"""
磁力计原始数据记录器（硬铁/软铁标定采集用，上位机/电脑端）

配合板端 EthernetUartDriver_SendMagTelemetry() 推送的 NMEA 风格行（~10Hz）：
    $UGMAG,seq,t_ms,mx,my,mz*HH
其中 mx,my,mz 为 IMU 模块系(FLU)原始磁场，单位 uT（模块量程假定见 imu_core.c，
全局缩放不影响标定：yaw 模型只用方向，A 矩阵会吸收每轴 scale）。

架构与 ekf_logger.py 一致：PC 作 TCP 服务端 bind:8800，板端网口模块作客户端连入。
注意 8800 同一时刻只能一个程序占用 —— 采集时不要同时开 contcp_t.py / ekf_logger.py。

针对标定现场的特性：
  - 实时覆盖度反馈：把 (m - 运行中心估计) 的方向按 [上/中/下]x12 个方位扇区分桶，
    终端实时显示哪些姿态区还没采到，指导现场"还要往哪个方向转"。
  - 双写：原始字节流 raw_mag_*.log（绝不丢），校验通过帧 mag_*.csv。
  - seq 丢帧/板重启检测。

用法：
    python station/tools/mag_logger.py [绑定IP] [--port 8800] [--outdir DIR]
例：
    python station/tools/mag_logger.py                       # 绑定 0.0.0.0:8800
    python station/tools/mag_logger.py 192.168.57.100
采集完 Ctrl+C 退出，然后用 mag_calib_fit.py 拟合。
"""

import argparse
import math
import os
import socket
import sys
import time
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STATION_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir))
if STATION_DIR not in sys.path:
    sys.path.insert(0, STATION_DIR)

from ugcp_protocol import (  # noqa: E402
    MSG_STREAM_CONTROL,
    STREAM_MAG,
    SequenceCounter,
    build_frame,
    pack_stream_control_payload,
)


DEFAULT_OUTDIR = os.path.join(STATION_DIR, "logs", "mag")

CSV_HEADER = ["host_iso", "host_epoch", "seq", "t_ms", "mx", "my", "mz", "norm"]

# 覆盖度分桶：仰角 3 带（>30° / ±30° / <-30°） x 方位 12 扇区(每 30°)
N_AZ = 12
STREAM_RETRY_SECONDS = 2.0


def nmea_checksum_ok(line):
    """校验 $...*HH：异或 $ 与 * 之间的所有字符，与十六进制 HH 比较。"""
    star = line.rfind("*")
    if star < 1 or star + 3 > len(line):
        return False
    cks = 0
    for ch in line[1:star]:
        cks ^= ord(ch)
    try:
        return cks == int(line[star + 1:star + 3], 16)
    except ValueError:
        return False


def parse_ugmag(line):
    """解析一行 $UGMAG,seq,t_ms,mx,my,mz*HH，返回 (seq,t_ms,mx,my,mz)；格式不符返回 None。"""
    star = line.rfind("*")
    parts = line[1:star].split(",")
    if len(parts) != 6 or parts[0] != "UGMAG":
        return None
    try:
        return (int(parts[1]), int(parts[2]),
                float(parts[3]), float(parts[4]), float(parts[5]))
    except ValueError:
        return None


class Coverage(object):
    """姿态覆盖度统计：用每轴 (min+max)/2 做运行硬铁中心粗估，
    对 (m - center) 的方向分桶计数。中心估计随数据更新，桶计数只增不减，
    所以显示是"已探索方向"的近似指示，不追求精确。"""

    def __init__(self):
        self.mn = [float("inf")] * 3
        self.mx = [float("-inf")] * 3
        self.bins = [[0] * N_AZ for _ in range(3)]  # [上/中/下][方位]

    def add(self, m):
        # 固件 AppendFixed 对非有限值输出 "nan"，此处跳过以免污染 min/max 与分桶
        if not all(math.isfinite(x) for x in m):
            return
        for i in range(3):
            if m[i] < self.mn[i]:
                self.mn[i] = m[i]
            if m[i] > self.mx[i]:
                self.mx[i] = m[i]
        c = [(self.mn[i] + self.mx[i]) * 0.5 for i in range(3)]
        v = [m[i] - c[i] for i in range(3)]
        n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        if n < 1e-9:
            return
        elev = math.asin(max(-1.0, min(1.0, v[2] / n)))   # 磁场向量仰角
        az = math.atan2(v[1], v[0])                        # 方位角 [-pi,pi)
        band = 0 if elev > math.radians(30) else (2 if elev < math.radians(-30) else 1)
        k = int((az + math.pi) / (2 * math.pi) * N_AZ) % N_AZ
        self.bins[band][k] += 1

    def bar(self):
        """三行覆盖度字符条：#=该扇区已有样本，.=还没有。"""
        names = ["mz+带", "水平带", "mz-带"]
        lines = []
        for b in range(3):
            cells = "".join("#" if self.bins[b][k] > 0 else "." for k in range(N_AZ))
            filled = sum(1 for k in range(N_AZ) if self.bins[b][k] > 0)
            lines.append("  %s [%s] %2d/%d" % (names[b], cells, filled, N_AZ))
        return "\n".join(lines)


def make_server(bind_ip, port):
    """PC 作 TCP 服务端，板端网口模块（客户端）连入 —— 与 ekf_logger.py 相同架构。"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((bind_ip, port))
    srv.listen(1)
    srv.settimeout(1.0)
    print("[服务端] 已绑定 %s:%d，等待板子连接..." % (bind_ip, port))
    print("[提示] 若报 WinError 10049：PC 网卡需设静态 IP 192.168.57.100/24")
    return srv


def send_stream_control(conn, sequence, mask):
    """向板端声明完整遥测状态，返回本次 UGCP sequence。"""
    seq = sequence.next()
    conn.sendall(build_frame(
        MSG_STREAM_CONTROL,
        pack_stream_control_payload(mask),
        seq,
    ))
    return seq


def stream_retry_due(last_valid_time, last_request_time, now):
    """合法数据和最近请求都已沉默满 2 秒时才重申，避免请求风暴。"""
    return (
        now - last_valid_time >= STREAM_RETRY_SECONDS and
        now - last_request_time >= STREAM_RETRY_SECONDS
    )


def main():
    ap = argparse.ArgumentParser(description="磁标定原始数据记录器（PC 作 TCP 服务端）")
    ap.add_argument("bind", nargs="?", default="0.0.0.0",
                    help="本机绑定IP，默认0.0.0.0(所有网卡)；也可填 192.168.57.100")
    ap.add_argument("--port", type=int, default=8800, help="TCP 端口(默认8800)")
    ap.add_argument("--outdir", default=DEFAULT_OUTDIR, help="日志输出目录(默认 station/logs/mag)")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = os.path.join(args.outdir, "raw_mag_%s.log" % stamp)
    csv_path = os.path.join(args.outdir, "mag_%s.csv" % stamp)
    print("[文件] 原始流 -> %s" % raw_path)
    print("[文件] CSV   -> %s" % csv_path)

    raw_f = open(raw_path, "ab")
    csv_f = open(csv_path, "w", encoding="utf-8", newline="")
    csv_f.write(",".join(CSV_HEADER) + "\n")
    csv_f.flush()

    cov = Coverage()
    n_ok = 0
    n_bad = 0
    n_drop = 0
    last_print = time.time()
    last_m = (0.0, 0.0, 0.0)

    srv = make_server(args.bind, args.port)
    tx_sequence = SequenceCounter()
    try:
        while True:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            print("[连接] 板子已连接：%s" % (addr,))
            conn.settimeout(1.0)
            buf = b""
            last_seq = None
            try:
                stream_seq = send_stream_control(conn, tx_sequence, STREAM_MAG)
                print("[遥测] 已请求 MAG-only，seq=%d；等待首个合法 $UGMAG" % stream_seq)
                last_stream_request = time.monotonic()
                last_valid_mag = last_stream_request
                while True:
                    now_mono = time.monotonic()
                    if stream_retry_due(
                            last_valid_mag, last_stream_request, now_mono):
                        stream_seq = send_stream_control(
                            conn, tx_sequence, STREAM_MAG)
                        print("[遥测] 连续 2 秒无合法 $UGMAG，重发 MAG-only，seq=%d"
                              % stream_seq)
                        last_stream_request = now_mono

                    try:
                        chunk = conn.recv(4096)
                    except socket.timeout:
                        chunk = None
                    if chunk == b"":
                        print("[连接] 板子断开，回到等待连接")
                        break
                    if chunk:
                        raw_f.write(chunk)
                        raw_f.flush()
                        buf += chunk
                        while b"\n" in buf:
                            rawline, buf = buf.split(b"\n", 1)
                            line = rawline.decode("ascii", errors="replace").strip("\r\n ")
                            if not line.startswith("$UGMAG"):
                                continue  # $UGEKF / AT 回显等，忽略
                            if not nmea_checksum_ok(line):
                                n_bad += 1
                                continue
                            row = parse_ugmag(line)
                            if row is None:
                                n_bad += 1
                                continue
                            seq, t_ms, mx, my, mz = row
                            if last_seq is not None:
                                if seq < last_seq:
                                    print("[事件] seq 回退(%d->%d)，疑似板子重启"
                                          % (last_seq, seq))
                                elif seq > last_seq + 1:
                                    n_drop += seq - last_seq - 1
                            last_seq = seq

                            norm = math.sqrt(mx * mx + my * my + mz * mz)
                            csv_f.write("%s,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f\n"
                                        % (datetime.now().isoformat(timespec="milliseconds"),
                                           time.time(), seq, t_ms, mx, my, mz, norm))
                            csv_f.flush()
                            n_ok += 1
                            last_valid_mag = time.monotonic()
                            last_m = (mx, my, mz)
                            cov.add((mx, my, mz))

                    # 周期状态：样本数 + 当前读数 + 覆盖度条
                    now = time.time()
                    if now - last_print >= 2.0:
                        norm = math.sqrt(sum(x * x for x in last_m))
                        print("[状态] 样本=%d 坏帧=%d 丢帧=%d | 当前 m=(%.1f, %.1f, %.1f) |m|=%.1f uT"
                              % (n_ok, n_bad, n_drop,
                                 last_m[0], last_m[1], last_m[2], norm))
                        print(cov.bar())
                        last_print = now
            except OSError as e:
                print("[连接] 异常(%s)，回到等待连接" % e)
            finally:
                try:
                    send_stream_control(conn, tx_sequence, 0)
                    print("[遥测] 已请求关闭 MAG 流")
                except (OSError, ValueError):
                    pass
                try:
                    conn.close()
                except OSError:
                    pass
    except KeyboardInterrupt:
        print("\n[退出] 共写入 %d 帧（坏 %d，丢 %d）" % (n_ok, n_bad, n_drop))
        print("[下一步] python station/tools/mag_calib_fit.py %s" % csv_path)
    finally:
        try:
            srv.close()
        except OSError:
            pass
        raw_f.close()
        csv_f.close()


if __name__ == "__main__":
    main()
