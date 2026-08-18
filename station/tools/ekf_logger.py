#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF 状态估计遥测记录器（上位机 / 电脑端）

配合板端 EthernetUartDriver_SendEkfTelemetry() 推送的 NMEA 风格行：
    $UGEKF,seq,t_ms,status,fl,roll,pitch,yaw,pN,pE,depth,vN,vE,vD,
           bgx,bgy,bgz,bax,bay,baz,Ppos,Pvel,Patt,
           nisD,accD,rejD,dstreak,dreset,dmeas,dt,imuhz,tlast,cal_crc,cal_valid*HH

设计要点（面向真机下水）：
  - 本脚本在 PC 上作 TCP server（默认端口 8800），板上的 UART 转以太网模块作 client 连入。
  - 自动重接：板子复位 / 缆松动会断 TCP，脚本回到 accept 等待模块再次连入。
  - 双写：收到的原始字节立刻 append 到 raw_*.log 并 flush（绝不丢原始流）；
          解析校验通过的帧另写 ekf_*.csv（带电脑墙钟接收时间戳，便于离线对齐/绘图）。
  - 容错：非 $UGEKF 行（AT 回显、OK、心跳等）直接跳过；异或校验失败计数但不写 CSV。
  - 掉帧 / 重启检测：用 seq 连续性判丢帧，用 seq 归零判板子重启。

用法：
    python station/tools/ekf_logger.py [PC本机绑定IP] [--port 8800] [--outdir DIR]
说明：
    [...] 表示可选参数，不要照抄方括号；端口用 --port 指定。
    --outdir 是日志输出目录，不填默认 station/logs/ekf（脚本自动创建）。
例：
    python station/tools/ekf_logger.py 0.0.0.0 --port 8800
    python station/tools/ekf_logger.py 192.168.57.100 --port 8800 --outdir D:\glider_logs
"""

import argparse
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
    STREAM_EKF,
    SequenceCounter,
    build_frame,
    pack_stream_control_payload,
)


DEFAULT_OUTDIR = os.path.join(STATION_DIR, "logs", "ekf")

# 板端 $UGEKF 之后的字段顺序，必须与 ethernet_uart_driver.c 的构帧完全一致
BASE_FIELDS = [
    "seq", "t_ms", "status", "fl",
    "roll_deg", "pitch_deg", "yaw_deg",
    "pN", "pE", "depth",
    "vN", "vE", "vD",
    "bgx", "bgy", "bgz",
    "bax", "bay", "baz",
    "Ppos", "Pvel", "Patt",
    "nisD", "accD", "rejD", "dstreak", "dreset",
    "dmeas", "dt", "imuhz", "tlast",
]
ACK_FIELDS = ["cal_crc", "cal_valid"]
FIELDS = BASE_FIELDS + ACK_FIELDS

# 整数字段（其余按 float 解析，'nan' 也能被 float() 接受）
INT_FIELDS = {"seq", "t_ms", "status", "fl",
              "accD", "rejD", "dstreak", "dreset",
              "cal_crc", "cal_valid"}

# out.status 枚举（与 UgEkfStatus 对应）
STATUS_NAMES = {
    0: "UNINIT", 1: "WAIT_BOOT_DELAY", 2: "WAIT_STILL",
    3: "ALIGNING", 4: "RUNNING", 5: "FAULT",
}
STREAM_RETRY_SECONDS = 2.0

# CSV 表头：墙钟时间 + fl 展开成 5 个布尔列 + 其余字段
CSV_HEADER = (
    ["host_iso", "host_epoch"]
    + ["seq", "t_ms", "status", "status_name",
       "healthy", "valid", "still", "finite", "aligned"]
    + FIELDS[4:]  # 从 roll_deg 起的全部数值字段
)


def nmea_checksum_ok(line):
    """校验 $...*HH：异或 $ 与 * 之间的所有字符，与十六进制 HH 比较。"""
    star = line.rfind("*")
    if star < 1 or star + 3 > len(line):
        return False
    body = line[1:star]
    cks = 0
    for ch in body:
        cks ^= ord(ch)
    try:
        return cks == int(line[star + 1:star + 3], 16)
    except ValueError:
        return False


def parse_ugekf(line):
    """解析旧 31 字段或带 cal_crc/cal_valid 的新 33 字段 $UGEKF。"""
    star = line.rfind("*")
    parts = line[1:star].split(",")  # 去掉 '$'，去掉 '*HH'
    if not parts or parts[0] != "UGEKF":
        return None
    vals = parts[1:]
    if len(vals) == len(BASE_FIELDS):
        frame_fields = BASE_FIELDS
    elif len(vals) == len(FIELDS):
        frame_fields = FIELDS
    else:
        return None  # 字段数对不上，可能截断帧
    row = {}
    try:
        for name, raw in zip(frame_fields, vals):
            if name in INT_FIELDS:
                row[name] = int(raw)
            else:
                row[name] = float(raw)  # 'nan' 也可被解析
    except ValueError:
        return None
    for name in ACK_FIELDS:
        row.setdefault(name, None)
    return row


def make_csv_row(host_iso, host_epoch, row):
    """把解析结果 + fl 位展开成与 CSV_HEADER 对齐的列表。"""
    fl = row["fl"]
    out = [
        host_iso, "%.3f" % host_epoch,
        row["seq"], row["t_ms"], row["status"],
        STATUS_NAMES.get(row["status"], "?"),
        (fl >> 0) & 1, (fl >> 1) & 1, (fl >> 2) & 1,
        (fl >> 3) & 1, (fl >> 4) & 1,
    ]
    out.extend(row[name] for name in FIELDS[4:])
    return out


def make_server(bind_ip, port):
    """创建并返回监听中的 TCP 服务端 socket。
    架构：PC 作服务端，板子的网口模块作客户端连进来（与 contcp_t.py 一致）。"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((bind_ip, port))
    srv.listen(1)
    srv.settimeout(1.0)  # accept 1s 超时，便于响应 Ctrl+C
    print("[服务端] 已绑定 %s:%d，等待板子连接..." % (bind_ip, port))
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
    ap = argparse.ArgumentParser(description="EKF 遥测记录器（PC 作 TCP 服务端）")
    ap.add_argument("bind", nargs="?", default="0.0.0.0",
                    help="本机绑定IP，默认0.0.0.0(所有网卡)；也可填 192.168.57.100")
    ap.add_argument("--port", type=int, default=8800, help="TCP 端口(默认8800)")
    ap.add_argument("--outdir", default=DEFAULT_OUTDIR, help="日志输出目录(默认 station/logs/ekf)")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = os.path.join(args.outdir, "raw_%s.log" % stamp)
    csv_path = os.path.join(args.outdir, "ekf_%s.csv" % stamp)
    print("[文件] 原始流 -> %s" % raw_path)
    print("[文件] CSV   -> %s" % csv_path)

    raw_f = open(raw_path, "ab")
    csv_f = open(csv_path, "w", encoding="utf-8", newline="")
    csv_f.write(",".join(CSV_HEADER) + "\n")
    csv_f.flush()

    n_ok = 0          # 成功写入 CSV 的帧数
    n_bad = 0         # 校验失败帧数
    n_drop = 0        # 累计丢帧数（seq 跳变）
    last_print = time.time()
    last_ok_count = 0

    srv = make_server(args.bind, args.port)
    tx_sequence = SequenceCounter()
    try:
        while True:
            # —— 等板子连接（板复位/断缆会断开，循环重新 accept） ——
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            print("[连接] 板子已连接：%s" % (addr,))
            conn.settimeout(1.0)
            buf = b""
            last_seq = None     # 每条连接重置：板复位后 seq 会归零
            try:
                stream_seq = send_stream_control(conn, tx_sequence, STREAM_EKF)
                print("[遥测] 已请求 EKF-only，seq=%d" % stream_seq)
                last_stream_request = time.monotonic()
                last_valid_ekf = last_stream_request
                while True:
                    now_mono = time.monotonic()
                    if stream_retry_due(
                            last_valid_ekf, last_stream_request, now_mono):
                        stream_seq = send_stream_control(
                            conn, tx_sequence, STREAM_EKF)
                        print("[遥测] 连续 2 秒无合法 $UGEKF，重发 EKF-only，seq=%d"
                              % stream_seq)
                        last_stream_request = now_mono

                    try:
                        chunk = conn.recv(4096)
                    except socket.timeout:
                        # 无数据：借机打印状态心跳
                        now = time.time()
                        if now - last_print >= 2.0:
                            rate = (n_ok - last_ok_count) / (now - last_print)
                            print("[状态] ok=%d bad=%d drop=%d  ~%.1f Hz"
                                  % (n_ok, n_bad, n_drop, rate))
                            last_print = now
                            last_ok_count = n_ok
                        continue
                    if not chunk:
                        print("[连接] 板子断开，回到等待连接")
                        break  # 回到 accept

                    raw_f.write(chunk)
                    raw_f.flush()  # 原始流立即落盘，绝不丢

                    buf += chunk
                    while b"\n" in buf:
                        rawline, buf = buf.split(b"\n", 1)
                        line = rawline.decode("ascii", errors="replace").strip("\r\n ")
                        if not line.startswith("$UGEKF"):
                            continue  # AT 回显 / OK / 心跳等，忽略
                        if not nmea_checksum_ok(line):
                            n_bad += 1
                            continue
                        row = parse_ugekf(line)
                        if row is None:
                            n_bad += 1
                            continue

                        # 丢帧 / 重启检测
                        seq = row["seq"]
                        if last_seq is not None:
                            if seq < last_seq:
                                print("[事件] seq 回退(%d->%d)，疑似板子重启"
                                      % (last_seq, seq))
                            elif seq > last_seq + 1:
                                n_drop += seq - last_seq - 1
                        last_seq = seq

                        host_epoch = time.time()
                        host_iso = datetime.now().isoformat(timespec="milliseconds")
                        out = make_csv_row(host_iso, host_epoch, row)
                        csv_f.write(",".join(str(x) for x in out) + "\n")
                        csv_f.flush()  # 边采边落盘，掉电最多丢最后一行
                        n_ok += 1
                        last_valid_ekf = time.monotonic()
            except OSError as e:
                print("[连接] 异常(%s)，回到等待连接" % e)
            finally:
                try:
                    send_stream_control(conn, tx_sequence, 0)
                    print("[遥测] 已请求关闭 EKF 流")
                except (OSError, ValueError):
                    pass
                try:
                    conn.close()
                except OSError:
                    pass
    except KeyboardInterrupt:
        print("\n[退出] Ctrl+C，已写 %d 帧（坏 %d，丢 %d）" % (n_ok, n_bad, n_drop))
    finally:
        try:
            srv.close()
        except OSError:
            pass
        raw_f.close()
        csv_f.close()


if __name__ == "__main__":
    main()
