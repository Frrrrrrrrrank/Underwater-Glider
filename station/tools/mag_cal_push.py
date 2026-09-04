#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过 UGCP下发磁力计标定参数
python station\tools\mag_cal_push.py station\logs\mag\mag_时间戳_calib.json --bind 192.168.57.100 --port 8800

PC 是 TCP 服务端，网口模块作为客户端接入。每次尝试严格执行：关闭遥测、
发送四个参数块、提交、重新打开遥测，再从新格式 $UGEKF 尾部等待 CRC/valid ACK。
"""

import argparse
import json
import os
import socket
import sys
import time


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STATION_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir))
REPO_ROOT = os.path.abspath(os.path.join(STATION_DIR, os.pardir))
if STATION_DIR not in sys.path:
    sys.path.insert(0, STATION_DIR)

from ugcp_protocol import (  # noqa: E402
    MSG_PARAM_BLOCK,
    MSG_PARAM_COMMIT,
    MSG_TELEM_CONTROL,
    PARAM_GROUP_MAG_CAL,
    SequenceCounter,
    build_frame,
    crc32_iso_hdlc,
    pack_mag_cal_table,
    pack_param_block_payload,
    pack_param_commit_payload,
    pack_telem_control_payload,
)


MAG_CAL_BLOCK_COUNT = 4
MAG_CAL_BLOCK_BYTES = 16
MAG_CAL_TABLE_BYTES = MAG_CAL_BLOCK_COUNT * MAG_CAL_BLOCK_BYTES
UGEKF_ACK_FIELD_COUNT = 33


def load_calibration_json(path, allow_failed_quality=False):
    """读取 fit JSON 并返回板端定义的 64 字节 MAG_CAL v1 表。"""
    with open(path, "r", encoding="utf-8") as stream:
        document = json.load(stream)

    if not isinstance(document, dict):
        raise ValueError("JSON 顶层必须是对象")
    schema = document.get("schema")
    if schema != "underwater_glider.mag_calibration":
        raise ValueError("schema 必须是 underwater_glider.mag_calibration")
    version = document.get("version")
    if type(version) is not int or version != 1:
        raise ValueError("仅支持 version=1")
    frame = document.get("coordinate_frame")
    if frame != "module_FLU":
        raise ValueError("coordinate_frame 必须是 module_FLU")
    unit = document.get("hard_iron_unit")
    if unit != "uT":
        raise ValueError("hard_iron_unit 必须是 uT")
    if document.get("soft_iron_order") != "row_major":
        raise ValueError("soft_iron_order 必须是 row_major")

    quality = document.get("quality")
    if not isinstance(quality, dict):
        raise ValueError("JSON 缺少 quality 对象")
    quality_class = quality.get("class")
    if quality_class not in ("pass", "marginal", "fail"):
        raise ValueError("quality.class 必须是 pass/marginal/fail")
    if quality_class == "fail" and not allow_failed_quality:
        raise ValueError(
            "标定质量为 fail，默认禁止下发；确认风险后可显式使用 --force")

    try:
        hard_iron = document["hard_iron_ut"]
        soft_iron = document["soft_iron"]
    except KeyError as exc:
        raise ValueError("JSON 缺少字段 %s" % exc) from exc

    table = pack_mag_cal_table(
        hard_iron,
        soft_iron,
        document.get("reserved", [0.0, 0.0, 0.0, 0.0]),
    )
    if len(table) != MAG_CAL_TABLE_BYTES:
        raise ValueError("MAG_CAL 表必须恰好为 64 字节")
    return table


def _send_message(conn, sequence, message_type, payload=b""):
    seq = sequence.next()
    frame = build_frame(message_type, payload, seq)
    conn.sendall(frame)
    return seq, frame


def send_transaction(conn, table_bytes, transaction, sequence=None,
                     inter_frame_delay=0.01, quiet_after_off=0.25):
    """发送一次完整事务并返回期望 CRC；ACK 由调用方另行等待。

    ``conn`` 只需提供 ``sendall``，便于单元测试使用捕获型 fake socket。
    """
    table_bytes = bytes(table_bytes)
    if len(table_bytes) != MAG_CAL_TABLE_BYTES:
        raise ValueError("table_bytes 必须恰好为 64 字节")
    transaction = int(transaction)
    if not 0 <= transaction <= 0xFF:
        raise ValueError("transaction 必须在 0..255")
    if sequence is None:
        sequence = SequenceCounter()

    expected_crc = crc32_iso_hdlc(table_bytes)
    _send_message(
        conn,
        sequence,
        MSG_TELEM_CONTROL,
        pack_telem_control_payload(False),
    )
    if quiet_after_off > 0:
        time.sleep(quiet_after_off)

    for block_index in range(MAG_CAL_BLOCK_COUNT):
        offset = block_index * MAG_CAL_BLOCK_BYTES
        block = table_bytes[offset:offset + MAG_CAL_BLOCK_BYTES]
        payload = pack_param_block_payload(
            PARAM_GROUP_MAG_CAL,
            transaction,
            block_index,
            block,
        )
        _send_message(conn, sequence, MSG_PARAM_BLOCK, payload)
        if inter_frame_delay > 0:
            time.sleep(inter_frame_delay)

    commit = pack_param_commit_payload(
        PARAM_GROUP_MAG_CAL,
        transaction,
        MAG_CAL_BLOCK_COUNT,
        expected_crc,
    )
    _send_message(conn, sequence, MSG_PARAM_COMMIT, commit)

    # ACK 挂在 $UGEKF 尾部，所以提交后必须先恢复遥测。
    _send_message(
        conn,
        sequence,
        MSG_TELEM_CONTROL,
        pack_telem_control_payload(True),
    )
    return expected_crc


def _nmea_checksum_ok(line):
    star = line.rfind("*")
    if star < 1 or star + 3 != len(line):
        return False
    checksum = 0
    for char in line[1:star]:
        checksum ^= ord(char)
    try:
        return checksum == int(line[star + 1:star + 3], 16)
    except ValueError:
        return False


def _parse_uint(text):
    text = text.strip()
    if text.lower().startswith("0x"):
        return int(text, 16)
    return int(text, 10)


def _parse_ugekf_ack(line):
    line = line.strip()
    if not line.startswith("$UGEKF") or not _nmea_checksum_ok(line):
        return None
    star = line.rfind("*")
    fields = line[1:star].split(",")
    if not fields or fields[0] != "UGEKF":
        return None
    if len(fields) - 1 != UGEKF_ACK_FIELD_COUNT:
        return None
    try:
        cal_crc = _parse_uint(fields[-2]) & 0xFFFFFFFF
        cal_valid = int(fields[-1])
    except ValueError:
        return None
    if cal_valid not in (0, 1):
        return None
    return cal_crc, cal_valid


def wait_for_ack(conn, expected_crc, timeout, initial_buffer=b"",
                 return_buffer=False):
    """流式等待匹配 ACK；可选择返回超时后尚未拼成整行的尾部缓存。"""
    expected_crc = int(expected_crc) & 0xFFFFFFFF
    timeout = float(timeout)
    if timeout <= 0:
        raise ValueError("timeout 必须大于 0")

    deadline = time.monotonic() + timeout
    buffer = bytes(initial_buffer)
    while True:
        while b"\n" in buffer:
            raw_line, buffer = buffer.split(b"\n", 1)
            line = raw_line.decode("ascii", errors="replace").strip("\r ")
            ack = _parse_ugekf_ack(line)
            if ack is None:
                continue
            cal_crc, cal_valid = ack
            print("[ACK] cal_crc=0x%08X cal_valid=%d" % (cal_crc, cal_valid))
            if cal_valid == 1 and cal_crc == expected_crc:
                if return_buffer:
                    return True, buffer
                return True

        remaining = deadline - time.monotonic()
        if remaining <= 0:
            if return_buffer:
                return False, buffer
            return False
        if hasattr(conn, "settimeout"):
            conn.settimeout(min(0.25, remaining))
        try:
            chunk = conn.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            raise ConnectionError("板端连接已断开")

        buffer += chunk


def _make_server(bind_ip, port):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((bind_ip, port))
    server.listen(1)
    return server


def _parse_args():
    parser = argparse.ArgumentParser(description="下发磁力计标定参数")
    parser.add_argument("calibration", help="mag_calib_fit 生成的 JSON")
    parser.add_argument("--bind", default="0.0.0.0", help="PC 监听地址")
    parser.add_argument("--port", type=int, default=8800)
    parser.add_argument("--timeout", type=float, default=2.0,
                        help="每次等待 ACK 的秒数")
    parser.add_argument("--retries", type=int, default=3,
                        help="完整事务最多尝试次数")
    parser.add_argument("--inter-frame-delay", type=float, default=0.01)
    parser.add_argument("--quiet-after-off", type=float, default=0.25)
    parser.add_argument("--transaction", type=int, default=None)
    parser.add_argument(
        "--force",
        action="store_true",
        help="明确允许下发 quality.class=fail 的标定（默认拒绝）",
    )
    return parser.parse_args()


def main():
    args = _parse_args()
    if args.retries < 1:
        raise SystemExit("--retries 必须至少为 1")
    if args.timeout <= 0:
        raise SystemExit("--timeout 必须大于 0")

    try:
        table = load_calibration_json(
            args.calibration,
            allow_failed_quality=args.force,
        )
    except (OSError, ValueError) as exc:
        print("[失败] 标定文件无效：%s" % exc)
        return 2
    expected_crc = crc32_iso_hdlc(table)
    transaction = args.transaction
    if transaction is None:
        transaction = int(time.monotonic_ns()) & 0xFF
    if not 0 <= transaction <= 0xFF:
        raise SystemExit("--transaction 必须在 0..255")

    print("[参数] table=64B txn=%d crc32=0x%08X" %
          (transaction, expected_crc))
    if args.force:
        print("[安全警告] 已启用 --force，允许覆盖拟合质量失败保护")
    print("[提示] 端口 8800 不能同时运行其他控制/记录工具")

    server = _make_server(args.bind, args.port)
    conn = None
    sequence = SequenceCounter()
    try:
        print("[服务端] 监听 %s:%d，等待板端连接..." %
              (args.bind, args.port))
        conn, address = server.accept()
        print("[连接] 板端已连接：%s" % (address,))

        ack_buffer = b""
        for attempt in range(1, args.retries + 1):
            print("[事务] 第 %d/%d 次尝试" % (attempt, args.retries))
            sent_crc = send_transaction(
                conn,
                table,
                transaction,
                sequence=sequence,
                inter_frame_delay=args.inter_frame_delay,
                quiet_after_off=args.quiet_after_off,
            )
            matched, ack_buffer = wait_for_ack(
                conn,
                sent_crc,
                args.timeout,
                initial_buffer=ack_buffer,
                return_buffer=True,
            )
            if matched:
                print("[成功] 板端已应用 CRC=0x%08X" % sent_crc)
                return 0
            print("[重试] 未收到匹配 ACK，整组重发")

        print("[失败] 超过最大重试次数")
        return 4
    except (OSError, ConnectionError, ValueError) as exc:
        print("[失败] %s" % exc)
        return 2
    finally:
        # 即使异常发生在 off 与 on 之间，也尽力恢复遥测。
        if conn is not None:
            try:
                _send_message(
                    conn,
                    sequence,
                    MSG_TELEM_CONTROL,
                    pack_telem_control_payload(True),
                )
            except OSError:
                pass
            try:
                conn.close()
            except OSError:
                pass
        server.close()


if __name__ == "__main__":
    sys.exit(main())
