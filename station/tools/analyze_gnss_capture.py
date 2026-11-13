"""离线分析 UGRAW CSV/JSONL/raw.log；输出 summary.json 与逐记录 epochs.csv。

频率以实际记录的 MCU 接收时刻为依据，队列耗时为同一 MCU 时钟 tx-rx。
--static 仅计算用户声明静止时的重复性，不推断绝对定位精度或 GNSS 绝对延迟。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import struct
import sys
from collections import Counter
from datetime import date, datetime, timedelta
from pathlib import Path

STATION = Path(__file__).resolve().parents[1]
if str(STATION) not in sys.path:
    sys.path.insert(0, str(STATION))
from raw_telemetry import ugraw_parse

UINT32 = 1 << 32
COUNTERS = ("queue_drop", "uart_drop", "line_drop")
CSV_FIELDS = (
    "record_index", "source_file", "source_line", "segment_id", "capture_id", "seq",
    "rx_us", "tx_us", "t_ns", "pc_time", "queue_delay_ms", "queue_drop", "uart_drop",
    "line_drop", "flags", "src_len", "kept_len", "sentence", "talker", "formatter",
    "checksum_status", "rmc_status", "firmware_status", "reason", "utc_date", "utc_ms",
    "epoch_iso", "epoch_disposition", "lat_deg_e7", "lon_deg_e7", "speed_mps",
    "course_deg", "velocity_valid", "vel_n_mps", "vel_e_mps", "quality_source",
    "normalization", "fields_json", "payload_hex",
)


def fp32(value):
    """匹配固件可选速度字段的 float32 舍入和 0.3m/s 边界。"""
    return struct.unpack("<f", struct.pack("<f", value))[0]


def parse_utc(value):
    if not re.fullmatch(r"[0-9]{6}(?:\.[0-9]{1,3})?", value):
        raise ValueError("utc_format")
    h, m, s = int(value[:2]), int(value[2:4]), int(value[4:6])
    if h > 23 or m > 59 or s > 59:
        raise ValueError("utc_range")
    fraction = int(value[7:].ljust(3, "0")) if len(value) > 6 else 0
    return ((h * 60 + m) * 60 + s) * 1000 + fraction


def parse_date(value):
    if not re.fullmatch(r"[0-9]{6}", value):
        raise ValueError("date_format")
    try:
        return date(2000 + int(value[4:6]), int(value[2:4]), int(value[:2]))
    except ValueError:
        raise ValueError("date_range") from None


def coordinate(value, direction, longitude=False):
    digits = 5 if longitude else 4
    if not re.fullmatch(r"[0-9]{%d}(?:\.[0-9]{1,9})?" % digits, value):
        raise ValueError("coordinate_format")
    if direction not in (("E", "W") if longitude else ("N", "S")):
        raise ValueError("coordinate_direction")
    parts = value.split(".")
    whole, fraction = int(parts[0]), int(parts[1]) if len(parts) > 1 else 0
    scale = 10 ** len(parts[1]) if len(parts) > 1 else 1
    degree, minute = divmod(whole, 100)
    limit = 180 if longitude else 90
    if minute >= 60 or degree > limit or (degree == limit and (minute or fraction)):
        raise ValueError("coordinate_range")
    result = degree * 10000000 + ((minute * scale + fraction) * 10000000 + 30 * scale) // (60 * scale)
    return -result if direction in ("S", "W") else result


def unsigned_decimal(value):
    if not re.fullmatch(r"[0-9]{1,9}(?:\.[0-9]{1,9})?", value):
        raise ValueError("velocity_format")
    parts = value.split(".")
    fraction = fp32(fp32(int(parts[1])) / fp32(10 ** len(parts[1]))) if len(parts) > 1 else 0
    return fp32(fp32(int(parts[0])) + fraction)


def classify_payload(payload, flags=0):
    """复核原 core 的去 CR/C 字符串和严格 GNRMC builder，不声称已被 EKF 接受。"""
    normalized = payload.replace(b"\r", b"").split(b"\x00", 1)[0].strip(b" \t\n\v\f")
    text = normalized.decode("latin1")
    header = text[1:].split(",", 1)[0].split("*", 1)[0] if text.startswith("$") else ""
    result = dict(sentence=header, talker=header[:2], formatter=header[2:] if len(header) == 5 else "",
                  checksum_status="not_nmea", rmc_status="not_rmc", firmware_status="not_rmc",
                  reason="", velocity_valid=False, quality_source="unknown",
                  normalization=";".join(name for name, present in
                      (("cr_removed", b"\r" in payload), ("nul_terminated", b"\x00" in payload)) if present))
    if flags & 1:
        result.update(rmc_status="unknown", firmware_status="unknown", reason="capture_truncated")
        return result
    if not text.startswith("$"):
        result["reason"] = "not_nmea"
        return result
    body, star, trailer = text[1:].partition("*")
    fields = body.split(",")
    result["fields_json"] = json.dumps(fields, ensure_ascii=True, separators=(",", ":"))
    if not star:
        result["checksum_status"] = "missing"
    elif not re.fullmatch(r"[0-9A-Fa-f]{2}", trailer):
        result["checksum_status"] = "malformed"
    else:
        checksum = 0
        for byte in body.encode("latin1"):
            checksum ^= byte
        result["checksum_status"] = "valid" if checksum == int(trailer, 16) else "mismatch"
    if result["formatter"] != "RMC":
        return result
    result.update(rmc_status="rejected", firmware_status="rejected")
    if result["checksum_status"] != "valid":
        result["reason"] = "checksum_" + result["checksum_status"]
        return result
    try:
        if len(fields) < 10:
            raise ValueError("missing_fields")
        utc_ms, day = parse_utc(fields[1]), parse_date(fields[9])
        stamp = datetime.combine(day, datetime.min.time()) + timedelta(milliseconds=utc_ms)
        result.update(utc_date=day.year * 10000 + day.month * 100 + day.day, utc_ms=utc_ms,
                      epoch_iso=stamp.isoformat(timespec="milliseconds") + "Z",
                      _epoch=day.toordinal() * 86400000 + utc_ms)
        if fields[2] not in ("A", "V"):
            raise ValueError("status")
        no_fix = fields[2] == "V"
        if len(fields) > 12 and fields[12]:
            mode = fields[12]
            if mode == "N":
                no_fix = True
            elif mode not in ("A", "D", "F", "R", "P") and not (no_fix and mode in ("E", "M", "S")):
                raise ValueError("mode")
        if len(fields) > 13 and fields[13]:
            status = fields[13]
            if status == "V":
                no_fix = True
            elif status != "S" and not (no_fix and status in ("C", "U")):
                raise ValueError("navigation_status")
        if len(fields) > 14:
            raise ValueError("extra_fields")
        if no_fix:
            result["rmc_status"] = "no_fix"
        else:
            result.update(lat_deg_e7=coordinate(fields[3], fields[4]),
                          lon_deg_e7=coordinate(fields[5], fields[6], True), rmc_status="position")
            try:
                knots, course = unsigned_decimal(fields[7]), unsigned_decimal(fields[8])
                speed = fp32(knots * fp32(1852.0 / 3600.0))
                if course >= 360 or speed < fp32(.3):
                    raise ValueError("velocity_range_or_low_speed")
                angle = fp32(course * fp32(.01745329251994329577))
                result.update(speed_mps=speed, course_deg=course, velocity_valid=True,
                              vel_n_mps=fp32(speed * fp32(math.cos(angle))),
                              vel_e_mps=fp32(speed * fp32(math.sin(angle))))
            except ValueError:
                pass  # 坏速度不污染独立的位置有效性。
        result["firmware_status"] = result["rmc_status"] if header == "GNRMC" else "unsupported_talker"
        if header != "GNRMC":
            result["reason"] = "firmware_requires_GNRMC"
    except ValueError as exc:
        result["reason"] = str(exc)
    return result


def choose_sources(path):
    """一个会话只选择一种主数据源；同族带时间戳 CSV 视为分片。"""
    path = Path(path).resolve()
    if path.is_file():
        return [path], []
    if not path.is_dir():
        raise ValueError("输入路径不存在: " + str(path))
    families = ("telemetry_ugraw*.csv", "uart7_raw*.csv", "events*.jsonl", "raw*.log")
    all_sources = sorted({file for pattern in families for file in path.glob(pattern) if file.is_file()})
    for pattern in families:
        chosen = sorted(file for file in path.glob(pattern) if file.is_file())
        if chosen:
            return chosen, [file for file in all_sources if file not in chosen]
    raise ValueError("目录中没有 UGRAW CSV、events*.jsonl 或 raw*.log")


def file_events(source, problems):
    """统一日志容器；保留原文件顺序，不按 UTC 排序掩盖乱序。"""
    with source.open(encoding="utf-8-sig", errors="replace", newline="") as handle:
        if source.suffix.lower() == ".csv":
            for index, row in enumerate(csv.DictReader(handle), 2):
                yield index, row
        elif source.suffix.lower() in (".jsonl", ".ndjson"):
            for index, line in enumerate(handle, 1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                    if not isinstance(row, dict):
                        raise ValueError("记录不是对象")
                    yield index, row
                except (ValueError, TypeError) as exc:
                    problems.append({"file": str(source), "line": index, "reason": str(exc)})
        else:
            for index, line in enumerate(handle, 1):
                start = line.find("$UGRAW")
                if start >= 0:
                    # 外包原样交严格解析器；rstrip 会把双 CR 坏包悄悄修成有效帧。
                    yield index, {"raw": line[start:], "received_at": line[:start].strip()}


def raw_text(event):
    row = event.get("row") if isinstance(event.get("row"), dict) else event
    raw = event.get("raw") or row.get("raw")
    return raw if isinstance(raw, str) else ""


def recorder_end(event):
    return (event.get("type") == "session.ended" or
            event.get("kind") in ("recording_end", "recorder_end", "session_end"))


def receiver_incomplete(event):
    """JSON 使用 bool，CSV 使用文本；空列/缺列仍表示未知而非明确残片。"""
    value = event.get("complete")
    return value is False or value == 0 or (isinstance(value, str) and value.strip().lower() in ("false", "0"))


def load_capture(path):
    sources, ignored = choose_sources(path)
    records, problems, ends, invalid, non_capture = [], [], [], 0, 0
    for source in sources:
        for number, event in file_events(source, problems):
            if recorder_end(event):
                ends.append(event)
            raw = raw_text(event)
            if not raw.startswith("$UGRAW"):
                if source.suffix.lower() == ".csv" or event.get("topic") == "ugraw":
                    invalid += 1
                else:
                    non_capture += 1
                continue
            # CLI CSV 的空 schema 表示接收端未认可该外包（包括仅缺 LF 的断线残片）。
            receiver_invalid = receiver_incomplete(event) or ("schema" in event and event["schema"] in ("", None))
            row = ugraw_parse(raw) if not receiver_invalid else None
            if row is None:
                invalid += 1
                continue
            row = dict(row)
            try:
                t_ns = event.get("t_ns")
                row["t_ns"] = int(t_ns) if re.fullmatch(r"[0-9]+", str(t_ns)) else None
            except (ValueError, TypeError):
                row["t_ns"] = None
            row.update(pc_time=event.get("host_iso") or event.get("received_at") or "",
                       source_file=str(source), source_line=number)
            records.append(row)
    metadata = dict(selected_sources=[str(p) for p in sources], ignored_sources=[str(p) for p in ignored],
                    invalid_outer_primary=invalid, unrelated_records=non_capture, container_errors=problems)
    base = Path(path) if Path(path).is_dir() else Path(path).parent
    sidecar = base / "session.json"
    metadata["pc_recorder"] = None
    if sidecar.is_file():
        try:
            value = json.loads(sidecar.read_text(encoding="utf-8-sig"))
            if not isinstance(value, dict):
                raise ValueError("session.json 不是对象")
            metadata["pc_recorder"] = value
        except (ValueError, OSError) as exc:
            problems.append({"file": str(sidecar), "reason": str(exc)})
    # 主 CSV 只含解析成功的外包；另审计一个事件/原始日志族，仅数坏包，不重复纳入有效帧。
    audit = []
    if sources[0].suffix.lower() == ".csv":
        for pattern in ("events*.jsonl", "raw*.log"):
            audit = sorted(base.glob(pattern))
            if audit:
                break
    metadata["invalid_outer_audit"] = None
    metadata["outer_audit_sources"] = [str(p) for p in audit]
    metadata["recorder_end_events"] = ends
    if audit:
        bad = 0
        for source in audit:
            for _, event in file_events(source, problems):
                raw = raw_text(event)
                if raw.startswith("$UGRAW") and (ugraw_parse(raw) is None or receiver_incomplete(event)):
                    bad += 1
                if recorder_end(event):
                    metadata["recorder_end_events"].append(event)
        metadata["invalid_outer_audit"] = bad
    return records, metadata


def distribution(values):
    values = sorted(values)
    if not values:
        return {"count": 0, "min": None, "median": None, "p95": None, "max": None}
    position = (len(values) - 1) * .95
    low = int(position)
    p95 = values[low] + (values[min(low + 1, len(values) - 1)] - values[low]) * (position - low)
    return {"count": len(values), "min": values[0], "median": statistics.median(values),
            "p95": p95, "max": values[-1]}


def timing(groups, scale, gap_s):
    """不跨分段计算间隔；零间隔保留计数，不能拿 1/0 宣称无限频率。"""
    differences, count, pairs, elapsed = [], 0, 0, 0
    for values in groups:
        count += len(values)
        if len(values) >= 2:
            differences.extend((b - a) / scale for a, b in zip(values, values[1:]))
            pairs += len(values) - 1
            elapsed += (values[-1] - values[0]) / scale
    return {"samples": count, "intervals_s": distribution(differences),
            "observed_rate_hz": pairs / elapsed if elapsed > 0 else None,
            "zero_intervals": sum(value == 0 for value in differences),
            "gap_threshold_s": gap_s, "gap_count": sum(value > gap_s for value in differences)}


def static_repeatability(points):
    if not points:
        return {"samples": 0, "reason": "no_independent_valid_positions"}
    # WGS84 ECEF 差投影到第一点 NE，跨日期变更线也不会跳到地球另一边。
    lat0, lon0 = (math.radians(v / 1e7) for v in points[0])
    def ecef(lat, lon):
        n = 6378137.0 / math.sqrt(1 - 6.6943799901413165e-3 * math.sin(lat) ** 2)
        return n * math.cos(lat) * math.cos(lon), n * math.cos(lat) * math.sin(lon), n * (1 - 6.6943799901413165e-3) * math.sin(lat)
    anchor = ecef(lat0, lon0)
    north, east = [], []
    for lat_e7, lon_e7 in points:
        pos = ecef(math.radians(lat_e7 / 1e7), math.radians(lon_e7 / 1e7))
        dx, dy, dz = (p - a for p, a in zip(pos, anchor))
        if math.sqrt(dx * dx + dy * dy + dz * dz) > 100000:
            return {"samples": len(points), "reason": "extent_exceeds_100km_check_static_selection"}
        north.append(-math.sin(lat0) * math.cos(lon0) * dx - math.sin(lat0) * math.sin(lon0) * dy + math.cos(lat0) * dz)
        east.append(-math.sin(lon0) * dx + math.cos(lon0) * dy)
    cn, ce = statistics.median(north), statistics.median(east)
    return {"samples": len(points), "reference_lat_e7": points[0][0], "reference_lon_e7": points[0][1],
            "center_n_m": cn, "center_e_m": ce,
            "north_sample_std_m": statistics.stdev(north) if len(north) > 1 else None,
            "east_sample_std_m": statistics.stdev(east) if len(east) > 1 else None,
            "radius_about_median_m": distribution([math.hypot(n - cn, e - ce) for n, e in zip(north, east)]),
            "meaning": "用户声明静止的重复性；无真值，非绝对精度；未自动剔除离群点"}


def analyze_records(records, metadata=None, static=False, gap_s=5.0):
    metadata = dict(metadata or {})
    rows, segments = [], []
    counts, reasons, sentences, checksums = Counter(), Counter(), Counter(), Counter()
    seen = set()
    previous, segment, highwater = None, None, None
    queue_delays, points = [], []
    no_fix_recoveries, position_gaps = [], []
    pending_no_fix, previous_fix = None, None
    duplicate_capture = 0
    loss = Counter()
    for record in records:
        boundary = None
        if previous is None:
            boundary = "start"
        elif record["capture_id"] != previous["capture_id"]:
            boundary = "capture_changed"
        elif record["rx_us"] < previous["rx_us"]:
            boundary = "mcu_rx_backwards"
        else:
            delta = (record["seq"] - previous["seq"]) % UINT32
            if delta >= UINT32 // 2:
                boundary = "sequence_backwards"
        key = (record["capture_id"], record["seq"], record["rx_us"], record["payload_hex"])
        # 同一传输记录重复落盘不等于接收机重发；接收机重发会有新的原始 seq。
        if not boundary and key in seen:
            duplicate_capture += 1
            continue
        if boundary:
            if pending_no_fix is not None:
                pending_no_fix.update(recovered=False, end_reason=boundary,
                    observed_until_rx_us=previous["rx_us"],
                    observed_duration_s=(previous["rx_us"] - pending_no_fix["start_rx_us"]) / 1e6)
                no_fix_recoveries.append(pending_no_fix)
            segment = {"segment_id": len(segments) + 1, "capture_id": record["capture_id"],
                       "boundary": boundary, "first_counters": {k: record[k] for k in COUNTERS},
                       "counter_increases": {k: 0 for k in COUNTERS}, "sequence_missing": 0,
                       "raw_rx": [], "rmc_rx": [], "fix_rx": [], "event_rx": [], "epochs": [], "pc_rx": []}
            segments.append(segment)
            highwater, seen = None, set()
            pending_no_fix, previous_fix = None, None
        else:
            delta = (record["seq"] - previous["seq"]) % UINT32
            if delta == 0:
                loss["sequence_conflicts"] += 1
            elif delta > 1:
                segment["sequence_missing"] += delta - 1
                loss["sequence_missing"] += delta - 1
            for name in COUNTERS:
                difference = record[name] - previous[name]
                if difference < 0:
                    if previous[name] >= 0xF0000000 and record[name] < 0x10000000:
                        difference += UINT32
                    else:
                        loss[name + "_counter_resets"] += 1
                        continue
                segment["counter_increases"][name] += difference
                loss[name + "_increase"] += difference
        seen.add(key)
        previous = record
        row = dict(record, record_index=len(rows) + 1, segment_id=segment["segment_id"], epoch_disposition="not_eligible")
        classified = classify_payload(bytes.fromhex(record["payload_hex"]), record["flags"])
        row.update(classified)
        if record["tx_us"] >= record["rx_us"]:
            row["queue_delay_ms"] = (record["tx_us"] - record["rx_us"]) / 1000
            queue_delays.append(row["queue_delay_ms"])
        else:
            row["queue_delay_ms"] = None
            loss["tx_before_rx"] += 1
        segment["raw_rx"].append(record["rx_us"])
        if record.get("t_ns") is not None:
            if segment["pc_rx"] and record["t_ns"] < segment["pc_rx"][-1]:
                loss["pc_clock_backwards"] += 1
            segment["pc_rx"].append(record["t_ns"])
        sentences[classified["sentence"] or "non_nmea"] += 1
        checksums[classified["checksum_status"]] += 1
        counts["captured_lines"] += 1
        if classified["reason"]:
            reasons[classified["reason"]] += 1
        if classified["formatter"] == "RMC":
            counts["rmc_arrivals"] += 1
            segment["rmc_rx"].append(record["rx_us"])
            counts["rmc_" + classified["rmc_status"]] += 1
            counts["firmware_" + classified["firmware_status"]] += 1
        if classified["firmware_status"] in ("position", "no_fix"):
            epoch = classified["_epoch"]
            if highwater is not None and epoch <= highwater:
                row["epoch_disposition"] = "duplicate" if epoch == highwater else "out_of_order"
                counts[row["epoch_disposition"]] += 1
            else:
                highwater = epoch
                row["epoch_disposition"] = "new"
                counts["new_epochs"] += 1
                segment["epochs"].append(epoch)
                segment["event_rx"].append(record["rx_us"])
                counts["new_" + classified["firmware_status"]] += 1
                if classified["firmware_status"] == "position":
                    if pending_no_fix is not None:
                        pending_no_fix.update(recovered=True, end_rx_us=record["rx_us"],
                            end_utc=classified["epoch_iso"], duration_s=(record["rx_us"] - pending_no_fix["start_rx_us"]) / 1e6)
                        no_fix_recoveries.append(pending_no_fix)
                        pending_no_fix = None
                    if previous_fix is not None and record["rx_us"] - previous_fix > gap_s * 1e6:
                        position_gaps.append({"segment_id": segment["segment_id"], "capture_id": record["capture_id"],
                            "previous_fix_rx_us": previous_fix, "next_fix_rx_us": record["rx_us"],
                            "gap_s": (record["rx_us"] - previous_fix) / 1e6})
                    previous_fix = record["rx_us"]
                    segment["fix_rx"].append(record["rx_us"])
                    points.append((classified["lat_deg_e7"], classified["lon_deg_e7"]))
                    counts["new_velocity_valid"] += int(classified["velocity_valid"])
                elif pending_no_fix is None:
                    pending_no_fix = {"segment_id": segment["segment_id"], "capture_id": record["capture_id"],
                        "start_rx_us": record["rx_us"], "start_utc": classified["epoch_iso"],
                        "previous_fix_rx_us": previous_fix, "no_fix_epochs": 1}
                else:
                    pending_no_fix["no_fix_epochs"] += 1
        rows.append(row)
    if pending_no_fix is not None:
        pending_no_fix.update(recovered=False, end_reason="end_of_capture",
            observed_until_rx_us=previous["rx_us"],
            observed_duration_s=(previous["rx_us"] - pending_no_fix["start_rx_us"]) / 1e6)
        no_fix_recoveries.append(pending_no_fix)
    for name in ("captured_lines", "rmc_arrivals", "rmc_position", "rmc_no_fix", "rmc_rejected", "rmc_unknown",
                 "firmware_position", "firmware_no_fix", "firmware_rejected", "firmware_unsupported_talker",
                 "new_epochs", "new_position", "new_no_fix", "duplicate", "out_of_order", "new_velocity_valid"):
        counts.setdefault(name, 0)
    for name in ("sequence_missing", "sequence_conflicts", "tx_before_rx", "pc_clock_backwards") + tuple(k + "_increase" for k in COUNTERS):
        loss.setdefault(name, 0)
    summary = {"schema": 1, "source": metadata, "counts": dict(counts), "reject_or_unavailable_reasons": dict(reasons),
               "sentence_counts": dict(sentences), "checksum_counts": dict(checksums),
               "duplicate_capture_records": duplicate_capture, "loss": dict(loss),
               "quality_source": "unknown_RMC_no_accuracy_report", "queue_delay_ms": distribution(queue_delays),
               "no_fix_recoveries": no_fix_recoveries, "position_gaps": position_gaps,
               "valid_position_fraction_all_rmc": counts["firmware_position"] / counts["rmc_arrivals"] if counts["rmc_arrivals"] else None,
               "valid_position_fraction_new_epochs": counts["new_position"] / counts["new_epochs"] if counts["new_epochs"] else None,
               "timing": {}, "segments": [], "static_repeatability": None,
               "notes": ["频率是采集到的报文/独立UTC，存在丢包时不能还原接收机真实全部输出。",
                         "UTC身份去重是离线输入资格复核，不表示这些样本实际通过了EKF状态/新鲜度/NIS检查。",
                         "queue_delay_ms是MCU原始行接收至发送时间戳间隔，不含PC/TCP到达延迟，也不是GNSS绝对延迟。",
                         "队列行丢弃、UART字节丢弃、decoder超长行和原始序号缺口单位/范围不同，不能直接相加。",
                         "GGA/GST仅保留句型和fields_json原文，不与RMC自动关联，不生成位置方差。",
                         "NO_FIX恢复指首个新有效输入位置；不等于EKF重锚或辅助恢复。跨重启/采集段不计算恢复时长。",
                         "duplicate指等于当前UTC高水位，低于高水位记out_of_order；采集seq重复另列。",
                         "每段首读累计计数可能包含文件开始前的损失；counter_increases仅统计观测区间内变化。"]}
    for key, scale in (("raw_rx", 1e6), ("rmc_rx", 1e6), ("fix_rx", 1e6), ("event_rx", 1e6), ("epochs", 1000)):
        summary["timing"][key] = timing([s[key] for s in segments], scale, gap_s)
    # PC 单调时钟若倒退，输出不可用，不以排序隐藏异常。
    summary["timing"]["pc_rx"] = None if loss["pc_clock_backwards"] else timing([s["pc_rx"] for s in segments], 1e9, gap_s)
    for segment in segments:
        public = {k: v for k, v in segment.items() if not isinstance(v, list)}
        public.update(records=len(segment["raw_rx"]), first_rx_us=segment["raw_rx"][0], last_rx_us=segment["raw_rx"][-1])
        summary["segments"].append(public)
    if static:
        summary["static_repeatability"] = static_repeatability(points)
    recorder = metadata.get("pc_recorder") or (metadata.get("recorder_end_events") or [{}])[-1]
    # queue_drop 在每个 capture 清零，所以首读非零已证明该 capture 有损失。
    # UART/decoder 累计计数可能来自本文件开始前，只呈报历史事实，不硬加本区间。
    initial_queue_loss = any(s["first_counters"]["queue_drop"] for s in segments)
    initial_history_loss = any(s["first_counters"][k] for s in segments for k in ("uart_drop", "line_drop"))
    observed_problem = (any(loss.values()) or reasons["capture_truncated"] or metadata.get("invalid_outer_primary", 0)
                        or metadata.get("invalid_outer_audit", 0) or metadata.get("container_errors")
                        or initial_queue_loss
                        or any(recorder.get(k) for k in ("dropped_records", "rejected_records", "pending_records",
                                                       "invalid_ugraw_count", "last_error")))
    summary["capture_integrity"] = ("issues_observed" if observed_problem else
        "history_loss_observed_interval_unknown" if initial_history_loss else
        "no_observed_issues_recorder_reports_complete" if recorder.get("complete") is True else
        "pc_recorder_incomplete" if recorder.get("complete") is False else "unknown")
    return summary, rows


def write_outputs(input_path, output_dir, summary, rows):
    output_dir = Path(output_dir).resolve()
    input_path = Path(input_path).resolve()
    targets = [output_dir / "summary.json", output_dir / "epochs.csv"]
    protected = {Path(p).resolve() for p in summary["source"].get("selected_sources", [])}
    if (input_path.is_dir() and output_dir == input_path) or input_path in targets or any(p in protected for p in targets):
        raise ValueError("输出目录/文件会覆盖输入；请指定独立的 --output-dir")
    if any(p.exists() for p in targets):
        raise ValueError("输出文件已存在；请指定新的 --output-dir，避免覆盖已有结果")
    output_dir.mkdir(parents=True, exist_ok=True)
    with targets[0].open("x", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, allow_nan=False, indent=2)
        handle.write("\n")
    with targets[1].open("x", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="UGRAW CSV/JSONL/raw.log 文件或一个会话目录")
    parser.add_argument("--output-dir", type=Path, help="独立输出目录；默认在输入旁创建 gnss_analysis")
    parser.add_argument("--static", action="store_true", help="用户确认整段静止，计算独立有效位置的重复性")
    parser.add_argument("--gap-seconds", type=float, default=5.0, help="报告 gap 的阈值，默认5秒，不据此删除样本")
    args = parser.parse_args(argv)
    try:
        if not math.isfinite(args.gap_seconds) or args.gap_seconds <= 0:
            raise ValueError("--gap-seconds 必须为有限正数")
        records, metadata = load_capture(args.input)
        summary, rows = analyze_records(records, metadata, args.static, args.gap_seconds)
        destination = args.output_dir or (args.input / "gnss_analysis" if args.input.is_dir()
                                          else args.input.parent / (args.input.stem + "_gnss_analysis"))
        write_outputs(args.input, destination, summary, rows)
    except (OSError, ValueError, csv.Error) as exc:
        parser.exit(2, "分析失败: " + str(exc) + "\n")
    print("已写入 " + str(destination.resolve()))
    print("捕获行=%d，新有效位置=%d，NO_FIX=%d，重复UTC=%d，乱序=%d，完整性=%s" % (
        summary["counts"]["captured_lines"], summary["counts"]["new_position"], summary["counts"]["new_no_fix"],
        summary["counts"]["duplicate"], summary["counts"]["out_of_order"], summary["capture_integrity"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
