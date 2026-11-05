"""离线复核 auto_sti 的 pitch/roll 指令与 UGWRM；仅使用 Python 标准库。

时间区间指 PC 收到位置变化的观测区间，不能解释为纯执行器机械延迟。
"""
from __future__ import annotations

import argparse
import csv
import html
import json
import math
import statistics
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Options:
    pitch_threshold: float = 1.0
    roll_threshold: float = 1.0
    consecutive_samples: int = 3
    max_gap_s: float = 0.6
    window_s: float = 8.0


CSV_FIELDS = [
    "session_id", "run_id", "axis", "tx_event_id", "host_iso", "change",
    "from_rpm", "to_rpm", "tx_start_ns", "tx_end_ns", "quality", "reason",
    "observed_lower_s", "observed_upper_s", "first_observed_ns", "confirmed_ns",
    "sample_interval_median_s", "sample_interval_max_s", "threshold_pulses",
    "consecutive_samples", "pulse_rate_per_s", "rate_window_s", "feedback_kind",
]


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def control_target(event, axis):
    fields = event.get("fields") or {}
    enabled = fields.get(axis + "motor_enable")
    rpm = fields.get(axis + "motor_speed_rpm")
    if enabled not in (0, 1) or not finite(rpm):
        return None
    return rpm if enabled else 0


def is_control(event):
    return event.get("kind") == "tx" and "pitchmotor_enable" in (event.get("fields") or {})


def is_release(event):
    return is_control(event) and (event.get("fields") or {}).get("eth_control_flag") == 0


def boundary(event):
    # 模式/链路改变和释放之后，无法把后续动作继续归因于旧指令。
    if event.get("kind") in ("mode", "link", "recording_error", "recorder_error"):
        return event.get("kind") + "_boundary"
    if event.get("kind") == "tx" and event.get("success") is not True:
        return "send_failed_boundary"
    if is_release(event):
        return "control_released"
    if is_control(event) and event.get("source") != "auto_sti":
        return "other_control_source"
    row = event.get("row") or {}
    if event.get("kind") == "rx" and event.get("topic") == "ugctl" and row.get("mode", 0) != 0:
        return "pid_control_active"
    return None


def feedback_problem(row, axis):
    if not isinstance(row, dict):
        return "bad_ugwrm_frame"
    if row.get("schema") not in (2, 3):
        return "unsupported_schema"
    if row.get("calib_state") == 1:
        return "calibration_active"
    if axis == "pitch":
        if row.get("data_fresh") != 1 or not finite(row.get("ref_lost")):
            return "pitch_stale_or_reference_unknown"
        if row.get("home_state") in (1, 2):
            return "calibration_active"
        if row.get("alarm", 0):
            return "pitch_alarm"
        value = row.get("pulse")
    else:
        if row.get("schema") != 3:
            return "roll_unavailable_old_schema"
        if row.get("roll_valid") != 1:
            return "roll_invalid"
        if row.get("roll_home_state") in (1, 2, 3, 4, 5):
            return "calibration_active"
        value = row.get("roll_pulse")
    return None if finite(value) else "missing_or_nonfinite_pulse"


def reference_signature(row, axis):
    # 上电观测范围可随运动扩展，不能把其 min/max 更新误判为重新校准。
    names = ["schema", "calib_state", "calib_hold"]
    if axis == "pitch":
        names += ["ref_kind", "ref_lost", "pulse_home", "home_state"]
        if row.get("ref_kind") == 2:
            names += ["span_valid", "span_pulses"]
    else:
        names += ["roll_pulse_min", "roll_pulse_max", "roll_span_pulses", "roll_home_state"]
    return tuple(row.get(name) for name in names)


def make_points(events):
    points = {"pitch": [], "roll": []}
    previous = None
    receive_counts = Counter(event["t_ns"] for event in events if event.get("kind") == "rx"
        and (event.get("topic") == "ugwrm" or str(event.get("raw", "")).startswith("$UGWRM")))
    # 先按日志提交顺序检查接收时钟，之后才按 PC 时间排序。
    for event in events:
        if event.get("kind") != "rx":
            continue
        if event.get("topic") != "ugwrm" and not str(event.get("raw", "")).startswith("$UGWRM"):
            continue
        row = event.get("row")
        common = None
        if not isinstance(row, dict):
            common = "bad_ugwrm_frame"
        elif not finite(row.get("seq")) or not finite(row.get("t_ms")):
            common = "missing_board_clock_or_sequence"
        elif receive_counts[event["t_ns"]] > 1:
            common = "batched_or_nonincreasing_rx_time"
        if previous is not None and common is None:
            previous_row = previous.get("row") or {}
            if event["t_ns"] <= previous["t_ns"]:
                common = "batched_or_nonincreasing_rx_time"
            elif previous.get("session_id") != event.get("session_id"):
                common = "session_changed"
            elif finite(previous_row.get("seq")) and finite(previous_row.get("t_ms")):
                # uint32 正常回绕可用；重复、乱序和复位不能作为新样本。
                seq_delta = (int(row["seq"]) - int(previous_row["seq"])) & 0xFFFFFFFF
                time_delta = (int(row["t_ms"]) - int(previous_row["t_ms"])) & 0xFFFFFFFF
                if seq_delta == 0 or time_delta == 0:
                    common = "duplicate_telemetry"
                elif seq_delta >= 0x80000000 or time_delta >= 0x80000000:
                    common = "board_clock_or_sequence_reset"
                elif seq_delta != 1:
                    common = "telemetry_sequence_gap"
        for axis in points:
            value = row.get("pulse" if axis == "pitch" else "roll_pulse") if isinstance(row, dict) else None
            problem = common or feedback_problem(row, axis)
            points[axis].append(dict(t_ns=event["t_ns"], value=value, problem=problem,
                signature=reference_signature(row, axis) if isinstance(row, dict) else (),
                row=row or {}, event=event))
        previous = event
    for values in points.values():
        values.sort(key=lambda point: point["t_ns"])
    return points


def load_recording(input_dir):
    events, problems = [], []
    try:
        metadata = json.loads((input_dir / "session.json").read_text(encoding="utf-8-sig"))
        if not isinstance(metadata, dict):
            raise ValueError("session.json 必须是对象")
    except (OSError, ValueError) as exc:
        metadata = {}
        problems.append("session.json 不可用: " + str(exc))
    with (input_dir / "events.jsonl").open(encoding="utf-8-sig") as handle:
        for number, line in enumerate(handle, 1):
            try:
                event = json.loads(line)
                if not isinstance(event, dict) or not isinstance(event.get("t_ns"), int) or event["t_ns"] < 0:
                    raise ValueError("事件缺少有效单调时间 t_ns")
                events.append(event)
            except (ValueError, TypeError) as exc:
                problems.append("events.jsonl 第 %d 行不可用: %s" % (number, exc))
    return events, metadata, problems


def continuity_problem(points, options):
    for point in points:
        if point["problem"]:
            return point["problem"]
    for left, right in zip(points, points[1:]):
        delta = (right["t_ns"] - left["t_ns"]) / 1e9
        if delta <= 0 or delta > options.max_gap_s:
            return "telemetry_gap"
        if left["signature"] != right["signature"]:
            return "calibration_or_reference_changed"
    return None


def measure(candidate, axis_points, start_ns, end_ns, end_reason, options):
    result = candidate
    tx_start, tx_end = result["tx_start_ns"], result["tx_end_ns"]
    count, threshold = options.consecutive_samples, result["threshold_pulses"]
    baseline = [point for point in axis_points if start_ns <= point["t_ns"] < tx_start][-count:]
    if len(baseline) < count:
        result["reason"] = "insufficient_precommand_baseline"
        return result
    problem = continuity_problem(baseline, options)
    if problem or (tx_start - baseline[-1]["t_ns"]) / 1e9 > options.max_gap_s:
        result["reason"] = problem or "stale_precommand_baseline"
        return result
    spread = max(point["value"] for point in baseline) - min(point["value"] for point in baseline)
    if result["change"] == "start" and spread > threshold:
        result["reason"] = "precommand_already_moving"
        return result
    if result["change"] == "stop" and spread <= threshold:
        result["reason"] = "no_precommand_motion_evidence"
        return result
    if result["change"] == "stop" and abs(baseline[-1]["value"] - baseline[-2]["value"]) <= threshold:
        # 基线早期运动不代表命令前仍在运动；可能已触限位或自行停住。
        result["reason"] = "precommand_stop_ambiguous"
        return result
    # sendall 阻塞期间也可能已经收到反馈；保留它并用发送起止给出观测区间。
    samples = [point for point in axis_points if tx_start < point["t_ns"] < end_ns]
    chain, movement = list(baseline), []
    unchanged = baseline[-1]
    for point in samples:
        problem = continuity_problem([chain[-1], point], options)
        if problem:
            result["reason"] = problem
            return result
        chain.append(point)
        if result["change"] == "start":
            delta = point["value"] - baseline[-1]["value"]
            if abs(delta) > threshold:
                if movement and delta * (movement[0]["value"] - baseline[-1]["value"]) < 0:
                    result["reason"] = "motion_direction_not_consistent"
                    return result
                movement.append(point)
            else:
                # 穿阈后又回到基线不能把更晚的动作当作第一次响应。
                if movement:
                    result["reason"] = "motion_not_sustained"
                    return result
                unchanged = point
            if len(movement) < count:
                continue
            first, preceding = movement[0], unchanged
        else:
            movement.append(point)
            if len(movement) < count:
                continue
            stable = movement[-count:]
            if max(item["value"] for item in stable) - min(item["value"] for item in stable) > threshold:
                continue
            first = stable[0]
            preceding = movement[-count - 1] if len(movement) > count else baseline[-1]
        intervals = [(right["t_ns"] - left["t_ns"]) / 1e9 for left, right in zip(chain, chain[1:])]
        rate_window_s = (point["t_ns"] - first["t_ns"]) / 1e9
        result.update(quality="observed", reason="pc_observation_only",
            observed_lower_s=max(0.0, (preceding["t_ns"] - tx_end) / 1e9),
            observed_upper_s=(first["t_ns"] - tx_start) / 1e9,
            first_observed_ns=first["t_ns"], confirmed_ns=point["t_ns"],
            sample_interval_median_s=statistics.median(intervals), sample_interval_max_s=max(intervals),
            pulse_rate_per_s=(point["value"] - first["value"]) / rate_window_s,
            rate_window_s=rate_window_s)
        return result
    result["reason"] = end_reason or "response_not_confirmed_within_window"
    return result


def analyze(events, metadata, options=None, input_problems=None):
    options = options or Options()
    problems = list(input_problems or [])
    if (metadata.get("complete") is not True or metadata.get("closed") is not True
            or metadata.get("dropped_records", 0) or metadata.get("rejected_records", 0)
            or metadata.get("stop_timed_out", False)):
        problems.append("记录尚未正常关闭或有丢失；本次全部响应估计标记不可用。")
    sessions = {event.get("session_id") for event in events}
    if len(sessions) > 1:
        problems.append("同一输入包含多个会话；禁止跨会话配对。")
    if metadata.get("session_id") is not None and sessions != {metadata["session_id"]}:
        problems.append("事件会话 ID 与 session.json 不一致。")
    for name in ("written_records", "accepted_records"):
        if name in metadata and (not isinstance(metadata[name], int) or metadata[name] != len(events)):
            problems.append("事件条数与 session.json 的 %s 不一致；可能被截断或混入其他记录。" % name)
    receive_times = [event["t_ns"] for event in events if event.get("kind") == "rx"]
    if any(right < left for left, right in zip(receive_times, receive_times[1:])):
        problems.append("接收事件的 PC 单调时钟倒退；禁止重排后继续估计。")
    points = make_points(events)
    ordered = sorted(events, key=lambda event: (event["t_ns"], event.get("event_id", 0)))
    results, current, run_starts = [], {}, {}
    baseline_floor, active_key = 0, None
    for index, event in enumerate(ordered):
        if boundary(event):
            current.clear()
            baseline_floor = event["t_ns"]
        if not is_control(event) or event.get("source") != "auto_sti" or is_release(event):
            continue
        run_id = event.get("run_id")
        key = (event.get("session_id"), run_id)
        if key != active_key:
            current.clear()
            baseline_floor = event["t_ns"]
            active_key = key
        run_starts.setdefault(key, event["t_ns"])
        for axis in ("pitch", "roll"):
            target = control_target(event, axis)
            axis_key = key + (axis,)
            previous = current.get(axis_key)
            if target == previous and event.get("success") is True:
                continue
            current[axis_key] = target if event.get("success") is True else None
            if target == 0 and previous is None and event.get("success") is True:
                continue
            change = "start" if target and previous in (None, 0) else "stop" if target == 0 else "speed_change"
            result = dict.fromkeys(CSV_FIELDS, "")
            result.update(session_id=event.get("session_id"), run_id=run_id, axis=axis,
                tx_event_id=event.get("event_id"), host_iso=event.get("host_iso", ""),
                change=change, from_rpm=previous, to_rpm=target,
                tx_start_ns=event.get("tx_start_ns", event["t_ns"]), tx_end_ns=event.get("tx_end_ns", event["t_ns"]),
                quality="unconfirmed", threshold_pulses=getattr(options, axis + "_threshold"),
                consecutive_samples=options.consecutive_samples,
                feedback_kind="encoder_pulse" if axis == "pitch" else "output_pulse_estimate")
            if problems:
                result["reason"] = "incomplete_or_invalid_recording"
            elif not run_id:
                result["reason"] = "missing_run_id"
            elif event.get("success") is not True:
                result["reason"] = "send_failed_may_be_partial"
            elif target is None:
                result["reason"] = "invalid_command_fields"
            elif previous is None:
                result["reason"] = "missing_prior_command"
            elif change == "speed_change":
                result["reason"] = "nonzero_speed_change_not_supported"
            elif result["tx_end_ns"] < result["tx_start_ns"]:
                result["reason"] = "invalid_send_clock"
            else:
                limit = result["tx_start_ns"] + int(options.window_s * 1e9)
                end_reason = None
                for following in ordered[index + 1:]:
                    if following["t_ns"] >= limit:
                        break
                    reason = boundary(following)
                    if is_control(following):
                        if following.get("run_id") != run_id:
                            reason = "run_changed"
                        elif control_target(following, axis) != target:
                            reason = reason or "next_command_before_confirmation"
                    if reason:
                        limit, end_reason = following["t_ns"], reason
                        break
                measure(result, points[axis], max(run_starts[key], baseline_floor), limit, end_reason, options)
            results.append(result)
    return results, points, problems


def write_svg(path, events, points, results, options):
    """保留断点的独立 SVG：红色目标，橙色板端 cmd_rpm，绿色 fb_rpm。"""
    ordered = sorted(events, key=lambda event: event["t_ns"])
    origin = ordered[0]["t_ns"] if ordered else 0
    duration = max(1.0, (ordered[-1]["t_ns"] - origin) / 1e9) if ordered else 1.0
    left, width, panel_height = 100, 1100, 125
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="810" viewBox="0 0 1280 810">',
        '<rect width="1280" height="810" fill="#10151d"/>',
        '<g font-family="Microsoft YaHei, sans-serif" fill="#e6edf3" font-size="13">',
        '<text x="25" y="28" font-size="20">auto_sti 指令与 UGWRM — PC 接收时间轴</text>',
        '<text x="25" y="53">红: PC 目标 RPM　橙: pitch 板端 cmd_rpm（安装转换后）　绿: fb_rpm　蓝: 位置脉冲</text>',
        '<text x="25" y="74">Roll 位置为输出脉冲估算，不能证明机械响应。竖线: 目标变化；圆点: 不可用遥测。</text>']
    x = lambda stamp: left + ((stamp - origin) / 1e9) / duration * width
    def sample_segments(axis, field):
        segments, segment, previous = [], [], None
        for point in points[axis]:
            value = point["value"] if field is None else point["row"].get(field)
            valid = not point["problem"] and finite(value)
            broken = previous is not None and continuity_problem([previous, point], options)
            if not valid or broken:
                if segment:
                    segments.append(segment)
                segment = []
            if valid:
                segment.append((point["t_ns"], value))
            previous = point
        if segment:
            segments.append(segment)
        return segments

    for panel, (axis, is_position) in enumerate((("pitch", False), ("pitch", True), ("roll", False), ("roll", True))):
        top = 115 + panel * 160
        series = []
        if is_position:
            series.extend((segment, "#70b8ff") for segment in sample_segments(axis, None))
        else:
            command, previous, previous_run = [], None, None
            for event in ordered:
                changed_run = is_control(event) and previous_run is not None and event.get("run_id") != previous_run
                if boundary(event) or changed_run:
                    if command:
                        command.append((event["t_ns"], previous))
                        series.append((command, "#ff7777"))
                    command, previous = [], None
                    if is_release(event) or event.get("success") is False:
                        continue
                if is_control(event) and event.get("source") == "auto_sti" and not is_release(event):
                    target = control_target(event, axis)
                    if finite(target) and event.get("success") is True:
                        if previous is not None and target != previous:
                            command.append((event["t_ns"], previous))
                        if target != previous:
                            command.append((event["t_ns"], target))
                        previous = target
                        previous_run = event.get("run_id")
            if command and ordered:
                command.append((ordered[-1]["t_ns"], previous))
            series.append((command, "#ff7777"))
            if axis == "pitch":
                for field, color in (("cmd_rpm", "#ffb454"), ("fb_rpm", "#75d69c")):
                    series.extend((segment, color) for segment in sample_segments(axis, field))
        values = [value for segment, _ in series for _, value in segment]
        low, high = (min(values), max(values)) if values else (-1, 1)
        if low == high:
            low, high = low - 1, high + 1
        y = lambda value: top + panel_height - (value - low) / (high - low) * panel_height
        label = axis + (" 原始编码器脉冲" if axis == "pitch" else " 输出估算脉冲") if is_position else axis + " RPM"
        svg.extend(['<text x="100" y="%d">%s</text>' % (top - 10, label),
            '<rect x="100" y="%d" width="1100" height="125" fill="none" stroke="#526170"/>' % top,
            '<text x="8" y="%d">%.5g</text>' % (top + 12, high),
            '<text x="8" y="%d">%.5g</text>' % (top + panel_height, low)])
        for segment, color in series:
            # 大日志限缩图文件；原始事件和结果 CSV 仍保留全部精度。
            stride = max(1, len(segment) // 3000)
            sampled = segment[::stride]
            if segment and (not sampled or sampled[-1] != segment[-1]):
                sampled.append(segment[-1])
            coordinates = " ".join("%.2f,%.2f" % (x(stamp), y(value)) for stamp, value in sampled)
            svg.append('<polyline fill="none" stroke="%s" stroke-width="1.5" points="%s"/>' % (color, coordinates))
        for result in results:
            if result["axis"] == axis:
                stamp = result["tx_start_ns"]
                svg.append('<line x1="%.2f" y1="%d" x2="%.2f" y2="%d" stroke="#aaa" stroke-dasharray="3,5"><title>%s</title></line>' % (
                    x(stamp), top, x(stamp), top + panel_height,
                    html.escape("%s event=%s %s" % (result["change"], result["tx_event_id"], result["reason"])) ))
        if is_position:
            for point in points[axis]:
                if point["problem"]:
                    svg.append('<circle cx="%.2f" cy="%d" r="3" fill="#fa7"><title>%s</title></circle>' % (x(point["t_ns"]), top + panel_height, html.escape(point["problem"])))
        for tick in range(6):
            elapsed = duration * tick / 5
            svg.append('<text x="%.1f" y="%d">%.1f s</text>' % (left + width * tick / 5, top + panel_height + 17, elapsed))
    svg.extend(['<text x="25" y="795">曲线仅供定位事件；是否能估计响应以 response_summary.csv 的 quality / reason 为准。</text>', '</g></svg>'])
    path.write_text("\n".join(svg), encoding="utf-8")


def write_outputs(output_dir, events, results, points, problems, options):
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "response_summary.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, CSV_FIELDS)
        writer.writeheader()
        writer.writerows(results)
    observed = [row for row in results if row["quality"] == "observed"]
    lines = [
        "auto_sti pitch / roll 离线实验复核", "",
        "有效观测 %d / 指令变化 %d；原始发送心跳保存在 events.jsonl，心跳不重复算动作。" % (len(observed), len(results)),
        "所有时间差只用 PC 单调时钟。observed_lower_s / observed_upper_s 是接收样本夹住的观测区间，",
        "包含发送、板端执行、反馈采样、遥测与接收等待；不是纯机械延迟，也不是硬件响应的严格上下界。",
        "tx_start_ns / tx_end_ns 是 sendall 调用区间，成功不表示板端确认执行。",
        "pitch 使用原始编码器 pulse；roll 使用输出脉冲估算，不能证明真实机械运动或排除失步。",
        "板端 cmd_rpm 已经过安装方向转换，不能直接假定与 PC 目标符号相同。", "",
        "pulse_rate_per_s 为首次变化/稳定样本到 confirmed_ns 的平均脉冲变化率（脉冲/s），",
        "rate_window_s 为该确认窗口长度；不是毫米/s，roll 仍只是输出脉冲估算。", "",
        "判据：脉冲变化严格大于阈值；默认 pitch=%g、roll=%g 脉冲，连续确认=%d 样本。" % (
            options.pitch_threshold, options.roll_threshold, options.consecutive_samples),
        "最大相邻接收间隔=%g s；单次搜索窗口=%g s。阈值仅为分析参数，需结合现场噪声调整。" % (options.max_gap_s, options.window_s),
        "起动须有同 run 静止基线；停止须有命令前运动证据和连续稳定窗口。先进入 ACTIVE 静止约1秒。",
        "非零转速到非零转速、反向指令、基线不足、坏帧/重复/批量同时间戳、长缺口、",
        "校准变化、时钟复位、发送失败或模式/控制来源切换时不强行估计。未确认不等于执行器未动。",
        "CSV quality=observed 才有区间；unconfirmed 的 reason 说明原因。原始 JSONL 是最终复核依据。",
        "SVG: response_timeline.svg；图表大日志会抽样，图线不作为质量判断。", "",
    ]
    if problems:
        lines.append("记录质量问题：")
        lines.extend("- " + problem for problem in problems)
    for row in results:
        value = ("观测区间 %.3f..%.3f s；样本中位间隔 %.3f s；确认窗口平均 %.3f 脉冲/s" % (
            row["observed_lower_s"], row["observed_upper_s"], row["sample_interval_median_s"], row["pulse_rate_per_s"])) if row["quality"] == "observed" else "未确认: " + row["reason"]
        lines.append("%s %s %s -> %s RPM (event %s): %s" % (
            row["axis"], row["change"], row["from_rpm"], row["to_rpm"], row["tx_event_id"], value))
    (output_dir / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8-sig")
    write_svg(output_dir / "response_timeline.svg", events, points, results, options)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True, help="含 events.jsonl 和 session.json 的实验目录")
    parser.add_argument("--output-dir", type=Path, help="默认为实验目录下 response_analysis")
    parser.add_argument("--pitch-threshold", type=float, default=1.0, help="pitch 脉冲变化阈值，默认1")
    parser.add_argument("--roll-threshold", type=float, default=1.0, help="roll 脉冲变化阈值，默认1")
    parser.add_argument("--consecutive-samples", type=int, default=3, help="基线/确认连续样本数，默认3")
    parser.add_argument("--max-gap-s", type=float, default=0.6, help="允许的最大样本间隔，默认0.6秒")
    parser.add_argument("--window-s", type=float, default=8.0, help="单次搜索窗口，默认8秒")
    args = parser.parse_args(argv)
    if (args.consecutive_samples < 2 or any(not math.isfinite(value) or value < 0 for value in
            (args.pitch_threshold, args.roll_threshold)) or any(not math.isfinite(value) or value <= 0 for value in
            (args.max_gap_s, args.window_s))):
        parser.error("阈值须为有限非负数，连续样本数至少2，时间参数须为有限正数")
    options = Options(args.pitch_threshold, args.roll_threshold, args.consecutive_samples, args.max_gap_s, args.window_s)
    try:
        events, metadata, problems = load_recording(args.input_dir)
        results, points, problems = analyze(events, metadata, options, problems)
        destination = args.output_dir or args.input_dir / "response_analysis"
        write_outputs(destination, events, results, points, problems, options)
    except (OSError, ValueError, TypeError) as exc:
        parser.exit(1, "分析失败: %s\n" % exc)
    print("响应分析 -> %s（有效观测 %d / 指令变化 %d）" % (
        destination.resolve(), sum(row["quality"] == "observed" for row in results), len(results)))
    return 0


if __name__ == "__main__":
    # Windows 重定向/终端捕获仍输出 UTF-8，避免独立 CLI 的中文乱码。
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
