#!/usr/bin/env python3
"""分析并绘制水下滑翔机状态估计相关的CSV 日志

脚本按板端启动次数划分实验会话，使用板端时间戳拼接上位机重连日志，
为每个会话生成运动总览图，并为深度更新质量最好的会话生成 pitch-vD 相图

在相对路径中 Python plot_ekf_logs.py
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import pandas as pd


FILE_RE = re.compile(r"ekf_(\d{8})_(\d{6})\.csv$")
REQUIRED_COLUMNS = {
    "host_iso",
    "seq",
    "t_ms",
    "status_name",
    "healthy",
    "valid",
    "roll_deg",
    "pitch_deg",
    "depth",
    "vD",
    "nisD",
    "accD",
    "rejD",
    "dstreak",
    "dreset",
    "dmeas",
}

# IMU 指向压力计的 FRD 机体系杠杆臂，数值来自 ekf_bridge.c。
PRESSURE_LEVER_FRD_M = np.array([-0.32429, 0.01950, 0.02500])
DEPTH_NIS_GATE = 6.635
LOGGER_GAP_S = 0.5


def configure_plot_style() -> None:
    installed = {font.name for font in font_manager.fontManager.ttflist}
    for candidate in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC"):
        if candidate in installed:
            plt.rcParams["font.sans-serif"] = [candidate, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["figure.facecolor"] = "white"
    plt.rcParams["axes.grid"] = True
    plt.rcParams["grid.alpha"] = 0.22
    plt.rcParams["grid.linewidth"] = 0.7


def discover_logs(input_dir: Path, date: str | None) -> Tuple[str, List[Path]]:
    candidates: Dict[str, List[Path]] = {}
    for path in input_dir.glob("ekf_????????_??????.csv"):
        match = FILE_RE.fullmatch(path.name)
        if match:
            candidates.setdefault(match.group(1), []).append(path)

    if not candidates:
        raise FileNotFoundError(f"在 {input_dir} 中没有找到 EKF CSV 日志")

    selected_date = date or max(candidates)
    if selected_date not in candidates:
        available = ", ".join(sorted(candidates))
        raise FileNotFoundError(
            f"没有找到 {selected_date} 的日志；可用日期：{available}"
        )
    return selected_date, sorted(candidates[selected_date])


def read_log(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path)
    missing = REQUIRED_COLUMNS.difference(frame.columns)
    if missing:
        raise ValueError(f"{path.name} 缺少字段：{sorted(missing)}")
    if frame.empty:
        raise ValueError(f"{path.name} 是空文件")

    frame = frame.copy()
    frame["source_file"] = path.name
    frame["host_time"] = pd.to_datetime(frame["host_iso"], errors="coerce")
    return frame


def group_boot_sessions(logs: Sequence[Tuple[Path, pd.DataFrame]]) -> List[List[pd.DataFrame]]:
    """按板端启动划分会话，同时容许重连处存在少量重复数据。"""
    sessions: List[List[pd.DataFrame]] = []
    current: List[pd.DataFrame] = []
    previous_last_t = -1
    previous_last_seq = -1

    for _, frame in logs:
        first_t = int(frame["t_ms"].iloc[0])
        first_seq = int(frame["seq"].iloc[0])
        board_restarted = (
            current
            and first_t < previous_last_t - 5000
            and first_seq < previous_last_seq - 500
        )
        if board_restarted:
            sessions.append(current)
            current = []

        current.append(frame)
        previous_last_t = int(frame["t_ms"].iloc[-1])
        previous_last_seq = int(frame["seq"].iloc[-1])

    if current:
        sessions.append(current)
    return sessions


def combine_session(parts: Sequence[pd.DataFrame]) -> pd.DataFrame:
    frame = pd.concat(parts, ignore_index=True)
    frame = frame.sort_values(["t_ms", "host_time", "seq"])
    frame = frame.drop_duplicates(subset=["t_ms"], keep="last").reset_index(drop=True)

    frame["elapsed_min"] = (frame["t_ms"] - frame["t_ms"].iloc[0]) / 60000.0
    frame["board_gap_s"] = frame["t_ms"].diff() / 1000.0
    frame["reject_event"] = frame["rejD"].diff().fillna(0) > 0
    frame["reset_event"] = frame["dreset"].diff().fillna(0) > 0

    # 用 电脑接收时刻 - 板端运行时间 的中位数估算板端启动时刻。
    # 中位数可以抑制上位机重连后缓存帧集中到达造成的时间偏差。
    valid_host = frame["host_time"].notna()
    host_ns = frame.loc[valid_host, "host_time"].astype("int64").to_numpy()
    board_ns = (
        frame.loc[valid_host, "t_ms"].to_numpy(dtype=np.int64) * 1_000_000
    )
    boot_time_ns = int(round(np.median(host_ns - board_ns) / 1_000_000)) * 1_000_000
    event_ns = boot_time_ns + frame["t_ms"].to_numpy(dtype=np.int64) * 1_000_000
    frame["event_time"] = pd.to_datetime(event_ns)

    roll = np.deg2rad(frame["roll_deg"].to_numpy(dtype=float))
    pitch = np.deg2rad(frame["pitch_deg"].to_numpy(dtype=float))
    rx, ry, rz = PRESSURE_LEVER_FRD_M
    lever_down = (
        -np.sin(pitch) * rx
        + np.cos(pitch) * np.sin(roll) * ry
        + np.cos(pitch) * np.cos(roll) * rz
    )
    frame["pressure_lever_down"] = lever_down
    frame["dmeas_at_imu"] = frame["dmeas"] - lever_down

    # 不让被拒绝的异常压力值（例如 312 m）破坏图的纵轴尺度。
    frame["dmeas_plot"] = frame["dmeas_at_imu"].where(frame["dstreak"] == 0)
    return frame


def centered_slope(
    t_s: np.ndarray, values: np.ndarray, window_s: float
) -> np.ndarray:
    """计算中心差分斜率，并将跨越日志缺口的结果置空。"""
    slope = np.full(values.shape, np.nan, dtype=float)
    finite = np.isfinite(t_s) & np.isfinite(values)
    t = t_s[finite]
    y = values[finite]
    if len(t) < 3:
        return slope

    half = window_s / 2.0
    left_target = t - half
    right_target = t + half
    left_value = np.interp(left_target, t, y)
    right_value = np.interp(right_target, t, y)

    left_index = np.clip(np.searchsorted(t, left_target), 0, len(t) - 1)
    right_index = np.clip(np.searchsorted(t, right_target), 0, len(t) - 1)
    near_samples = (
        np.abs(t[left_index] - left_target) < LOGGER_GAP_S / 2.0
    ) & (np.abs(t[right_index] - right_target) < LOGGER_GAP_S / 2.0)
    inside = (left_target >= t[0]) & (right_target <= t[-1])
    calculated = (right_value - left_value) / window_s
    finite_positions = np.flatnonzero(finite)
    slope[finite_positions[inside & near_samples]] = calculated[inside & near_samples]
    return slope


def shade_gaps_and_invalid(axes: Sequence[plt.Axes], frame: pd.DataFrame) -> None:
    x = frame["elapsed_min"].to_numpy(dtype=float)
    gaps = np.flatnonzero(frame["board_gap_s"].to_numpy(dtype=float) > LOGGER_GAP_S)
    for ax in axes:
        for index in gaps:
            ax.axvspan(x[index - 1], x[index], color="0.65", alpha=0.25, lw=0)
        ax.fill_between(
            x,
            0,
            1,
            where=frame["valid"].to_numpy(dtype=int) == 0,
            transform=ax.get_xaxis_transform(),
            color="#f4a261",
            alpha=0.13,
            step="mid",
            lw=0,
        )


def gap_midpoints(frame: pd.DataFrame) -> np.ndarray:
    """返回每个日志缺口在相对时间轴上的中点。"""
    x = frame["elapsed_min"].to_numpy(dtype=float)
    gaps = np.flatnonzero(frame["board_gap_s"].to_numpy(dtype=float) > LOGGER_GAP_S)
    if len(gaps) == 0:
        return np.array([], dtype=float)
    return (x[gaps - 1] + x[gaps]) / 2.0


def add_break_marks(ax: plt.Axes, positions: np.ndarray, axis_y: float) -> None:
    """在时间轴指定位置绘制“//”中断符号。"""
    for position in positions:
        ax.text(
            position,
            axis_y,
            "//",
            transform=ax.get_xaxis_transform(),
            ha="center",
            va="center",
            fontsize=12,
            fontweight="bold",
            color="0.2",
            bbox={"facecolor": "white", "edgecolor": "none", "pad": 0.4},
            clip_on=False,
            zorder=10,
        )


def format_event_clock(value: pd.Timestamp) -> str:
    """将事件时刻格式化到毫秒。"""
    return value.strftime("%H:%M:%S.%f")[:-3]


def add_gap_annotations(ax: plt.Axes, frame: pd.DataFrame) -> None:
    """标出每次中断前后的事件时刻以及中断持续时间。"""
    x = frame["elapsed_min"].to_numpy(dtype=float)
    gap_indices = np.flatnonzero(
        frame["board_gap_s"].to_numpy(dtype=float) > LOGGER_GAP_S
    )
    for index in gap_indices:
        midpoint = (x[index - 1] + x[index]) / 2.0
        before = format_event_clock(frame["event_time"].iloc[index - 1])
        after = format_event_clock(frame["event_time"].iloc[index])
        duration = float(frame["board_gap_s"].iloc[index])
        ax.text(
            midpoint,
            0.96,
            f"中断前：{before}\n中断后：{after}\n持续：{duration:.1f} s",
            transform=ax.get_xaxis_transform(),
            ha="center",
            va="top",
            fontsize=8.5,
            linespacing=1.25,
            color="0.15",
            bbox={
                "boxstyle": "round,pad=0.3",
                "facecolor": "white",
                "edgecolor": "0.55",
                "alpha": 0.92,
            },
            zorder=9,
        )


def add_event_clock_axis(
    ax: plt.Axes, frame: pd.DataFrame, breaks: np.ndarray
) -> plt.Axes:
    """在图的上方增加与板端时间对应的估算事件时钟。"""
    clock_axis = ax.twiny()
    clock_axis.set_xlim(ax.get_xlim())
    ticks = ax.get_xticks()
    left, right = ax.get_xlim()
    ticks = ticks[(ticks >= left) & (ticks <= right)]
    event_start = frame["event_time"].iloc[0]
    labels = [
        (event_start + pd.to_timedelta(float(value), unit="m")).strftime("%H:%M:%S")
        for value in ticks
    ]
    clock_axis.set_xticks(ticks)
    clock_axis.set_xticklabels(labels)
    clock_axis.set_xlabel("估算事件时刻（时:分:秒）")
    clock_axis.grid(False)
    add_break_marks(clock_axis, breaks, axis_y=1.0)
    return clock_axis


def counter_delta(frame: pd.DataFrame, column: str) -> int:
    values = pd.to_numeric(frame[column], errors="coerce").dropna()
    if values.empty:
        return 0
    return max(0, int(values.max() - values.min()))


def summarize_session(index: int, frame: pd.DataFrame) -> Dict[str, object]:
    running = frame[frame["status_name"] == "RUNNING"]
    useful = running if not running.empty else frame
    accepted = counter_delta(frame, "accD")
    rejected = counter_delta(frame, "rejD")
    resets = counter_delta(frame, "dreset")
    total_depth_updates = accepted + rejected
    acceptance_ratio = accepted / total_depth_updates if total_depth_updates else 0.0
    valid_pct = 100.0 * float(useful["valid"].mean())
    duration_s = (float(frame["t_ms"].iloc[-1]) - float(frame["t_ms"].iloc[0])) / 1000

    source_files = list(dict.fromkeys(frame["source_file"].tolist()))
    return {
        "session": index,
        "files": ";".join(source_files),
        "start_host": frame["host_iso"].iloc[0],
        "end_host": frame["host_iso"].iloc[-1],
        "start_event": frame["event_time"].iloc[0].isoformat(timespec="milliseconds"),
        "end_event": frame["event_time"].iloc[-1].isoformat(timespec="milliseconds"),
        "duration_s": round(duration_s, 1),
        "rows": len(frame),
        "running_rows": len(running),
        "valid_pct_running": round(valid_pct, 2),
        "depth_min_m": round(float(useful["depth"].min()), 3),
        "depth_max_m": round(float(useful["depth"].max()), 3),
        "vD_min_mps": round(float(useful["vD"].min()), 3),
        "vD_max_mps": round(float(useful["vD"].max()), 3),
        "pitch_min_deg": round(float(useful["pitch_deg"].min()), 2),
        "pitch_max_deg": round(float(useful["pitch_deg"].max()), 2),
        "max_abs_roll_deg": round(float(useful["roll_deg"].abs().max()), 2),
        "depth_accepts": accepted,
        "depth_rejects": rejected,
        "depth_resets": resets,
        "depth_acceptance_pct": round(100.0 * acceptance_ratio, 3),
        "largest_log_gap_s": round(float(frame["board_gap_s"].max()), 1),
        "Ppos_end": round(float(frame["Ppos"].iloc[-1]), 3)
        if "Ppos" in frame
        else np.nan,
        "_quality_score": duration_s * acceptance_ratio * valid_pct / 100.0,
    }


def plot_overview(
    date: str,
    session_index: int,
    frame: pd.DataFrame,
    summary: Dict[str, object],
    output_dir: Path,
    slope_window_s: float,
    dpi: int,
) -> Path:
    x = frame["elapsed_min"].to_numpy(dtype=float)
    t_s = frame["t_ms"].to_numpy(dtype=float) / 1000.0
    depth_slope = centered_slope(
        t_s, frame["depth"].to_numpy(dtype=float), slope_window_s
    )

    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    ax_depth, ax_velocity, ax_attitude, ax_quality = axes

    ax_depth.plot(x, frame["depth"], color="#005f73", lw=1.35, label="EKF 估计深度")
    ax_depth.plot(
        x,
        frame["dmeas_plot"],
        color="#ee9b00",
        lw=0.8,
        alpha=0.7,
        label="压力深度，杠杆臂修正",
    )
    ax_depth.set_ylabel("深度（m，向下为正）")
    ax_depth.invert_yaxis()
    ax_depth.legend(loc="best", ncol=2)

    ax_velocity.plot(x, frame["vD"], color="#9b2226", lw=1.1, label="EKF vD")
    ax_velocity.plot(
        x,
        depth_slope,
        color="#0a9396",
        lw=0.9,
        alpha=0.8,
        label=f"深度变化率（{slope_window_s:g} s）",
    )
    ax_velocity.axhline(0, color="0.25", lw=0.7)
    ax_velocity.set_ylabel("垂向速度（m/s）")
    ax_velocity.legend(loc="best", ncol=2)

    ax_attitude.plot(x, frame["pitch_deg"], color="#3a0ca3", lw=1.0, label="俯仰角")
    ax_attitude.plot(x, frame["roll_deg"], color="#4361ee", lw=1.0, label="横滚角")
    ax_attitude.axhline(0, color="0.25", lw=0.7)
    ax_attitude.set_ylabel("姿态角（°）")
    ax_attitude.legend(loc="best", ncol=2)

    ax_quality.plot(x, frame["nisD"], color="#6a4c93", lw=0.85, label="深度 NIS")
    ax_quality.axhline(
        DEPTH_NIS_GATE,
        color="#d00000",
        lw=0.9,
        ls="--",
        label=f"NIS 门限 {DEPTH_NIS_GATE:g}",
    )
    rejected = frame["reject_event"]
    reset = frame["reset_event"]
    if rejected.any():
        ax_quality.scatter(
            x[rejected],
            frame.loc[rejected, "nisD"],
            s=13,
            color="#d00000",
            zorder=3,
            label="深度更新拒绝",
        )
    if reset.any():
        for reset_x in x[reset]:
            ax_quality.axvline(reset_x, color="#ff006e", lw=1.0, alpha=0.8)
    ax_quality.set_yscale("symlog", linthresh=0.1)
    ax_quality.set_ylabel("深度 NIS")
    ax_quality.set_xlabel("板端相对时间（min）")
    ax_quality.legend(loc="best", ncol=3)

    shade_gaps_and_invalid(axes, frame)
    breaks = gap_midpoints(frame)
    add_break_marks(ax_quality, breaks, axis_y=0.0)
    add_event_clock_axis(ax_depth, frame, breaks)
    add_gap_annotations(ax_depth, frame)
    files = str(summary["files"]).replace(";", ", ")
    fig.suptitle(
        f"EKF 运动状态总览 — {date}，会话 {session_index:02d}\n"
        f"{files} | 深度更新：接受 {summary['depth_accepts']}，"
        f"拒绝 {summary['depth_rejects']}，重置 {summary['depth_resets']}",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.93))

    output_path = output_dir / f"{date}_session_{session_index:02d}_overview.png"
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return output_path


def plot_phase(
    date: str,
    session_index: int,
    frame: pd.DataFrame,
    output_dir: Path,
    dpi: int,
) -> Path:
    useful = frame[
        (frame["status_name"] == "RUNNING")
        & (frame["valid"] == 1)
        & np.isfinite(frame["pitch_deg"])
        & np.isfinite(frame["vD"])
    ]
    fig, ax = plt.subplots(figsize=(9, 6.5))
    points = ax.scatter(
        useful["pitch_deg"],
        useful["vD"],
        c=useful["elapsed_min"],
        cmap="viridis",
        s=8,
        alpha=0.55,
        edgecolors="none",
    )
    ax.axhline(0, color="0.25", lw=0.8)
    ax.axvline(0, color="0.25", lw=0.8)
    ax.set_xlabel("俯仰角（°）")
    ax.set_ylabel("vD（m/s，向下为正）")
    ax.set_title(
        f"俯仰角–垂向速度相图 — {date}，会话 {session_index:02d}"
    )
    # Matplotlib 3.5 在颜色条继承全局网格设置时会发出弃用警告。
    with plt.rc_context({"axes.grid": False}):
        colorbar = fig.colorbar(points, ax=ax)
    colorbar.set_label("板端相对时间（min）")
    fig.tight_layout()

    output_path = output_dir / f"{date}_session_{session_index:02d}_pitch_vD.png"
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return output_path


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="分组、汇总并绘制水下滑翔机 EKF 日志。"
    )
    parser.add_argument(
        "--date",
        help="日志日期，格式为 YYYYMMDD；默认使用最新日期。",
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=script_dir.parent / "csv",
        help="包含 ekf_YYYYMMDD_HHMMSS.csv 日志的目录。",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=script_dir / "plots",
        help="PNG 图和汇总 CSV 的输出目录。",
    )
    parser.add_argument(
        "--slope-window",
        type=float,
        default=1.0,
        help="深度中心差分窗口，单位为秒（默认：1.0）。",
    )
    parser.add_argument("--dpi", type=int, default=160)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.slope_window <= 0:
        raise ValueError("--slope-window 必须为正数")

    configure_plot_style()
    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    date, paths = discover_logs(input_dir, args.date)
    loaded = [(path, read_log(path)) for path in paths]
    grouped = group_boot_sessions(loaded)
    sessions = [combine_session(parts) for parts in grouped]
    summaries = [
        summarize_session(index, frame)
        for index, frame in enumerate(sessions, start=1)
    ]

    generated: List[Path] = []
    for index, (frame, summary) in enumerate(zip(sessions, summaries), start=1):
        generated.append(
            plot_overview(
                date,
                index,
                frame,
                summary,
                output_dir,
                args.slope_window,
                args.dpi,
            )
        )

    best_zero_index = int(np.argmax([item["_quality_score"] for item in summaries]))
    best_session_index = best_zero_index + 1
    generated.append(
        plot_phase(
            date,
            best_session_index,
            sessions[best_zero_index],
            output_dir,
            args.dpi,
        )
    )

    public_summaries = [
        {key: value for key, value in item.items() if not key.startswith("_")}
        for item in summaries
    ]
    summary_path = output_dir / f"{date}_sessions_summary.csv"
    pd.DataFrame(public_summaries).to_csv(summary_path, index=False, encoding="utf-8-sig")

    print(f"日期：{date}")
    print(f"输入日志：{len(paths)} 个")
    print(f"板端实验会话：{len(sessions)} 个")
    print(f"推荐会话：{best_session_index:02d}")
    print(f"汇总表：{summary_path}")
    for path in generated:
        print(f"图：{path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
