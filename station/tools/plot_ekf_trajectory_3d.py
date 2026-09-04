#!/usr/bin/env python3
"""绘制 EKF 北-东-深度三维轨迹及三个二维投影。

默认从 station/logs/ekf 读取日志，把 PNG 写入 station/plots。脚本沿用
plot_ekf_logs.py 的上电会话识别和重连日志拼接逻辑；只有相邻两帧均处于
RUNNING、valid=1、坐标有限且时间间隔未超门限时才连接轨迹。
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np
import pandas as pd

from plot_ekf_logs import (
    combine_session,
    configure_plot_style,
    discover_logs,
    group_boot_sessions,
    read_log,
)


POSITION_COLUMNS = ("pN", "pE", "depth")


def trajectory_segments(
    frame: pd.DataFrame, gap_seconds: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """返回有效点掩码、三维线段、线段时间和可连接线段掩码。"""
    finite = np.ones(len(frame), dtype=bool)
    for column in (*POSITION_COLUMNS, "t_ms"):
        values = pd.to_numeric(frame[column], errors="coerce").to_numpy(dtype=float)
        finite &= np.isfinite(values)

    valid_points = (
        (frame["status_name"].to_numpy() == "RUNNING")
        & (pd.to_numeric(frame["valid"], errors="coerce").to_numpy() == 1)
        & finite
    )
    points = frame[["pE", "pN", "depth"]].to_numpy(dtype=float)
    elapsed = frame["elapsed_min"].to_numpy(dtype=float)
    gap = frame["t_ms"].diff().to_numpy(dtype=float) / 1000.0

    # 线段严格由原始相邻帧组成；中间出现 invalid 时不会跨过去连接。
    connected = (
        valid_points[:-1]
        & valid_points[1:]
        & np.isfinite(gap[1:])
        & (gap[1:] >= 0.0)
        & (gap[1:] <= gap_seconds)
    )
    segments = np.stack((points[:-1], points[1:]), axis=1)[connected]
    segment_time = ((elapsed[:-1] + elapsed[1:]) * 0.5)[connected]
    return valid_points, segments, segment_time, connected


def add_colored_2d_segments(
    ax: plt.Axes,
    segments_3d: np.ndarray,
    segment_time: np.ndarray,
    dimensions: Sequence[int],
    norm: Normalize,
) -> None:
    """把三维轨迹线段的指定两个分量投影到二维坐标轴。"""
    if len(segments_3d) == 0:
        return
    projected = segments_3d[:, :, list(dimensions)]
    collection = LineCollection(
        projected,
        cmap="viridis",
        norm=norm,
        linewidth=1.7,
        alpha=0.9,
    )
    collection.set_array(segment_time)
    ax.add_collection(collection)


def padded_limits(values: np.ndarray) -> Tuple[float, float]:
    """计算带少量边距的坐标范围，并处理整轴无变化的情况。"""
    low = float(np.nanmin(values))
    high = float(np.nanmax(values))
    span = high - low
    padding = 0.05 * span if span > 1e-9 else max(abs(low) * 0.05, 0.05)
    return low - padding, high + padding


def mark_start_end_3d(ax: plt.Axes, points: np.ndarray) -> None:
    """在三维坐标系标出整段有效数据的起点和终点。"""
    start = points[0]
    end = points[-1]
    ax.scatter(*start, s=65, color="#2a9d8f", marker="o", label="起点", zorder=5)
    ax.scatter(*end, s=75, color="#d62828", marker="^", label="终点", zorder=5)


def mark_start_end_2d(
    ax: plt.Axes, points: np.ndarray, dimensions: Sequence[int]
) -> None:
    """在二维投影中标出整段有效数据的起点和终点。"""
    start = points[0, list(dimensions)]
    end = points[-1, list(dimensions)]
    ax.scatter(*start, s=32, color="#2a9d8f", marker="o", zorder=5)
    ax.scatter(*end, s=38, color="#d62828", marker="^", zorder=5)


def plot_trajectory(
    date: str,
    session_index: int,
    frame: pd.DataFrame,
    output_dir: Path,
    gap_seconds: float,
    dpi: int,
) -> Path | None:
    """绘制一个板端上电会话；没有有效位置样本时返回 None。"""
    missing = [column for column in POSITION_COLUMNS if column not in frame.columns]
    if missing:
        raise ValueError(f"会话 {session_index:02d} 缺少位置字段：{missing}")

    valid, segments, segment_time, connected = trajectory_segments(
        frame, gap_seconds
    )
    valid_points = frame.loc[valid, ["pE", "pN", "depth"]].to_numpy(dtype=float)
    valid_time = frame.loc[valid, "elapsed_min"].to_numpy(dtype=float)
    if len(valid_points) == 0:
        print(f"跳过会话 {session_index:02d}：没有 RUNNING 且 valid=1 的有限位置样本")
        return None

    time_min = float(np.nanmin(valid_time))
    time_max = float(np.nanmax(valid_time))
    if time_max <= time_min:
        time_max = time_min + 1e-9
    norm = Normalize(vmin=time_min, vmax=time_max)

    fig = plt.figure(figsize=(17, 11.5))
    grid = fig.add_gridspec(
        3,
        3,
        width_ratios=(1.25, 1.25, 1.0),
        hspace=0.34,
        wspace=0.30,
    )
    ax_3d = fig.add_subplot(grid[:, :2], projection="3d")
    ax_plan = fig.add_subplot(grid[0, 2])
    ax_e_depth = fig.add_subplot(grid[1, 2])
    ax_n_depth = fig.add_subplot(grid[2, 2])

    if len(segments):
        line_3d = Line3DCollection(
            segments,
            cmap="viridis",
            norm=norm,
            linewidth=2.0,
            alpha=0.92,
        )
        line_3d.set_array(segment_time)
        ax_3d.add_collection3d(line_3d)
    else:
        ax_3d.scatter(
            valid_points[:, 0],
            valid_points[:, 1],
            valid_points[:, 2],
            c=valid_time,
            cmap="viridis",
            norm=norm,
            s=10,
        )

    mark_start_end_3d(ax_3d, valid_points)
    east_limits = padded_limits(valid_points[:, 0])
    north_limits = padded_limits(valid_points[:, 1])
    depth_limits = padded_limits(valid_points[:, 2])
    ax_3d.set_xlim(*east_limits)
    ax_3d.set_ylim(*north_limits)
    # NED 中深度向下为正；反转显示范围，使深处在画面下方。
    ax_3d.set_zlim(depth_limits[1], depth_limits[0])
    spans = np.array(
        [
            east_limits[1] - east_limits[0],
            north_limits[1] - north_limits[0],
            depth_limits[1] - depth_limits[0],
        ]
    )
    minimum_span = max(float(spans.max()) * 0.04, 1e-6)
    ax_3d.set_box_aspect(np.maximum(spans, minimum_span))
    ax_3d.set_xlabel("东向位置 pE（m）", labelpad=10)
    ax_3d.set_ylabel("北向位置 pN（m）", labelpad=10)
    ax_3d.set_zlabel("深度（m，向下为正）", labelpad=10)
    ax_3d.set_title("三维相对轨迹", pad=16)
    ax_3d.view_init(elev=24, azim=-58)
    ax_3d.legend(loc="upper left")

    projections = (
        (ax_plan, (0, 1), "水平面投影", "东向位置 pE（m）", "北向位置 pN（m）", False),
        (ax_e_depth, (0, 2), "东向–深度投影", "东向位置 pE（m）", "深度（m）", True),
        (ax_n_depth, (1, 2), "北向–深度投影", "北向位置 pN（m）", "深度（m）", True),
    )
    for axis, dims, title, xlabel, ylabel, invert_depth in projections:
        add_colored_2d_segments(axis, segments, segment_time, dims, norm)
        if len(segments) == 0:
            axis.scatter(
                valid_points[:, dims[0]],
                valid_points[:, dims[1]],
                c=valid_time,
                cmap="viridis",
                norm=norm,
                s=8,
            )
        mark_start_end_2d(axis, valid_points, dims)
        axis.set_xlim(*padded_limits(valid_points[:, dims[0]]))
        second_limits = padded_limits(valid_points[:, dims[1]])
        axis.set_ylim(
            (second_limits[1], second_limits[0])
            if invert_depth
            else second_limits
        )
        axis.set_title(title)
        axis.set_xlabel(xlabel)
        axis.set_ylabel(ylabel)
        axis.set_aspect("equal", adjustable="datalim")

    scalar = ScalarMappable(norm=norm, cmap="viridis")
    scalar.set_array([])
    # Matplotlib 3.5 在颜色条继承全局网格设置时会发出弃用警告。
    with plt.rc_context({"axes.grid": False}):
        colorbar = fig.colorbar(
            scalar,
            ax=[ax_3d, ax_plan, ax_e_depth, ax_n_depth],
            fraction=0.025,
            pad=0.025,
        )
    colorbar.set_label("板端相对时间（min）")

    connected_vectors = segments[:, 1, :2] - segments[:, 0, :2]
    horizontal_path = (
        float(np.linalg.norm(connected_vectors, axis=1).sum())
        if len(connected_vectors)
        else 0.0
    )
    horizontal_displacement = float(
        np.linalg.norm(valid_points[-1, :2] - valid_points[0, :2])
    )
    gap_count = int(
        np.count_nonzero(
            pd.to_numeric(frame["board_gap_s"], errors="coerce").to_numpy()
            > gap_seconds
        )
    )
    files = ", ".join(dict.fromkeys(frame["source_file"].tolist()))
    duration_min = (
        float(frame["t_ms"].iloc[-1]) - float(frame["t_ms"].iloc[0])
    ) / 60000.0
    fig.suptitle(
        f"EKF 北–东–深度轨迹 — {date}，会话 {session_index:02d}\n"
        f"{files}",
        fontsize=14,
        y=0.98,
    )
    fig.text(
        0.02,
        0.015,
        f"时长 {duration_min:.2f} min | 有效点 {len(valid_points)} | "
        f"日志缺口 {gap_count} | 水平净位移 {horizontal_displacement:.3f} m | "
        f"有效线段累计水平路程 {horizontal_path:.3f} m | 最大深度 {valid_points[:, 2].max():.3f} m\n"
        "说明：pN/pE 在缺少 GPS、DVL 或声学定位约束时是惯性积分得到的相对位置，长期漂移不等于真实航迹。",
        fontsize=9.5,
    )

    output_path = output_dir / f"{date}_session_{session_index:02d}_trajectory_3d.png"
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return output_path


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    station_dir = script_dir.parent
    parser = argparse.ArgumentParser(
        description="按板端上电会话绘制 EKF 北-东-深度三维轨迹。"
    )
    parser.add_argument(
        "--date",
        help="日志日期，格式为 YYYYMMDD；默认使用最新日期。",
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=station_dir / "logs" / "ekf",
        help="包含 ekf_YYYYMMDD_HHMMSS.csv 的目录。",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=station_dir / "plots",
        help="三维轨迹 PNG 输出目录。",
    )
    parser.add_argument(
        "--gap-seconds",
        type=float,
        default=0.5,
        help="相邻帧超过该间隔时断开轨迹，单位秒（默认：0.5）。",
    )
    parser.add_argument("--dpi", type=int, default=160, help="PNG 分辨率（默认：160）。")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.gap_seconds <= 0:
        raise ValueError("--gap-seconds 必须为正数")
    if args.dpi <= 0:
        raise ValueError("--dpi 必须为正数")

    configure_plot_style()
    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    date, paths = discover_logs(input_dir, args.date)
    loaded: List[Tuple[Path, pd.DataFrame]] = []
    for path in paths:
        frame = read_log(path)
        missing = [column for column in POSITION_COLUMNS if column not in frame.columns]
        if missing:
            raise ValueError(f"{path.name} 缺少位置字段：{missing}")
        loaded.append((path, frame))

    grouped = group_boot_sessions(loaded)
    sessions = [combine_session(parts) for parts in grouped]
    generated: List[Path] = []
    for index, frame in enumerate(sessions, start=1):
        path = plot_trajectory(
            date,
            index,
            frame,
            output_dir,
            args.gap_seconds,
            args.dpi,
        )
        if path is not None:
            generated.append(path)

    print(f"日期：{date}")
    print(f"输入日志：{len(paths)} 个")
    print(f"板端实验会话：{len(sessions)} 个")
    for path in generated:
        print(f"三维轨迹图：{path}")
    if not generated:
        raise ValueError("所选日志没有可绘制的有效三维位置数据")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
