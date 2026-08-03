#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
磁力计硬铁/软铁标定,椭球拟合,离线,PC 端
python tools\mag_calib_fit.py logs\csv\mag_时间戳.csv

输入：mag_logger.py 采集的 mag_*.csv（列 mx,my,mz，模块系/FLU，uT）
输出：硬铁偏置 b(3) + 软铁矩阵 A(3x3)，使  m_cal = A @ (m_raw - b)  近似落在
      半径 B_target(=去心后平均模长) 的球面上。同时打印可直接粘贴进
      ekf_bridge.c 的 C 数组、质量判据，并（若装了 matplotlib）保存对比图

原理简述
  硬铁 = 机体固连恒定偏置 -> 球心平移；软铁 = 机体软磁对外场的线性畸变 -> 球变椭球。
  对样本拟合一般二次曲面 (代数最小二乘, SVD 取最小奇异向量):
      a x^2 + b y^2 + c z^2 + 2f yz + 2g xz + 2h xy + 2p x + 2q y + 2r z + d = 0
  取 M=[[a,h,g],[h,b,f],[g,f,c]], 球心 b = -M^{-1} [p,q,r]，归一化成
      (m-b)^T Q (m-b) = 1,  Q = M / (b^T M b - d)
  Q 对称正定时 A = B_target * Q^{1/2}（对称平方根），即白化矩阵

覆盖度检查：
  方向单位向量的二阶矩最小特征值过小说明数据近似共面（比如只转了水平圈），
  完整椭球 9 参数不可辨识 -> 自动/手动退化到 --simple（仅硬铁 + 每轴 scale）

    python mag_calib_fit.py <mag_*.csv> [更多csv ...] [--simple] [--no-plot]
"""

import argparse
import json
import os
import sys
from datetime import datetime

import numpy as np

# 覆盖度阈值：方向二阶矩最小特征值（均匀球面=1/3）
COVERAGE_WARN = 0.03    # 低于此值警告"倾斜姿态太少"
COVERAGE_MIN = 0.005    # 低于此值椭球拟合不可信，强制退化 simple


def load_csv(paths):
    """读取一个或多个 mag CSV，拼接返回 Nx3 数组（列名 mx,my,mz）。"""
    chunks = []
    for p in paths:
        with open(p, "r", encoding="utf-8") as f:
            header = f.readline().strip().split(",")
        try:
            idx = [header.index(c) for c in ("mx", "my", "mz")]
        except ValueError:
            sys.exit("[错误] %s 缺少 mx/my/mz 列（表头: %s）" % (p, header))
        data = np.genfromtxt(p, delimiter=",", skip_header=1,
                             usecols=idx, dtype=np.float64)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        chunks.append(data)
        print("[输入] %s: %d 行" % (p, data.shape[0]))
    m = np.vstack(chunks)
    m = m[np.all(np.isfinite(m), axis=1)]
    return m


def fit_ellipsoid(m):
    """代数最小二乘椭球拟合。返回 (center b, 对称正定 Q)；拟合退化抛 ValueError。"""
    x, y, z = m[:, 0], m[:, 1], m[:, 2]
    # 设计矩阵列序: x², y², z², 2yz, 2xz, 2xy, 2x, 2y, 2z, 1
    D = np.column_stack([x * x, y * y, z * z,
                         2 * y * z, 2 * x * z, 2 * x * y,
                         2 * x, 2 * y, 2 * z,
                         np.ones_like(x)])
    # 数值预条件：按平均模长缩放，避免 x² 与 1 量级差太大
    s = np.mean(np.linalg.norm(m, axis=1))
    S = np.diag([1/s**2]*3 + [1/s**2]*3 + [1/s]*3 + [1.0])
    _, _, Vt = np.linalg.svd(D @ S, full_matrices=False)
    v = S @ Vt[-1]  # 还原缩放后的最小奇异向量

    a, b_, c, f, g, h, p, q, r, d = v
    M = np.array([[a, h, g],
                  [h, b_, f],
                  [g, f, c]])
    n = np.array([p, q, r])
    try:
        center = -np.linalg.solve(M, n)
    except np.linalg.LinAlgError:
        raise ValueError("二次曲面矩阵奇异")
    k = float(center @ M @ center - d)
    if abs(k) < 1e-15:
        raise ValueError("归一化系数为零")
    Q = M / k
    eigval, eigvec = np.linalg.eigh(Q)
    if np.any(eigval <= 0):
        raise ValueError("拟合结果非椭球（特征值 %s），姿态覆盖可能不足" % eigval)
    return center, (eigvec * eigval) @ eigvec.T  # 重组保证数值对称


def sqrtm_spd(Q):
    """对称正定矩阵的对称平方根。"""
    eigval, eigvec = np.linalg.eigh(Q)
    return (eigvec * np.sqrt(eigval)) @ eigvec.T


def fit_simple(m):
    """退化标定：每轴 (min+max)/2 为硬铁，每轴半径归一到平均半径（对角软铁）。"""
    mn, mx = m.min(axis=0), m.max(axis=0)
    b = (mn + mx) / 2.0
    radii = (mx - mn) / 2.0
    radii = np.where(radii < 1e-9, 1.0, radii)
    B = float(np.mean(radii))
    A = np.diag(B / radii)
    return b, A, B


def coverage_min_eig(m, b):
    """方向单位向量二阶矩的最小特征值（均匀球面=1/3，共面->0）。"""
    v = m - b
    n = np.linalg.norm(v, axis=1, keepdims=True)
    u = v / np.maximum(n, 1e-12)
    return float(np.linalg.eigvalsh(u.T @ u / u.shape[0]).min())


def report_norm(tag, norms):
    mean, std = float(np.mean(norms)), float(np.std(norms))
    print("  %-8s |m| 均值=%.2f uT  标准差=%.2f  波动比 std/mean=%.2f%%"
          % (tag, mean, std, 100.0 * std / mean))
    return std / mean


def c_snippet(b, A, n_samples):
    """生成可替换 ekf_bridge.c 默认参考值的 C 数组。"""
    date = datetime.now().strftime("%Y-%m-%d %H:%M")
    lines = []
    lines.append("/* ===== 磁力计硬铁/软铁标定结果（模块系/FLU，uT）=====")
    lines.append(" * 生成: %s, 样本数 %d, 工具 station/tools/mag_calib_fit.py" % (date, n_samples))
    lines.append(" * 用法: m_cal = s_mag_soft_iron * (m_raw - s_mag_hard_iron_ut)")
    lines.append(" * 必须在 mount_R 之前(模块系)应用，然后再 UgEkf_PushMag。")
    lines.append(" * 这里只是固件默认参考值；上电 valid=0，正常流程仍须用 JSON 参数事务下发。")
    lines.append(" * 机体结构/带磁部件变动后必须重标。 */")
    lines.append("static float s_mag_hard_iron_ut[3] = { %.4ff, %.4ff, %.4ff };"
                 % tuple(b))
    lines.append("static float s_mag_soft_iron[3][3] = {")
    for i in range(3):
        lines.append("  { %.6ff, %.6ff, %.6ff },%s"
                     % (A[i, 0], A[i, 1], A[i, 2], "" if i < 2 else ""))
    lines.append("};")
    return "\n".join(lines)


def try_plot(m_raw, m_cal, b, out_png):
    """保存 raw/校正 对比图（matplotlib 可选依赖，没装就跳过）。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[提示] 未安装 matplotlib，跳过绘图（pip install matplotlib）")
        return
    fig, axes = plt.subplots(2, 2, figsize=(11, 10))
    pairs = [(0, 1, "mx-my"), (0, 2, "mx-mz"), (1, 2, "my-mz")]
    for ax, (i, j, name) in zip(axes.flat[:3], pairs):
        ax.scatter(m_raw[:, i], m_raw[:, j], s=2, c="tab:red", label="raw")
        ax.scatter(m_cal[:, i], m_cal[:, j], s=2, c="tab:blue", label="calibrated")
        ax.axhline(0, lw=0.5, c="gray")
        ax.axvline(0, lw=0.5, c="gray")
        ax.set_title(name)
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(markerscale=4)
    ax = axes.flat[3]
    ax.plot(np.linalg.norm(m_raw - b, axis=1), lw=0.6, c="tab:red",
            label="|m_raw - b|")
    ax.plot(np.linalg.norm(m_cal, axis=1), lw=0.6, c="tab:blue", label="|m_cal|")
    ax.set_title("field magnitude per sample")
    ax.set_xlabel("sample")
    ax.set_ylabel("uT")
    ax.legend()
    fig.suptitle("mag calibration: raw(red) vs calibrated(blue)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print("[图] 已保存 %s" % out_png)


def main():
    ap = argparse.ArgumentParser(description="磁力计硬铁/软铁椭球拟合")
    ap.add_argument("csv", nargs="+", help="mag_logger.py 生成的 CSV（可多个拼接）")
    ap.add_argument("--simple", action="store_true",
                    help="只做硬铁+每轴scale（数据覆盖差时用）")
    ap.add_argument("--no-plot", action="store_true", help="不生成对比图")
    args = ap.parse_args()

    m = load_csv(args.csv)
    if m.shape[0] < 200:
        print("[警告] 样本只有 %d 个（10Hz 下建议至少 10 分钟 ≈ 6000 个），"
              "结果可能不可靠" % m.shape[0])
    print("[数据] 共 %d 个样本" % m.shape[0])
    print("  各轴范围: x[%.1f, %.1f] y[%.1f, %.1f] z[%.1f, %.1f] uT"
          % (m[:, 0].min(), m[:, 0].max(), m[:, 1].min(), m[:, 1].max(),
             m[:, 2].min(), m[:, 2].max()))

    # —— 拟合（自动选择完整椭球 / simple 退化） ——
    mode = "simple" if args.simple else "ellipsoid"
    coverage = coverage_min_eig(m, m.mean(axis=0))
    if mode == "ellipsoid":
        print("[覆盖度] 方向二阶矩最小特征值 = %.4f（均匀球面=0.333）" % coverage)
        if coverage < COVERAGE_MIN:
            print("[退化] 覆盖度过低（数据近似共面），完整椭球不可辨识，改用 simple 模式；"
                  "建议补采俯仰/横滚姿态后重拟合")
            mode = "simple"
        elif coverage < COVERAGE_WARN:
            print("[警告] 倾斜姿态覆盖偏少，软铁非对角项可信度有限；"
                  "建议补采俯仰 ±30~45° 的旋转数据")

    if mode == "ellipsoid":
        try:
            b, Q = fit_ellipsoid(m)
            # 第二遍：按马氏残差剔除野值后重拟合（磕碰/瞬时电流干扰）
            r = np.einsum("ni,ij,nj->n", m - b, Q, m - b)  # 椭球上应≈1
            keep = np.abs(r - 1.0) < 4.0 * np.std(r - 1.0) + 1e-12
            if keep.sum() >= 200 and keep.sum() < m.shape[0]:
                print("[野值] 剔除 %d/%d 个离群样本后重拟合"
                      % (m.shape[0] - keep.sum(), m.shape[0]))
                b, Q = fit_ellipsoid(m[keep])
            B_target = float(np.mean(np.linalg.norm(m - b, axis=1)))
            A = B_target * sqrtm_spd(Q)
        except ValueError as e:
            print("[退化] 椭球拟合失败（%s），改用 simple 模式" % e)
            mode = "simple"
    if mode == "simple":
        b, A, B_target = fit_simple(m)

    m_cal = (m - b) @ A.T
    coverage = coverage_min_eig(m, b)

    # —— 质量判据 ——
    print("\n[结果] 模式=%s" % mode)
    print("  硬铁 b = (%.3f, %.3f, %.3f) uT，|b|=%.1f uT" % (*b, np.linalg.norm(b)))
    eig = np.linalg.eigvalsh(A)
    print("  软铁 A 特征值 = (%.4f, %.4f, %.4f)，比值 max/min = %.3f"
          % (*(eig / np.mean(eig)), eig.max() / eig.min()))
    print("  目标场强 B_target = %.2f uT" % B_target)
    print("[质量] 校正前后模长稳定性（判据：校正后 std/mean < 2~3%% 合格）：")
    raw_ratio = report_norm("校正前", np.linalg.norm(m, axis=1))
    ratio = report_norm("校正后", np.linalg.norm(m_cal, axis=1))
    if ratio < 0.03:
        print("  => 合格，可以上板")
    elif ratio < 0.05:
        print("  => 勉强，建议换更干净的场地/补采数据后重标")
    else:
        print("  => 不合格：检查采集环境（钢结构/电流），或姿态覆盖不足")

    # —— C 数组输出 ——
    snippet = c_snippet(b, A, m.shape[0])
    print("\n----- 复制以下内容进 ekf_bridge.c -----\n%s\n----------------------------------------" % snippet)

    base = os.path.splitext(args.csv[0])[0]
    txt_path = base + "_calib.txt"
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write(snippet + "\n\nmode=%s  N=%d  std/mean=%.4f\n"
                % (mode, m.shape[0], ratio))
    print("[文件] 结果已存 %s" % txt_path)

    json_path = base + "_calib.json"
    quality_class = (
        "pass" if ratio < 0.03
        else "marginal" if ratio < 0.05
        else "fail"
    )
    document = {
        "schema": "underwater_glider.mag_calibration",
        "version": 1,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "generated_by": "station/tools/mag_calib_fit.py",
        "coordinate_frame": "module_FLU",
        "hard_iron_unit": "uT",
        "soft_iron_order": "row_major",
        "hard_iron_ut": [float(value) for value in b],
        "soft_iron": [
            [float(A[row, column]) for column in range(3)]
            for row in range(3)
        ],
        "reserved": [0.0, 0.0, 0.0, 0.0],
        "sample_count": int(m.shape[0]),
        "fit_mode": mode,
        "source_csv": [os.path.abspath(path) for path in args.csv],
        "quality": {
            "class": quality_class,
            "raw_std_over_mean": float(raw_ratio),
            "calibrated_std_over_mean": float(ratio),
            "target_field_ut": float(B_target),
            "coverage_min_eigenvalue": float(coverage),
            "soft_iron_eigenvalues": [float(value) for value in eig],
            "soft_iron_condition": float(eig.max() / eig.min()),
        },
    }
    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2)
    print("[文件] 可直接下发的 JSON 已存 %s" % json_path)

    if not args.no_plot:
        try_plot(m, m_cal, b, base + "_calib.png")


if __name__ == "__main__":
    main()
