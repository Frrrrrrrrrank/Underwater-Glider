# EKF 日志分析与绘图

脚本会按板端 `t_ms/seq` 识别重新上电，将上位机断线重连产生的多个 CSV
拼成同一个实验会话，并自动生成：

- 每个实验会话的四联运动总览图；
- 深度更新质量和实验范围汇总表；
- 质量最好会话的 `pitch-vD` 相图。

总览图中的压力深度已经按 `ekf_bridge.c` 中的压力计杠杆臂换算到 IMU
参考点。底部 X 轴为板端相对时间，顶部 X 轴为根据板端 `t_ms` 和电脑时间
稳健对齐得到的估算事件时刻。灰色区域和 `//` 表示日志缺口，淡橙色区域
表示 `valid=0`。每个缺口旁会列出中断前最后一帧时刻、中断后第一帧时刻
和持续时间；被 EKF 拒绝的压力异常值不会参与深度曲线缩放。

从仓库 `Underwater-Glider` 目录运行：

```powershell
python logs/analysis/plot_ekf_logs.py --date 20260701
```

不传 `--date` 时自动选择最新日期：

```powershell
python logs/analysis/plot_ekf_logs.py
```

默认输出目录为 `logs/analysis/plots/`。可用参数：

```text
--input-dir DIR       日志目录
--output-dir DIR      图表与汇总表目录
--slope-window SEC    深度变化率的中心差分窗口，默认 1 秒
--dpi DPI             PNG 分辨率，默认 160
```
