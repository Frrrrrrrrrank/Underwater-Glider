"""Xbox 输入只读诊断：打印摇杆与映射结果，不导入控制台或访问网络。"""

import argparse
import math
from pathlib import Path
import sys
import time


# 允许从仓库根目录直接执行脚本，不触发 contcp_t 的任何初始化。
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gamepad_control import GamepadConfig, GamepadService, map_gamepad_to_target


def _positive_duration(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("持续时间必须是有限正数")
    return result


def build_parser():
    parser = argparse.ArgumentParser(description="Xbox 手柄只读诊断，不发送任何控制指令")
    parser.add_argument("--index", type=int, choices=range(4), help="固定 XInput 槽位 0 到 3")
    parser.add_argument("--max-percent", type=float, default=100, help="显示的最大输出百分率，默认 100")
    parser.add_argument("--deadzone", type=float, default=0.10, help="左右和前后轴各自的死区，默认 0.10")
    parser.add_argument("--duration", type=_positive_duration, help="运行指定秒数后退出")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        config = GamepadConfig(deadzone=args.deadzone, max_percent=args.max_percent)
    except ValueError as exc:
        parser.error(str(exc))
    try:
        service = GamepadService(config)
    except OSError as exc:
        print("[手柄诊断] {}".format(exc), file=sys.stderr)
        return 2

    selected_index = args.index
    # 本程序新建采集服务，序号从零开始，首次显示前发生的 X 短按也要保留。
    exit_baseline = 0
    last_notice = None
    start_time = time.monotonic()
    print("[手柄诊断] 仅显示输入与计算目标，不连接控制板。按 X 或 Ctrl+C 退出。")
    try:
        service.start()
        while args.duration is None or time.monotonic() - start_time < args.duration:
            if not service.is_alive():
                print("[手柄诊断] 采集线程已停止。")
                return 1
            if selected_index is None:
                indices = service.connected_indices()
                if len(indices) > 1:
                    print("[手柄诊断] 检测到多个槽位 {}，请用 --index 明确选择。".format(indices))
                    return 2
                if len(indices) == 1:
                    selected_index = indices[0]
                    print("[手柄诊断] 已固定槽位 {}，不会自动切换设备。".format(selected_index))
                else:
                    if last_notice != "等待手柄连接":
                        last_notice = "等待手柄连接"
                        print("[手柄诊断] {}。".format(last_notice))
                    time.sleep(config.send_period_s)
                    continue

            sample = service.snapshot(selected_index)
            if not sample.connected:
                notice = "槽位 {} 不可用：{}；保持当前槽位等待连接".format(selected_index, sample.error)
                if last_notice != notice:
                    last_notice = notice
                    print("[手柄诊断] " + notice)
            elif time.monotonic() - sample.sample_time > config.input_timeout_s:
                print("[手柄诊断] 输入采样已过期。")
                return 1
            else:
                last_notice = None
                if sample.button_x or sample.exit_seq != exit_baseline:
                    print("[手柄诊断] X 退出。")
                    return 0
                target = map_gamepad_to_target(sample, config)
                print(
                    "槽位={} 横向={:+.3f} 前后={:+.3f} X/Y/A={}/{}/{} "
                    "左={:+.1f}% 右={:+.1f}% pitch={}RPM 失连序号={}".format(
                        selected_index, sample.left_x, sample.forward,
                        int(sample.button_x), int(sample.button_y), int(sample.button_a),
                        target.left_percent, target.right_percent, target.pitch_rpm,
                        sample.loss_seq,
                    ),
                    flush=True,
                )
            time.sleep(config.send_period_s)
    except KeyboardInterrupt:
        print("\n[手柄诊断] Ctrl+C 退出。")
    finally:
        service.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
