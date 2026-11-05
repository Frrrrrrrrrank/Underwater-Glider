"""尾舵扳机、B 回中事件和只读诊断的离线验证，不接触真实手柄或网络。"""

from dataclasses import replace
from pathlib import Path
import sys
import unittest
from unittest import mock


STATION_DIR = Path(__file__).resolve().parent.parent
for directory in (STATION_DIR, STATION_DIR / "tools"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

import gamepad_control as gamepad
import gamepad_monitor


def sample(**changes):
    return replace(gamepad.GamepadSnapshot(index=0, connected=True), **changes)


class GamepadRudderMappingTests(unittest.TestCase):
    def test_config_rejects_invalid_deadzone_and_rate(self):
        for value in (True, "0.1", -0.1, 1, float("nan"), float("inf")):
            with self.subTest(deadzone=value), self.assertRaises(ValueError):
                gamepad.GamepadConfig(trigger_deadzone=value)
        for value in (True, "30", 0, -30, float("nan"), float("inf")):
            with self.subTest(rate=value), self.assertRaises(ValueError):
                gamepad.GamepadConfig(rudder_rate_deg_s=value)

    def test_trigger_deadzone_has_zero_linear_middle_and_full_endpoint(self):
        for value in (0, 0.05, 0.1):
            self.assertEqual(gamepad.apply_trigger_deadzone(value, 0.1), 0)
        self.assertAlmostEqual(gamepad.apply_trigger_deadzone(0.55, 0.1), 0.5)
        self.assertEqual(gamepad.apply_trigger_deadzone(1, 0.1), 1)
        for value in (-0.01, 1.01, True, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                gamepad.apply_trigger_deadzone(value, 0.1)

    def test_each_trigger_maps_to_signed_proportional_rate_and_release_zero(self):
        config = gamepad.GamepadConfig()
        for left, right, expected in ((0, 0, 0), (0.1, 0.1, 0),
                                      (0, 1, 30), (1, 0, -30),
                                      (0.1, 0.55, 15), (0.55, 0.1, -15)):
            with self.subTest(left=left, right=right):
                target = gamepad.map_gamepad_to_target(
                    sample(left_trigger=left, right_trigger=right), config)
                self.assertAlmostEqual(target.rudder_rate_deg_s, expected)
                self.assertFalse(target.rudder_center)

    def test_two_active_triggers_stop_even_when_pressures_are_unequal(self):
        for left, right in ((1, 1), (0.11, 1), (1, 0.11)):
            with self.subTest(left=left, right=right):
                target = gamepad.map_gamepad_to_target(
                    sample(left_trigger=left, right_trigger=right), gamepad.GamepadConfig())
                self.assertEqual(target.rudder_rate_deg_s, 0)
                self.assertFalse(target.rudder_center)

    def test_trigger_parameters_are_independent_of_joystick_deadzone(self):
        config = gamepad.GamepadConfig(deadzone=0.8, trigger_deadzone=0.2, rudder_rate_deg_s=20)
        target = gamepad.map_gamepad_to_target(sample(right_trigger=0.6, forward=0.6), config)
        self.assertAlmostEqual(target.rudder_rate_deg_s, 10)
        self.assertEqual(target.left_percent, 0)

    def test_b_centers_only_rudder_while_x_suppresses_everything(self):
        config = gamepad.GamepadConfig()
        baseline_sample = sample(forward=0.6, left_x=0.3, button_y=True, dpad_left=True)
        baseline = gamepad.map_gamepad_to_target(baseline_sample, config)
        for left, right in ((0, 1), (1, 0), (1, 1)):
            with self.subTest(left=left, right=right):
                target = gamepad.map_gamepad_to_target(
                    replace(baseline_sample, button_b=True, left_trigger=left, right_trigger=right),
                    config)
                self.assertEqual(target, replace(baseline, rudder_center=True))
        target = gamepad.map_gamepad_to_target(
            replace(baseline_sample, button_x=True, button_b=True, right_trigger=1), config)
        self.assertEqual(target, gamepad.ControlTarget())

    def test_invalid_trigger_button_and_event_values_are_not_neutral(self):
        invalids = [(key, value) for key in ("left_trigger", "right_trigger")
                    for value in (True, -0.01, 1.01, float("nan"), float("inf"))]
        invalids += [("button_b", 1), ("button_b", "按下")]
        invalids += [("rudder_center_seq", value) for value in (-1, 0.5, True, "1")]
        for key, value in invalids:
            with self.subTest(key=key, value=value):
                for operation in (gamepad.map_gamepad_to_target, gamepad.is_gamepad_neutral):
                    with self.assertRaises(ValueError):
                        operation(sample(**{key: value}), gamepad.GamepadConfig())

    def test_neutral_gate_requires_both_triggers_and_all_buttons_released(self):
        config = gamepad.GamepadConfig(deadzone=0.2, trigger_deadzone=0.1)
        self.assertTrue(gamepad.is_gamepad_neutral(
            sample(left_x=0.2, forward=-0.2, left_trigger=0.1, right_trigger=0.1,
                   rudder_center_seq=3), config))
        for changes in ({"left_trigger": 0.11}, {"right_trigger": 0.11},
                        {"left_trigger": 1, "right_trigger": 1}, {"button_b": True},
                        {"button_x": True}, {"button_y": True, "button_a": True},
                        {"dpad_left": True, "dpad_right": True}, {"forward": 0.21}):
            with self.subTest(changes=changes):
                self.assertFalse(gamepad.is_gamepad_neutral(sample(**changes), config))

    def test_existing_positional_arguments_keep_their_meaning(self):
        target = gamepad.ControlTarget(25, -60, 1, -50, 1, 300)
        self.assertEqual(target.roll_rpm, 300)
        self.assertEqual(target.rudder_rate_deg_s, 0)
        self.assertFalse(target.rudder_center)
        old_sample = gamepad.GamepadSnapshot(0, True, 0.1, 0.2, False, True, False,
                                            1.0, 2, 3, "", True, False)
        self.assertTrue(old_sample.dpad_left)
        self.assertEqual((old_sample.left_trigger, old_sample.right_trigger,
                          old_sample.button_b, old_sample.rudder_center_seq), (0, 0, False, 0))


class GamepadRudderSamplingTests(unittest.TestCase):
    def setUp(self):
        self.current = sample()
        backend = mock.Mock()
        backend.poll.side_effect = lambda index: self.current if index == 0 else gamepad.GamepadSnapshot(index)
        self.service = gamepad.GamepadService(backend=backend)

    def poll(self):
        self.service.poll_once()
        return self.service.snapshot(0)

    def test_b_short_press_counts_once_and_survives_release_and_disconnect(self):
        self.assertEqual(self.poll().rudder_center_seq, 0)
        self.current = sample(button_b=True)
        self.assertEqual(self.poll().rudder_center_seq, 1)
        self.assertEqual(self.poll().rudder_center_seq, 1)
        self.current = sample()
        released = self.poll()
        self.assertFalse(released.button_b)
        self.assertEqual(released.rudder_center_seq, 1)
        self.current = gamepad.GamepadSnapshot(index=0)
        failed = self.poll()
        self.assertFalse(failed.connected)
        self.assertEqual(failed.rudder_center_seq, 1)
        self.assertEqual(failed.sample_time, released.sample_time)
        self.current = sample()
        self.assertEqual(self.poll().rudder_center_seq, 1)
        self.current = sample(button_b=True)
        self.assertEqual(self.poll().rudder_center_seq, 2)

    def test_b_held_on_reconnect_counts_once_and_backend_cannot_reset_counter(self):
        self.current = sample(button_b=True, rudder_center_seq=99)
        self.assertEqual(self.poll().rudder_center_seq, 1)
        self.current = gamepad.GamepadSnapshot(index=0)
        self.poll()
        self.current = sample(button_b=True)
        self.assertEqual(self.poll().rudder_center_seq, 2)
        self.assertEqual(self.poll().rudder_center_seq, 2)

    def test_invalid_trigger_marks_input_loss_without_fabricating_center_event(self):
        self.current = sample(button_b=True)
        before = self.poll()
        self.current = sample(right_trigger=float("nan"), button_b=True)
        failed = self.poll()
        self.assertFalse(failed.connected)
        self.assertEqual(failed.loss_seq, before.loss_seq + 1)
        self.assertEqual(failed.sample_time, before.sample_time)
        self.assertEqual(failed.rudder_center_seq, before.rudder_center_seq)

    def test_x_and_b_events_remain_independent(self):
        self.current = sample(button_x=True, button_b=True)
        both = self.poll()
        self.assertEqual((both.exit_seq, both.rudder_center_seq), (1, 1))
        self.current = sample(button_b=True)
        self.poll()
        self.current = sample(button_x=True)
        final = self.poll()
        self.assertEqual((final.exit_seq, final.rudder_center_seq), (2, 1))

    def test_unsigned_trigger_endpoints_and_b_mask_decode_without_loading_dll(self):
        backend = object.__new__(gamepad.XInputBackend)
        raw = {"left": 0, "right": 0, "buttons": 0}

        def get_state(index, pointer):
            pointer._obj.Gamepad.bLeftTrigger = raw["left"]
            pointer._obj.Gamepad.bRightTrigger = raw["right"]
            pointer._obj.Gamepad.wButtons = raw["buttons"]
            return 0

        backend._get_state = get_state
        for left, right in ((0, 255), (255, 0), (128, 64), (255, 255)):
            for buttons in (0, 0x2000, 0xD00C, 0xF00C):
                with self.subTest(left=left, right=right, buttons=buttons):
                    raw.update(left=left, right=right, buttons=buttons)
                    result = backend.poll(0)
                    self.assertEqual(result.left_trigger, left / 255.0)
                    self.assertEqual(result.right_trigger, right / 255.0)
                    self.assertEqual(result.button_b, bool(buttons & 0x2000))


class GamepadRudderMonitorTests(unittest.TestCase):
    def test_parser_exposes_trigger_settings_and_formatter_labels_input_intent(self):
        args = gamepad_monitor.build_parser().parse_args(
            ["--trigger-deadzone", "0.2", "--rudder-rate-deg-s", "20"])
        config = gamepad.GamepadConfig(trigger_deadzone=args.trigger_deadzone,
                                       rudder_rate_deg_s=args.rudder_rate_deg_s)
        current = sample(right_trigger=0.6, rudder_center_seq=4)
        target = gamepad.map_gamepad_to_target(current, config)
        line = gamepad_monitor._format_sample(current, target, config)
        self.assertIn("LT=0.000 RT=0.600", line)
        self.assertIn("死区后LT/RT=0.000/0.500", line)
        self.assertIn("B=0", line)
        self.assertIn("目标角速度意图=+10.0度/秒", line)
        self.assertIn("回中意图=0 回中序号=4", line)


if __name__ == "__main__":
    unittest.main()
