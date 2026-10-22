#!/usr/bin/env python3
"""遥测缺失、新鲜度与字段质量的回归测试，不接真实设备。"""

import json
import math
import sys
import unittest
from pathlib import Path


STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

from groundstation.legacy_telemetry import (  # noqa: E402
    EKF_FIELDS,
    UGCTL_FIELDS,
    UGWRM_FIELDS_V2 as UGWRM_FIELDS,
    decode_telemetry_line,
)
from groundstation.models import DEFAULT_REQUIRED_TOPICS, LinkStatus  # noqa: E402
from groundstation.state_store import StateStore  # noqa: E402


class FakeClock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value


def make_line(prefix, fields, overrides):
    """构造带真实 XOR 校验的线上语句，确保故障值走原解析器。"""

    defaults = {"schema": 2, "fl": 2, "data_fresh": 1, "span_valid": 1}
    defaults.update(overrides)
    body = prefix + "," + ",".join(str(defaults.get(name, 0)) for name in fields)
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return "$" + body + "*{:02X}".format(checksum)


class TelemetryCompletenessTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.store = StateStore(clock=self.clock)
        self.store.set_link(LinkStatus.CONNECTED, session_id="session-1")

    def publish_all(self):
        self.store.publish_telemetry("ugekf", {"fl": 2, "depth": 1.0})
        self.store.publish_telemetry("ugctl", {"armed": 0, "fault": 1})
        self.store.publish_telemetry("ugwrm", {"span_valid": 1, "data_fresh": 1})

    def test_default_requires_all_three_topics(self):
        self.store.publish_telemetry("ugekf", {"fl": 2, "depth": 1.0})
        _, partial = self.store.read()
        self.assertEqual(partial["state"]["required_topics"], list(DEFAULT_REQUIRED_TOPICS))
        self.assertEqual(partial["state"]["missing_topics"], ["ugctl", "ugwrm"])
        self.assertFalse(partial["state"]["overall_fresh"])
        self.assertEqual(partial["state"]["stale_reason"], "missing_telemetry")

        self.publish_all()
        _, complete = self.store.read()
        self.assertEqual(complete["state"]["missing_topics"], [])
        self.assertTrue(complete["state"]["overall_fresh"])

        self.clock.value += 0.501
        _, stale = self.store.read()
        self.assertFalse(stale["state"]["overall_fresh"])
        self.assertEqual(stale["state"]["stale_reason"], "stale_telemetry")

    def test_additional_stale_topic_does_not_block_required_topics(self):
        self.store.publish_telemetry("diagnostic", {"counter": 1})
        self.clock.value += 1.0
        self.publish_all()
        _, snapshot = self.store.read()
        self.assertEqual(snapshot["state"]["stale_topics"], ["diagnostic"])
        self.assertTrue(snapshot["state"]["overall_fresh"])

    def test_disconnected_link_is_not_fresh_even_with_recent_samples(self):
        self.publish_all()
        self.store.set_link(LinkStatus.DISCONNECTED)
        _, snapshot = self.store.read()
        self.assertTrue(snapshot["telemetry"]["ugekf"]["fresh"])
        self.assertFalse(snapshot["state"]["overall_fresh"])
        self.assertEqual(snapshot["state"]["stale_reason"], "device_link_disconnected")

    def test_empty_required_topics_cannot_make_empty_state_fresh(self):
        store = StateStore(clock=self.clock, required_topics=())
        store.set_link(LinkStatus.CONNECTED)
        _, empty = store.read()
        self.assertFalse(empty["state"]["overall_fresh"])
        self.assertEqual(empty["state"]["stale_reason"], "no_telemetry")
        store.publish_telemetry("diagnostic", {"counter": 1})
        self.assertTrue(store.read()[1]["state"]["overall_fresh"])
        self.clock.value += 1.0
        self.assertFalse(store.read()[1]["state"]["overall_fresh"])

    def test_new_session_must_receive_its_own_topics(self):
        self.publish_all()
        self.store.clear_session_telemetry()
        self.store.set_link(LinkStatus.CONNECTED, session_id="session-2")
        _, empty = self.store.read()
        self.assertEqual(empty["telemetry"], {})
        self.assertEqual(empty["state"]["missing_topics"], list(DEFAULT_REQUIRED_TOPICS))
        self.assertFalse(empty["state"]["overall_fresh"])

        self.store.publish_telemetry("ugekf", {"fl": 2}, session_id="session-2")
        _, partial = self.store.read()
        self.assertEqual(partial["telemetry"]["ugekf"]["session_id"], "session-2")
        self.assertFalse(partial["state"]["overall_fresh"])


class TelemetryFieldQualityTests(unittest.TestCase):
    def setUp(self):
        self.store = StateStore(clock=FakeClock())

    def test_non_finite_values_are_json_safe_without_losing_other_data(self):
        original = {"reading": float("nan"), "extra": {"values": [float("inf"), -float("inf"), 42]}, "future_field": "kept"}
        self.store.publish_telemetry("diagnostic", original)
        _, snapshot = self.store.read()
        sample = snapshot["telemetry"]["diagnostic"]
        self.assertIsNone(sample["data"]["reading"])
        self.assertEqual(sample["data"]["extra"]["values"], [None, None, 42])
        self.assertEqual(sample["data"]["future_field"], "kept")
        self.assertEqual(sample["invalid_fields"], {
            "reading": "non_finite",
            "extra.values[0]": "non_finite",
            "extra.values[1]": "non_finite",
        })
        self.assertEqual(sample["quality"], {"status": "invalid", "reasons": ["non_finite"]})
        self.assertEqual(json.loads(json.dumps(snapshot, allow_nan=False)), snapshot)
        self.assertTrue(math.isnan(original["reading"]))

        # 返回对象也不能反向污染状态仓库。
        sample["data"]["extra"]["values"][2] = 99
        self.assertEqual(self.store.read()[1]["telemetry"]["diagnostic"]["data"]["extra"]["values"][2], 42)

    def test_real_parser_keeps_non_finite_fault_rows_for_each_topic(self):
        cases = (
            ("UGEKF", EKF_FIELDS, "roll_deg"),
            ("UGCTL", UGCTL_FIELDS, "depth"),
            ("UGWRM", UGWRM_FIELDS, "stroke_percent"),
        )
        for prefix, fields, field in cases:
            with self.subTest(topic=prefix):
                decoded = decode_telemetry_line(make_line(prefix, fields, {field: "nan"}))
                self.assertIsNotNone(decoded)
                self.store.publish_telemetry(decoded.topic, decoded.data)
                sample = self.store.read()[1]["telemetry"][decoded.topic]
                self.assertIsNone(sample["data"][field])
                self.assertEqual(sample["invalid_fields"][field], "non_finite")
                self.assertEqual(sample["data"]["seq"], 0)
                json.dumps(sample, allow_nan=False)

    def test_fresh_sample_and_overall_fresh_do_not_imply_field_validity(self):
        self.store.set_link(LinkStatus.CONNECTED)
        self.store.publish_telemetry("ugekf", {"fl": 0, "roll_deg": 12.0, "depth": 3.0, "status": 3, "rejD": 4})
        self.store.publish_telemetry("ugctl", {"armed": 0, "fault": 1})
        self.store.publish_telemetry("ugwrm", {"data_fresh": 1, "span_valid": 1})
        _, snapshot = self.store.read()
        sample = snapshot["telemetry"]["ugekf"]
        self.assertTrue(snapshot["state"]["overall_fresh"])
        self.assertTrue(sample["fresh"])
        self.assertEqual(sample["quality"]["status"], "invalid")
        self.assertEqual(sample["invalid_fields"], {"roll_deg": "ekf_not_valid", "depth": "ekf_not_valid"})
        self.assertEqual(sample["data"]["roll_deg"], 12.0)
        self.assertEqual(sample["data"]["status"], 3)
        self.assertEqual(sample["data"]["rejD"], 4)
        self.assertFalse(snapshot["state"]["capabilities"]["realtime_control"])

    def test_worm_span_gate_does_not_hide_raw_position_or_diagnostics(self):
        self.store.publish_telemetry("ugwrm", {"span_valid": 0, "data_fresh": 1, "pulse": 123, "stroke_percent": 0.0, "mm_from_front": 0.0, "alarm": 1, "home_fail": 0})
        sample = self.store.read()[1]["telemetry"]["ugwrm"]
        self.assertEqual(sample["invalid_fields"], {"stroke_percent": "worm_span_invalid", "mm_from_front": "worm_span_invalid"})
        self.assertEqual(sample["data"]["pulse"], 123)
        self.assertEqual(sample["data"]["alarm"], 1)

    def test_worm_feedback_gate_and_non_finite_reason_are_independent(self):
        self.store.publish_telemetry("ugwrm", {"span_valid": 1, "data_fresh": 0, "pulse": 123, "stroke_percent": float("nan"), "mm_from_front": 0.0, "poll_fail": 5})
        sample = self.store.read()[1]["telemetry"]["ugwrm"]
        self.assertEqual(sample["invalid_fields"], {"stroke_percent": "non_finite", "pulse": "worm_feedback_stale", "mm_from_front": "worm_feedback_stale"})
        self.assertEqual(sample["data"]["poll_fail"], 5)

    def test_missing_validity_flag_is_unknown_not_assumed_valid(self):
        self.store.publish_telemetry("ugekf", {"depth": 1.0})
        sample = self.store.read()[1]["telemetry"]["ugekf"]
        self.assertEqual(sample["quality"], {"status": "unknown", "reasons": ["ekf_validity_not_reported"]})

    def test_disarmed_control_status_is_valid_diagnostic_data(self):
        row = {"armed": 0, "output_valid": 0, "fault": 1, "depth": 3.0}
        self.store.publish_telemetry("ugctl", row)
        sample = self.store.read()[1]["telemetry"]["ugctl"]
        self.assertEqual(sample["data"], row)
        self.assertEqual(sample["invalid_fields"], {})
        self.assertEqual(sample["quality"]["status"], "valid")


if __name__ == "__main__":
    unittest.main()
