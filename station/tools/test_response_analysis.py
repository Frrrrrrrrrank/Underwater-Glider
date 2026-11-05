"""合成时间线验证：区间是 PC 观测，坏数据不能生成有效机械响应结论。"""
from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))
import analyze_response as analysis


COMPLETE = dict(complete=True, closed=True, dropped_records=0, rejected_records=0)


def stamp(seconds):
    return int((10 + seconds) * 1e9)


def tx(seconds, pitch=0, roll=0, **changes):
    event = dict(kind="tx", session_id="session", run_id="run1", source="auto_sti",
        phase="ACTIVE", t_ns=stamp(seconds), tx_start_ns=stamp(seconds),
        tx_end_ns=stamp(seconds + 0.01), success=True,
        fields=dict(pitchmotor_enable=int(pitch != 0), pitchmotor_speed_rpm=pitch,
                    rollmotor_enable=int(roll != 0), rollmotor_speed_rpm=roll,
                    eth_control_flag=1))
    event.update(changes)
    return event


def rx(seconds, pulse=0, roll=0, **changes):
    row = dict(schema=3, seq=0, t_ms=int(1000 + seconds * 1000), pulse=pulse,
        data_fresh=1, ref_kind=2, ref_lost=0, span_valid=1, span_pulses=281321,
        pulse_home=0, home_state=3, alarm=0, cmd_rpm=-100, fb_rpm=-100,
        roll_pulse=roll, roll_valid=1, roll_pulse_min=0, roll_pulse_max=1000,
        roll_span_pulses=1000, roll_home_state=6, calib_state=2, calib_hold=0)
    row.update(changes)
    return dict(kind="rx", session_id="session", t_ns=stamp(seconds), row=row,
                topic="ugwrm", raw="$UGWRM,...", batch_id=int(seconds * 1000))


def ordered(events):
    events.sort(key=lambda item: item["t_ns"])
    sequence = 0
    for number, event in enumerate(events, 1):
        event["event_id"] = number
        if event.get("kind") == "rx" and isinstance(event.get("row"), dict):
            sequence += 1
            event["row"]["seq"] = sequence
    return events


def recording(both=False):
    target = 100 if both else 0
    events = [tx(0), rx(.2), rx(.4), rx(.6), tx(.7, 100, target),
        rx(.8), rx(1.0, 10, 10 if both else 0), rx(1.2, 20, 20 if both else 0),
        rx(1.4, 30, 30 if both else 0), tx(1.5),
        rx(1.6, 40, 40 if both else 0), rx(1.8, 50, 50 if both else 0),
        rx(2.0, 50, 50 if both else 0), rx(2.2, 50, 50 if both else 0)]
    return ordered(events)


def pitch_start(events, metadata=None, **kwargs):
    rows, _, _ = analysis.analyze(events, metadata or COMPLETE, **kwargs)
    return next(row for row in rows if row["axis"] == "pitch" and row["change"] == "start")


class ResponseAnalysisTests(unittest.TestCase):
    def test_start_and_stop_have_sample_intervals_and_confirmation_time(self):
        rows, _, problems = analysis.analyze(recording(True), COMPLETE)
        self.assertFalse(problems)
        self.assertEqual(len(rows), 4)
        self.assertTrue(all(row["quality"] == "observed" for row in rows))
        start = rows[0]
        self.assertAlmostEqual(start["observed_lower_s"], .09)
        self.assertAlmostEqual(start["observed_upper_s"], .3)
        self.assertEqual(start["first_observed_ns"], stamp(1.0))
        self.assertEqual(start["confirmed_ns"], stamp(1.4))
        self.assertAlmostEqual(start["pulse_rate_per_s"], 50)
        self.assertAlmostEqual(start["rate_window_s"], .4)
        stop = next(row for row in rows if row["axis"] == "pitch" and row["change"] == "stop")
        self.assertAlmostEqual(stop["observed_upper_s"], .3)
        self.assertEqual(stop["confirmed_ns"], stamp(2.2))
        self.assertEqual(stop["pulse_rate_per_s"], 0)
        self.assertEqual(rows[1]["feedback_kind"], "output_pulse_estimate")

    def test_heartbeats_do_not_repeat_command_changes(self):
        events = recording()
        events.extend(tx(time, 100) for time in (.75, .9, 1.1, 1.3))
        rows, _, _ = analysis.analyze(ordered(events), COMPLETE)
        self.assertEqual(len(rows), 2)
        self.assertTrue(all(row["quality"] == "observed" for row in rows))

    def test_stale_pitch_rejects_estimate_but_roll_has_independent_validity(self):
        events = recording(True)
        next(event for event in events if event["t_ns"] == stamp(1.0))["row"]["data_fresh"] = 0
        rows, _, _ = analysis.analyze(events, COMPLETE)
        self.assertEqual(rows[0]["reason"], "pitch_stale_or_reference_unknown")
        self.assertEqual(rows[1]["quality"], "observed")

    def test_duplicate_or_reset_or_sequence_gap_blocks_observation(self):
        for field, value, reason in (("seq", 4, "duplicate_telemetry"),
                ("seq", 1, "board_clock_or_sequence_reset"),
                ("seq", 6, "telemetry_sequence_gap"),
                ("t_ms", 0, "board_clock_or_sequence_reset")):
            with self.subTest(field=field, value=value):
                events = recording()
                next(event for event in events if event["t_ns"] == stamp(1.0))["row"][field] = value
                self.assertEqual(pitch_start(events)["reason"], reason)

    def test_uint32_counter_wrap_is_not_a_reset(self):
        events = recording()
        for number, event in enumerate(event for event in events if event["kind"] == "rx"):
            event["row"]["seq"] = (0xFFFFFFFD + number) & 0xFFFFFFFF
            event["row"]["t_ms"] = (0xFFFFFF00 + 200 * number) & 0xFFFFFFFF
        self.assertEqual(pitch_start(events)["quality"], "observed")

    def test_reference_loss_is_a_counter_and_only_changes_break_continuity(self):
        events = recording()
        for event in events:
            if event["kind"] == "rx":
                event["row"]["ref_lost"] = 3
        self.assertEqual(pitch_start(events)["quality"], "observed")
        next(event for event in events if event["t_ns"] == stamp(1.0))["row"]["ref_lost"] = 4
        self.assertEqual(pitch_start(events)["reason"], "calibration_or_reference_changed")

    def test_missing_baseline_and_no_prior_command_are_explicit(self):
        events = recording()
        self.assertEqual(pitch_start([event for event in events if event["t_ns"] >= stamp(.7)])["reason"], "missing_prior_command")
        self.assertEqual(pitch_start([event for event in events if event["t_ns"] != stamp(.2)])["reason"], "insufficient_precommand_baseline")

    def test_failed_send_cannot_be_a_valid_response(self):
        events = recording()
        next(event for event in events if event["t_ns"] == stamp(.7))["success"] = False
        self.assertEqual(pitch_start(events)["reason"], "send_failed_may_be_partial")

    def test_later_failed_heartbeat_cuts_off_pending_measurement(self):
        events = recording() + [tx(1.1, 100, success=False)]
        self.assertEqual(pitch_start(ordered(events))["reason"], "send_failed_boundary")

    def test_schema2_cannot_create_roll_estimate(self):
        events = recording(True)
        for event in events:
            if event["kind"] == "rx":
                event["row"]["schema"] = 2
                event["row"]["roll_pulse"] = None
                event["row"]["roll_valid"] = None
        rows, _, _ = analysis.analyze(events, COMPLETE)
        self.assertEqual(rows[0]["quality"], "observed")
        self.assertEqual(rows[1]["reason"], "roll_unavailable_old_schema")

    def test_mode_link_and_run_changes_cut_off_pairing(self):
        for event, expected in ((dict(kind="mode", phase="EXIT", t_ns=stamp(1.1)), "mode_boundary"),
                (dict(kind="link", connected=False, t_ns=stamp(1.1)), "link_boundary"),
                (tx(1.1, 100, run_id="run2"), "run_changed"),
                (tx(1.1, 100, source="auto"), "other_control_source")):
            with self.subTest(expected=expected):
                event["session_id"] = "session"
                self.assertEqual(pitch_start(ordered(recording() + [event]))["reason"], expected)

    def test_release_is_a_boundary_not_a_stop_measurement(self):
        release = tx(1.1)
        release["fields"]["eth_control_flag"] = 0
        rows, _, _ = analysis.analyze(ordered(recording() + [release]), COMPLETE)
        self.assertEqual(rows[0]["reason"], "control_released")
        self.assertFalse(any(row["tx_start_ns"] == stamp(1.1) for row in rows))

    def test_nonzero_speed_steps_are_not_mislabeled_as_start(self):
        rows, _, _ = analysis.analyze(ordered(recording() + [tx(1.45, -100)]), COMPLETE)
        change = next(row for row in rows if row["tx_start_ns"] == stamp(1.45))
        self.assertEqual(change["change"], "speed_change")
        self.assertEqual(change["reason"], "nonzero_speed_change_not_supported")

    def test_incomplete_recording_suppresses_all_estimates(self):
        for changes in (dict(complete=False), dict(closed=False), dict(dropped_records=1), dict(rejected_records=1)):
            with self.subTest(changes=changes):
                metadata = dict(COMPLETE, **changes)
                rows, _, problems = analysis.analyze(recording(), metadata)
                self.assertTrue(problems)
                self.assertTrue(all(row["reason"] == "incomplete_or_invalid_recording" for row in rows))

    def test_gap_batched_timestamp_and_bad_frame_do_not_confirm(self):
        events = recording()
        target = next(event for event in events if event["t_ns"] == stamp(1.0))
        target["row"] = None
        self.assertEqual(pitch_start(events)["reason"], "bad_ugwrm_frame")
        events = recording()
        target = next(event for event in events if event["t_ns"] == stamp(1.0))
        target["t_ns"] = stamp(.8)
        self.assertEqual(pitch_start(events)["reason"], "batched_or_nonincreasing_rx_time")
        self.assertEqual(pitch_start(recording(), options=analysis.Options(max_gap_s=.15))["reason"], "telemetry_gap")

    def test_calibration_changes_or_moving_baseline_are_rejected(self):
        events = recording()
        next(event for event in events if event["t_ns"] == stamp(1.0))["row"]["pulse_home"] = 500
        self.assertEqual(pitch_start(events)["reason"], "calibration_or_reference_changed")
        events = recording()
        next(event for event in events if event["t_ns"] == stamp(.6))["row"]["pulse"] = 10
        self.assertEqual(pitch_start(events)["reason"], "precommand_already_moving")

    def test_threshold_noise_cannot_be_called_motion(self):
        events = recording()
        for event in events:
            if event["kind"] == "rx":
                event["row"]["pulse"] = min(event["row"]["pulse"], 1)
        self.assertEqual(pitch_start(events)["reason"], "next_command_before_confirmation")
        rows, _, _ = analysis.analyze(events, COMPLETE)
        self.assertEqual(rows[-1]["reason"], "no_precommand_motion_evidence")

    def test_stop_with_plateau_already_before_command_is_ambiguous(self):
        events = ordered([tx(0), rx(.2), rx(.4), rx(.6), tx(.7, 100),
            rx(.8, 0), rx(1.0, 10), rx(1.2, 10), tx(1.3),
            rx(1.4, 10), rx(1.6, 10), rx(1.8, 10)])
        rows, _, _ = analysis.analyze(events, COMPLETE)
        stop = next(row for row in rows if row["axis"] == "pitch" and row["change"] == "stop")
        self.assertEqual(stop["quality"], "unconfirmed")
        self.assertEqual(stop["reason"], "precommand_stop_ambiguous")
        self.assertEqual(stop["observed_upper_s"], "")

    def test_mode_start_requires_new_baseline(self):
        event = dict(kind="mode", phase="ACTIVE", run_id="run1", session_id="session", t_ns=stamp(.65))
        events = ordered(recording() + [event, tx(.66)])
        self.assertEqual(pitch_start(events)["reason"], "insufficient_precommand_baseline")

    def test_feedback_received_during_send_is_retained(self):
        events = recording()
        next(event for event in events if event["t_ns"] == stamp(.7))["tx_end_ns"] = stamp(1.1)
        result = pitch_start(events)
        self.assertEqual(result["quality"], "observed")
        self.assertEqual(result["first_observed_ns"], stamp(1.0))
        self.assertEqual(result["observed_lower_s"], 0)
        self.assertAlmostEqual(result["observed_upper_s"], .3)

    def test_pc_clock_regression_invalidates_recording(self):
        events = recording()
        next(event for event in events if event["t_ns"] == stamp(1.0))["t_ns"] = 1
        self.assertEqual(pitch_start(events)["reason"], "incomplete_or_invalid_recording")

    def test_batch_duplicate_cannot_supply_last_confirmation_sample(self):
        events = recording()
        events.append(rx(1.4, 40))
        self.assertEqual(pitch_start(ordered(events))["reason"], "batched_or_nonincreasing_rx_time")

    def test_manifest_count_and_session_id_detect_complete_line_truncation(self):
        events = recording()
        metadata = dict(COMPLETE, session_id="session", written_records=len(events), accepted_records=len(events))
        self.assertEqual(pitch_start(events, metadata)["quality"], "observed")
        self.assertEqual(pitch_start(events[:-1], metadata)["reason"], "incomplete_or_invalid_recording")
        for changes in (dict(session_id="another_session"), dict(accepted_records=len(events) + 1)):
            with self.subTest(changes=changes):
                self.assertEqual(pitch_start(events, dict(metadata, **changes))["reason"], "incomplete_or_invalid_recording")

    def test_cli_creates_csv_text_and_valid_standalone_svg(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / "events.jsonl").write_text("\n".join(json.dumps(event) for event in recording(True)), encoding="utf-8")
            (directory / "session.json").write_text(json.dumps(COMPLETE), encoding="utf-8")
            self.assertEqual(analysis.main(["--input-dir", str(directory)]), 0)
            output = directory / "response_analysis"
            with (output / "response_summary.csv").open(encoding="utf-8-sig", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 4)
            ET.parse(output / "response_timeline.svg")
            self.assertIn("不能证明真实机械运动", (output / "summary.txt").read_text(encoding="utf-8-sig"))
            with (directory / "events.jsonl").open("a", encoding="utf-8") as handle:
                handle.write("\n{broken")
            events, metadata, problems = analysis.load_recording(directory)
            self.assertTrue(problems)
            rows, _, _ = analysis.analyze(events, metadata, input_problems=problems)
            self.assertTrue(all(row["quality"] == "unconfirmed" for row in rows))


if __name__ == "__main__":
    unittest.main()
