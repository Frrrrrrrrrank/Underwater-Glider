"""实验记录真实文件、并发、背压与磁盘异常回归，不连接设备。"""

import csv
from datetime import datetime
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest


STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

from experiment_recorder import ExperimentRecorder  # noqa: E402


class BlockingRecorder(ExperimentRecorder):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.entered = threading.Event()
        self.release = threading.Event()

    def _write_event(self, event):
        self.entered.set()
        if not self.release.wait(3):
            raise RuntimeError("测试写盘等待超时")
        super()._write_event(event)


class FailingRecorder(ExperimentRecorder):
    def _write_event(self, event):
        raise OSError("模拟磁盘已满")


class ExperimentRecorderTests(unittest.TestCase):
    def setUp(self):
        buildtmp = STATION_DIR.parents[1] / ".buildtmp"
        buildtmp.mkdir(exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(prefix="recorder_", dir=buildtmp)
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def start_recorder(self, recorder_type=ExperimentRecorder, **kwargs):
        recorder = recorder_type(self.root, **kwargs)
        directory = Path(recorder.start(peer="127.0.0.1:12345"))
        self.addCleanup(recorder.stop)
        if isinstance(recorder, BlockingRecorder):
            self.addCleanup(recorder.release.set)
        return recorder, directory

    @staticmethod
    def read_events(directory):
        return [json.loads(line) for line in
                (directory / "events.jsonl").read_text(encoding="utf-8").splitlines()]

    @staticmethod
    def read_csv(path):
        with path.open(encoding="utf-8-sig", newline="") as handle:
            return list(csv.DictReader(handle))

    @staticmethod
    def read_session(directory):
        return json.loads((directory / "session.json").read_text(encoding="utf-8"))

    def test_real_files_keep_heartbeats_failed_send_exit_and_unknown_receive(self):
        recorder, directory = self.start_recorder(metadata={"deadzone": 0.1})
        tx = {
            "kind": "tx", "t_ns": 101, "host_iso": "2026-10-23T13:00:00+08:00",
            "tx_start_ns": 101, "tx_end_ns": 105, "seq": 65535, "msg_type": 7,
            "payload_hex": "012345", "fields": {"pitchmotor_enable": 1,
            "pitchmotor_speed_rpm": -100, "eth_control_flag": 1},
            "success": True, "error": "", "source": "auto_sti", "phase": "ACTIVE",
            "run_id": "run-1", "input": {"sample_time": 0.00000009},
        }
        self.assertTrue(recorder.record(tx))
        self.assertTrue(recorder.record(dict(tx, seq=0)))
        self.assertTrue(recorder.record(dict(tx, seq=1, success=False, error="部分发送未知")))
        self.assertTrue(recorder.record({"kind": "mode", "phase": "EXIT", "reason": "X"}))
        self.assertTrue(recorder.record({"kind": "rx", "t_ns": 123, "topic": "ugwrm",
                                        "row": {"schema": 2, "seq": 4, "pulse": 88},
                                        "batch_id": 1, "line_index": 0, "raw": "$UGWRM,..."}))
        self.assertTrue(recorder.record({"kind": "rx", "topic": "ugwrm",
                                        "row": {"schema": 3, "seq": 5, "roll_pulse": 11},
                                        "raw": "$UGWRM,3,..."}))
        self.assertTrue(recorder.record({"kind": "rx", "topic": "unknown",
                                        "row": None, "raw": "$NEW,1,2*FF"}))
        recorder.stop()
        events = self.read_events(directory)
        self.assertEqual(len(events), 7)
        self.assertEqual([item["event_id"] for item in events], list(range(1, 8)))
        self.assertEqual(events[0]["t_ns"], 101)
        self.assertEqual(events[0]["host_iso"], tx["host_iso"])
        self.assertEqual(events[1]["seq"], 0)
        self.assertFalse(events[2]["success"])
        self.assertIsNotNone(datetime.fromisoformat(events[3]["host_iso"]).utcoffset())
        commands = self.read_csv(directory / "commands.csv")
        self.assertEqual(len(commands), 3)
        self.assertEqual(commands[0]["pitchmotor_speed_rpm"], "-100")
        self.assertEqual(commands[0]["tx_end_ns"], "105")
        self.assertEqual(commands[2]["error"], "部分发送未知")
        telemetry = self.read_csv(directory / "telemetry_ugwrm.csv")
        self.assertEqual(json.loads(telemetry[1]["row_json"])["roll_pulse"], 11)
        self.assertEqual(telemetry[0]["pulse"], "88")
        self.assertEqual(telemetry[0]["roll_pulse"], "")
        self.assertEqual(telemetry[1]["roll_pulse"], "11")
        self.assertEqual(self.read_csv(directory / "telemetry_unknown.csv")[0]["raw"], "$NEW,1,2*FF")
        summary = self.read_session(directory)
        self.assertTrue(summary["complete"])
        self.assertEqual(summary["status"], "stopped")
        self.assertEqual(summary["accepted_records"], summary["written_records"])
        self.assertEqual(summary["metadata"]["deadzone"], 0.1)
        self.assertEqual(summary["peer"], "127.0.0.1:12345")

    def test_nonfinite_and_caller_mutation_do_not_corrupt_raw_evidence(self):
        recorder, directory = self.start_recorder(BlockingRecorder)
        event = {"kind": "rx", "topic": "ugrds", "row": {"x": float("nan"),
                 "nested": [float("inf"), 12]}, "raw": "$UGRDS,nan,inf*00"}
        self.assertTrue(recorder.record(event))
        self.assertTrue(recorder.entered.wait(1))
        event["row"]["nested"][1] = 999
        recorder.release.set()
        recorder.stop()
        saved = self.read_events(directory)[0]
        self.assertIsNone(saved["row"]["x"])
        self.assertEqual(saved["row"]["nested"], [None, 12])
        self.assertEqual(saved["raw"], "$UGRDS,nan,inf*00")
        self.assertEqual(saved["nonfinite_fields"], [".row.x", ".row.nested[0]"])
        self.assertNotIn("NaN", (directory / "events.jsonl").read_text(encoding="utf-8"))
        self.assertTrue(self.read_session(directory)["complete"])

    def test_queue_overflow_is_nonblocking_and_permanently_marks_incomplete(self):
        recorder, directory = self.start_recorder(BlockingRecorder, queue_size=1)
        self.assertTrue(recorder.record({"kind": "mode", "phase": "ACTIVE"}))
        self.assertTrue(recorder.entered.wait(1))
        self.assertTrue(recorder.record({"kind": "tx", "seq": 2}))
        started = time.monotonic()
        self.assertFalse(recorder.record({"kind": "tx", "seq": 3}))
        self.assertLess(time.monotonic() - started, 0.2)
        state = recorder.snapshot()
        self.assertEqual(state["status"], "error")
        self.assertTrue(state["accepting"])
        self.assertEqual(state["dropped_records"], 1)
        recorder.release.set()
        recorder.stop()
        summary = self.read_session(directory)
        self.assertFalse(summary["complete"])
        self.assertEqual(summary["written_records"], 2)
        self.assertEqual(summary["dropped_records"], 1)

    def test_disk_failure_does_not_raise_in_control_path(self):
        recorder, directory = self.start_recorder(FailingRecorder)
        self.assertTrue(recorder.record({"kind": "tx", "seq": 1}))
        self.assertTrue(recorder._done.wait(1))
        self.assertFalse(recorder.record({"kind": "tx", "seq": 2}))
        recorder.stop()
        state = recorder.snapshot()
        self.assertEqual(state["status"], "error")
        self.assertEqual(state["dropped_records"], 1)
        self.assertIn("模拟磁盘已满", state["last_error"])
        self.assertFalse(self.read_session(directory)["complete"])

    def test_startup_failure_is_reported_before_control_session(self):
        invalid_root = self.root / "already_a_file"
        invalid_root.write_text("occupied", encoding="utf-8")
        recorder = ExperimentRecorder(invalid_root)
        with self.assertRaises(OSError):
            recorder.start()
        recorder.stop()
        self.assertEqual(recorder.snapshot()["status"], "error")
        self.assertFalse(recorder.record({"kind": "tx"}))

    def test_stop_is_bounded_and_timeout_remains_in_final_manifest(self):
        recorder, directory = self.start_recorder(BlockingRecorder)
        recorder.record({"kind": "tx", "seq": 1})
        self.assertTrue(recorder.entered.wait(1))
        started = time.monotonic()
        recorder.stop(timeout_s=0.01)
        self.assertLess(time.monotonic() - started, 0.2)
        self.assertFalse(recorder.record({"kind": "tx", "seq": 2}))
        self.assertTrue(recorder.snapshot()["stop_timed_out"])
        recorder.release.set()
        recorder.stop()
        summary = self.read_session(directory)
        self.assertTrue(summary["closed"])
        self.assertTrue(summary["stop_timed_out"])
        self.assertFalse(summary["complete"])
        self.assertEqual(summary["written_records"], 1)

    def test_external_incomplete_mark_preserves_queued_records_and_final_error(self):
        recorder, directory = self.start_recorder(BlockingRecorder)
        recorder.record({"kind": "tx", "seq": 1})
        self.assertTrue(recorder.entered.wait(1))
        recorder.mark_incomplete("接收线程未退出，尾部数据可能遗漏")
        self.assertTrue(recorder.record({"kind": "mode", "phase": "EXIT"}))
        self.assertTrue(recorder.snapshot()["accepting"])
        recorder.release.set()
        recorder.stop()
        summary = self.read_session(directory)
        self.assertEqual(summary["status"], "error")
        self.assertFalse(summary["complete"])
        self.assertEqual(summary["written_records"], 2)
        self.assertIn("尾部数据", summary["last_error"])

    def test_multi_producer_event_order_preserves_distinct_capture_times(self):
        recorder, directory = self.start_recorder()
        def produce(offset):
            for index in range(30):
                recorder.record({"kind": "mode", "t_ns": offset - index})
        threads = [threading.Thread(target=produce, args=(1000 * i,)) for i in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        recorder.stop()
        events = self.read_events(directory)
        self.assertEqual([item["event_id"] for item in events], list(range(1, 121)))
        self.assertEqual(sorted(item["t_ns"] for item in events),
                         sorted(1000 * i - j for i in range(4) for j in range(30)))
        self.assertTrue(self.read_session(directory)["complete"])

    def test_unknown_topic_cannot_escape_session_directory(self):
        recorder, directory = self.start_recorder()
        for topic in ("../../outside", "UGEKF", "ugekf"):
            self.assertTrue(recorder.record({"kind": "rx", "topic": topic, "row": {}, "raw": "raw"}))
        recorder.stop()
        self.assertEqual(len(list(directory.glob("telemetry_*.csv"))), 3)
        self.assertFalse((self.root / "outside.csv").exists())
        self.assertTrue(self.read_session(directory)["complete"])

    def test_invalid_event_never_raises_and_cannot_claim_complete_capture(self):
        recorder, directory = self.start_recorder()
        self.assertFalse(recorder.record({"kind": "mode", "bad": object()}))
        self.assertTrue(recorder.record({"kind": "mode", "phase": "EXIT"}))
        recorder.stop()
        self.assertFalse(recorder.record({"kind": "tx"}))
        self.assertEqual(recorder.snapshot()["rejected_records"], 1)
        self.assertFalse(self.read_session(directory)["complete"])

    def test_all_file_operations_use_worker_thread(self):
        calls = []
        class ThreadCheckingRecorder(ExperimentRecorder):
            def _open_outputs(self):
                calls.append(("open", threading.current_thread().name))
                super()._open_outputs()
            def _write_event(self, event):
                calls.append(("write", threading.current_thread().name))
                super()._write_event(event)
            def _flush(self):
                calls.append(("flush", threading.current_thread().name))
                super()._flush()
            def _write_session(self):
                calls.append(("session", threading.current_thread().name))
                super()._write_session()
        recorder, directory = self.start_recorder(ThreadCheckingRecorder)
        self.assertTrue(recorder.record({"kind": "rx", "topic": "ugwrm", "row": {}}))
        recorder.stop()
        self.assertEqual({operation for operation, _ in calls}, {"open", "write", "flush", "session"})
        self.assertEqual({name for _, name in calls}, {"experiment-recorder"})
        self.assertTrue(self.read_session(directory)["complete"])

    def test_independent_rsdd_log_retains_good_bad_and_truncated_raw_data(self):
        rsdd_root = self.root / "logs" / "rsdd"
        recorder, directory = self.start_recorder(rdss_root=rsdd_root)
        raw_frames = ("$UGRDS,1,1,42*56\r\n", "$UGRDS,broken*ZZ", "$UGRDS,1,2")
        for index, raw in enumerate(raw_frames):
            self.assertTrue(recorder.record({
                "kind": "rx", "topic": "ugrds", "t_ns": 500 + index,
                "host_iso": "2026-10-23T13:00:00+08:00", "batch_id": 1,
                "line_index": index, "raw": raw, "complete": index != 2,
                "row": {"schema": 1, "seq": 1} if index == 0 else None,
            }))
        recorder.record({"kind": "rx", "topic": "ugwrm", "raw": "$UGWRM,other", "row": {}})
        recorder.stop()
        log_path = Path(recorder.snapshot()["rdss_log_path"])
        self.assertEqual(log_path.parent, rsdd_root)
        self.assertTrue(log_path.name.startswith("rsdd_"))
        self.assertEqual(log_path.suffix, ".log")
        lines = log_path.read_text(encoding="utf-8").splitlines()
        saved = [json.loads(line) for line in lines]
        self.assertEqual([event["raw"] for event in saved], list(raw_frames))
        self.assertEqual([event["t_ns"] for event in saved], [500, 501, 502])
        self.assertEqual([event["parsed"] for event in saved], [True, False, False])
        self.assertEqual([event["complete"] for event in saved], [True, True, False])
        self.assertEqual(len(self.read_events(directory)), 4)
        self.assertEqual(len(self.read_csv(directory / "telemetry_ugrds.csv")), 3)
        self.assertEqual(self.read_session(directory)["rdss_log_path"], str(log_path))


if __name__ == "__main__":
    unittest.main()
