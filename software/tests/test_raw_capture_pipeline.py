"""生产UGRAW发送函数→两种地面站异步记录→离线统计的跨层回归。"""

import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from test_raw_capture import build_harness, TMP_ROOT

STATION = Path(__file__).resolve().parents[2] / "station"
for directory in (STATION, STATION / "tools"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from raw_telemetry import ugraw_parse
from experiment_recorder import ExperimentRecorder
from groundstation.legacy_telemetry import decode_telemetry_line
from groundstation.session_recorder import SessionRecorder
import analyze_gnss_capture as analysis


class RawCapturePipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        TMP_ROOT.mkdir(exist_ok=True)
        cls.temp = tempfile.TemporaryDirectory(prefix="raw_pipeline_", dir=TMP_ROOT)
        cls.addClassCleanup(cls.temp.cleanup)
        cls.work = Path(cls.temp.name)
        binary, cls.env = build_harness(cls.work / "firmware")
        result = subprocess.run([str(binary), "timeline"], capture_output=True,
                                env=cls.env, check=True, timeout=30)
        cls.frames = result.stdout.decode("ascii").splitlines(keepends=True)
        if len(cls.frames) != 4:
            raise AssertionError("生产发送函数必须给出四条完整外包")

    def verify_analysis(self, directory, output_name):
        records, metadata = analysis.load_capture(directory)
        summary, rows = analysis.analyze_records(records, metadata, gap_s=3)
        self.assertEqual(len(records), 4)
        self.assertEqual([bytes.fromhex(row["payload_hex"]) for row in rows],
                         [bytes.fromhex(ugraw_parse(frame)["payload_hex"]) for frame in self.frames])
        self.assertEqual([row["epoch_disposition"] for row in rows],
                         ["new", "duplicate", "new", "new"])
        counts = summary["counts"]
        self.assertEqual((counts["rmc_arrivals"], counts["new_epochs"],
                          counts["new_position"], counts["new_no_fix"], counts["duplicate"]),
                         (4, 3, 2, 1, 1))
        self.assertAlmostEqual(summary["timing"]["rmc_rx"]["observed_rate_hz"], .6)
        self.assertAlmostEqual(summary["timing"]["event_rx"]["observed_rate_hz"], .4)
        self.assertEqual(summary["no_fix_recoveries"][0]["duration_s"], 4)
        self.assertEqual(summary["position_gaps"][0]["gap_s"], 5)
        self.assertEqual(summary["queue_delay_ms"]["median"], 10)
        self.assertEqual(summary["capture_integrity"], "no_observed_issues_recorder_reports_complete")
        # 真正执行命令行入口，检查磁盘结果与内存结果一致，且没有重复读CSV/events。
        output = self.work / output_name
        result = subprocess.run([sys.executable, "-B", str(STATION / "tools" / "analyze_gnss_capture.py"),
                                 str(directory), "--output-dir", str(output), "--gap-seconds", "3"],
                                capture_output=True, env=self.env, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr.decode("utf-8", errors="replace"))
        saved = json.loads((output / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(saved["counts"], counts)
        with (output / "epochs.csv").open(encoding="utf-8-sig", newline="") as handle:
            self.assertEqual(len(list(csv.DictReader(handle))), 4)

    def test_production_sender_through_cli_recorder_and_analyzer(self):
        recorder = ExperimentRecorder(self.work / "cli")
        directory = recorder.start("synthetic-firmware")
        try:
            for frame in self.frames:
                row = ugraw_parse(frame)
                self.assertIsNotNone(row)
                self.assertTrue(recorder.record({"kind": "rx", "topic": "ugraw", "row": row,
                    "raw": frame, "complete": True, "t_ns": row["rx_us"] * 1000,
                    "host_iso": "2026-11-06T12:00:00+08:00"}))
        finally:
            recorder.stop()
        self.assertTrue(recorder.snapshot()["complete"])
        self.verify_analysis(directory, "cli_analysis")

    def test_production_sender_through_web_recorder_and_analyzer(self):
        recorder = SessionRecorder(self.work / "web")
        recorder.begin_session("synthetic", "synthetic-firmware")
        try:
            for frame in self.frames:
                decoded = decode_telemetry_line(frame)
                self.assertIsNotNone(decoded)
                recorder.record(frame, decoded, received_ns=decoded.data["rx_us"] * 1000,
                                received_at="2026-11-06T12:00:00+08:00")
            recorder.end_session("test_finished")
        finally:
            recorder.stop()
        self.verify_analysis(recorder.snapshot()["directory"], "web_analysis")


if __name__ == "__main__":
    unittest.main()
