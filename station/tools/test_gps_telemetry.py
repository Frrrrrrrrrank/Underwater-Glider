"""UGPS 协议、共享质量、会话去重和实际 CSV 落盘回归；不连接真实设备。"""

import csv
from pathlib import Path
import sys
import tempfile
import unittest

STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

import gps_telemetry as gps
import contcp_t
import monitor
from monitor_bridge import MonitorBridge
from groundstation.legacy_telemetry import decode_telemetry_line
from groundstation.session_recorder import SessionRecorder
from groundstation.state_store import StateStore
from groundstation.ethernet_link import EthernetRealtimeLink


GOLD_BODY = (
    "UGPS,1,10,1000,2,0,247,7,120,50,20261105,36000123,"
    "220000001,1145000001,1,220000000,1145000000,"
    "1000,-2000,1100,-1900,100,100,1250,250000,40000,-5000,90000,3,1,0,2,0,4"
)


def checked(body):
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return "$" + body + "*{:02X}".format(checksum)


def frame(**changes):
    row = dict(zip(gps.UGPS_FIELDS, map(int, GOLD_BODY.split(",")[1:])))
    row.update(changes)
    return checked("UGPS," + ",".join(str(row[name]) for name in gps.UGPS_FIELDS))


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class GpsProtocolTests(unittest.TestCase):
    def test_golden_schema_and_shared_cli_web_preserve_units(self):
        expected = gps.ugps_parse(checked(GOLD_BODY))
        self.assertEqual(len(expected), 33)
        self.assertEqual(expected["utc_date"], 20261105)
        self.assertEqual(expected["lat_e7"], 220000001)
        self.assertEqual(expected["origin_lat_e7"], 220000000)
        self.assertEqual(expected["meas_e_mm"], -2000)
        self.assertEqual(expected["pne_cm2"], -5000)
        self.assertEqual(expected["input_reject_count"], 4)
        for suffix in ("", "\n", "\r\n"):
            wire = checked(GOLD_BODY) + suffix
            self.assertEqual(contcp_t.ugps_parse(wire), expected)
            decoded = decode_telemetry_line(wire)
            self.assertEqual(decoded.topic, "ugps")
            self.assertEqual(decoded.data, expected)
            self.assertEqual(decoded.device_tick_ms, 1000)
            self.assertEqual(decoded.stale_after_ms, 3000)
        self.assertEqual(gps.ugps_csv_row(expected)[1:],
                         [expected[field] for field in gps.UGPS_FIELDS])

    def test_corruption_is_rejected_equally_by_cli_and_web(self):
        valid = frame()
        body = valid[1:valid.index("*")]
        bad_frames = (
            " " + valid, valid + " ", valid + "\r", valid + "\r\r\n",
            valid + "\n\n", valid + "tail", valid[:-2] + "ZZ",
            valid[:-2] + ("FF" if valid[-2:] != "FF" else "00"),
            checked(body + ",0"), checked(body.rsplit(",", 1)[0]),
            checked(body.replace("UGPS,1", "UGPS,１")),
            checked(body.replace("UGPS,1", "UGPS,+1")),
            checked(body.replace("UGPS,1", "UGPS,1.0")),
            frame(schema=2), frame(state=6), frame(reason=12), frame(flags=256),
            frame(seq=-1), frame(rx_age_ms=1 << 32), frame(nis_milli=-1),
            frame(pnn_cm2=-1), frame(pne_cm2=1 << 31),
            frame(lat_e7=900000001), frame(origin_lon_e7=-1800000001),
        )
        for wire in bad_frames:
            with self.subTest(wire=wire):
                self.assertIsNone(gps.ugps_parse(wire))
                self.assertIsNone(decode_telemetry_line(wire))
        for wire in (None, b"ASCII", "x" * 513):
            self.assertIsNone(gps.ugps_parse(wire))

    def test_invalid_placeholders_and_unknown_ages_remain_diagnostics(self):
        row = gps.ugps_parse(frame(flags=0, rx_age_ms=0xFFFFFFFF,
                                   fuse_age_ms=0xFFFFFFFF, lat_e7=2147483647))
        self.assertIsNotNone(row)
        invalid = gps.ugps_invalid_fields(row)
        for field in ("lat_e7", "origin_id", "meas_n_mm", "est_n_mm",
                      "innov_n_mm", "nis_milli", "pnn_cm2", "fuse_age_ms", "rx_age_ms"):
            self.assertIn(field, invalid)
        self.assertEqual(row["lat_e7"], 2147483647)
        self.assertEqual(row["fuse_age_ms"], 0xFFFFFFFF)


class GpsMonitorTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.bridge = MonitorBridge(port=0, clock=self.clock)
        self.bridge.set_link("connected", peer="offline")
        self.addCleanup(self.bridge.stop)

    def test_optional_topic_duplicate_timeout_and_session_boundary(self):
        initial = self.bridge.snapshot()
        self.assertNotIn("ugps", initial["state"]["required_topics"])
        self.assertIn("旧固件可能不支持", monitor.render_panel(initial))
        row = gps.ugps_parse(frame())
        self.assertTrue(self.bridge.publish("ugps", row))
        self.clock.now += 2.5
        self.assertFalse(self.bridge.publish("ugps", row))
        self.clock.now += 0.6
        self.assertFalse(self.bridge.snapshot()["telemetry"]["ugps"]["fresh"])
        self.bridge.publish("ugps", dict(row, seq=11, rx_age_ms=7000, state=3))
        panel = monitor.render_panel(self.bridge.snapshot(), width=160)
        self.assertIn("失锁推算", panel)
        self.assertIn("接收龄：7000ms", panel)
        self.bridge.set_link("disconnected")
        self.assertNotIn("+22.0000001°", monitor.render_panel(self.bridge.snapshot(), width=160))
        self.bridge.set_link("connected")
        self.assertNotIn("ugps", self.bridge.snapshot()["telemetry"])

    def test_panel_separates_measurements_fusion_and_unverified_reference(self):
        self.bridge.publish("ugps", gps.ugps_parse(frame()))
        panel = monitor.render_panel(self.bridge.snapshot(), width=160)
        for text in ("自身定位（纬,经）：+22.0000001°", "接收龄：120ms", "融合龄：50ms",
                     "NE测量(m)：+1.000, -2.000", "NE估计(m)：+1.100, -1.900",
                     "NIS：1.250", "R配置(m²)：25.0000", "4.0000/-0.5000/9.0000",
                     "安装杆臂/真北未实测确认", "非导航就绪", "不是接收机报告精度"):
            self.assertIn(text, panel)
        self.bridge.publish("ugps", gps.ugps_parse(frame(seq=11, flags=0,
                            rx_age_ms=0xFFFFFFFF, fuse_age_ms=0xFFFFFFFF)))
        panel = monitor.render_panel(self.bridge.snapshot(), width=160)
        for text in ("无有效测量", "尚未锚定", "无有效创新", "协方差无效", "尚未融合", "年龄未知"):
            self.assertIn(text, panel)
        self.assertNotIn("+22.0000001°", panel)
        self.assertNotIn("NE估计(m)：+1.100", panel)

    def test_web_store_uses_shared_quality_and_duplicate_does_not_refresh(self):
        store = StateStore(clock=self.clock)
        store.set_link("connected", session_id="gps-session")
        row = gps.ugps_parse(frame(flags=0))
        args = dict(session_id="gps-session", stale_after_ms=3000)
        generation = store.publish_telemetry("ugps", row, **args)
        self.clock.now += 3.1
        self.assertEqual(store.publish_telemetry("ugps", row, **args), generation)
        sample = store.read()[1]["telemetry"]["ugps"]
        self.assertFalse(sample["fresh"])
        self.assertIn("est_n_mm", sample["invalid_fields"])
        store.clear_session_telemetry()
        store.publish_telemetry("ugps", row, session_id="new", stale_after_ms=3000)
        self.assertTrue(store.read()[1]["telemetry"]["ugps"]["fresh"])


class GpsWebRecordingTests(unittest.TestCase):
    def test_web_writes_dedicated_csv_and_keeps_damaged_raw_line(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = SessionRecorder(Path(temporary))
            recorder.begin_session("gps-session", "offline")
            valid = frame()
            recorder.record(valid, decode_telemetry_line(valid))
            recorder.record(valid + "broken", None)
            recorder.end_session("peer_closed")
            recorder.stop()
            state = recorder.snapshot()
            self.assertEqual(state["rows"], {"ugps": 1})
            with Path(state["files"]["ugps"]).open(encoding="utf-8", newline="") as handle:
                rows = list(csv.reader(handle))
            self.assertEqual(rows[0], gps.UGPS_CSV_HEADER)
            self.assertEqual(rows[1][rows[0].index("pne_cm2")], "-5000")
            self.assertIn("broken", Path(state["files"]["raw"]).read_text(encoding="utf-8"))

    def test_web_receiver_does_not_drop_bad_nonascii_bytes_into_valid_gps(self):
        class InputOnlySocket:
            def __init__(self, wire):
                self.chunks = iter((wire, b""))

            def recv(self, size):
                return next(self.chunks)

        store = StateStore()
        store.set_link("connected", session_id="offline")
        link = EthernetRealtimeLink(store, host="127.0.0.1", port=0)
        good = (frame(seq=12) + "\r\n").encode("ascii")
        corrupt = (frame(seq=99) + "\r\n").encode("ascii").replace(b"UGPS,", b"UGPS,\xff")
        self.assertEqual(link._receive_lines(InputOnlySocket(corrupt + good), "offline"), "peer_closed")
        self.assertEqual(store.read()[1]["telemetry"]["ugps"]["data"]["seq"], 12)


if __name__ == "__main__":
    unittest.main()
