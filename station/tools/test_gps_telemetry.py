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
GOLD_BODY_V2 = (
    "UGPS,2,10,1000,2,0,8191,7,120,50,20261105,36000123,"
    "220000001,1145000001,1,220000000,1145000000,"
    "1000,-2000,1100,-1900,100,100,1250,250000,40000,-5000,90000,3,1,0,2,0,4,"
    "0,1,50,3,1250,-500,1200,-450,500,40000,2500,-100,3600,2,1,2,"
    "100000,-200000,10000,-20000,1000,-2000,1250,-500,2000,255"
)


def checked(body):
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return "$" + body + "*{:02X}".format(checksum)


def frame(**changes):
    row = dict(zip(gps.UGPS_SCHEMA1_FIELDS, map(int, GOLD_BODY.split(",")[1:])))
    schema = changes.get("schema", 1)
    fields = gps.UGPS_FIELDS_BY_SCHEMA.get(schema, gps.UGPS_FIELDS)
    if schema != 1:
        row.update(dict.fromkeys(gps.UGPS_SCHEMA2_EXTRA_FIELDS, 0))
        row.update(schema=schema, flags=8191, velocity_source=1, vel_fuse_age_ms=50,
                   no_fix_count=3, meas_vn_mmps=1250, meas_ve_mmps=-500,
                   est_vn_mmps=1200, est_ve_mmps=-450, vel_nis_milli=500,
                   vel_r_var_cm2ps2=40000, pvn_cm2ps2=2500, pvne_cm2ps2=-100,
                   pve_cm2ps2=3600, vel_accept_count=2, vel_reject_count=1,
                   recovery_reason=2, reset_before_n_mm=100000, reset_before_e_mm=-200000,
                   reset_before_vn_mmps=10000, reset_before_ve_mmps=-20000,
                   reset_after_n_mm=1000, reset_after_e_mm=-2000,
                   reset_after_vn_mmps=1250, reset_after_ve_mmps=-500,
                   reset_age_ms=2000, reset_valid_mask=255)
    row.update(changes)
    return checked("UGPS," + ",".join(str(row[name]) for name in fields))


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
                         [expected.get(field, "") for field in gps.UGPS_FIELDS])

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
            frame(schema=3), frame(state=6), frame(reason=12), frame(flags=256),
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

    def test_schema2_golden_order_and_shared_decoder_preserve_all_extensions(self):
        wire = checked(GOLD_BODY_V2)
        row = gps.ugps_parse(wire)
        self.assertEqual(len(row), 59)
        self.assertEqual(gps.UGPS_FIELDS[:33], gps.UGPS_SCHEMA1_FIELDS)
        self.assertEqual(row["quality_source"], 0)
        self.assertEqual(row["velocity_source"], 1)
        self.assertEqual(row["meas_vn_mmps"], 1250)
        self.assertEqual(row["est_ve_mmps"], -450)
        self.assertEqual(row["vel_r_var_cm2ps2"], 40000)
        self.assertEqual(row["pvne_cm2ps2"], -100)
        self.assertEqual(row["recovery_reason"], 2)
        self.assertEqual(row["reset_before_vn_mmps"], 10000)
        self.assertEqual(row["reset_after_e_mm"], -2000)
        self.assertEqual(row["reset_valid_mask"], 255)
        self.assertEqual(contcp_t.ugps_parse(wire), row)
        self.assertEqual(decode_telemetry_line(wire).data, row)
        self.assertEqual(gps.ugps_invalid_fields(row), {})
        self.assertEqual(gps.ugps_csv_row(row)[1:], [row[name] for name in gps.UGPS_FIELDS])

    def test_schema2_rejects_wrong_length_enum_unsigned_and_overflow(self):
        body = GOLD_BODY_V2
        invalid = (
            checked(body.rsplit(",", 1)[0]), checked(body + ",0"),
            checked(body.replace("UGPS,2,", "UGPS,1,", 1)),
            checked(GOLD_BODY.replace("UGPS,1,", "UGPS,2,", 1)),
            frame(schema=2, flags=8192), frame(schema=2, state=6),
            frame(schema=2, reason=15), frame(schema=2, quality_source=2),
            frame(schema=2, velocity_source=3), frame(schema=2, recovery_reason=6),
            frame(schema=2, reset_valid_mask=256), frame(schema=2, vel_nis_milli=-1),
            frame(schema=2, pvn_cm2ps2=-1), frame(schema=2, vel_r_var_cm2ps2=-1),
            frame(schema=2, pvne_cm2ps2=1 << 31), frame(schema=2, est_vn_mmps=1 << 31),
            frame(schema=2, reset_before_n_mm=-(1 << 31) - 1),
            frame(schema=2, reset_age_ms=1 << 32),
        )
        for wire in invalid:
            with self.subTest(wire=wire):
                self.assertIsNone(gps.ugps_parse(wire))
                self.assertIsNone(decode_telemetry_line(wire))
        self.assertIsNone(gps.ugps_parse("x" * (gps.UGPS_MAX_LINE + 1)))

    def test_schema2_maximum_frame_can_exceed_old_512_byte_limit(self):
        row = dict.fromkeys(gps.UGPS_FIELDS, 0xFFFFFFFF)
        signed = ("lat_e7", "lon_e7", "origin_lat_e7", "origin_lon_e7", "meas_n_mm",
                  "meas_e_mm", "est_n_mm", "est_e_mm", "innov_n_mm", "innov_e_mm",
                  "pne_cm2", "meas_vn_mmps", "meas_ve_mmps", "est_vn_mmps",
                  "est_ve_mmps", "pvne_cm2ps2")
        row.update(dict.fromkeys(signed + tuple(gps.UGPS_RESET_FIELDS), -(1 << 31)))
        row.update(schema=2, state=5, reason=14, flags=0, quality_source=1,
                   velocity_source=2, recovery_reason=5, reset_valid_mask=255)
        wire = checked("UGPS," + ",".join(str(row[name]) for name in gps.UGPS_FIELDS))
        self.assertGreater(len(wire), 512)
        self.assertLess(len(wire), gps.UGPS_MAX_LINE)
        self.assertEqual(gps.ugps_parse(wire), row)
        self.assertEqual(decode_telemetry_line(wire).data, row)

    def test_velocity_and_recovery_quality_are_independent_of_position_validity(self):
        row = gps.ugps_parse(frame(schema=2, flags=247, vel_fuse_age_ms=0xFFFFFFFF,
                                   reset_age_ms=0xFFFFFFFF))
        invalid = gps.ugps_invalid_fields(row)
        self.assertNotIn("est_n_mm", invalid)
        self.assertNotIn("lat_e7", invalid)
        for field in ("est_vn_mmps", "meas_vn_mmps", "pvn_cm2ps2", "vel_nis_milli",
                      "reset_after_n_mm", "reset_age_ms", "vel_fuse_age_ms"):
            self.assertIn(field, invalid)
        # 逐位恢复有效性独立于当前估计；无效历史原值仍可审计。
        row = gps.ugps_parse(frame(schema=2, reset_valid_mask=0b11110101))
        invalid = gps.ugps_invalid_fields(row)
        self.assertEqual(invalid["reset_before_e_mm"], "gps_recovery_value_unavailable")
        self.assertEqual(invalid["reset_before_ve_mmps"], "gps_recovery_value_unavailable")
        self.assertNotIn("reset_before_n_mm", invalid)
        self.assertNotIn("reset_after_vn_mmps", invalid)
        self.assertEqual(row["reset_before_e_mm"], -200000)


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

    def test_schema1_and_schema2_keep_distinct_state_meanings_and_missing_extensions(self):
        self.bridge.publish("ugps", gps.ugps_parse(frame(state=1)))
        panel = monitor.render_panel(self.bridge.snapshot(), width=220)
        self.assertIn("状态：质量观察", panel)
        self.assertIn("速度可信：--（旧协议未上报）", panel)
        # 同序号、时刻的协议升级不是同一份快照。
        self.assertTrue(self.bridge.publish("ugps", gps.ugps_parse(frame(schema=2, state=1,
                            flags=247, velocity_source=0, vel_fuse_age_ms=0xFFFFFFFF))))
        panel = monitor.render_panel(self.bridge.snapshot(), width=220)
        self.assertIn("状态：仅位置锚定", panel)
        self.assertIn("位置锚定：是", panel)
        self.assertIn("速度可信：否", panel)
        self.assertIn("速度来源：未知", panel)
        self.assertIn("速度融合龄：--（年龄未知）", panel)
        self.assertIn("NE速度估计(m/s)：--（速度尚不可信）", panel)
        self.assertNotIn("状态：连续跟踪", panel)

    def test_schema2_panel_keeps_measurement_current_estimate_and_recovery_separate(self):
        self.bridge.publish("ugps", gps.ugps_parse(frame(schema=2, reset_valid_mask=253,
                                                       quality_source=1)))
        panel = monitor.render_panel(self.bridge.snapshot(), width=240)
        for text in ("状态：连续跟踪", "测量有效：是", "位置锚定：是", "速度可信：是",
                     "速度来源：RMC", "NE速度测量(m/s)：+1.250, -0.500",
                     "NE速度估计(m/s)：+1.200, -0.450", "速度融合龄：50ms",
                     "最近恢复：辅助失锁", "恢复龄：2000ms", "无定位计数：3",
                     "恢复前 NE(m)：+100.000, --（该恢复值无效）",
                     "恢复后锚点 NE(m)：+1.000, -2.000", "NE估计(m)：+1.100, -1.900",
                     "速度接受/拒绝：2/1", "质量来源：设备报告", "位置R(m²)：25.0000"):
            self.assertIn(text, panel)
        self.assertNotIn("R配置", panel)
        self.clock.now += 3.1
        stale = monitor.render_panel(self.bridge.snapshot(), width=240)
        self.assertNotIn("+100.000", stale)
        self.assertNotIn("+1.200", stale)

    def test_schema2_web_quality_and_protocol_transition_do_not_lose_new_fields(self):
        store = StateStore(clock=self.clock)
        store.set_link("connected", session_id="gps-session")
        args = dict(session_id="gps-session", stale_after_ms=3000)
        store.publish_telemetry("ugps", gps.ugps_parse(frame()), **args)
        row = gps.ugps_parse(frame(schema=2, flags=247, reset_valid_mask=0))
        generation = store.publish_telemetry("ugps", row, **args)
        sample = store.read()[1]["telemetry"]["ugps"]
        self.assertEqual(sample["data"]["schema"], 2)
        self.assertEqual(sample["data"]["est_vn_mmps"], 1200)
        self.assertIn("est_vn_mmps", sample["invalid_fields"])
        self.assertNotIn("est_n_mm", sample["invalid_fields"])
        self.clock.now += 3.1
        self.assertEqual(store.publish_telemetry("ugps", row, **args), generation)
        self.assertFalse(store.read()[1]["telemetry"]["ugps"]["fresh"])


class GpsWebRecordingTests(unittest.TestCase):
    def test_web_records_mixed_schemas_with_blank_old_extension_columns(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = SessionRecorder(Path(temporary))
            recorder.begin_session("mixed", "offline")
            for wire in (frame(), frame(schema=2, seq=11)):
                recorder.record(wire, decode_telemetry_line(wire))
            recorder.end_session("peer_closed")
            recorder.stop()
            state = recorder.snapshot()
            with Path(state["files"]["ugps"]).open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["schema"], "1")
            self.assertEqual(rows[0]["velocity_source"], "")
            self.assertEqual(rows[0]["reset_after_n_mm"], "")
            self.assertEqual(rows[1]["schema"], "2")
            self.assertEqual(rows[1]["est_ve_mmps"], "-450")
            self.assertEqual(rows[1]["reset_after_n_mm"], "1000")

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
        good = (frame(schema=2, seq=12) + "\r\n").encode("ascii")
        corrupt = (frame(schema=2, seq=99) + "\r\n").encode("ascii").replace(b"UGPS,", b"UGPS,\xff")
        self.assertEqual(link._receive_lines(InputOnlySocket(corrupt + good), "offline"), "peer_closed")
        self.assertEqual(store.read()[1]["telemetry"]["ugps"]["data"]["seq"], 12)


if __name__ == "__main__":
    unittest.main()
