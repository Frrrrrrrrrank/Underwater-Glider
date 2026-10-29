"""北斗原始遥测的协议边界，避免损坏数据变成实验里的有效定位。"""

from pathlib import Path
import sys
import unittest


STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

import rdss_telemetry as rdss


def frame(**changes):
    row = dict.fromkeys(rdss.UGRDS_FIELDS, 0)
    row.update(schema=2, seq=10, t_ms=1000, signal_good=1,
               has_cached_fix=1, local_latitude_deg_e7=220000001,
               local_longitude_deg_e7=1145000001)
    row.update(changes)
    body = "UGRDS," + ",".join(str(row[name]) for name in rdss.UGRDS_FIELDS)
    return checked_body(body)


def checked_body(body):
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return "$" + body + "*{:02X}".format(checksum)


class RdssTelemetryTests(unittest.TestCase):
    def test_schema_two_matches_board_fields_and_preserves_integer_coordinates(self):
        wire = frame(target_valid=1, target_latitude_deg_e7=-900000000,
                     target_longitude_deg_e7=-1800000000, sent_count=14,
                     send_ok_count=11, last_result=3)
        row = rdss.ugrds_parse(wire + "\r\n")
        self.assertEqual(len(row), 26)
        self.assertEqual(row["local_latitude_deg_e7"], 220000001)
        self.assertEqual(row["target_longitude_deg_e7"], -1800000000)
        self.assertEqual(row["sent_count"], 14)
        self.assertEqual(row["send_ok_count"], 11)
        self.assertEqual(rdss.ugrds_parse(wire + "\n"), row)
        csv_row = rdss.ugrds_csv_row(row)
        self.assertEqual(len(csv_row), len(rdss.UGRDS_CSV_HEADER))
        self.assertEqual(csv_row[1:], [row[name] for name in rdss.UGRDS_FIELDS])

    def test_rejects_bad_checksum_trailing_garbage_nonascii_and_invalid_field_count(self):
        valid = frame()
        body = valid[1:valid.index("*")]
        for wire in (None, b"ASCII", " " + valid, valid + " ", valid + "garbage",
                     valid + "\r", valid + "\n\n", valid[:-2] + "GG",
                     valid[:-2] + ("FF" if valid[-2:] != "FF" else "00"),
                     checked_body(body + ",0"), checked_body(body.rsplit(",", 1)[0]),
                     checked_body(body.replace("UGRDS,2", "UGRDS,２")),
                     checked_body(body.replace("UGRDS,2", "UGRDS,+2")),
                     checked_body(body.replace("UGRDS,2", "UGRDS,2.0")),
                     checked_body(body.replace("UGRDS,2", "UGRDS, 2"))):
            with self.subTest(wire=repr(wire)):
                self.assertIsNone(rdss.ugrds_parse(wire))

    def test_rejects_invalid_schema_flags_enums_and_integer_overflow(self):
        cases = [{"schema": 1}, {"schema": 3}, {"seq": -1}, {"t_ms": 1 << 32},
                 {"last_result": 7}, {"tx_mode": 3}, {"last_hal_status": 4},
                 {"uart_fail_count": 1 << 32}, {"target_latitude_deg_e7": -(1 << 31) - 1}]
        cases.extend({field: 2} for field in (
            "signal_good", "ready_to_send", "pending", "waiting_ack", "card_id_known",
            "card_mismatch", "has_cached_fix", "target_valid"))
        for changes in cases:
            with self.subTest(changes=changes):
                self.assertIsNone(rdss.ugrds_parse(frame(**changes)))

    def test_valid_coordinate_ranges_and_invalid_raw_values_are_separate(self):
        for changes in ({"local_latitude_deg_e7": 900000001},
                        {"local_longitude_deg_e7": -1800000001},
                        {"target_valid": 1, "target_latitude_deg_e7": 900000001},
                        {"target_valid": 1, "target_longitude_deg_e7": 1800000001}):
            with self.subTest(changes=changes):
                self.assertIsNone(rdss.ugrds_parse(frame(**changes)))
        row = rdss.ugrds_parse(frame(has_cached_fix=0, target_valid=0,
                                   local_latitude_deg_e7=2147483647,
                                   target_longitude_deg_e7=-2147483648))
        self.assertEqual(row["local_latitude_deg_e7"], 2147483647)
        self.assertEqual(row["target_longitude_deg_e7"], -2147483648)


if __name__ == "__main__":
    unittest.main()
