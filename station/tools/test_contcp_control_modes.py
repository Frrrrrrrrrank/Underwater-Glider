import contextlib
import io
import os
import sys
import unittest
from unittest import mock


HERE = os.path.dirname(os.path.abspath(__file__))
STATION_DIR = os.path.dirname(HERE)
if STATION_DIR not in sys.path:
    sys.path.insert(0, STATION_DIR)

import contcp_t
import ugcp_protocol as ugcp


class FakeSocket:
    def __init__(self):
        self.sent = []

    def sendall(self, data):
        self.sent.append(bytes(data))


CONTROL_VALUES = (
    0, 0, 0, 0,
    0, 0, 10, 10,
    0, 0, 0, 0, 1,
)


class ContcpControlModeTests(unittest.TestCase):
    def setUp(self):
        contcp_t.UGCP_TX_SEQUENCE = ugcp.SequenceCounter()
        contcp_t.PID_MODE_ACTIVE.clear()

    def tearDown(self):
        contcp_t.PID_MODE_ACTIVE.clear()

    def test_default_is_stream_and_manual_can_select_latched(self):
        conn = FakeSocket()
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(
                contcp_t.send_control_packet(conn, *CONTROL_VALUES))
            self.assertTrue(
                contcp_t.send_control_packet(
                    conn, *CONTROL_VALUES, latched=True))

        parsed = [ugcp.parse_frame(frame) for frame in conn.sent]
        self.assertEqual(
            [item[0] for item in parsed],
            [ugcp.MSG_CONTROL19_STREAM, ugcp.MSG_CONTROL19_LATCHED],
        )
        self.assertEqual(parsed[0][2], parsed[1][2])

    def test_safe_exit_sends_latched_neutral_then_release(self):
        conn = FakeSocket()
        with mock.patch.object(contcp_t.time, "sleep"), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(contcp_t.send_neutral_and_release(conn))

        parsed = [ugcp.parse_frame(frame) for frame in conn.sent]
        self.assertEqual(
            [item[0] for item in parsed],
            [ugcp.MSG_CONTROL19_LATCHED, ugcp.MSG_CONTROL19_LATCHED],
        )
        self.assertEqual(parsed[0][2][-1], 1)
        self.assertEqual(parsed[1][2][-1], 0)
        self.assertEqual(parsed[0][2][:-1], bytes(18))
        self.assertEqual(parsed[1][2], bytes(19))

    def test_snapshot_display_is_full_in_auto_and_compact_in_pid(self):
        snap = contcp_t.EthernetUartSystemSnapshot()
        snap.displacement_mm_x10 = 893
        snap.displacement_valid = 1
        snap.limit_switch_bits = 0x07

        auto_output = io.StringIO()
        with contextlib.redirect_stdout(auto_output):
            contcp_t.print_snapshot_for_active_mode(snap)
        self.assertIn('"displacement_mm_x10": 893', auto_output.getvalue())
        self.assertNotIn("[输入状态]", auto_output.getvalue())

        contcp_t.PID_MODE_ACTIVE.set()
        pid_output = io.StringIO()
        with contextlib.redirect_stdout(pid_output):
            contcp_t.print_snapshot_for_active_mode(snap)
        self.assertIn(
            "[输入状态] displacement=89.3 mm valid=1",
            pid_output.getvalue(),
        )
        self.assertIn("limit_switch_bits=0x07", pid_output.getvalue())
        self.assertNotIn('"displacement_mm_x10"', pid_output.getvalue())


if __name__ == "__main__":
    unittest.main()
