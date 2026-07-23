import os
import sys
import unittest


HERE = os.path.dirname(os.path.abspath(__file__))
STATION_DIR = os.path.dirname(HERE)
REPO_ROOT = os.path.dirname(STATION_DIR)
for path in (HERE, STATION_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

import ekf_logger
import mag_logger
import ugcp_protocol as ugcp


class FakeSocket:
    def __init__(self):
        self.sent = []

    def sendall(self, data):
        self.sent.append(bytes(data))


class StreamOwnerTests(unittest.TestCase):
    def test_mag_and_ekf_loggers_declare_complete_masks(self):
        cases = (
            (mag_logger, ugcp.STREAM_MAG),
            (ekf_logger, ugcp.STREAM_EKF),
        )
        for module, expected_mask in cases:
            with self.subTest(module=module.__name__):
                conn = FakeSocket()
                sequence = ugcp.SequenceCounter(10)
                self.assertEqual(
                    module.send_stream_control(
                        conn, sequence, expected_mask),
                    10,
                )
                self.assertEqual(
                    module.send_stream_control(conn, sequence, 0),
                    11,
                )
                parsed = [ugcp.parse_frame(frame) for frame in conn.sent]
                self.assertEqual(
                    [item[0] for item in parsed],
                    [ugcp.MSG_STREAM_CONTROL, ugcp.MSG_STREAM_CONTROL],
                )
                self.assertEqual(
                    [item[2] for item in parsed],
                    [bytes((expected_mask,)), bytes((0,))],
                )

    def test_retry_requires_both_data_and_request_to_be_stale(self):
        for module in (mag_logger, ekf_logger):
            with self.subTest(module=module.__name__):
                self.assertFalse(module.stream_retry_due(0.0, 0.0, 1.999))
                self.assertTrue(module.stream_retry_due(0.0, 0.0, 2.0))
                self.assertFalse(module.stream_retry_due(1.5, 0.0, 2.0))
                self.assertFalse(module.stream_retry_due(0.0, 1.5, 2.0))
                self.assertTrue(module.stream_retry_due(2.0, 2.0, 4.0))


if __name__ == "__main__":
    unittest.main()
