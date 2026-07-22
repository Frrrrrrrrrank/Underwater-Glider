import json
import os
import socket
import sys
import tempfile
import unittest


HERE = os.path.dirname(os.path.abspath(__file__))
STATION_DIR = os.path.dirname(HERE)
REPO_ROOT = os.path.dirname(STATION_DIR)
if HERE not in sys.path:
    sys.path.insert(0, HERE)
if STATION_DIR not in sys.path:
    sys.path.insert(0, STATION_DIR)

import mag_cal_push
import ugcp_protocol as ugcp


class FakeSocket:
    def __init__(self, chunks=None):
        self.sent = []
        self.chunks = list(chunks or [])
        self.timeout = None

    def sendall(self, data):
        self.sent.append(bytes(data))

    def settimeout(self, timeout):
        self.timeout = timeout

    def recv(self, _size):
        if self.chunks:
            return self.chunks.pop(0)
        raise socket.timeout()


def make_ugekf_ack(cal_crc, cal_valid):
    values = ['0'] * 31 + [str(cal_crc), str(cal_valid)]
    body = 'UGEKF,' + ','.join(values)
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return ('$%s*%02X' % (body, checksum)).encode('ascii') + bytes((13, 10))


class MagCalPushTests(unittest.TestCase):
    def setUp(self):
        self.table = ugcp.pack_mag_cal_table(
            [1.0, 2.0, 3.0],
            [[1.0, 0.0, 0.0],
             [0.0, 1.0, 0.0],
             [0.0, 0.0, 1.0]],
        )
        self.document = {
            'schema': 'underwater_glider.mag_calibration',
            'version': 1,
            'coordinate_frame': 'module_FLU',
            'hard_iron_unit': 'uT',
            'soft_iron_order': 'row_major',
            'hard_iron_ut': [1.0, 2.0, 3.0],
            'soft_iron': [[1.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0],
                          [0.0, 0.0, 1.0]],
            'reserved': [0.0, 0.0, 0.0, 0.0],
            'quality': {'class': 'pass'},
        }

    @staticmethod
    def _load_document(document, allow_failed_quality=False):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'cal.json')
            with open(path, 'w', encoding='utf-8') as stream:
                json.dump(document, stream)
            return mag_cal_push.load_calibration_json(
                path,
                allow_failed_quality=allow_failed_quality,
            )

    def test_transaction_order_and_reassembly(self):
        connection = FakeSocket()
        expected_crc = mag_cal_push.send_transaction(
            connection,
            self.table,
            transaction=9,
            sequence=ugcp.SequenceCounter(100),
            inter_frame_delay=0,
            quiet_after_off=0,
        )
        parsed = [ugcp.parse_frame(frame) for frame in connection.sent]
        self.assertEqual(
            [item[0] for item in parsed],
            [ugcp.MSG_TELEM_CONTROL]
            + [ugcp.MSG_PARAM_BLOCK] * 4
            + [ugcp.MSG_PARAM_COMMIT, ugcp.MSG_TELEM_CONTROL],
        )
        self.assertEqual(parsed[0][2], bytes((0,)))
        self.assertEqual(parsed[-1][2], bytes((1,)))
        self.assertEqual([item[1] for item in parsed], list(range(100, 107)))

        block_payloads = [item[2] for item in parsed[1:5]]
        self.assertEqual([payload[:3] for payload in block_payloads],
                         [bytes((1, 9, index)) for index in range(4)])
        self.assertEqual(
            b''.join(payload[3:] for payload in block_payloads),
            self.table,
        )
        commit = parsed[5][2]
        self.assertEqual(commit[:3], bytes((1, 9, 4)))
        self.assertEqual(int.from_bytes(commit[3:7], 'little'), expected_crc)

    def test_wait_for_ack_handles_fragmentation_and_ignores_old_ack(self):
        expected_crc = ugcp.crc32_iso_hdlc(self.table)
        stream = (
            make_ugekf_ack(expected_crc ^ 1, 1)
            + make_ugekf_ack(expected_crc, 0)
            + make_ugekf_ack(expected_crc, 1)
        )
        connection = FakeSocket([
            stream[:7],
            stream[7:53],
            stream[53:],
        ])
        self.assertTrue(
            mag_cal_push.wait_for_ack(connection, expected_crc, timeout=0.2))

    def test_load_calibration_json(self):
        table = self._load_document(self.document)
        self.assertEqual(len(table), 64)
        self.assertEqual(table, self.table)

    def test_json_rejects_non_v1_and_wrong_matrix_order(self):
        for field, value in (
                ('version', 1.5),
                ('soft_iron_order', 'column_major')):
            with self.subTest(field=field):
                document = dict(self.document)
                document[field] = value
                with self.assertRaises(ValueError):
                    self._load_document(document)

    def test_json_rejects_non_object_root(self):
        with self.assertRaises(ValueError):
            self._load_document([])

    def test_failed_quality_requires_force(self):
        document = dict(self.document)
        document['quality'] = {'class': 'fail'}
        with self.assertRaises(ValueError):
            self._load_document(document)
        self.assertEqual(
            self._load_document(document, allow_failed_quality=True),
            self.table,
        )

    def test_ack_partial_line_survives_retry_boundary(self):
        expected_crc = ugcp.crc32_iso_hdlc(self.table)
        ack = make_ugekf_ack(expected_crc, 1)
        connection = FakeSocket([ack[:17]])
        matched, buffered = mag_cal_push.wait_for_ack(
            connection,
            expected_crc,
            timeout=0.01,
            return_buffer=True,
        )
        self.assertFalse(matched)
        self.assertEqual(buffered, ack[:17])

        connection.chunks.append(ack[17:])
        matched, buffered = mag_cal_push.wait_for_ack(
            connection,
            expected_crc,
            timeout=0.2,
            initial_buffer=buffered,
            return_buffer=True,
        )
        self.assertTrue(matched)
        self.assertEqual(buffered, b'')


if __name__ == '__main__':
    unittest.main()
