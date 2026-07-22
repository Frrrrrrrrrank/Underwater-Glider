import os
import struct
import sys
import unittest
import zlib


HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS_DIR = os.path.join(HERE, 'tools')
if HERE not in sys.path:
    sys.path.insert(0, HERE)
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)

import contcp_t
import ekf_logger
import ugcp_protocol as ugcp


def make_nmea(body):
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return '$%s*%02X' % (body, checksum)


class UgcpProtocolTests(unittest.TestCase):
    def test_crc_standard_vector(self):
        self.assertEqual(ugcp.crc32_iso_hdlc(b'123456789'), 0xCBF43926)

    def test_frame_offsets_crc_and_round_trip(self):
        sample_payload = bytes((0xAA, 0xBB))
        frame = ugcp.build_frame(0x5A, sample_payload, seq=0x1234)
        self.assertEqual(frame[:4], b'UGCP')
        self.assertEqual(frame[4], 1)
        self.assertEqual(frame[5], 0x5A)
        self.assertEqual(struct.unpack_from('<H', frame, 6)[0], 0x1234)
        self.assertEqual(struct.unpack_from('<H', frame, 8)[0], 2)
        self.assertEqual(frame[10:12], sample_payload)
        self.assertEqual(
            struct.unpack_from('<I', frame, 12)[0],
            zlib.crc32(frame[4:12]) & 0xFFFFFFFF,
        )
        self.assertEqual(ugcp.parse_frame(frame), (0x5A, 0x1234, sample_payload))

        corrupt = bytearray(frame)
        corrupt[10] ^= 1
        with self.assertRaises(ugcp.UgcpProtocolError):
            ugcp.parse_frame(corrupt)

    def test_sequence_rollover(self):
        sequence = ugcp.SequenceCounter(0xFFFF)
        self.assertEqual(sequence.next(), 0xFFFF)
        self.assertEqual(sequence.next(), 0)

    def test_control_policy_types_and_stream_mask(self):
        self.assertEqual(ugcp.MSG_CONTROL19_STREAM, 0x01)
        self.assertEqual(ugcp.MSG_CONTROL19, ugcp.MSG_CONTROL19_STREAM)
        self.assertEqual(ugcp.MSG_CONTROL19_LATCHED, 0x04)
        self.assertEqual(ugcp.MSG_STREAM_CONTROL, ugcp.MSG_TELEM_CONTROL)
        self.assertEqual(
            [ugcp.pack_stream_control_payload(mask) for mask in range(4)],
            [b'\x00', b'\x01', b'\x02', b'\x03'],
        )
        self.assertEqual(ugcp.pack_telem_control_payload(False), b'\x00')
        self.assertEqual(ugcp.pack_telem_control_payload(True), b'\x01')
        for invalid in (-1, 4, 255):
            with self.assertRaises(ugcp.UgcpProtocolError):
                ugcp.pack_stream_control_payload(invalid)

    def test_control19_signed_rpm_and_validation(self):
        payload = ugcp.pack_control_payload(
            1, -12345, 1, -678,
            0, 1, 25, 30, 0, 1, 1, 0, 1,
        )
        self.assertEqual(len(payload), 19)
        self.assertEqual(struct.unpack_from('<i', payload, 1)[0], -12345)
        self.assertEqual(struct.unpack_from('<i', payload, 6)[0], -678)
        self.assertEqual(payload[12:14], bytes((25, 30)))

        for field_values in (
            (1, 0, 1, 0, 0, 1, 101, 30, 0, 1, 1, 0, 1),
            (1, 0, 1, 0, 0, 1, 25, 30, 2, 1, 1, 0, 1),
            (1, 0, 1, 0, 0, 1, 25, 30, 0, 1, 1, 2, 1),
        ):
            with self.assertRaises(ugcp.UgcpProtocolError):
                ugcp.pack_control_payload(*field_values)

    def test_mag_table_blocks_and_commit(self):
        hard = [5.5920, 2.4283, 10.0223]
        matrix = [
            [1.091314, -0.004086, 0.161096],
            [-0.004086, 1.319577, -0.013531],
            [0.161096, -0.013531, 0.787296],
        ]
        table = ugcp.pack_mag_cal_table(hard, matrix)
        self.assertEqual(len(table), 64)
        blocks = ugcp.split_param_table(table)
        self.assertEqual(len(blocks), 4)
        self.assertEqual(b''.join(blocks), table)

        payloads = [
            ugcp.pack_param_block_payload(
                ugcp.PARAM_GROUP_MAG_CAL, 7, index, block)
            for index, block in enumerate(blocks)
        ]
        self.assertTrue(all(len(payload) == 19 for payload in payloads))
        self.assertEqual(b''.join(payload[3:] for payload in payloads), table)

        crc = ugcp.crc32_iso_hdlc(table)
        commit = ugcp.pack_param_commit_payload(
            ugcp.PARAM_GROUP_MAG_CAL, 7, 4, crc)
        self.assertEqual(len(commit), 7)
        self.assertEqual(struct.unpack('<BBBI', commit), (1, 7, 4, crc))

    def test_mag_table_rejects_unsafe_values(self):
        identity = [[1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]]
        with self.assertRaises(ugcp.UgcpProtocolError):
            ugcp.pack_mag_cal_table([float('nan'), 0, 0], identity)
        with self.assertRaises(ugcp.UgcpProtocolError):
            ugcp.pack_mag_cal_table([0, 0, 0], identity, [0, 0, 0, -0.0])
        with self.assertRaises(ugcp.UgcpProtocolError):
            ugcp.pack_mag_cal_table(
                [0, 0, 0],
                [[1, 0.2, 0], [0, 1, 0], [0, 0, 1]],
            )
        with self.assertRaises(ugcp.UgcpProtocolError):
            ugcp.pack_mag_cal_table(
                [0, 0, 0],
                [[-1, 0, 0], [0, -1, 0], [0, 0, 1]],
            )


class TelemetryCompatibilityTests(unittest.TestCase):
    def _line(self, with_ack):
        values = ['0'] * len(contcp_t.EKF_BASE_FIELDS)
        values[0] = '12'
        values[1] = '345'
        if with_ack:
            values += ['4294967295', '1']
        return make_nmea('UGEKF,' + ','.join(values))

    def test_old_31_field_frame(self):
        line = self._line(False)
        self.assertTrue(contcp_t.ekf_checksum_ok(line))
        self.assertTrue(ekf_logger.nmea_checksum_ok(line))
        for parser in (contcp_t.ekf_parse, ekf_logger.parse_ugekf):
            row = parser(line)
            self.assertEqual(row['seq'], 12)
            self.assertIsNone(row['cal_crc'])
            self.assertIsNone(row['cal_valid'])

    def test_new_33_field_ack(self):
        line = self._line(True)
        for parser in (contcp_t.ekf_parse, ekf_logger.parse_ugekf):
            row = parser(line)
            self.assertEqual(row['cal_crc'], 0xFFFFFFFF)
            self.assertEqual(row['cal_valid'], 1)


if __name__ == '__main__':
    unittest.main()
