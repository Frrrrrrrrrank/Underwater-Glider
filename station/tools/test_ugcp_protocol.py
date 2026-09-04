import os
import struct
import sys
import unittest
import zlib


HERE = os.path.dirname(os.path.abspath(__file__))
STATION_DIR = os.path.dirname(HERE)
for path in (HERE, STATION_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

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
        self.assertEqual(ugcp.MSG_CASCADE_COMMAND, 0x05)
        self.assertEqual(ugcp.MSG_STREAM_CONTROL, ugcp.MSG_TELEM_CONTROL)
        self.assertEqual(
            [ugcp.pack_stream_control_payload(mask) for mask in range(8)],
            [bytes((mask,)) for mask in range(8)],
        )
        self.assertEqual(ugcp.STREAM_ALL, 0x07)
        self.assertEqual(ugcp.pack_telem_control_payload(False), b'\x00')
        self.assertEqual(ugcp.pack_telem_control_payload(True), b'\x01')
        for invalid in (-1, 8, 255):
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

    def test_cascade_command_offsets_round_trip_and_validation(self):
        payload = ugcp.pack_cascade_command_payload(
            ugcp.CASCADE_MODE_DEPTH,
            True,
            0x1234,
            5.5,
            -30.0,
            42.0,
            0x89ABCDEF,
        )
        self.assertEqual(len(payload), 24)
        self.assertEqual(payload[:4], b'\x01\x02\x01\x00')
        self.assertEqual(struct.unpack_from('<H', payload, 4)[0], 0x1234)
        self.assertEqual(payload[6:8], b'\x00\x00')
        self.assertAlmostEqual(struct.unpack_from('<f', payload, 8)[0], 5.5)
        self.assertAlmostEqual(struct.unpack_from('<f', payload, 12)[0], -30.0)
        self.assertAlmostEqual(struct.unpack_from('<f', payload, 16)[0], 42.0)
        self.assertEqual(struct.unpack_from('<I', payload, 20)[0], 0x89ABCDEF)

        decoded = ugcp.unpack_cascade_command_payload(payload)
        self.assertEqual(decoded['mode'], ugcp.CASCADE_MODE_DEPTH)
        self.assertTrue(decoded['armed'])
        self.assertEqual(decoded['command_id'], 0x1234)
        self.assertAlmostEqual(decoded['target_pitch_deg'], -30.0)

        invalid_cases = (
            (3, True, 0, 0, 0, 0, 0),
            (ugcp.CASCADE_MODE_PITCH_HOLD, 2, 0, 0, 0, 0, 0),
            (ugcp.CASCADE_MODE_DEPTH, True, 0, -1, 0, 0, 0),
            (ugcp.CASCADE_MODE_DEPTH, True, 0, 1, 91, 0, 0),
            (ugcp.CASCADE_MODE_DEPTH, True, 0, 1, 0, -101, 0),
            (ugcp.CASCADE_MODE_DEPTH, True, 0, 1, float('nan'), 0, 0),
        )
        for arguments in invalid_cases:
            with self.subTest(arguments=arguments):
                with self.assertRaises(ugcp.UgcpProtocolError):
                    ugcp.pack_cascade_command_payload(*arguments)

        bad_reserved = bytearray(payload)
        bad_reserved[6] = 1
        with self.assertRaises(ugcp.UgcpProtocolError):
            ugcp.unpack_cascade_command_payload(bad_reserved)

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

    def test_pid_table_blocks_crc_round_trip_and_validation(self):
        parameters = {
            'buoyancy_kp_mm_per_m': 5.0,
            'buoyancy_ki_mm_per_m_s': 0.1,
            'buoyancy_kd_mm_s_per_m': 1.2,
            'buoyancy_integral_limit_m_s': 2.0,
            'buoyancy_derivative_tau_s': 0.4,
            'pitch_kp_rpm_per_deg': 2.5,
            'pitch_ki_rpm_per_deg_s': 0.2,
            'pitch_kd_rpm_s_per_deg': 0.8,
            'pitch_integral_limit_deg_s': 20.0,
            'pitch_derivative_tau_s': 0.1,
            'depth_to_pitch_alpha_deg_per_m': 4.0,
            'max_pitch_deg': 30.0,
            'max_worm_rpm': 200.0,
            'max_thrust_percent': 60.0,
            'surface_depth_m': 0.2,
            'depth_switch_threshold_m': 0.3,
            'pitch_actuator_sign': -1.0,
            'displacement_neutral_mm': 90.0,
        }
        table = ugcp.pack_pid_param_table(parameters)
        self.assertEqual(len(table), 80)
        self.assertEqual(len(ugcp.split_param_table(table)), 5)
        self.assertEqual(b''.join(ugcp.split_param_table(table)), table)
        decoded = ugcp.unpack_pid_param_table(table)
        for name, expected in parameters.items():
            self.assertAlmostEqual(decoded[name], expected, places=5)

        crc = ugcp.crc32_iso_hdlc(table)
        commit = ugcp.pack_param_commit_payload(
            ugcp.PARAM_GROUP_PID,
            9,
            ugcp.PID_PARAM_BLOCK_COUNT,
            crc,
        )
        self.assertEqual(
            struct.unpack('<BBBI', commit),
            (ugcp.PARAM_GROUP_PID, 9, 5, crc),
        )

        for name, value in (
                ('pitch_kp_rpm_per_deg', -1.0),
                ('depth_to_pitch_alpha_deg_per_m', -1.0),
                ('depth_to_pitch_alpha_deg_per_m', 91.0),
                ('max_pitch_deg', 0.0),
                ('max_pitch_deg', 91.0),
                ('max_worm_rpm', 601.0),
                ('max_thrust_percent', 101.0),
                ('surface_depth_m', -0.1),
                ('pitch_actuator_sign', 0.0)):
            unsafe = dict(parameters)
            unsafe[name] = value
            with self.subTest(name=name, value=value):
                with self.assertRaises(ugcp.UgcpProtocolError):
                    ugcp.pack_pid_param_table(unsafe)

        # neutral 越界必须能送到板端，由底层 MIN/MAX 宏返回专用拒绝码。
        board_rejected = dict(parameters)
        board_rejected['displacement_neutral_mm'] = 140.0
        self.assertEqual(len(ugcp.pack_pid_param_table(board_rejected)), 80)

        bad_reserved = bytearray(table)
        struct.pack_into('<f', bad_reserved, 18 * 4, 1.0)
        with self.assertRaises(ugcp.UgcpProtocolError):
            ugcp.unpack_pid_param_table(bad_reserved)

        missing = dict(parameters)
        missing.pop('pitch_kd_rpm_s_per_deg')
        with self.assertRaises(ugcp.UgcpProtocolError):
            ugcp.pack_pid_param_table(missing)


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
