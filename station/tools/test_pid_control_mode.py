import contextlib
import io
import os
import struct
import sys
import threading
import time
import unittest


HERE = os.path.dirname(os.path.abspath(__file__))
STATION_DIR = os.path.dirname(HERE)
if STATION_DIR not in sys.path:
    sys.path.insert(0, STATION_DIR)

import contcp_t
import ugcp_protocol as ugcp


def make_nmea(body):
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return '$%s*%02X' % (body, checksum)


def make_ctl_row(**updates):
    row = {}
    for name in contcp_t.UGCTL_FIELDS:
        row[name] = 0 if name in contcp_t.UGCTL_INT_FIELDS else 0.0
    row.update({
        'schema': 2,
        'mode': ugcp.CASCADE_MODE_OFF,
        'fault': 1,
        'core_fault': 1,
    })
    row.update(updates)
    return row


def ctl_line(row):
    values = ','.join(str(row[name]) for name in contcp_t.UGCTL_FIELDS)
    return make_nmea('UGCTL,' + values)


class FakeBoardSocket:
    def __init__(self, store):
        self.store = store
        self.sent = []
        self.board_row = make_ctl_row()

    def sendall(self, data):
        frame = bytes(data)
        self.sent.append(frame)
        msg_type, _, payload = ugcp.parse_frame(frame)
        if msg_type == ugcp.MSG_CASCADE_COMMAND:
            command = ugcp.unpack_cascade_command_payload(payload)
            armed = int(command['armed'] and
                        command['mode'] != ugcp.CASCADE_MODE_OFF and
                        self.board_row['pid_valid'] == 1 and
                        command['required_pid_crc'] == self.board_row['active_crc'])
            self.board_row.update({
                'seq': self.board_row['seq'] + 1,
                'mode': command['mode'],
                'armed': armed,
                'output_valid': armed,
                'fault': 0 if armed else 1,
                'core_fault': 0 if armed else 1,
                'command_id': command['command_id'],
                'required_crc': command['required_pid_crc'],
                'target_depth': command['target_depth_m'],
                'target_pitch_deg': command['target_pitch_deg'],
                'pitch_sp_deg': command['target_pitch_deg'],
                'thrust_pct': command['signed_thrust_percent'] if armed else 0.0,
            })
            self.store.publish(self.board_row)
        elif msg_type == ugcp.MSG_PARAM_COMMIT:
            group, _, block_count, table_crc = struct.unpack('<BBBI', payload)
            if (group == ugcp.PARAM_GROUP_PID and
                    block_count == ugcp.PID_PARAM_BLOCK_COUNT):
                self.board_row.update({
                    'seq': self.board_row['seq'] + 1,
                    'pid_valid': 1,
                    'active_crc': table_crc,
                    'config_apply': self.board_row['config_apply'] + 1,
                    'config_reject': 0,
                })
                self.store.publish(self.board_row)


class UgctlTests(unittest.TestCase):
    def test_strict_ugctl_parse(self):
        row = make_ctl_row(
            seq=12,
            t_ms=345,
            mode=ugcp.CASCADE_MODE_PITCH_HOLD,
            pitch_sp_deg=-5.0,
            pitch_deg=-3.5,
            active_crc=0x89ABCDEF,
        )
        line = ctl_line(row)
        parsed = contcp_t.ugctl_parse(line)
        self.assertEqual(parsed['seq'], 12)
        self.assertEqual(parsed['active_crc'], 0x89ABCDEF)
        self.assertAlmostEqual(parsed['pitch_sp_deg'], -5.0)

        self.assertIsNone(contcp_t.ugctl_parse(line[:-1] + '0'))
        wrong_schema = dict(row)
        wrong_schema['schema'] = 1
        self.assertIsNone(contcp_t.ugctl_parse(ctl_line(wrong_schema)))

    def test_store_waits_for_new_generation(self):
        store = contcp_t.UgctlStore()

        def publish_later():
            time.sleep(0.02)
            store.publish(make_ctl_row(seq=7))

        thread = threading.Thread(target=publish_later)
        thread.start()
        result = store.wait_for(lambda row: row['seq'] == 7, 0, 0.5)
        thread.join()
        self.assertEqual(result['seq'], 7)


class PidSessionTests(unittest.TestCase):
    def setUp(self):
        contcp_t.UGCP_TX_SEQUENCE = ugcp.SequenceCounter()
        self.store = contcp_t.UgctlStore()
        self.conn = FakeBoardSocket(self.store)
        self.store.publish(self.conn.board_row)
        self.example = os.path.join(STATION_DIR, 'pid_params.example.json')

    def _frame_types(self):
        return [ugcp.parse_frame(frame)[0] for frame in self.conn.sent]

    def _cascade_commands(self):
        commands = []
        for frame in self.conn.sent:
            msg_type, _, payload = ugcp.parse_frame(frame)
            if msg_type == ugcp.MSG_CASCADE_COMMAND:
                commands.append(ugcp.unpack_cascade_command_payload(payload))
        return commands

    def test_parameter_5_plus_1_ack_then_arm_and_refresh(self):
        session = contcp_t.PidSession(self.conn, self.store, send_period_s=0.02)
        with contextlib.redirect_stdout(io.StringIO()):
            crc = session.load_parameters(self.example)
            self.assertTrue(session.push_parameters())
        self.assertEqual(
            self._frame_types(),
            [ugcp.MSG_PARAM_BLOCK] * 5 + [ugcp.MSG_PARAM_COMMIT],
        )
        local = session.local_snapshot()
        self.assertEqual(local['loaded_crc'], crc)
        self.assertEqual(local['confirmed_crc'], crc)

        session.set_pitch_target(-5.0)
        session.set_thrust(12.0)
        session.start()
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(session.arm())
        time.sleep(0.07)
        armed_commands = [
            command for command in self._cascade_commands()
            if command['armed']]
        self.assertGreaterEqual(len(armed_commands), 2)
        first_id = armed_commands[0]['command_id']
        self.assertTrue(all(command['command_id'] == first_id
                            for command in armed_commands))
        self.assertTrue(all(command['mode'] == ugcp.CASCADE_MODE_PITCH_HOLD
                            for command in armed_commands))
        self.assertTrue(all(abs(command['target_pitch_deg'] + 5.0) < 1e-6
                            for command in armed_commands))
        self.assertTrue(all(abs(command['signed_thrust_percent'] - 12.0) < 1e-6
                            for command in armed_commands))

        session.set_pitch_target(-3.0)
        time.sleep(0.04)
        newest = [
            command for command in self._cascade_commands()
            if command['armed']][-1]
        self.assertNotEqual(newest['command_id'], first_id)
        self.assertAlmostEqual(newest['target_pitch_deg'], -3.0)

        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(session.disarm(force=True))
        count_after_off = len(self._cascade_commands())
        time.sleep(0.05)
        self.assertEqual(len(self._cascade_commands()), count_after_off)
        self.assertEqual(self._cascade_commands()[-1]['mode'], ugcp.CASCADE_MODE_OFF)
        self.assertFalse(self._cascade_commands()[-1]['armed'])
        session.close()

    def test_arm_requires_current_parameter_ack(self):
        session = contcp_t.PidSession(self.conn, self.store)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(session.arm())
            session.load_parameters(self.example)
            self.assertFalse(session.arm())


if __name__ == '__main__':
    unittest.main()
