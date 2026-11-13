"""UART7原始外包、单接收链路、租约与异步记录的回归测试。"""

import contextlib
import csv
import io
import json
import socket
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

STATION = Path(__file__).resolve().parents[1]
if str(STATION) not in sys.path:
    sys.path.insert(0, str(STATION))

import contcp_t as cli
import ugcp_protocol as ugcp
from experiment_recorder import ExperimentRecorder
from raw_telemetry import UGRAW_WIRE_FIELDS, ugraw_parse
from groundstation.ethernet_link import EthernetRealtimeLink, DEFAULT_STREAM_MASK
from groundstation.legacy_telemetry import decode_telemetry_line
from groundstation.session_recorder import SessionRecorder
from groundstation.state_store import StateStore


def envelope(body):
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return '$' + body + '*%02X\r\n' % checksum


def raw_frame(payload=b'$GNRMC,bad*00\r', **overrides):
    row = dict.fromkeys(UGRAW_WIRE_FIELDS, 0)
    row.update(schema=1, capture_id=7, seq=8, rx_us_hi=2, rx_us_lo=3,
               tx_us_hi=2, tx_us_lo=500, src_len=len(payload),
               kept_len=min(len(payload), 384), flags=int(len(payload) > 384))
    row.update(overrides)
    return envelope('UGRAW,' + ','.join(str(row[name]) for name in UGRAW_WIRE_FIELDS)
                    + ',' + payload[:384].hex())


class FakeSocket:
    def __init__(self, chunks=(), allowed=0x7F):
        self.chunks = iter(chunks)
        self.masks = []
        self.allowed = allowed
        self.active_mask = 0
        self.recv_owners = set()
        self.closed = False

    def sendall(self, frame):
        msg_type, _seq, payload = ugcp.parse_frame(frame)
        if msg_type == ugcp.MSG_STREAM_CONTROL:
            mask = payload[0]
            self.masks.append(mask)
            if mask & ~self.allowed == 0:
                self.active_mask = mask

    def recv(self, _size):
        self.recv_owners.add(threading.get_ident())
        item = next(self.chunks, b'')
        if isinstance(item, Exception):
            raise item
        return item

    def settimeout(self, _value):
        pass

    def shutdown(self, _how):
        pass

    def close(self):
        self.closed = True


class RawParserTests(unittest.TestCase):
    def test_preserves_bad_inner_checksum_nul_nonascii_and_cr(self):
        payload = b'$GNRMC,bad*00\x00\xff\r'
        frame = raw_frame(payload)
        row = ugraw_parse(frame)
        self.assertEqual(bytes.fromhex(row['payload_hex']), payload)
        self.assertEqual(row['rx_us'], (2 << 32) | 3)
        self.assertEqual(row['tx_us'], (2 << 32) | 500)
        self.assertEqual(row['raw'], frame)
        self.assertEqual(decode_telemetry_line(frame).data, row)

    def test_empty_and_maximum_and_truncated_payload(self):
        for length in (0, 384, 385, 1000):
            with self.subTest(length=length):
                row = ugraw_parse(raw_frame(b'\xff' * length))
                self.assertEqual(row['src_len'], length)
                self.assertEqual(row['kept_len'], min(384, length))
                self.assertEqual(row['flags'], int(length > 384))

    def test_rejects_field_count_schema_reserved_flags_lengths_and_uint32(self):
        cases = [raw_frame(schema=2), raw_frame(flags=2), raw_frame(flags=1),
                 raw_frame(kept_len=0), raw_frame(src_len=0), raw_frame(seq=-1),
                 raw_frame(seq=0x100000000), raw_frame(rx_us_lo='1.0'),
                 raw_frame(b'x' * 385, flags=0)]
        body = raw_frame().split('*')[0][1:]
        cases.extend((envelope(body + ',0'), envelope(body.replace(',7,', ',+7,', 1)),
                      envelope(body[:-1]), envelope(body[:-1] + 'Z')))
        for line in cases:
            with self.subTest(line=line):
                self.assertIsNone(ugraw_parse(line))

    def test_outer_corruption_is_not_repaired(self):
        frame = raw_frame()
        for invalid in (None, b'bytes', ' ' + frame, frame + ' ', frame[:-4] + 'XX\r\n',
                        frame.replace('\r\n', '\r\r\n'), frame[:15] + 'Z' + frame[16:],
                        '$UGRAW,' + '0' * 1100):
            with self.subTest(line=invalid):
                self.assertIsNone(ugraw_parse(invalid))


class RawCliTests(unittest.TestCase):
    def setUp(self):
        self.scope = patch.multiple(
            cli, RAW_CAPTURE_REQUESTED=False, RAW_CAPTURE_CONFIRMED=False,
            RAW_LAST_RENEW_S=0.0, TELEMETRY_STREAM_MASK=0,
            EXPERIMENT_RECORDER=None, EXPERIMENT_MANAGED=True,
            EXPERIMENT_LAST_ERROR=None, running=True,
            UGCP_TX_SEQUENCE=ugcp.SequenceCounter())
        self.scope.start()
        self.addCleanup(self.scope.stop)
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)

    def test_default_off_and_initial_compatibility_renew_and_restore(self):
        conn = FakeSocket()
        self.assertTrue(cli.send_stream_control(conn, 0x2D))
        self.assertEqual(conn.masks, [0x0D, 0x1D, 0x3D])
        conn.masks.clear()
        self.assertTrue(cli.send_stream_control(conn, 0x3D, raw_enabled=True))
        self.assertEqual(conn.masks, [0x0D, 0x1D, 0x3D, 0x7D])
        cli.RAW_CAPTURE_CONFIRMED = True
        conn.masks.clear()
        self.assertTrue(cli.renew_raw_capture(conn, cli.RAW_LAST_RENEW_S + 4.99))
        self.assertEqual(conn.masks, [])
        self.assertTrue(cli.renew_raw_capture(conn, cli.RAW_LAST_RENEW_S + 5))
        self.assertEqual(conn.masks, [0x7D])
        self.assertTrue(cli.send_stream_control(conn, ugcp.STREAM_EKF))
        self.assertTrue(cli.send_stream_control(conn, 0x3D))
        self.assertEqual(conn.masks[-2:], [0x51, 0x7D])
        self.assertTrue(cli.send_stream_control(conn, 0x7D, raw_enabled=False))
        self.assertEqual(conn.masks[-3:], [0x0D, 0x1D, 0x3D])
        # 旧的模式保存值含RAW，也不能重新开启用户已经关闭的采集。
        self.assertTrue(cli.send_stream_control(conn, 0x7D))
        self.assertFalse(cli.get_stream_control_mask() & ugcp.STREAM_RAW)

    def test_old_firmware_raw_request_does_not_break_mode_changes(self):
        conn = FakeSocket(allowed=0x0F)
        cli.send_stream_control(conn, 0x2D, raw_enabled=True)
        self.assertEqual(conn.active_mask, 0x0D)
        self.assertFalse(cli.RAW_CAPTURE_CONFIRMED)
        cli.send_stream_control(conn, ugcp.STREAM_EKF | ugcp.STREAM_RDSS)
        self.assertEqual(conn.active_mask, ugcp.STREAM_EKF)
        cli.send_stream_control(conn, ugcp.STREAM_WRM | ugcp.STREAM_RDSS, raw_enabled=False)
        self.assertEqual(conn.active_mask, ugcp.STREAM_WRM)

    def test_exit_all_off_disables_renewal(self):
        conn = FakeSocket()
        cli.send_stream_control(conn, 0x2D, raw_enabled=True)
        cli.send_stream_control(conn, 0, keep_battery=False)
        self.assertEqual(conn.masks[-1], 0)
        count = len(conn.masks)
        cli.renew_raw_capture(conn, cli.RAW_LAST_RENEW_S + 100)
        self.assertEqual(len(conn.masks), count)

    def test_rawon_requires_working_async_recorder(self):
        conn = FakeSocket()
        self.assertFalse(cli.set_raw_capture(conn, True))
        cli.EXPERIMENT_RECORDER = Mock()
        cli.EXPERIMENT_RECORDER.snapshot.return_value = {'status': 'error', 'last_error': 'disk full'}
        self.assertFalse(cli.set_raw_capture(conn, True))
        self.assertEqual(conn.masks, [])
        cli.EXPERIMENT_RECORDER.snapshot.return_value = {'status': 'recording'}
        self.assertTrue(cli.set_raw_capture(conn, True))
        self.assertTrue(conn.active_mask & ugcp.STREAM_RAW)

    def test_fragmented_rx_sole_owner_records_raw_and_invalid_and_partial(self):
        valid = raw_frame(b'$GNRMC,broken\x00\xff\r')
        invalid = valid.replace(',7,', ',9,', 1)
        partial = '$UGRAW,1,7'
        chunks = [valid[:19].encode(), valid[19:].encode() + invalid.encode(), partial.encode()]
        conn = FakeSocket(chunks)
        with tempfile.TemporaryDirectory() as temporary:
            recorder = ExperimentRecorder(Path(temporary))
            recorder.start('test')
            cli.EXPERIMENT_RECORDER = recorder
            cli.receive_thread(conn)
            recorder.stop()
            state = recorder.snapshot()
            with (Path(state['directory']) / 'telemetry_ugraw.csv').open(encoding='utf-8', newline='') as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 3)
            self.assertEqual(rows[0]['raw'], valid)
            self.assertEqual(rows[0]['payload_hex'], b'$GNRMC,broken\x00\xff\r'.hex())
            self.assertEqual(rows[1]['raw'], invalid)
            self.assertEqual(rows[1]['payload_hex'], '')
            self.assertEqual(rows[2]['raw'], partial)
            events = [json.loads(line) for line in (Path(state['directory']) / 'events.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertFalse([event for event in events if event.get('topic') == 'ugraw'][-1]['complete'])
            self.assertTrue(int(rows[0]['t_ns']) > 0)
            summary = json.loads((Path(state['directory']) / 'session.json').read_text(encoding='utf-8'))
            self.assertEqual(summary['invalid_ugraw_count'], 2)
            self.assertTrue(summary['complete'])
        self.assertTrue(cli.RAW_CAPTURE_CONFIRMED)
        self.assertEqual(len(conn.recv_owners), 1)

    def test_continuous_rx_checks_renew_without_waiting_for_timeout(self):
        conn = FakeSocket([raw_frame().encode(), raw_frame(seq=9).encode()])
        cli.send_stream_control(conn, 0x2D, raw_enabled=True)
        conn.masks.clear()
        original = cli.renew_raw_capture
        with patch.object(cli, 'renew_raw_capture', side_effect=lambda sock: original(sock, cli.RAW_LAST_RENEW_S + 5)) as renew:
            cli.receive_thread(conn)
        self.assertEqual(renew.call_count, 3)
        self.assertEqual(conn.masks, [0x7D] * 3)
        self.assertEqual(len(conn.recv_owners), 1)


class RawWebTests(unittest.TestCase):
    def test_opt_in_and_old_firmware_and_full_mask_renew(self):
        store = StateStore()
        old = FakeSocket(allowed=0x0F)
        link = EthernetRealtimeLink(store, raw_capture_enabled=True)
        link._send_stream_subscription(old)
        self.assertEqual(old.masks, [0x0D, 0x2D, 0x6D])
        self.assertEqual(old.active_mask, 0x0D)
        link._renew_raw_subscription(old, link._raw_last_renew_s + 4.99)
        self.assertEqual(len(old.masks), 3)
        link._renew_raw_subscription(old, link._raw_last_renew_s + 5)
        self.assertEqual(old.masks, [0x0D, 0x2D, 0x6D, 0x6D])
        default = FakeSocket()
        EthernetRealtimeLink(store)._send_stream_subscription(default)
        self.assertEqual(default.masks, [0x0D, DEFAULT_STREAM_MASK])

    def test_continuous_rx_renew_and_single_receiver_without_state_publication(self):
        store = StateStore()
        recorder = Mock()
        link = EthernetRealtimeLink(store, raw_capture_enabled=True, recorder=recorder)
        frame = raw_frame()
        conn = FakeSocket([frame[:20].encode(), frame[20:].encode(), frame.encode()])
        original = link._renew_raw_subscription
        with patch.object(link, '_renew_raw_subscription', side_effect=lambda sock: original(sock, link._raw_last_renew_s + 5)) as renew:
            self.assertEqual(link._receive_lines(conn, 'session'), 'peer_closed')
        self.assertEqual(renew.call_count, 4)
        self.assertEqual(conn.masks, [0x6D] * 4)
        self.assertEqual(len(conn.recv_owners), 1)
        self.assertEqual(recorder.record.call_count, 2)
        self.assertEqual(recorder.record.call_args.args[0], frame)
        self.assertTrue(recorder.record.call_args.kwargs['received_ns'] > 0)
        self.assertNotIn('ugraw', store.read()[1]['telemetry'])

    def test_actual_csv_events_metadata_and_bad_outer_retention(self):
        frame = raw_frame(b'$GNRMC,broken\x00\xff\r')
        invalid = frame.replace(',7,', ',9,', 1)
        partial = '$UGRAW,1,7'
        with tempfile.TemporaryDirectory() as temporary:
            recorder = SessionRecorder(Path(temporary))
            link = EthernetRealtimeLink(StateStore(), recorder=recorder, raw_capture_enabled=True)
            conn = FakeSocket([frame.encode() + invalid.encode(), partial.encode()])
            link._handle_connection(conn, ('test', 1))
            recorder.stop()
            state = recorder.snapshot()
            csv_path = Path(state['files']['ugraw'])
            self.assertRegex(csv_path.name, r'^uart7_raw_\d{8}_\d{6}\.csv$')
            with csv_path.open(encoding='utf-8', newline='') as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]['raw'], frame)
            self.assertEqual(bytes.fromhex(rows[0]['payload_hex']), b'$GNRMC,broken\x00\xff\r')
            self.assertTrue(int(rows[0]['t_ns']) > 0)
            events = [json.loads(line) for line in Path(state['files']['events']).read_text(encoding='utf-8').splitlines()]
            rx = [event for event in events if event['type'] == 'rx']
            self.assertEqual([event['raw'] for event in rx], [frame, invalid, partial])
            self.assertIsNone(rx[1]['row'])
            self.assertFalse(rx[2]['complete'])
            ended = events[-1]
            self.assertEqual(ended['type'], 'session.ended')
            self.assertEqual(ended['invalid_ugraw_count'], 2)
            self.assertEqual(ended['dropped_records'], 0)
            self.assertTrue(ended['complete'])
            self.assertEqual(conn.masks[-1], 0)

    def test_queue_loss_persisted_in_session_end_without_blocking_submit(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = SessionRecorder(Path(temporary), queue_size=1)
            recorder._begin('test', 'peer', 'now')
            self.assertTrue(recorder._submit(('line', 'first', None, 'now', 1)))
            self.assertFalse(recorder._submit(('line', 'lost', None, 'now', 2)))
            recorder._queue.get_nowait()
            recorder._queue.task_done()
            recorder._submit(('end', 'done', 'now'))
            recorder._stopping.set()
            recorder._run()
            state = recorder.snapshot()
            events = [json.loads(line) for line in Path(state['files']['events']).read_text(encoding='utf-8').splitlines()]
            self.assertEqual(events[-1]['dropped_records'], 1)
            self.assertEqual(events[-1]['status'], 'error')
            self.assertFalse(events[-1]['complete'])
            self.assertTrue(events[-1]['last_error'])

    def test_drop_before_begin_is_not_erased_and_next_session_is_independent(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = SessionRecorder(Path(temporary), queue_size=1)
            entered, release = threading.Event(), threading.Event()
            original = recorder._begin

            def delayed_begin(*args):
                entered.set()
                release.wait(2)
                original(*args)

            with patch.object(recorder, '_begin', side_effect=delayed_begin):
                recorder.begin_session('first', 'peer')
                self.assertTrue(entered.wait(1))
                recorder.record(raw_frame(), decode_telemetry_line(raw_frame()))
                recorder.record('lost-before-begin', None)
                release.set()
                recorder._queue.join()
                recorder.end_session('done')
                recorder._queue.join()
            first_events = Path(recorder.snapshot()['files']['events'])
            recorder.begin_session('second', 'peer')
            recorder.end_session('done')
            recorder.stop()
            first = json.loads(first_events.read_text(encoding='utf-8').splitlines()[-1])
            second = json.loads(Path(recorder.snapshot()['files']['events']).read_text(encoding='utf-8').splitlines()[-1])
            self.assertEqual(first['dropped_records'], 1)
            self.assertFalse(first['complete'])
            self.assertEqual(second['dropped_records'], 0)
            self.assertTrue(second['complete'])

    def test_full_queue_at_end_cannot_produce_successful_end_marker(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = SessionRecorder(Path(temporary), queue_size=1)
            entered, release = threading.Event(), threading.Event()
            original = recorder._begin

            def delayed_begin(*args):
                entered.set()
                release.wait(2)
                original(*args)

            with patch.object(recorder, '_begin', side_effect=delayed_begin):
                recorder.begin_session('first', 'peer')
                self.assertTrue(entered.wait(1))
                recorder.record('queued', None)
                recorder.end_session('queue-full')
                release.set()
                recorder.stop()
            state = recorder.snapshot()
            self.assertEqual(state['dropped_records'], 1)
            self.assertEqual(state['status'], 'error')
            events = [json.loads(line) for line in Path(state['files']['events']).read_text(encoding='utf-8').splitlines()]
            self.assertFalse(any(event['type'] == 'session.ended' for event in events))

    def test_disabled_recording_warns_and_env_requires_opt_in(self):
        with self.assertLogs('groundstation.ethernet_link', level='WARNING') as log:
            EthernetRealtimeLink(StateStore(), raw_capture_enabled=True)
        self.assertIn('不落盘', log.output[0])
        try:
            from groundstation.app import create_app
        except ImportError:
            self.skipTest('FastAPI需在station/.venv中验证')
        for value, expected in [('0', False), ('1', True)]:
            with patch.dict('os.environ', {'URGC_RAW_CAPTURE_ENABLED': value, 'URGC_RECORDING_ENABLED': '0'}):
                app = create_app(device_link_enabled=True)
                self.assertEqual(bool(app.state.device_link._stream_mask & ugcp.STREAM_RAW), expected)


if __name__ == '__main__':
    unittest.main()
