"""原始 GNSS 采集离线分析：合成日志、真实 codec 对照和命令行输出回归。"""
from __future__ import annotations

import csv
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parent
ROOT = TOOLS.parents[1]
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))
import analyze_gnss_capture as analysis


def checksum(body):
    value = 0
    for byte in body:
        value ^= byte
    return b"$" + body + b"*%02X" % value


def rmc(**changes):
    fields = ["GNRMC", "120000.000", "A", "3015.0000", "N", "12030.0000", "E", "", "", "061126", "", "", "A"]
    for index, value in changes.items():
        while int(index) >= len(fields):
            fields.append("")
        fields[int(index)] = value
    return checksum(",".join(fields).encode("ascii"))


def capture(payload=None, seq=1, rx_us=1000000, tx_us=None, capture_id=1,
            queue_drop=0, uart_drop=0, line_drop=0, flags=0, src_len=None):
    payload = payload if payload is not None else rmc()
    tx_us = rx_us + 10000 if tx_us is None else tx_us
    src_len = len(payload) if src_len is None else src_len
    body = ",".join(str(v) for v in (1,capture_id,seq,rx_us >> 32,rx_us & 0xFFFFFFFF,
        tx_us >> 32,tx_us & 0xFFFFFFFF,queue_drop,uart_drop,line_drop,flags,src_len,len(payload)))
    outer = checksum(("UGRAW," + body + "," + payload.hex()).encode("ascii")).decode("ascii")
    return {"raw": outer, "t_ns": rx_us * 1000 + 123, "host_iso": "2026-11-06T12:00:00+08:00"}


def decoded(*events):
    output = []
    for index, event in enumerate(events):
        row = analysis.ugraw_parse(event["raw"])
        if row is None:
            raise AssertionError("测试外包必须合法")
        output.append(dict(row, t_ns=event["t_ns"], pc_time=event["host_iso"], source_line=index + 1, source_file="synthetic"))
    return output


class CaptureAnalysisTests(unittest.TestCase):
    def setUp(self):
        base = ROOT.parent / ".buildtmp"
        base.mkdir(exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(prefix="gnss_analysis_", dir=base)
        self.addCleanup(self.tmp.cleanup)
        self.work = Path(self.tmp.name)

    def write_csv(self, events, name="telemetry_ugraw.csv"):
        path = self.work / name
        with path.open("w", encoding="utf-8-sig", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["t_ns", "host_iso", "raw"])
            writer.writeheader()
            writer.writerows(events)
        return path

    def write_events(self, events, name="events.jsonl"):
        path = self.work / name
        path.write_text("".join(json.dumps(event) + "\n" for event in events), encoding="utf-8")
        return path

    def test_precise_builder_rules_and_other_talker_visibility(self):
        self.assertEqual(analysis.classify_payload(rmc())["firmware_status"], "position")
        for utc in ("120000", "120000.1", "120000.12", "235959.999"):
            self.assertEqual(analysis.classify_payload(rmc(**{"1": utc}))["firmware_status"], "position")
        for utc in ("120000.", "120000.0001", "120060", "240000", "12000"):
            self.assertEqual(analysis.classify_payload(rmc(**{"1": utc}))["firmware_status"], "rejected")
        self.assertEqual(analysis.classify_payload(rmc(**{"9": "290200"}))["utc_date"], 20000229)
        self.assertEqual(analysis.classify_payload(rmc(**{"9": "290226"}))["reason"], "date_range")
        row = analysis.classify_payload(rmc(**{"0": "GPRMC"}))
        self.assertEqual((row["rmc_status"], row["firmware_status"]), ("position", "unsupported_talker"))
        row = analysis.classify_payload(rmc(**{"3": "3015.123456789", "4": "S", "6": "W"}))
        self.assertEqual((row["lat_deg_e7"], row["lon_deg_e7"]), (-302520576, -1205000000))

    def test_no_fix_modes_and_bad_optional_velocity_keep_position(self):
        for changes in ({"2": "V", "3": "", "4": "", "5": "", "6": ""}, {"12": "N"}, {"13": "V"}):
            row = analysis.classify_payload(rmc(**changes))
            self.assertEqual(row["firmware_status"], "no_fix")
            self.assertNotIn("lat_deg_e7", row)
        for mode in ("E", "M", "S", "X", "AA"):
            self.assertEqual(analysis.classify_payload(rmc(**{"12": mode}))["reason"], "mode")
        # builder 按字段顺序检查：A/E 不能靠后面的 nav-status V 变为合法。
        self.assertEqual(analysis.classify_payload(rmc(**{"12": "E", "13": "V"}))["reason"], "mode")
        for speed, course in (("", ""), ("NaN", "90"), ("1e2", "0"), ("0.5831", "90"), ("2", "360")):
            row = analysis.classify_payload(rmc(**{"7": speed, "8": course}))
            self.assertEqual(row["firmware_status"], "position")
            self.assertFalse(row["velocity_valid"])
        row = analysis.classify_payload(rmc(**{"7": "2", "8": "90"}))
        self.assertTrue(row["velocity_valid"])
        self.assertAlmostEqual(row["vel_e_mps"], 1.0288889, places=6)
        self.assertTrue(analysis.classify_payload(rmc(**{"7": "0.5832", "8": "90"}))["velocity_valid"])
        self.assertEqual(row["quality_source"], "unknown")

    def test_arrival_rate_and_new_epochs_differ_with_burst_duplicate_no_fix(self):
        events = [capture(rmc(**{"1": "120000"}), seq=1, rx_us=1000000),
                  capture(rmc(**{"1": "120000"}), seq=2, rx_us=2000000),
                  capture(rmc(**{"1": "120001"}), seq=3, rx_us=2000000),
                  capture(rmc(**{"1": "120002", "2": "V"}), seq=4, rx_us=3000000),
                  capture(rmc(**{"1": "120003"}), seq=5, rx_us=10000000)]
        summary, rows = analysis.analyze_records(decoded(*events))
        self.assertEqual(summary["counts"]["duplicate"], 1)
        self.assertEqual(summary["counts"]["new_position"], 3)
        self.assertEqual(summary["counts"]["new_no_fix"], 1)
        self.assertEqual(summary["timing"]["rmc_rx"]["zero_intervals"], 1)
        self.assertAlmostEqual(summary["timing"]["rmc_rx"]["observed_rate_hz"], 4 / 9)
        self.assertAlmostEqual(summary["timing"]["fix_rx"]["observed_rate_hz"], 2 / 9)
        self.assertEqual(summary["timing"]["epochs"]["observed_rate_hz"], 1)
        self.assertEqual(summary["timing"]["rmc_rx"]["gap_count"], 1)
        self.assertEqual(rows[2]["epoch_disposition"], "new")
        self.assertEqual(summary["queue_delay_ms"]["median"], 10)
        self.assertEqual(summary["capture_integrity"], "unknown")

    def test_checksum_reject_does_not_consume_utc_and_no_fix_does(self):
        line = rmc(**{"1": "120000"})
        bad = line[:-2] + b"ZZ"
        summary, rows = analysis.analyze_records(decoded(capture(bad), capture(line,seq=2),
            capture(rmc(**{"1": "120001", "2": "V"}),seq=3),
            capture(rmc(**{"1": "120001"}),seq=4),capture(rmc(**{"1": "115959"}),seq=5)))
        self.assertEqual(summary["counts"]["new_position"], 1)
        self.assertEqual(summary["counts"]["new_no_fix"], 1)
        self.assertEqual(summary["counts"]["duplicate"], 1)
        self.assertEqual(summary["counts"]["out_of_order"], 1)
        self.assertEqual(rows[1]["epoch_disposition"], "new")
        self.assertIsNone(summary["timing"]["raw_rx"]["observed_rate_hz"])

    def test_midnight_uses_real_calendar_and_mcu_uint64_does_not_wrap(self):
        high = (1 << 32) * 1000
        events = [capture(rmc(**{"1": "235959.999", "9": "311226"}),rx_us=high-1000),
                  capture(rmc(**{"1": "000000.000", "9": "010127"}),seq=2,rx_us=high)]
        summary, rows = analysis.analyze_records(decoded(*events))
        self.assertEqual(len(summary["segments"]), 1)
        self.assertAlmostEqual(summary["timing"]["epochs"]["intervals_s"]["median"], .001)
        self.assertEqual(rows[1]["epoch_iso"], "2027-01-01T00:00:00.000Z")
        self.assertEqual(summary["counts"]["new_position"], 2)

    def test_capture_change_boot_restart_sequence_wrap_and_loss_are_separate(self):
        events = [capture(seq=0xFFFFFFFF,rx_us=1000000,queue_drop=0xFFFFFFFF),
                  capture(seq=0,rx_us=2000000,queue_drop=0),
                  capture(seq=3,rx_us=3000000,queue_drop=2,uart_drop=7,line_drop=1),
                  capture(seq=1,rx_us=4000000,capture_id=2),
                  capture(seq=1,rx_us=100000,capture_id=2),
                  capture(seq=0,rx_us=200000,capture_id=2)]
        summary, _ = analysis.analyze_records(decoded(*events))
        self.assertEqual([s["boundary"] for s in summary["segments"]],
                         ["start", "capture_changed", "mcu_rx_backwards", "sequence_backwards"])
        self.assertEqual(summary["loss"]["sequence_missing"], 2)
        self.assertEqual(summary["loss"]["queue_drop_increase"], 3)
        self.assertEqual(summary["loss"]["uart_drop_increase"], 7)
        self.assertEqual(summary["loss"]["line_drop_increase"], 1)
        self.assertEqual(summary["counts"]["new_position"], 4)

    def test_raw_bytes_cr_nul_auxiliary_and_truncated_are_not_invented_fixes(self):
        payload = rmc() + b"\r\x00\xffsuffix"
        truncated = b"$GNRMC," + b"X" * 377
        events = [capture(payload), capture(checksum(b"GNGGA,120000,3015,N,12030,E,1,9,0.9,1,M"),seq=2),
                  capture(checksum(b"GNGST,120000,1,2,3,4,5,6,7"),seq=3),
                  capture(truncated,seq=4,flags=1,src_len=600)]
        summary, rows = analysis.analyze_records(decoded(*events))
        self.assertEqual(summary["counts"]["new_position"], 1)
        self.assertEqual(rows[0]["payload_hex"], payload.hex())
        self.assertEqual(rows[0]["normalization"], "cr_removed;nul_terminated")
        self.assertEqual(rows[3]["firmware_status"], "unknown")
        self.assertIn('"0.9"', rows[1]["fields_json"])
        self.assertEqual(summary["quality_source"], "unknown_RMC_no_accuracy_report")
        self.assertEqual(summary["capture_integrity"], "issues_observed")

    def test_static_is_opt_in_uses_unique_positions_and_handles_dateline(self):
        events = [capture(rmc(**{"3": "0000.0000", "5": "17959.9999"}),seq=1),
                  capture(rmc(**{"1": "120001", "3": "0000.0000", "5": "17959.9999", "6": "W"}),seq=2,rx_us=2000000)]
        records = decoded(*events)
        summary, _ = analysis.analyze_records(records)
        self.assertIsNone(summary["static_repeatability"])
        summary, _ = analysis.analyze_records(records, static=True)
        spread = summary["static_repeatability"]
        self.assertEqual(spread["samples"], 2)
        self.assertLess(spread["radius_about_median_m"]["max"], .3)
        self.assertIn("非绝对精度", spread["meaning"])

    def test_directory_csv_priority_audits_bad_outer_and_recorder_sidecar(self):
        event = capture()
        selected = self.write_csv([event], "uart7_raw_20261106_120000.csv")
        bad = dict(capture(seq=2), topic="ugraw")
        bad["raw"] = bad["raw"][:-2] + "ZZ"
        self.write_events([dict(event, kind="rx", topic="ugraw"), bad])
        (self.work / "raw.log").write_text(event["raw"] + "\n",encoding="utf-8")
        sidecar = {"complete": True, "status": "stopped", "dropped_records": 2,
                   "rejected_records": 0, "pending_records": 0, "last_error": ""}
        (self.work / "session.json").write_text(json.dumps(sidecar),encoding="utf-8")
        records, metadata = analysis.load_capture(self.work)
        self.assertEqual(len(records), 1)
        self.assertEqual(metadata["selected_sources"], [str(selected.resolve())])
        self.assertEqual(metadata["invalid_outer_audit"], 1)
        summary, _ = analysis.analyze_records(records,metadata)
        self.assertEqual(summary["capture_integrity"], "issues_observed")
        self.assertEqual(summary["source"]["pc_recorder"]["dropped_records"], 2)

    def test_web_end_event_and_no_end_unknown_and_partial_frame(self):
        event = dict(capture(), type="rx",kind="rx",topic="ugraw",complete=True)
        source = self.write_events([event])
        records, metadata = analysis.load_capture(source)
        self.assertEqual(analysis.analyze_records(records,metadata)[0]["capture_integrity"], "unknown")
        end = {"type": "session.ended", "complete": True,"dropped_records":0,"invalid_ugraw_count":0,
               "pending_records":0,"last_error":"","status":"idle"}
        self.write_events([event,end])
        records, metadata = analysis.load_capture(source)
        self.assertEqual(analysis.analyze_records(records,metadata)[0]["capture_integrity"],
                         "no_observed_issues_recorder_reports_complete")
        self.write_events([dict(event,complete=False),dict(end,invalid_ugraw_count=1)])
        records, metadata = analysis.load_capture(source)
        self.assertEqual(len(records), 0)
        self.assertEqual(metadata["invalid_outer_primary"], 1)
        self.assertEqual(analysis.analyze_records(records,metadata)[0]["capture_integrity"], "issues_observed")

    def test_rawlog_json_container_errors_and_duplicate_transport_record(self):
        event = capture()
        raw = self.work / "raw.log"
        raw.write_text("2026-11-06T12:00:00Z\t" + event["raw"] + "\n" + event["raw"] + "\n",encoding="utf-8")
        records, metadata = analysis.load_capture(raw)
        summary, rows = analysis.analyze_records(records,metadata)
        self.assertEqual(len(rows), 1)
        self.assertEqual(summary["duplicate_capture_records"], 1)
        source = self.write_events([dict(event,kind="rx",topic="ugraw")])
        with source.open("a",encoding="utf-8") as handle:
            handle.write('{"raw":\n')
        records, metadata = analysis.load_capture(source)
        self.assertEqual(len(metadata["container_errors"]), 1)
        self.assertEqual(analysis.analyze_records(records,metadata)[0]["capture_integrity"], "issues_observed")

    def test_csv_blank_schema_preserves_receiver_rejection_and_empty_uart_is_valid(self):
        path = self.work / "telemetry_ugraw.csv"
        with path.open("w",encoding="utf-8",newline="") as handle:
            writer = csv.DictWriter(handle,fieldnames=["schema","raw"])
            writer.writeheader()
            writer.writerow({"schema":"", "raw":capture()["raw"]})
            writer.writerow({"schema":"1", "raw":capture(b"",seq=2)["raw"]})
        records, metadata = analysis.load_capture(path)
        self.assertEqual(len(records),1)
        self.assertEqual(records[0]["kept_len"],0)
        self.assertEqual(metadata["invalid_outer_primary"],1)

    def test_rawlog_extra_carriage_return_is_not_repaired_into_valid_outer(self):
        path = self.work / "raw.log"
        valid = capture()["raw"]
        path.write_bytes(("time\t" + valid + "\r\n" + "time\t" + valid + "\r\r\n").encode("ascii"))
        records, metadata = analysis.load_capture(path)
        self.assertEqual(len(records),1)
        self.assertEqual(records[0]["raw"],valid + "\r\n")
        self.assertEqual(metadata["invalid_outer_primary"],1)

    def test_csv_false_complete_cannot_reaccept_checksum_complete_fragment(self):
        path = self.work / "telemetry_ugraw.csv"
        with path.open("w",encoding="utf-8",newline="") as handle:
            writer = csv.DictWriter(handle,fieldnames=["schema","complete","raw"])
            writer.writeheader()
            for seq, complete in enumerate(("False", "0", " false ", "True", ""),1):
                # 假设解析列仍有schema=1，也不能盖过接收者明确给出的残片标记。
                writer.writerow({"schema":"1", "complete":complete,"raw":capture(seq=seq)["raw"]})
        records, metadata = analysis.load_capture(path)
        self.assertEqual([row["seq"] for row in records],[4,5])
        self.assertEqual(metadata["invalid_outer_primary"],3)
        self.assertTrue(analysis.receiver_incomplete({"complete":False}))
        self.assertFalse(analysis.receiver_incomplete({}))

    def test_empty_single_and_bad_queue_clock_have_explicit_unknowns(self):
        summary, _ = analysis.analyze_records([])
        self.assertIsNone(summary["valid_position_fraction_all_rmc"])
        self.assertIsNone(summary["timing"]["fix_rx"]["observed_rate_hz"])
        summary, _ = analysis.analyze_records(decoded(capture(tx_us=900000)),static=True)
        self.assertEqual(summary["loss"]["tx_before_rx"], 1)
        self.assertEqual(summary["queue_delay_ms"]["count"], 0)
        self.assertIsNone(summary["static_repeatability"]["north_sample_std_m"])

    def test_initial_loss_counters_cannot_claim_loss_free_capture(self):
        meta = {"pc_recorder": {"complete": True, "dropped_records": 0}}
        summary, _ = analysis.analyze_records(decoded(capture(queue_drop=2)), meta)
        self.assertEqual(summary["capture_integrity"], "issues_observed")
        self.assertEqual(summary["loss"]["queue_drop_increase"], 0)
        self.assertEqual(summary["segments"][0]["first_counters"]["queue_drop"], 2)
        summary, _ = analysis.analyze_records(decoded(capture(uart_drop=9,line_drop=3)), meta)
        self.assertEqual(summary["capture_integrity"], "history_loss_observed_interval_unknown")
        self.assertEqual(summary["loss"]["uart_drop_increase"], 0)

    def test_no_fix_recovery_duration_duplicate_and_unrecovered_segment_boundary(self):
        events = [capture(rmc(**{"1":"120000"}),seq=1,rx_us=1000000),
                  capture(rmc(**{"1":"120001","2":"V"}),seq=2,rx_us=2000000),
                  capture(rmc(**{"1":"120001","2":"V"}),seq=3,rx_us=3000000),
                  capture(rmc(**{"1":"120002","2":"V"}),seq=4,rx_us=4000000),
                  capture(rmc(**{"1":"120003"}),seq=5,rx_us=10000000),
                  capture(rmc(**{"1":"120004","2":"V"}),seq=6,rx_us=11000000),
                  capture(rmc(**{"1":"120005","2":"V"}),seq=1,rx_us=1000000),
                  capture(rmc(**{"1":"120006","2":"V"}),seq=2,rx_us=2000000)]
        summary, _ = analysis.analyze_records(decoded(*events))
        first, boundary, tail = summary["no_fix_recoveries"]
        self.assertTrue(first["recovered"])
        self.assertEqual(first["duration_s"],8)
        self.assertEqual(first["no_fix_epochs"],2)
        self.assertFalse(boundary["recovered"])
        self.assertEqual(boundary["end_reason"],"mcu_rx_backwards")
        self.assertFalse(tail["recovered"])
        self.assertEqual(tail["end_reason"],"end_of_capture")
        self.assertEqual(tail["observed_duration_s"],1)
        self.assertEqual(summary["position_gaps"][0]["gap_s"],9)

    def test_cli_outputs_and_refuses_overwriting_input_or_previous_results(self):
        source = self.write_csv([capture(),capture(rmc(**{"1":"120001"}),seq=2,rx_us=2000000)])
        before = source.read_bytes()
        command = [sys.executable,"-B",str(TOOLS/"analyze_gnss_capture.py"),str(self.work),"--static"]
        result = subprocess.run(command,capture_output=True,text=True,encoding="utf-8",errors="replace")
        self.assertEqual(result.returncode,0,result.stderr)
        summary = json.loads((self.work/"gnss_analysis/summary.json").read_text(encoding="utf-8"))
        self.assertEqual(summary["counts"]["new_position"],2)
        with (self.work/"gnss_analysis/epochs.csv").open(encoding="utf-8-sig",newline="") as handle:
            self.assertEqual(len(list(csv.DictReader(handle))),2)
        self.assertEqual(source.read_bytes(),before)
        again = subprocess.run(command,capture_output=True)
        self.assertNotEqual(again.returncode,0)
        records, metadata = analysis.load_capture(source)
        summary, rows = analysis.analyze_records(records,metadata)
        with self.assertRaisesRegex(ValueError,"覆盖输入"):
            analysis.write_outputs(self.work,self.work,summary,rows)


class ActualCodecAgreementTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc") or ("D:/msys64/mingw64/bin/gcc.exe" if Path("D:/msys64/mingw64/bin/gcc.exe").is_file() else None)
        if compiler is None:
            raise unittest.SkipTest("实际 C codec 对照需要主机 gcc；其余纯 Python 测试不受影响")
        base = ROOT.parent / ".buildtmp"
        base.mkdir(exist_ok=True)
        cls.tmp = tempfile.TemporaryDirectory(prefix="gnss_analysis_codec_",dir=base)
        cls.addClassCleanup(cls.tmp.cleanup)
        work = Path(cls.tmp.name)
        fixture = work / "codec.c"
        fixture.write_text('''#include "glider_rdss_codec.h"
#include <stdio.h>
int main(int argc,char **argv) {
  GliderRdssDecodedFrame f; GnssSample s;
  if(argc!=2) return 2;
  int ok=GliderRdssCodec_TryDecodeLine(argv[1],&f) && GliderRdssCodec_TryMakeGnssSample(&f,1000,1,&s);
  if(!ok) puts("0");
  else printf("1 %lu %lu %lu %ld %ld\\n",(unsigned long)s.validity_flags,
    (unsigned long)s.utc_date_yyyymmdd,(unsigned long)s.utc_ms_of_day,
    (long)s.lat_deg_e7,(long)s.lon_deg_e7);
  return 0;
}
''',encoding="utf-8")
        cls.env = dict(os.environ)
        cls.env.update(TMP=str(work),TEMP=str(work),TMPDIR=str(work))
        cls.env["PATH"] = str(Path(compiler).parent) + os.pathsep + cls.env.get("PATH","")
        cls.binary = work / ("codec.exe" if os.name == "nt" else "codec")
        result = subprocess.run([compiler,"-std=c11","-O1","-I",str(ROOT/"software/Glider-Core"),
            "-I",str(ROOT/"software/Glider-App/common"),str(fixture),
            str(ROOT/"software/Glider-Core/glider_rdss_codec.c"),"-lm","-o",str(cls.binary)],
            env=cls.env,capture_output=True,text=True,encoding="utf-8",errors="replace")
        if result.returncode:
            raise RuntimeError(result.stderr)

    def test_python_classification_matches_actual_c_builder_boundary_matrix(self):
        cases = [rmc()]
        for field, values in {"0":["GPRMC","GNGGA"],"1":["120000","120000.1","120000.12","120000.123","120000.1234","120060"],
            "2":["V","AA","a"],"3":["3015.123456789","3015.1234567890","9000.0","9000.1","3060.0"],
            "4":["S","NN"],"7":["0.5831","0.5832","NaN","1e2","999999999.999999999"],
            "9":["290200","290226","311226"],"12":["","A","D","F","R","P","N","E","M","S"],
            "13":["S","V","C","U"],"14":["","extra"]}.items():
            cases.extend(rmc(**{field:value}) for value in values)
        cases.extend([rmc(**{"2":"V","12":"E","13":"C"}),rmc(**{"12":"E","13":"V"})])
        cases.extend([rmc().split(b"*")[0],rmc()[:-2]+b"ZZ",rmc()+b"X"])
        for payload in cases:
            with self.subTest(payload=payload):
                run = subprocess.run([str(self.binary),payload.decode("ascii")],env=self.env,capture_output=True,text=True)
                self.assertEqual(run.returncode,0,run.stderr)
                actual = [int(v) for v in run.stdout.split()]
                row = analysis.classify_payload(payload)
                accepted = row["firmware_status"] in ("position","no_fix")
                self.assertEqual(accepted,bool(actual[0]))
                if accepted:
                    flags = 73 if row["firmware_status"] == "no_fix" else 31 | (32 if row["velocity_valid"] else 0)
                    self.assertEqual(actual[1:],[flags,row["utc_date"],row["utc_ms"],row.get("lat_deg_e7",0),row.get("lon_deg_e7",0)])


if __name__ == "__main__":
    unittest.main()
