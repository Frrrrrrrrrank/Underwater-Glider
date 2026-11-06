#!/usr/bin/env python3
"""在内存执行真实前端字段规则，覆盖估计就绪、PID 反馈与双轴参考。"""

import json
import shutil
import subprocess
import unittest
from pathlib import Path


FRONTEND_DIR = Path(__file__).resolve().parents[1] / "frontend"
NODE_RUNNER = r"""
const fs = require('node:fs');
const Module = require('node:module');
const assert = require('node:assert/strict');
const esbuild = require(process.argv[1]);
const filename = process.argv[2];
const result = esbuild.buildSync({entryPoints: [filename], bundle: true, write: false,
  platform: 'node', format: 'cjs', target: 'node18', logLevel: 'silent'});
const compiled = new Module(filename);
compiled._compile(result.outputFiles[0].text, filename);
const payload = JSON.parse(fs.readFileSync(0, 'utf8'));
new Function('api', 'fixture', 'assert', payload.script)(compiled.exports, payload.fixture, assert);
"""


class FrontendFieldQualityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = shutil.which("node")
        cls.esbuild = FRONTEND_DIR / "node_modules" / "esbuild"
        if not cls.node or not (cls.esbuild / "package.json").is_file():
            raise unittest.SkipTest("前端测试需要 Node 与已有 npm ci 依赖；不会自动联网安装")
        cls.fixture = {
            "fresh": True,
            "age_ms": 20,
            "invalid_fields": {},
            "data": {
                "fl": 2, "mode": 1, "armed": 1, "output_valid": 1, "fault": 0,
                "depth": 0, "pitch_deg": 0, "displacement_mm": 0,
                "depth_error": 0, "pitch_error_deg": 0, "displacement_error_mm": 0,
                "pitch_sp_deg": 0, "target_displacement_mm": 0,
                "target_depth": 3, "target_pitch_deg": -10,
                "ref_kind": 2, "span_valid": 1, "data_fresh": 1,
                "stroke_percent": 25, "mm_from_front": 14.7, "pulse": 70330,
                "roll_percent": 50, "roll_valid": 1,
            },
        }

    def run_frontend(self, script):
        # 仅转换内存中的 TS；不写 dist、缓存或临时文件。
        result = subprocess.run(
            [self.node, "-e", NODE_RUNNER, str(self.esbuild), str(FRONTEND_DIR / "src" / "domain" / "fields.ts")],
            input=json.dumps({"fixture": self.fixture, "script": script}),
            text=True, encoding="utf-8", capture_output=True, timeout=20,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_estimate_ready_uses_only_fresh_estimate_flags(self):
        self.run_frontend("""
for (const flags of [2, 3, 0x80000002, 0xffffffff]) {
  assert.equal(api.estimateReady({ fresh: true, data: { fl: flags } }), true);
}
for (const flags of [0, 1, 4, 0xfffffffd]) {
  assert.equal(api.estimateReady({ fresh: true, data: { fl: flags } }), false);
}
assert.equal(api.estimateReady(undefined), false);
assert.equal(api.estimateReady({ fresh: true, data: {} }), false);
""")

    def test_estimate_ready_rejects_invalid_flag_types_and_uint32_overflow(self):
        self.run_frontend("""
for (const flags of [undefined, null, '2', true, 2.5, -2, NaN, Infinity, -Infinity, 0x100000002]) {
  fixture.data.fl = flags;
  assert.equal(api.estimateReady(fixture), false, String(flags));
  assert.equal(api.numeric(fixture, 'ugekf', 'pitch_deg'), null, String(flags));
}
""")

    def test_estimate_ready_rejects_stale_or_invalid_flag_field(self):
        self.run_frontend("""
fixture.fresh = false;
assert.equal(api.estimateReady(fixture), false);
fixture.fresh = true;
fixture.invalid_fields.fl = 'non_finite';
assert.equal(api.estimateReady(fixture), false);
""")

    def test_pid_off_hides_computed_feedback_without_rewriting_raw_values(self):
        self.run_frontend("""
fixture.data.mode = 0;
fixture.data.armed = 0;
fixture.data.output_valid = 0;
fixture.data.fault = 1;
const original = JSON.stringify(fixture);
for (const key of ['depth', 'pitch_deg', 'displacement_mm', 'depth_error', 'pitch_error_deg',
  'displacement_error_mm', 'pitch_sp_deg', 'target_displacement_mm']) {
  assert.equal(api.fieldIssue(fixture, 'ugctl', key), 'PID 已关闭', key);
  assert.equal(api.numeric(fixture, 'ugctl', key), null, key);
  assert.notEqual(api.formatField('ugctl', key, fixture.data[key]), '—', key);
}
for (const key of ['mode', 'target_depth', 'target_pitch_deg']) {
  assert.equal(api.fieldIssue(fixture, 'ugctl', key), null, key);
}
assert.equal(JSON.stringify(fixture), original);
""")

    def test_unknown_pid_mode_hides_computed_feedback_but_keeps_requests(self):
        self.run_frontend("""
for (const mode of [undefined, null, '1', -1, 3]) {
  fixture.data.mode = mode;
  fixture.data.armed = 0;
  fixture.data.output_valid = 0;
  fixture.data.fault = 1;
  for (const key of ['depth', 'pitch_deg', 'displacement_mm', 'depth_error', 'pitch_error_deg',
    'displacement_error_mm', 'pitch_sp_deg', 'target_displacement_mm']) {
    assert.equal(api.fieldIssue(fixture, 'ugctl', key), 'PID 模式未知', key);
  }
  assert.equal(api.fieldIssue(fixture, 'ugctl', 'target_depth'), null);
  assert.equal(api.fieldIssue(fixture, 'ugctl', 'target_pitch_deg'), null);
}
""")

    def test_active_pid_modes_allow_valid_computed_values_including_zero(self):
        self.run_frontend("""
for (const mode of [1, 2]) {
  fixture.data.mode = mode;
  for (const key of ['depth', 'pitch_deg', 'displacement_mm', 'depth_error', 'pitch_error_deg',
    'displacement_error_mm', 'pitch_sp_deg', 'target_displacement_mm']) {
    assert.equal(api.fieldIssue(fixture, 'ugctl', key), null, key);
    assert.equal(api.numeric(fixture, 'ugctl', key), 0, key);
  }
}
""")

    def test_inactive_or_faulted_pid_feedback_is_hidden_without_changing_raw_values(self):
        self.run_frontend("""
for (const mode of [1, 2]) {
  for (const [flag, values] of [
    ['armed', [undefined, null, 0, '1']],
    ['output_valid', [undefined, null, 0, '1']],
    ['fault', [undefined, null, 1, 2, 3, 4, 5, 6, 7, 8, 9, '0']],
  ]) {
    for (const value of values) {
      const changed = structuredClone(fixture);
      changed.data.mode = mode;
      if (value === undefined) delete changed.data[flag];
      else changed.data[flag] = value;
      const original = JSON.stringify(changed);
      for (const key of ['depth', 'pitch_deg', 'displacement_mm', 'depth_error', 'pitch_error_deg',
        'displacement_error_mm', 'pitch_sp_deg', 'target_displacement_mm']) {
        assert.equal(api.fieldIssue(changed, 'ugctl', key), 'PID 反馈未更新', flag + '=' + value + ', ' + key);
        assert.equal(api.numeric(changed, 'ugctl', key), null, key);
        assert.notEqual(api.formatField('ugctl', key, changed.data[key]), '—', key);
      }
      assert.equal(api.fieldIssue(changed, 'ugctl', 'target_depth'), null);
      assert.equal(api.fieldIssue(changed, 'ugctl', 'target_pitch_deg'), null);
      assert.equal(JSON.stringify(changed), original);
    }
  }
}
""")

    def test_observed_pitch_range_keeps_raw_pulses_and_independent_roll(self):
        self.run_frontend("""
fixture.data.ref_kind = 1;
const original = JSON.stringify(fixture);
for (const key of ['stroke_percent', 'mm_from_front']) {
  assert.equal(api.fieldIssue(fixture, 'ugwrm', key), '行程未校准', key);
  assert.equal(api.numeric(fixture, 'ugwrm', key), null, key);
}
assert.equal(api.numeric(fixture, 'ugwrm', 'pulse'), 70330);
assert.equal(api.numeric(fixture, 'ugwrm', 'roll_percent'), 50);
assert.equal(JSON.stringify(fixture), original);
""")

    def test_calibrated_pitch_position_requires_span_feedback_and_fresh_frame(self):
        self.run_frontend("""
for (const key of ['stroke_percent', 'mm_from_front']) {
  assert.equal(api.fieldIssue(fixture, 'ugwrm', key), null, key);
  for (const [flag, values] of [
    ['ref_kind', [undefined, null, 0, 1, '2']],
    ['span_valid', [undefined, null, 0, '1']],
    ['data_fresh', [undefined, null, 0, '1']],
  ]) {
    for (const value of values) {
      const changed = structuredClone(fixture);
      changed.data[flag] = value;
      assert.notEqual(api.fieldIssue(changed, 'ugwrm', key), null, flag + '=' + value);
    }
  }
  const stale = structuredClone(fixture); stale.fresh = false;
  assert.equal(api.fieldIssue(stale, 'ugwrm', key), '数据过期');
}
""")

    def test_roll_validity_does_not_depend_on_pitch_driver_feedback(self):
        self.run_frontend("""
fixture.data.ref_kind = 1;
fixture.data.span_valid = 0;
fixture.data.data_fresh = 0;
assert.equal(api.numeric(fixture, 'ugwrm', 'roll_percent'), 50);
assert.equal(api.numeric(fixture, 'ugwrm', 'pulse'), null);
fixture.data.roll_valid = 0;
assert.equal(api.numeric(fixture, 'ugwrm', 'roll_percent'), null);
fixture.data.roll_valid = 1;
fixture.fresh = false;
assert.equal(api.numeric(fixture, 'ugwrm', 'roll_percent'), null);
""")

    def test_operator_labels_distinguish_commands_and_uncalibrated_range(self):
        self.run_frontend("""
assert.equal(api.definition('ugctl', 'mode').label, 'PID 模式');
assert.equal(api.definition('ugctl', 'pump_enable').label, '泵启停指令');
assert.equal(api.definition('ugctl', 'pump_direction').label, '泵方向指令');
assert.equal(api.definition('ugctl', 'thrust_pct').label, '推进指令');
assert.equal(api.formatField('ugwrm', 'ref_kind', 1), '未校准范围');
assert.equal(api.formatField('ugwrm', 'ref_kind', 2), '校准参考');
""")


if __name__ == "__main__":
    unittest.main()
