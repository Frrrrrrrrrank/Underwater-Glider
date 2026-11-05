#!/usr/bin/env python3
"""直接执行前端 TypeScript 的输入质量规则；无浏览器、真实手柄或板端连接。"""

import json
import shutil
import subprocess
import sys
import unittest
from pathlib import Path


STATION_DIR = Path(__file__).resolve().parents[1]
FRONTEND_DIR = STATION_DIR / "frontend"
sys.path.insert(0, str(STATION_DIR))

from gamepad_control import GamepadSnapshot  # noqa: E402
from groundstation.gamepad_preview import GamepadPreview  # noqa: E402


class FixtureService:
    """复用真实 Python JSON 契约，避免前端测试手工维护第二份消息定义。"""

    def __init__(self, config):
        self.config = config

    def start(self):
        pass

    def stop(self):
        pass

    def is_alive(self):
        return True

    def snapshot(self, index):
        values = dict(forward=1.0, button_y=True, right_trigger=1.0) if index == 0 else {}
        return GamepadSnapshot(index=index, connected=True, sample_time=100.0, **values)


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


class FrontendGamepadQualityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = shutil.which("node")
        cls.esbuild = FRONTEND_DIR / "node_modules" / "esbuild"
        if not cls.node or not (cls.esbuild / "package.json").is_file():
            raise unittest.SkipTest("前端测试需要 Node 与已有 npm ci 依赖；不会自动联网安装")
        preview = GamepadPreview(service_factory=FixtureService, clock=lambda: 100.0)
        preview.start()
        cls.fixture = preview.snapshot()
        preview.stop()

    def run_frontend(self, script):
        # esbuild 仅在内存转换 TS；不生成 dist、缓存或临时文件。
        result = subprocess.run(
            [self.node, "-e", NODE_RUNNER, str(self.esbuild), str(FRONTEND_DIR / "src" / "domain" / "gamepad.ts")],
            input=json.dumps({"fixture": self.fixture, "script": script}),
            text=True, encoding="utf-8", capture_output=True, timeout=20,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_real_backend_contract_and_zero_inputs_are_available(self):
        self.run_frontend("""
const parsed = api.parseGamepadPreview(fixture);
assert.ok(parsed);
const shown = api.ageGamepadPreview(parsed, 0, true);
assert.equal(shown.slots[0].intent.pitch_rpm, -100);
assert.equal(shown.slots[0].intent.rudder_rate_deg_s, 30);
assert.equal(shown.slots[0].intent.left_percent, 100);
assert.equal(shown.slots[0].neutral, false);
assert.equal(shown.slots[1].intent.pitch_rpm, 0);
assert.equal(shown.slots[1].sample.forward, 0);
assert.equal(shown.slots[1].neutral, true);
assert.equal(shown.slots[1].fresh, true);
""")

    def test_missing_and_incompatible_previews_are_rejected(self):
        self.run_frontend("""
for (const value of [undefined, null, false, [], {}, 'preview']) assert.equal(api.parseGamepadPreview(value), null);
const corruptions = [
  f => { f.schema_version = 2; }, f => { f.mode = 'control'; }, f => { f.source = 'browser'; },
  f => { delete f.config; }, f => { f.slots.pop(); }, f => { f.slots[1].index = 0; },
  f => { f.slots[0].index = 1.5; }, f => { f.slots[0].sample.forward = '1'; },
  f => { delete f.slots[0].sample.button_y; }, f => { f.slots[0].sample.forward = 1.01; },
  f => { f.slots[0].sample.left_trigger = -0.1; }, f => { f.slots[0].sample.loss_seq = -1; },
  f => { f.slots[0].intent.pitch_rpm = -101; }, f => { f.slots[0].intent.left_percent = 101; },
  f => { f.slots[0].intent.rudder_rate_deg_s = 31; }, f => { f.config.deadzone = 1; },
  f => { f.config.input_timeout_s = 0; }, f => { f.config.motor1_forward_dir = 2; },
  f => { f.slots[0].connected = false; }, f => { f.slots[0].age_ms = null; },
  f => { f.slots[0].sample = null; }, f => { f.slots[0].intent = null; },
  f => { f.slots[0].neutral = null; }, f => { f.slots[0].status = 'stale'; },
];
for (const corrupt of corruptions) {
  const value = structuredClone(fixture); corrupt(value);
  assert.equal(api.parseGamepadPreview(value), null, corrupt.toString());
}
""")

    def test_non_finite_fields_are_rejected(self):
        self.run_frontend("""
for (const bad of [NaN, Infinity, -Infinity]) {
  for (const corrupt of [
    f => { f.config.input_timeout_s = bad; }, f => { f.slots[0].age_ms = bad; },
    f => { f.slots[0].sample.forward = bad; }, f => { f.slots[0].intent.pitch_rpm = bad; },
  ]) {
    const value = structuredClone(fixture); corrupt(value);
    assert.equal(api.parseGamepadPreview(value), null);
  }
}
""")

    def test_age_uses_200ms_boundary_and_never_changes_cached_input(self):
        self.run_frontend("""
const parsed = api.parseGamepadPreview(fixture);
const original = JSON.stringify(parsed);
assert.equal(api.ageGamepadPreview(parsed, 200, true).slots[0].fresh, true);
const expired = api.ageGamepadPreview(parsed, 200.1, true).slots[0];
assert.equal(expired.fresh, false);
assert.equal(expired.status, 'stale');
assert.equal(expired.sample, null);
assert.equal(expired.intent, null);
assert.equal(expired.neutral, null);
assert.equal(JSON.stringify(parsed), original);
""")

    def test_each_slot_has_independent_age(self):
        self.run_frontend("""
fixture.slots[1].age_ms = 150;
const shown = api.ageGamepadPreview(api.parseGamepadPreview(fixture), 51, true);
assert.equal(shown.slots[0].fresh, true);
assert.equal(shown.slots[1].fresh, false);
assert.equal(shown.slots[1].intent, null);
""")

    def test_core_loss_and_invalid_local_clock_fail_closed(self):
        self.run_frontend("""
const parsed = api.parseGamepadPreview(fixture);
assert.equal(api.ageGamepadPreview(parsed, 0, false), null);
assert.equal(api.ageGamepadPreview(null, 0, true), null);
for (const elapsed of [-1, NaN, Infinity]) assert.equal(api.ageGamepadPreview(parsed, elapsed, true), null);
""")

    def test_non_running_service_cannot_leave_usable_inputs(self):
        self.run_frontend("""
for (const status of ['disabled', 'stopped', 'error']) {
  fixture.status = status;
  const shown = api.ageGamepadPreview(api.parseGamepadPreview(fixture), 0, true);
  for (const slot of shown.slots) {
    assert.equal(slot.fresh, false); assert.equal(slot.status, 'unavailable');
    assert.equal(slot.sample, null); assert.equal(slot.intent, null); assert.equal(slot.neutral, null);
  }
}
""")

    def test_stale_and_disconnected_slots_never_look_like_released_input(self):
        self.run_frontend("""
for (const status of ['stale', 'disconnected', 'invalid', 'waiting', 'unavailable']) {
  fixture.slots[0].fresh = false;
  fixture.slots[0].status = status;
  const slot = api.ageGamepadPreview(api.parseGamepadPreview(fixture), 0, true).slots[0];
  assert.equal(slot.sample, null); assert.equal(slot.intent, null); assert.equal(slot.neutral, null);
}
""")


if __name__ == "__main__":
    unittest.main()
