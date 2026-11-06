#!/usr/bin/env python3
"""用本地 Edge 无头浏览器验证桌面工作台，数据仅来自明确标记的本地测试夹具。"""

import argparse
import asyncio
import base64
import json
import math
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from contextlib import contextmanager
from pathlib import Path

STATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STATION_DIR))

import uvicorn
import websockets

from groundstation.app import create_app
from groundstation.gamepad_preview import GamepadPreview
from groundstation.models import LinkStatus
from groundstation.state_store import StateStore
from groundstation.legacy_telemetry import EKF_FIELDS, UGCTL_FIELDS, UGWRM_FIELDS, UGWRM_FIELDS_V2
from groundstation.session_recorder import disabled_recording_snapshot
from gamepad_control import GamepadSnapshot


def free_port():
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        return listener.getsockname()[1]


@contextmanager
def temporary_browser_profile():
    """浏览器子进程可能稍晚释放文件；只清理本次创建的临时配置目录。"""
    profile = tempfile.TemporaryDirectory(prefix="urgc-browser-")
    try:
        yield profile.name
    finally:
        for _ in range(20):
            try:
                profile.cleanup()
                break
            except OSError:
                time.sleep(0.2)
        else:
            print("临时浏览器配置尚被锁定，请稍后清理：" + profile.name, file=sys.stderr)


class DevTools:
    """按命令编号收取 CDP 应答，同时保留网络方法用于只读边界断言。"""

    def __init__(self, connection):
        self.connection = connection
        self.sequence = 0
        self.http_methods = []

    async def call(self, method, **params):
        self.sequence += 1
        await self.connection.send(json.dumps({"id": self.sequence, "method": method, "params": params}))
        while True:
            result = json.loads(await asyncio.wait_for(self.connection.recv(), timeout=10))
            if result.get('method') == 'Network.requestWillBeSent':
                self.http_methods.append(result['params']['request']['method'])
            if result.get("id") == self.sequence:
                if "error" in result:
                    raise RuntimeError(result["error"])
                return result.get("result", {})

    async def evaluate(self, expression):
        result = await self.call("Runtime.evaluate", expression=expression, returnByValue=True)
        if "exceptionDetails" in result:
            raise AssertionError(result["exceptionDetails"])
        return result.get("result", {}).get("value")


class LocalFixtureLink:
    """仅提供长文件名界面夹具，不打开任何设备端口或写记录文件。"""

    def start(self):
        pass

    def stop(self):
        pass

    def recording_snapshot(self):
        state = disabled_recording_snapshot()
        directory = str(STATION_DIR / 'plots' / 'LOCAL-TEST-FIXTURE-NO-FILES' / ('long-session-name-' * 12))
        state.update(enabled=True, status='recording', session_id='browser-verification-only',
                     directory=directory, raw_lines=30, rows={'ugekf': 10, 'ugctl': 10, 'ugwrm': 10},
                     files={topic: directory + '/' + topic + '_20261008_120000.csv' for topic in ('ugekf', 'ugctl', 'ugwrm')})
        return state


class LocalFixtureGamepad:
    """仅返回合成输入；不构造 XInputBackend，也不建立板端连接。"""

    def __init__(self, config, fixture_mode):
        self.config = config
        self.fixture_mode = fixture_mode
        self.running = False

    def start(self):
        self.running = True

    def stop(self):
        self.running = False

    def is_alive(self):
        return self.running

    def snapshot(self, index):
        if index == 3:
            return GamepadSnapshot(index=index, error="本地测试：此槽位无手柄")
        mode = self.fixture_mode.get('gamepad', 'pressed')
        sampled_at = time.monotonic() - (1.0 if mode == 'stale' and index == 0 else 0.0)
        values = {}
        if index == 0 and mode != 'released':
            values = dict(forward=1.0, button_y=True, dpad_left=True, right_trigger=1.0)
        elif index == 2:
            values = dict(button_b=True, right_trigger=1.0, rudder_center_seq=1)
        return GamepadSnapshot(index=index, connected=True, sample_time=sampled_at, **values)


async def inspect_page(cdp_url, url, output, server, fixture_mode, store):
    async with websockets.connect(cdp_url, max_size=20 * 1024 * 1024) as connection:
        cdp = DevTools(connection)
        await cdp.call("Page.enable")
        await cdp.call("Network.enable")
        await cdp.call("Page.navigate", url=url)
        for _ in range(100):
            if await cdp.evaluate("!!document.querySelector('[data-testid=workbench]')"):
                break
            await asyncio.sleep(0.1)
        else:
            raise AssertionError("工作台未加载，请先构建 frontend/dist")
        for _ in range(50):
            if await cdp.evaluate("document.querySelector('[data-testid=workbench]').dataset.coreLive === 'true'"):
                break
            await asyncio.sleep(0.1)
        else:
            raise AssertionError("工作台未收到测试遥测，不能验证断线转换")
        if not await cdp.evaluate("document.querySelector('nav button[aria-current=page]').textContent.trim() === '主页'"):
            raise AssertionError('默认页面不是主页')
        theme = await cdp.evaluate("""(() => {
            const active = getComputedStyle(document.querySelector('.navigation nav button.active'));
            return {background: getComputedStyle(document.body).backgroundColor,
                sidebar: document.querySelector('.workspace-label').textContent.trim(),
                tabs: [...document.querySelectorAll('.navigation nav button')].map(b => b.textContent.trim()),
                radius: active.borderRadius, leftBorder: active.borderLeftWidth, rightBorder: active.borderRightWidth,
                gradient: getComputedStyle(document.querySelector('.topbar')).backgroundImage};
        })()""")
        tabs = ['主页', '手柄输入', '运行监控', '遥测诊断', '会话记录']
        if theme['background'] != 'rgb(0, 0, 0)' or theme['sidebar'] != '工作台侧栏' or theme['tabs'] != tabs:
            raise AssertionError({'theme': theme})
        if any(theme[key] != '0px' for key in ('radius', 'leftBorder', 'rightBorder')) or 'linear-gradient' not in theme['gradient']:
            raise AssertionError({'theme': theme})
        results = [{'theme': theme}]

        async def assert_estimator_ready(expected):
            state = await cdp.evaluate("""(() => {
                const e = document.querySelector('[data-testid=freshness-status]');
                return {ready: e?.dataset.ready, text: e?.textContent.trim()};
            })()""")
            if state['ready'] != str(expected).lower() or state['text'] != ('估计就绪' if expected else '估计未就绪'):
                raise AssertionError({'estimator_expected': expected, 'actual': state})

        for width, height in ((1366, 768), (1920, 1080), (1280, 720)):
            await cdp.call("Emulation.setDeviceMetricsOverride", width=width, height=height, deviceScaleFactor=1, mobile=False)
            for tab in tabs:
                clicked = await cdp.evaluate(
                    "(() => { const button = [...document.querySelectorAll('nav button')].find(b => b.textContent.trim().endsWith("
                    + json.dumps(tab) + ")); if (!button) return false; button.click(); return true; })()"
                )
                if not clicked:
                    raise AssertionError("找不到工作台页签: " + tab)
                await asyncio.sleep(0.4)
                bounds = await cdp.evaluate("""(() => {
                    const d = document.documentElement, shell = document.querySelector('[data-testid=workbench]');
                    return {width: innerWidth, height: innerHeight, scrollWidth: d.scrollWidth, scrollHeight: d.scrollHeight,
                        shellHeight: shell.getBoundingClientRect().height, live: shell.dataset.coreLive};
                })()""")
                if bounds["scrollWidth"] > width + 1 or bounds["scrollHeight"] > height + 1:
                    raise AssertionError({"tab": tab, **bounds})
                # 开发方案和自证文案不再作为操作界面的常驻内容。
                redundant_copy = await cdp.evaluate("""(() => {
                    const text = document.body.innerText;
                    return ['设备概览 · 工作台入口', '布局预览', '点击分组查看方案',
                        '不会发送控制指令', '建议操作：', '建议独立调参',
                        '不把所有会动执行器', '此处不显示虚构', '不把未知显示为松开',
                        '本页不能改配置', '复用 GamepadConfig', 'LIVE FEED',
                        '— 表示未收到、已过期或字段无效'].filter(value => text.includes(value));
                })()""")
                if redundant_copy:
                    raise AssertionError({'redundant_copy': redundant_copy, 'tab': tab})
                results.append({"tab": tab, **bounds})
                if tab == '主页':
                    if not await cdp.evaluate("document.querySelector('[data-testid=home-attitude]').dataset.valid === 'true'"):
                        raise AssertionError('主页姿态仪未使用有效估计')
                    screenshot = await cdp.call('Page.captureScreenshot', format='png')
                    (output / ('home-%dx%d-test-fixture.png' % (width, height))).write_bytes(base64.b64decode(screenshot['data']))
                    if await cdp.evaluate("!!document.querySelector('[data-home-action]')"):
                        raise AssertionError('主页仍保留未实现的操作方案入口')
                    readouts = await cdp.evaluate("""['attitude', 'heading'].map(name => {
                        const e = document.querySelector('[data-testid=home-' + name + '-readout]');
                        if (!e) return {name, missing: true};
                        const r = e.getBoundingClientRect();
                        const texts = [e, ...e.querySelectorAll('*')].filter(node => [...node.childNodes].some(
                            child => child.nodeType === Node.TEXT_NODE && child.textContent.trim()));
                        return {name, svg: !!e.closest('svg'), width: r.width, height: r.height,
                            bottom: r.bottom, containerBottom: e.closest('.situation-view').getBoundingClientRect().bottom,
                            minFont: Math.min(...texts.map(node => parseFloat(getComputedStyle(node).fontSize)))};
                    })""")
                    if any(item.get('missing') or item['svg'] or item['minFont'] < 12 or
                           item['width'] <= 0 or item['height'] <= 0 or
                           item['bottom'] > item['containerBottom'] + 1 for item in readouts):
                        raise AssertionError({'home_readouts': readouts, 'width': width, 'height': height})
                    for action, panel in (('calibration', 'calibration'), ('overview', 'status')):
                        await cdp.evaluate("document.querySelector('[data-home-tab=" + action + "]').click()")
                        await asyncio.sleep(0.1)
                        clipped = await cdp.evaluate("""(() => {
                            const e = document.querySelector('.home-view');
                            return e.scrollWidth > e.clientWidth + 1 || e.scrollHeight > e.clientHeight + 1;
                        })()""")
                        if clipped:
                            raise AssertionError({'home_action_clipped': action, 'width': width, 'height': height})
                        panel_bounds = await cdp.evaluate("""(() => {
                            const e = document.querySelector('[data-testid=home-""" + panel + """-panel]');
                            if (!e) return {missing: true};
                            const r = e.getBoundingClientRect();
                            const container = e.closest('.operation-content');
                            return {width: e.clientWidth, contentWidth: e.scrollWidth,
                                height: e.clientHeight, contentHeight: e.scrollHeight,
                                containerHeight: container.clientHeight, containerContentHeight: container.scrollHeight,
                                left: r.left, right: r.right, top: r.top, bottom: r.bottom};
                        })()""")
                        if (panel_bounds.get('missing') or
                                panel_bounds['contentHeight'] > panel_bounds['height'] + 1 or
                                panel_bounds['contentWidth'] > panel_bounds['width'] + 1 or
                                panel_bounds['containerContentHeight'] > panel_bounds['containerHeight'] + 1 or
                                panel_bounds['left'] < 0 or panel_bounds['right'] > width + 1 or
                                panel_bounds['top'] < 0 or panel_bounds['bottom'] > height + 1):
                            raise AssertionError({'home_panel_clipped': panel_bounds, 'panel': panel,
                                                  'width': width, 'height': height})
                        if action == 'calibration' and width == 1366:
                            screenshot = await cdp.call('Page.captureScreenshot', format='png')
                            (output / 'home-homing-test-fixture.png').write_bytes(base64.b64decode(screenshot['data']))
                    results.append({'home_readouts': readouts, 'home_panels_fit': True,
                                    'width': width, 'height': height})
                if tab == "运行监控":
                    tables = await cdp.evaluate("""[...document.querySelectorAll('.measurements-panel .panel-scroll, .actuator-panel .panel-scroll, .link-panel .panel-scroll')].map(e => ({height: e.clientHeight, contentHeight: e.scrollHeight, width: e.clientWidth, contentWidth: e.scrollWidth}))""")
                    if len(tables) != 3 or any(item['contentHeight'] > item['height'] + 1 or item['contentWidth'] > item['width'] + 1 for item in tables):
                        raise AssertionError({'critical_tables_clipped': tables, 'width': width, 'height': height})
                    screenshot = await cdp.call("Page.captureScreenshot", format="png")
                    # 测试产物仅写入本次输出目录，不进入应用静态资源。
                    (output / ("workbench-%dx%d-test-fixture.png" % (width, height))).write_bytes(base64.b64decode(screenshot["data"]))
                if tab == '手柄输入':
                    panels = await cdp.evaluate("""(() => {
                        const names = ['input', 'intent'];
                        return names.map(name => {
                            const e = document.querySelector('[data-testid=gamepad-' + name + '-panel]');
                            if (!e) return {name, missing: true};
                            const r = e.getBoundingClientRect();
                            const clipped = [e, ...e.querySelectorAll('.panel-scroll')].some(
                                p => p.scrollWidth > p.clientWidth + 1 || p.scrollHeight > p.clientHeight + 1);
                            return {name, clipped, left: r.left, right: r.right, top: r.top, bottom: r.bottom};
                        });
                    })()""")
                    if any(panel.get('missing') or panel.get('clipped') or panel['left'] < 0 or
                           panel['right'] > width + 1 or panel['bottom'] > height + 1 for panel in panels):
                        raise AssertionError({'gamepad_panels_clipped': panels, 'width': width, 'height': height})
                    screenshot = await cdp.call('Page.captureScreenshot', format='png')
                    (output / ('gamepad-%dx%d-test-fixture.png' % (width, height))).write_bytes(base64.b64decode(screenshot['data']))
                    results.append({'gamepad_panels': panels, 'width': width, 'height': height})
                    details = await cdp.evaluate("""['config', 'mapping'].map(name => {
                        const e = document.querySelector('[data-testid=gamepad-' + name + '-panel]');
                        return {name, tag: e?.tagName, open: e?.open, summary: !!e?.querySelector('summary')};
                    })""")
                    if any(item['tag'] != 'DETAILS' or item['open'] or not item['summary'] for item in details):
                        raise AssertionError({'gamepad_details_not_collapsed': details})
                    for name in ('config', 'mapping'):
                        await cdp.evaluate("document.querySelector('[data-testid=gamepad-" + name + "-panel] summary').click()")
                        await asyncio.sleep(.1)
                        expanded = await cdp.evaluate("""(() => {
                            const details = document.querySelector('[data-testid=gamepad-""" + name + """-panel]');
                            const content = details.querySelector('.panel-scroll');
                            const r = details.getBoundingClientRect();
                            if (!content) return {missingContent: true};
                            content.scrollTop = content.scrollHeight;
                            const reached = content.scrollTop;
                            content.scrollTop = 0;
                            return {open: details.open, rows: content.querySelectorAll('tbody tr').length,
                                left: r.left, right: r.right, bottom: r.bottom, reached,
                                height: content.clientHeight, contentHeight: content.scrollHeight,
                                width: content.clientWidth, contentWidth: content.scrollWidth,
                                pageWidth: document.documentElement.scrollWidth,
                                pageHeight: document.documentElement.scrollHeight};
                        })()""")
                        if (expanded.get('missingContent') or not expanded['open'] or expanded['rows'] == 0 or
                                expanded['left'] < 0 or expanded['right'] > width + 1 or expanded['bottom'] > height + 1 or
                                expanded['contentWidth'] > expanded['width'] + 1 or expanded['height'] <= 0 or
                                expanded['pageWidth'] > width + 1 or expanded['pageHeight'] > height + 1 or
                                (expanded['contentHeight'] > expanded['height'] + 1 and expanded['reached'] == 0)):
                            raise AssertionError({'gamepad_detail': name, 'expanded': expanded,
                                                  'width': width, 'height': height})
                    # 两块详情可同时展开；详情滚动不应使实时输入和目标消失。
                    live_panels_fit = await cdp.evaluate("""['input', 'intent'].every(name => {
                        const e = document.querySelector('[data-testid=gamepad-' + name + '-panel]');
                        const r = e.getBoundingClientRect();
                        return r.height > 0 && r.top >= 0 && r.bottom <= innerHeight + 1;
                    })""")
                    if not live_panels_fit:
                        raise AssertionError('展开详情后实时手柄面板移出视口')
                    for name in ('config', 'mapping'):
                        await cdp.evaluate("document.querySelector('[data-testid=gamepad-" + name + "-panel] summary').click()")
                    results.append({'gamepad_details_expand_and_scroll': True, 'width': width, 'height': height})
                if tab == '遥测诊断':
                    await cdp.evaluate("document.querySelector('[data-testid=diagnostic-raw-toggle]').click()")
                    for topic, expected_field in (('ugekf', 'fl'), ('ugctl', 'target_depth'), ('ugwrm', 'stroke_percent')):
                        await cdp.evaluate("document.querySelector('[data-topic=" + topic + "]').click()")
                        await asyncio.sleep(.1)
                        raw = await cdp.evaluate("JSON.parse(document.querySelector('[data-testid=diagnostic-raw]').textContent)")
                        if (raw.get('topic') != topic or expected_field not in (raw.get('sample') or {}).get('data', {}) or
                                'backend' in raw or 'telemetry' in raw):
                            raise AssertionError({'diagnostic_topic': topic, 'raw': raw})
                    await cdp.evaluate("document.querySelector('[data-testid=diagnostic-raw-toggle]').click()")
                    results.append({'diagnostic_raw_follows_topic': True, 'width': width, 'height': height})

        # 核心在线、设备监听状态与估计器就绪分别判断。
        store.set_link(LinkStatus.STOPPED)
        await asyncio.sleep(.4)
        listener_stopped = await cdp.evaluate("""(() => ({
            coreLive: document.querySelector('[data-testid=workbench]').dataset.coreLive,
            device: document.querySelector('[data-testid=device-status]').dataset.status,
            notice: !!document.querySelector('[data-testid=station-notice]'),
        }))()""")
        if listener_stopped != {'coreLive': 'true', 'device': 'stopped', 'notice': False}:
            raise AssertionError({'listener_stopped': listener_stopped})
        await assert_estimator_ready(False)
        store.set_link(LinkStatus.CONNECTED, peer='LOCAL TEST FIXTURE', session_id='browser-verification-only')
        fixture_mode['case'] = 'not_ready'
        await asyncio.sleep(.4)
        if not store.read()[1]['state']['overall_fresh']:
            raise AssertionError('未就绪夹具没有保持三主题新鲜')
        await assert_estimator_ready(False)
        fixture_mode['case'] = 'ekf_only'
        await asyncio.sleep(.2)
        store.clear_session_telemetry()
        await asyncio.sleep(.4)
        if set(store.read()[1]['telemetry']) != {'ugekf'}:
            raise AssertionError('单估计主题夹具仍含其他遥测')
        await assert_estimator_ready(True)
        results.append({'listener_stopped_without_warning': True, 'fresh_topics_without_estimator_ready': True,
                        'estimator_ready_without_other_topics': True})
        fixture_mode['case'] = 'valid'
        await asyncio.sleep(.4)

        async def select_gamepad(index):
            await cdp.evaluate("document.querySelector('[data-testid=gamepad-slot-" + str(index) + "]').click()")
            await asyncio.sleep(.15)

        async def read_intent():
            return await cdp.evaluate("""(() => {
                const result = {};
                for (const key of ['left_percent', 'right_percent', 'pitch_rpm', 'roll_rpm', 'rudder_rate_deg_s']) {
                    const e = document.querySelector('[data-testid=gamepad-intent-' + key + ']');
                    result[key] = e && e.dataset.value !== '' ? Number(e.dataset.value) : null;
                }
                return result;
            })()""")

        await cdp.evaluate("[...document.querySelectorAll('nav button')].find(b => b.textContent.trim() === '手柄输入').click()")
        await asyncio.sleep(.1)
        await select_gamepad(0)
        pressed = await read_intent()
        if pressed != dict(left_percent=100, right_percent=100, pitch_rpm=-100, roll_rpm=300, rudder_rate_deg_s=30):
            raise AssertionError({'pressed_mapping': pressed})
        fixture_mode['gamepad'] = 'released'
        await asyncio.sleep(.4)
        released = await read_intent()
        if released != dict.fromkeys(pressed, 0):
            raise AssertionError({'released_mapping': released})
        fixture_mode['gamepad'] = 'pressed'
        await asyncio.sleep(.3)
        await select_gamepad(1)
        if await read_intent() != released:
            raise AssertionError('选择中立槽位后仍显示其他槽位运动意图')
        await select_gamepad(3)
        if any(value is not None for value in (await read_intent()).values()):
            raise AssertionError('未连接槽位被显示成可用中立输入')
        # 切换页面不应偷偷选择另一只手柄。
        await cdp.evaluate("[...document.querySelectorAll('nav button')].find(b => b.textContent.trim() === '主页').click()")
        await asyncio.sleep(.1)
        await cdp.evaluate("document.querySelector('[data-testid=home-gamepad-link]').click()")
        await asyncio.sleep(.1)
        if any(value is not None for value in (await read_intent()).values()):
            raise AssertionError('切换页面意外自动选择了已连接槽位')
        await select_gamepad(0)
        store.set_link(LinkStatus.DISCONNECTED, peer='LOCAL TEST FIXTURE')
        await asyncio.sleep(.4)
        if await read_intent() != pressed:
            raise AssertionError('板端断开错误阻断了本机手柄预览')
        store.set_link(LinkStatus.CONNECTED, peer='LOCAL TEST FIXTURE', session_id='browser-verification-only')
        fixture_mode['gamepad'] = 'stale'
        await asyncio.sleep(.5)
        if any(value is not None for value in (await read_intent()).values()):
            raise AssertionError('遥测持续更新时，陈旧手柄输入仍作为可用意图')
        await assert_estimator_ready(True)
        if not store.read()[1]['state']['overall_fresh']:
            raise AssertionError('手柄陈旧测试没有保持遥测新鲜，不能证明独立年龄判断')
        results.append({'gamepad_pressed': pressed, 'gamepad_released': released,
                        'slot_selection_persists': True, 'preview_without_board': True,
                        'stale_gamepad_with_live_telemetry_unavailable': True})
        fixture_mode['gamepad'] = 'pressed'
        await asyncio.sleep(.3)
        await cdp.evaluate("[...document.querySelectorAll('nav button')].find(b => b.textContent.trim().endsWith('运行监控')).click()")
        for case in ('missing_validity', 'non_finite', 'invalid_attitude', 'stopped_stream'):
            fixture_mode['case'] = case
            await asyncio.sleep(1.1)
            if case in ('missing_validity', 'stopped_stream'):
                await assert_estimator_ready(False)
            metric = 'pitch_deg' if case == 'invalid_attitude' else 'depth'
            unavailable = await cdp.evaluate("!!document.querySelector('[data-testid=metric-" + metric + "] .data-value.unavailable')")
            if not unavailable:
                raise AssertionError("故障数据仍作为正常深度显示: " + case)
            await cdp.evaluate("[...document.querySelectorAll('nav button')].find(b => b.textContent.trim() === '主页').click()")
            await asyncio.sleep(0.1)
            if not await cdp.evaluate("!!document.querySelector('[data-testid=home-metric-" + metric + "] .data-value.unavailable')"):
                raise AssertionError('主页故障数值仍然正常显示: ' + case)
            if case != 'non_finite' and await cdp.evaluate("document.querySelector('[data-testid=home-attitude]').dataset.valid !== 'false'"):
                raise AssertionError('主页无效姿态仍驱动仪表: ' + case)
            results.append({'case': case, 'monitor_and_home_unavailable': unavailable})
            fixture_mode['case'] = 'valid'
            await asyncio.sleep(0.5)
            if await cdp.evaluate("!!document.querySelector('[data-testid=metric-depth] .data-value.unavailable')"):
                raise AssertionError("测试遥测恢复后深度未恢复")
        # 校准遥测缺失/无效时不显示假位置，pitch 反馈过期不影响有效 roll 估算。
        for case, expected_available in (('valid', True), ('legacy_wrm', False),
                ('invalid_roll', False), ('missing_roll_validity', False), ('pitch_stale', True)):
            fixture_mode['case'] = case
            await asyncio.sleep(.4)
            await cdp.evaluate("[...document.querySelectorAll('nav button')].find(b => b.textContent.trim() === '主页').click()")
            await asyncio.sleep(.1)
            available = await cdp.evaluate("!document.querySelector('[data-testid=home-metric-roll_percent] .data-value.unavailable')")
            if available != expected_available:
                raise AssertionError({'roll_home_case': case, 'available': available})
            await cdp.evaluate("[...document.querySelectorAll('nav button')].find(b => b.textContent.trim() === '运行监控').click()")
            await asyncio.sleep(.1)
            pointer = await cdp.evaluate("!!document.querySelector('[data-testid=roll-track] i')")
            if pointer != expected_available:
                raise AssertionError({'roll_track_case': case, 'pointer': pointer})
            results.append({'roll_case': case, 'available': available, 'pointer': pointer})
        # 无效 PID 计算结果与未校准行程只撤销展示，原始主题数据仍供诊断。
        for case, metric, topic, raw_value, expected_available in (
                ('valid', 'displacement_mm', 'ugctl', 90, True),
                ('pid_off', 'displacement_mm', 'ugctl', 90, False),
                ('pid_disarmed', 'displacement_mm', 'ugctl', 90, False),
                ('pid_output_invalid', 'displacement_mm', 'ugctl', 90, False),
                ('pid_fault', 'displacement_mm', 'ugctl', 90, False),
                ('wrm_observed_range', 'stroke_percent', 'ugwrm', 37.5, False),
                ('valid', 'stroke_percent', 'ugwrm', 0, True)):
            fixture_mode['case'] = case
            await asyncio.sleep(.4)
            await cdp.evaluate("[...document.querySelectorAll('.navigation nav button')].find(b => b.textContent.trim() === '主页').click()")
            await asyncio.sleep(.1)
            available = await cdp.evaluate("!document.querySelector('[data-testid=home-metric-" + metric + "] .data-value.unavailable')")
            if available != expected_available:
                raise AssertionError({'quality_case': case, 'metric': metric, 'available': available})
            independent_values_available = await cdp.evaluate("""['roll_percent', 'depth'].every(key =>
                !document.querySelector('[data-testid=home-metric-' + key + '] .data-value.unavailable'))""")
            if not independent_values_available:
                raise AssertionError('PID 或俯仰轴质量错误影响了横滚轴/估计深度: ' + case)
            await assert_estimator_ready(True)
            if store.read()[1]['telemetry'][topic]['data'].get(metric) != raw_value:
                raise AssertionError('展示门控修改了原始遥测: ' + case)
            await cdp.evaluate("[...document.querySelectorAll('.navigation nav button')].find(b => b.textContent.trim() === '遥测诊断').click()")
            await asyncio.sleep(.1)
            await cdp.evaluate("document.querySelector('[data-topic=" + topic + "]').click()")
            await cdp.evaluate("document.querySelector('[data-testid=diagnostic-raw-toggle]').click()")
            await asyncio.sleep(.1)
            raw = await cdp.evaluate("JSON.parse(document.querySelector('[data-testid=diagnostic-raw]').textContent)")
            if raw.get('topic') != topic or (raw.get('sample') or {}).get('data', {}).get(metric) != raw_value:
                raise AssertionError({'quality_case_raw_lost': case, 'raw': raw})
            results.append({'quality_case': case, 'metric': metric, 'available': available,
                            'raw_value_preserved': True, 'roll_and_estimate_independent': True})
        fixture_mode['case'] = 'valid'
        await asyncio.sleep(.4)
        await cdp.evaluate("[...document.querySelectorAll('nav button')].find(b => b.textContent.trim() === '主页').click()")
        await asyncio.sleep(.1)
        server.should_exit = True
        for _ in range(60):
            live = await cdp.evaluate("document.querySelector('[data-testid=workbench]').dataset.coreLive")
            if live == "false":
                break
            await asyncio.sleep(0.1)
        else:
            raise AssertionError("后端关闭后工作台未撤销在线标识")
        results.append({"backend_closed_core_live": live})
        await asyncio.sleep(0.3)
        if not await cdp.evaluate("!!document.querySelector('[data-testid=metric-depth] .data-value.unavailable')"):
            raise AssertionError("核心断开后仍将深度显示为正常值")
        if not await cdp.evaluate("document.querySelector('[data-testid=home-attitude]').dataset.valid === 'false' && !document.querySelector('[data-testid=home-heading-pointer]')"):
            raise AssertionError('断开后主页仍显示有效姿态或航向指针')
        screenshot = await cdp.call("Page.captureScreenshot", format="png")
        (output / "home-disconnected-test-fixture.png").write_bytes(base64.b64decode(screenshot["data"]))
        await cdp.evaluate("[...document.querySelectorAll('nav button')].find(b => b.textContent.trim() === '手柄输入').click()")
        await asyncio.sleep(.1)
        if any(value is not None for value in (await read_intent()).values()):
            raise AssertionError('核心断开后仍保留可用手柄意图')
        results.append({'core_closed_gamepad_unavailable': True})
        if any(method not in ('GET', 'HEAD') for method in cdp.http_methods):
            raise AssertionError({'unexpected_write_request': cdp.http_methods})
        results.append({'only_read_requests': True, 'request_count': len(cdp.http_methods)})
        print(json.dumps(results, ensure_ascii=False, indent=2))
        try:
            await cdp.call("Browser.close")
        except websockets.exceptions.ConnectionClosed:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--edge", default=r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe")
    parser.add_argument("--output", type=Path, default=STATION_DIR.parent / ".buildtmp" / "urgc-workbench-check")
    args = parser.parse_args()
    if not Path(args.edge).is_file():
        parser.error("未找到 Edge，请用 --edge 指定路径")
    args.output.mkdir(parents=True, exist_ok=True)
    store = StateStore()
    store.set_link(LinkStatus.CONNECTED, peer="LOCAL TEST FIXTURE", session_id="browser-verification-only")
    stopped = threading.Event()
    fixture_mode = {'case': 'valid', 'gamepad': 'pressed'}

    def publish_fixture():
        started = time.monotonic()
        while not stopped.wait(0.1):
            case = fixture_mode['case']
            if case == 'stopped_stream':
                continue
            seconds = time.monotonic() - started
            ekf = dict.fromkeys(EKF_FIELDS, 0)
            ekf.update(t_ms=int(seconds * 1000), fl=3, status=4, depth=2 + math.sin(seconds) * 0.1, roll_deg=12, pitch_deg=-5, yaw_deg=90)
            ctl = dict.fromkeys(UGCTL_FIELDS, 0)
            ctl.update(schema=2, t_ms=ekf["t_ms"], target_depth=2, depth=ekf["depth"], depth_error=2-ekf["depth"],
                       mode=2, armed=1, output_valid=1, fault=0,
                       pitch_sp_deg=-5, pitch_deg=-5, displacement_mm=90, target_displacement_mm=90)
            wrm = dict.fromkeys(UGWRM_FIELDS, 0)
            wrm.update(schema=3, t_ms=ekf["t_ms"], span_valid=1, data_fresh=1, stroke_percent=0, mm_from_front=0, ref_kind=2, home_state=3,
                       roll_pulse=500, roll_pulse_min=0, roll_pulse_max=1000, roll_span_pulses=1000, roll_percent=50,
                       roll_valid=1, roll_home_state=6, calib_state=2, calib_hold=1)
            if case == 'missing_validity':
                ekf.pop('fl')
            elif case == 'not_ready':
                ekf['fl'] = 1
            elif case == 'non_finite':
                ekf['depth'] = float('nan')
            elif case == 'invalid_attitude':
                ekf['pitch_deg'] = float('nan')
            elif case == 'legacy_wrm':
                wrm = {name: wrm[name] for name in UGWRM_FIELDS_V2}
                wrm['schema'] = 2
            elif case == 'invalid_roll':
                wrm['roll_valid'] = 0
            elif case == 'missing_roll_validity':
                wrm.pop('roll_valid')
            elif case == 'pitch_stale':
                wrm['data_fresh'] = 0
            elif case == 'pid_off':
                ctl['mode'] = 0
            elif case == 'pid_disarmed':
                ctl['armed'] = 0
            elif case == 'pid_output_invalid':
                ctl['output_valid'] = 0
            elif case == 'pid_fault':
                ctl['fault'] = 1
            elif case == 'wrm_observed_range':
                wrm.update(ref_kind=1, stroke_percent=37.5, mm_from_front=22.05)
            for topic, data in (("ugekf", ekf), ("ugctl", ctl), ("ugwrm", wrm)):
                if case == 'ekf_only' and topic != 'ugekf':
                    continue
                store.publish_telemetry(topic, data, session_id="browser-verification-only")

    publisher = threading.Thread(target=publish_fixture, daemon=True)
    publisher.start()
    port = free_port()
    preview = GamepadPreview(service_factory=lambda config: LocalFixtureGamepad(config, fixture_mode))
    app = create_app(store, link_manager=LocalFixtureLink(), device_link_enabled=False, gamepad_preview=preview)
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="error", timeout_graceful_shutdown=1))
    server_thread = threading.Thread(target=server.run, daemon=True)
    server_thread.start()
    browser = None
    with temporary_browser_profile() as profile:
        try:
            deadline = time.monotonic() + 10
            while not server.started and time.monotonic() < deadline:
                time.sleep(0.1)
            if not server.started:
                raise RuntimeError("本地测试 HTTP 服务未启动")
            debug_port = free_port()
            browser = subprocess.Popen(
                [args.edge, "--headless=new", "--disable-gpu", "--disable-breakpad", "--disable-crash-reporter", "--no-first-run", "--no-default-browser-check",
                 "--remote-debugging-port=" + str(debug_port), "--user-data-dir=" + profile, "about:blank"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
            deadline = time.monotonic() + 15
            cdp_url = None
            while time.monotonic() < deadline:
                try:
                    with urllib.request.urlopen("http://127.0.0.1:%d/json/list" % debug_port, timeout=1) as response:
                        pages = json.load(response)
                    cdp_url = next(page["webSocketDebuggerUrl"] for page in pages if page["type"] == "page")
                    break
                except (OSError, StopIteration):
                    time.sleep(0.1)
            if cdp_url is None:
                raise RuntimeError("无头浏览器调试服务未启动")
            asyncio.run(inspect_page(cdp_url, "http://127.0.0.1:%d" % port, args.output, server, fixture_mode, store))
        finally:
            stopped.set()
            server.should_exit = True
            server_thread.join(5)
            if browser is not None:
                try:
                    browser.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    browser.terminate()
                    browser.wait(timeout=5)


if __name__ == "__main__":
    main()
