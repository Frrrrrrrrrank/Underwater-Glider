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
from groundstation.models import LinkStatus
from groundstation.state_store import StateStore
from groundstation.legacy_telemetry import EKF_FIELDS, UGCTL_FIELDS, UGWRM_FIELDS
from groundstation.session_recorder import disabled_recording_snapshot


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


async def inspect_page(cdp_url, url, output, server, fixture_mode):
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
            const active = getComputedStyle(document.querySelector('nav button.active'));
            return {background: getComputedStyle(document.body).backgroundColor,
                sidebar: document.querySelector('.workspace-label').textContent.trim(),
                tabs: [...document.querySelectorAll('nav button')].map(b => b.textContent.trim()),
                radius: active.borderRadius, leftBorder: active.borderLeftWidth, rightBorder: active.borderRightWidth,
                gradient: getComputedStyle(document.querySelector('.topbar')).backgroundImage};
        })()""")
        if theme['background'] != 'rgb(0, 0, 0)' or theme['sidebar'] != '工作台侧栏' or theme['tabs'] != ['主页', '运行监控', '遥测诊断', '会话记录']:
            raise AssertionError({'theme': theme})
        if any(theme[key] != '0px' for key in ('radius', 'leftBorder', 'rightBorder')) or 'linear-gradient' not in theme['gradient']:
            raise AssertionError({'theme': theme})
        results = [{'theme': theme}]
        for width, height in ((1366, 768), (1920, 1080), (1280, 720)):
            await cdp.call("Emulation.setDeviceMetricsOverride", width=width, height=height, deviceScaleFactor=1, mobile=False)
            for tab in ("主页", "运行监控", "遥测诊断", "会话记录"):
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
                results.append({"tab": tab, **bounds})
                if tab == '主页':
                    if not await cdp.evaluate("document.querySelector('[data-testid=home-attitude]').dataset.valid === 'true'"):
                        raise AssertionError('主页姿态仪未使用有效估计')
                    screenshot = await cdp.call('Page.captureScreenshot', format='png')
                    (output / ('home-%dx%d-test-fixture.png' % (width, height))).write_bytes(base64.b64decode(screenshot['data']))
                    for action in ('direct', 'pid', 'homing', 'maintenance', 'overview'):
                        await cdp.evaluate("document.querySelector('[data-home-action=" + action + "]').click()")
                        await asyncio.sleep(0.1)
                        clipped = await cdp.evaluate("""(() => {
                            const e = document.querySelector('.home-view');
                            return e.scrollWidth > e.clientWidth + 1 || e.scrollHeight > e.clientHeight + 1;
                        })()""")
                        if clipped:
                            raise AssertionError({'home_action_clipped': action, 'width': width, 'height': height})
                        if action == 'homing' and width == 1366:
                            screenshot = await cdp.call('Page.captureScreenshot', format='png')
                            (output / 'home-homing-test-fixture.png').write_bytes(base64.b64decode(screenshot['data']))
                if tab == "运行监控":
                    tables = await cdp.evaluate("""[...document.querySelectorAll('.measurements-panel .panel-scroll, .actuator-panel .panel-scroll, .link-panel .panel-scroll')].map(e => ({height: e.clientHeight, contentHeight: e.scrollHeight, width: e.clientWidth, contentWidth: e.scrollWidth}))""")
                    if len(tables) != 3 or any(item['contentHeight'] > item['height'] + 1 or item['contentWidth'] > item['width'] + 1 for item in tables):
                        raise AssertionError({'critical_tables_clipped': tables, 'width': width, 'height': height})
                    screenshot = await cdp.call("Page.captureScreenshot", format="png")
                    # 测试产物写入已忽略的 plots 目录，不进入应用静态资源。
                    (output / ("workbench-%dx%d-test-fixture.png" % (width, height))).write_bytes(base64.b64decode(screenshot["data"]))
        await cdp.evaluate("[...document.querySelectorAll('nav button')].find(b => b.textContent.trim().endsWith('运行监控')).click()")
        for case in ('missing_validity', 'non_finite', 'invalid_attitude', 'stopped_stream'):
            fixture_mode['case'] = case
            await asyncio.sleep(1.1)
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
    parser.add_argument("--output", type=Path, default=STATION_DIR / "plots" / "urgc-workbench-check")
    args = parser.parse_args()
    if not Path(args.edge).is_file():
        parser.error("未找到 Edge，请用 --edge 指定路径")
    args.output.mkdir(parents=True, exist_ok=True)
    store = StateStore()
    store.set_link(LinkStatus.CONNECTED, peer="LOCAL TEST FIXTURE", session_id="browser-verification-only")
    stopped = threading.Event()
    fixture_mode = {'case': 'valid'}

    def publish_fixture():
        started = time.monotonic()
        while not stopped.wait(0.1):
            if fixture_mode['case'] == 'stopped_stream':
                continue
            seconds = time.monotonic() - started
            ekf = dict.fromkeys(EKF_FIELDS, 0)
            ekf.update(t_ms=int(seconds * 1000), fl=3, status=4, depth=2 + math.sin(seconds) * 0.1, roll_deg=12, pitch_deg=-5, yaw_deg=90)
            ctl = dict.fromkeys(UGCTL_FIELDS, 0)
            ctl.update(schema=2, t_ms=ekf["t_ms"], target_depth=2, depth=ekf["depth"], depth_error=2-ekf["depth"],
                       mode=2, pitch_sp_deg=-5, pitch_deg=-5, displacement_mm=90, target_displacement_mm=90)
            wrm = dict.fromkeys(UGWRM_FIELDS, 0)
            wrm.update(schema=2, t_ms=ekf["t_ms"], span_valid=1, data_fresh=1, stroke_percent=50, mm_from_front=72.5, ref_kind=2, home_state=3)
            if fixture_mode['case'] == 'missing_validity':
                ekf.pop('fl')
            elif fixture_mode['case'] == 'non_finite':
                ekf['depth'] = float('nan')
            elif fixture_mode['case'] == 'invalid_attitude':
                ekf['pitch_deg'] = float('nan')
            for topic, data in (("ugekf", ekf), ("ugctl", ctl), ("ugwrm", wrm)):
                store.publish_telemetry(topic, data, session_id="browser-verification-only")

    publisher = threading.Thread(target=publish_fixture, daemon=True)
    publisher.start()
    port = free_port()
    app = create_app(store, link_manager=LocalFixtureLink(), device_link_enabled=False)
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
            asyncio.run(inspect_page(cdp_url, "http://127.0.0.1:%d" % port, args.output, server, fixture_mode))
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
