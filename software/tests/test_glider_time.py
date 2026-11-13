"""编译真实公共时钟：验证 HAL 毫秒回绕扩展和临界区配对。"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class GliderTimeTests(unittest.TestCase):
    def test_monotonic_microseconds_survive_hal_wrap(self):
        software = Path(__file__).resolve().parents[1]
        temporary_root = software.parents[1] / ".buildtmp"
        temporary_root.mkdir(exist_ok=True)
        compiler = shutil.which("gcc")
        self.assertIsNotNone(compiler, "需要主机 GCC")
        with tempfile.TemporaryDirectory(prefix="glider_time_", dir=temporary_root) as folder:
            tmp = Path(folder)
            (tmp / "main.h").write_text(
                '#include <stdint.h>\nuint32_t HAL_GetTick(void);\n', encoding="utf-8")
            (tmp / "FreeRTOS.h").write_text('', encoding="utf-8")
            (tmp / "task.h").write_text(
                'void enter(void); void leave(void);\n'
                '#define taskENTER_CRITICAL() enter()\n'
                '#define taskEXIT_CRITICAL() leave()\n', encoding="utf-8")
            harness = tmp / "test.c"
            harness.write_text(r'''
#include <assert.h>
#include "glider_time.h"
static uint32_t tick, depth, calls;
uint32_t HAL_GetTick(void) { return tick; }
void enter(void) { assert(depth == 0); depth++; calls++; }
void leave(void) { assert(depth == 1); depth--; }
int main(void) {
  tick = 1;
  assert(GliderTime_NowUs() == 1000);
  tick = UINT32_MAX - 1U;
  uint64_t a = GliderTime_NowUs();
  tick++;
  uint64_t b = GliderTime_NowUs();
  tick = 0;
  uint64_t c = GliderTime_NowUs();
  tick = 1;
  uint64_t d = GliderTime_NowUs();
  assert(b-a == 1000 && c-b == 1000 && d-c == 1000);
  assert(d == ((1ULL << 32) + 1) * 1000);
  assert(GliderTime_NowUs() == d);
  assert(GliderTime_NowMs() == 1); /* 旧毫秒接口继续保持 HAL 语义 */
  assert(depth == 0 && calls == 6);
  return 0;
}
''', encoding="utf-8")
            env = {**os.environ, "TMP": folder, "TEMP": folder, "TMPDIR": folder}
            env["PATH"] = str(Path(compiler).parent) + os.pathsep + os.environ.get("PATH", "")
            exe = tmp / "clock.exe"
            result = subprocess.run([
                compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", folder, "-I", str(software / "Glider-App/common"),
                str(harness), str(software / "Glider-App/common/glider_time.c"),
                "-o", str(exe)], capture_output=True, env=env)
            self.assertEqual(result.returncode, 0, (result.stdout + result.stderr).decode(errors="replace"))
            result = subprocess.run([str(exe)], capture_output=True, env=env)
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors="replace"))


if __name__ == "__main__":
    unittest.main()
