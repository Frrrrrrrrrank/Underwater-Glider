# UnderwaterRobotGroundControl Core

简称：URGC。

这是常驻地面站核心的只读遥测阶段。默认监听板端 TCP 客户端使用的
`192.168.57.100:8800`，复用 `contcp_t.py` 的解析规则接收 `$UGEKF`、
`$UGCTL`、`$UGWRM`，再把最新状态发布给网页。

建立连接后只发送一次 `MSG_STREAM_CONTROL`，掩码为 `EKF | CTL | WRM`
（`0x0D`），用于请求板端输出遥测；当前没有执行器、回零、PID、参数写入或任务控制接口。
URGC 是设备 socket 的唯一所有者，因此运行它时不要同时启动 `contcp_t.py`，否则两个程序会
竞争同一个 8800 端口。旧脚本仍保留，可在不启动 URGC 时单独使用。

UGWRM 兼容 schema2 的原 30 字段和 schema3 的 41 字段。schema3 在原字段后追加
roll 脉冲、两端位置、跨度、百分比、有效位、阶段/失败原因，以及整次校准状态/失败原因/保持位。
旧版没有的字段为 null，CSV 新增尾列为空；旧 CSV 原列名及顺序保持不变。
新固件的 UGWRM 默认 5Hz，订阅仍为原 WRM 位，不增加主题或网页控制接口。

单独运行 `python -B station/contcp_t.py` 时，`calib` 同时启动 pitch 与 roll 校准，
`calibstop` 取消两轴。pitch 从 PB4 释放后建立前端 0%，跨度为 281321；roll 每次从
PB3 释放点 0% 测到 PB5 释放点 100%，再回到 50%。只有两轴都成功才报告整次完成。
完成、失败或取消后需新中立命令重新接管；auto_sti 会先发中立并等待保持位解除。
校准没有通信租约，断链不代表停止。roll 为开环脉冲估算，50% 不代表艇体水平；参考只存 RAM。

## contcp_t 双终端监控（无需更新固件）

在仓库根目录分别打开两个终端：

```powershell
# 终端一：沿用控制台、手柄与日志
python -B station/contcp_t.py
```

```powershell
# 终端二：只读固定面板，默认每秒刷新5次；Ctrl+C退出面板
python -B station/monitor.py
```

`contcp_t` 自动提供本机 `http://127.0.0.1:8801/snapshot`，monitor 仅从这里读取。
仍只有 contcp_t 的接收线程访问板端连接；无需启动网页后端，也不需要 FastAPI 或额外安装包。
监控可先于板端连接启动；关闭或卡住 monitor 不会发送控制命令、改变订阅或关闭板端连接。
contcp_t 退出后面板显示不可用，并等待它再次启动；contcp_t 本身仍沿用单次 accept 的连接流程。

端口冲突时原控制台继续工作，监控接口会提示不可用。可在终端一先设置
`$env:UG_MONITOR_PORT = '8802'`，终端二使用 `python -B station/monitor.py --port 8802`。
`python -B station/monitor.py --once` 只打印一次，适合检查接口或重定向输出。

| 面板项目 | 数据含义与限制 |
| --- | --- |
| Pitch 百分比 | UGWRM；ref_kind=2 为校准行程，1 只能表示本次观测范围且标为未校准；无参考/跨度无效/反馈过期时不可用 |
| Roll 百分比、校准状态 | UGWRM schema3；schema2 没有的字段显示未上报；roll 百分比依赖自己的有效位 |
| 姿态与深度 | 有效 UGEKF 的 pitch/roll/yaw（°）和深度（m），不使用 PID OFF 时的清零占位值 |
| 电池 | UGBAT 滤波总电压（V）和有效性；考虑 valid、板端样本年龄及本机停流超时，无 SOC 估算 |
| 北斗 | UGRDS schema2；信号、就绪、模块回执/失败计数、卡号状态、缓存定位和收到的目标坐标。模块确认不等于对端送达，缓存定位年龄没有上报 |
| 实验记录 | 顶部显示记录状态、丢记录数和当前目录；记录失败会在控制台主动提示 |
| 四个限位 | UGWRM.limit_bits：bit0=PB3/Roll 0%，bit1=PB4/Pitch机头，bit2=PB5/Roll 100%，bit3=PD7/Pitch机尾；0触发、1释放；未知不显示成释放 |
| PID | 模式、阶段、ARM、故障、请求深度/俯仰及有效算法俯仰设定；参数有效位与CRC。请求深度不等于上浮阶段的即时目标 |
| 整套 Kp/Ki/Kd、限幅参数 | 当前固件没有读回接口，显示未上报；不把本机草稿或P/I/D输出分量当作生效参数 |
| 左右最终 PWM 百分比及µs | 当前固件没有上报，显示未上报；UGCTL.thrust_pct 为 PID 请求，不代表最终双路输出 |

默认逐层请求 EKF/CTL/WRM（0x0D）、含 BAT 的0x1D、再含 RDSS 的0x3D。
每次包含 RDSS 的订阅更新/恢复都会先发送兼容掩码，旧固件拒绝未知位时仍能保留支持的主题。
发送成功不代表板端支持某主题，以收到有效遥测为准。
保留显式 `ekfoff/wormoff` 的含义，面板随后显示过期。面板5Hz刷新不改变板端采样/上报频率；
实时主题接收超时500ms、电池3000ms、北斗2000ms，重复帧不续期；断连、缺失、无效数据不填假0。
旧固件是否具备 roll/BAT/RDSS 取决于实际烧录版本，本机源码包含这些功能不等于板上已经运行。
完整面板含记录状态约31行，建议增大终端高度；窗口过矮会显示截断提示。

## CLI 实验记录与 pitch / roll 响应复核

`contcp_t.py` 建立连接后自动记录，无须额外启动采集进程。每次连接建立独立目录：
`station/logs/control/experiments/日期_时间_会话编号/`。默认 CLI 的遥测 CSV 统一写入此目录，
不再同步逐行写旧的 ekf/control/battery 目录；历史文件保留。网页的会话记录机制不变。

| 文件 | 用途 |
| --- | --- |
| `commands.csv` | 每一次发送，包括不变的心跳、松键停止、中立/释放和发送失败；记录实际编码字段、UGCP序号、发送起止时间、模式和输入 |
| `telemetry_ugwrm.csv` 等 | 原始主题字段；pitch/roll脉冲、RPM、有效位等直接展开为列，`row_json`保留完整字段；缺项留空 |
| `events.jsonl` | 完整发送、接收和模式/链路事件，用于重新分析；坏帧和断连时未完整接收的片段保留 |
| `session.json` | 会话参数、目录、记录数量、关闭状态及完整性；`complete=true`只表示记录器正常收尾，不表示硬件验收 |

UGRDS 另有独立日志，按用户指定的 **rsdd** 拼写保存至：
`D:\ForSTM32\xinjian\Underwater-Glider\station\logs\rsdd\rsdd_日期_时间_会话编号.log`。
每行是一条 JSON，含 `host_iso`、`t_ns`、`raw`、`complete`、`parsed`；`raw`保留接收文本，
行尾以 JSON 转义表示。坏校验和断连残片同样保存；仅完整且校验通过的数据进入有效面板。
该路径会在记录器启动时自动创建，与实验会话同步关闭，真实实验日志不作临时文件清理。

发送时间紧贴 `sendall()`，接收时间在唯一 `recv()` 返回后采集；同一批收包的多帧共享时间，
不会用解析/写盘时间伪造分辨率。墙钟带时区，用于对应现场笔记；时间差使用 PC 单调时钟。
发送失败可能已经发出部分字节；成功也只表示本机提交发送，不是执行确认。
文件由有界队列和独立线程批量写入；满队列、磁盘故障和收尾超时均报告不完整，停止指令仍照常执行。
正常退出控制台后再分析；突然终止进程或断电不能保证最后一批数据落盘。

下午重点测试 pitch/roll 时，进入 `auto_sti` 的 ACTIVE 阶段后静止约1秒，然后单轴按住动作，
松开后再保持静止约1秒，重复数次。每段至少留出3个有效 UGWRM 样本；避免同时改变其他执行器，
以便判断变化来源。沿用现有操作映射、转速和联锁；本次记录功能不自动发送实验动作。

正常退出后，在仓库根运行（替换为控制台显示的真实会话目录）：

```powershell
python -B station/tools/analyze_response.py --input-dir "station/logs/control/experiments/实际会话目录"
```

输出在该目录的 `response_analysis/`：`response_summary.csv`、中文 `summary.txt` 和可直接打开的
`response_timeline.svg`。分析只使用标准库，不需要安装绘图库。默认变化阈值为1脉冲、连续确认3样本、
允许最大样本间隔0.6秒、搜索窗口8秒；可用 `--pitch-threshold`、`--roll-threshold`、
`--consecutive-samples`、`--max-gap-s`、`--window-s` 调整，先结合静止噪声决定阈值。

结果是 **PC观测到开始/停止变化的时间区间**，包含通信、采样与遥测等待，不能宣称是纯机械延迟。
当前源码UGWRM名义5Hz，实际间隔由记录计算。Pitch优先看编码器脉冲；板端`cmd_rpm`已做安装方向转换，
不能假定与PC逻辑RPM同号。Roll仅输出脉冲估算，不能证明真实机械运动或排除失步。
PID OFF时UGCTL的部分测量字段是清零占位，不用于本分析。缺基线、坏帧、重复/批量同时间戳、
长缺口、校准/来源变化、日志不完整会标记未确认；非零转速之间的改变暂不计算延迟，仍保留曲线和原始数据。
命令前最后两个位置样本已经稳定时，停止响应标为 `precommand_stop_ambiguous`，避免把触限位等提前停止误判为松键效果。
`pulse_rate_per_s`为首次变化/稳定样本至连续确认结束之间的平均脉冲变化率，`rate_window_s`给出窗口长度；不换算成未标定的毫米速度。

后续恢复烧录后再考虑（本版未实现，也不会发送新查询）：

- `pid_control_service.c/.h`：保存成功应用的参数表，增加一致快照读回；
- `thruster_driver.c/.h`、`main.c:StartServoTask`：记录最终已写 PWM、来源和有效性；
- `ethernet_uart_driver.c/.h`：新增只读参数查询/应答，并扩展 UGCTL 上报最终推进输出及当前阶段深度目标。

验证使用本机模拟遥测，不连接真实设备：

```powershell
python -B -m unittest discover -s station/tools -p 'test_monitor*.py' -v
```

## 启动

在 station 目录执行：

    python -m pip install -r groundstation/requirements.txt
    python -m groundstation

设备网卡地址或端口不同时，可在 PowerShell 中先设置：

    $env:URGC_DEVICE_HOST = "192.168.57.100"
    $env:URGC_DEVICE_PORT = "8800"
    python -m groundstation

只查看网页骨架、不监听设备端口时：

    $env:URGC_DEVICE_LINK_ENABLED = "0"
    python -m groundstation

服务默认只监听 127.0.0.1:8000：

- GET /api/v1/health：后端进程健康状态；
- GET /api/v1/state：完整只读状态快照；
- GET /api/v1/recording：本次设备会话的记录目录、文件、行数及写入错误；
- WS /api/v1/ws/state：状态变更与 1 秒新鲜度心跳；
- /docs：FastAPI 自动接口文档。

frontend/dist 存在时会由核心托管；开发阶段可单独运行 Vite。不要使用多 worker：设备 TCP
socket 必须只有一个所有者，多 worker 会产生多个互相竞争的核心实例。

## 会话记录

默认自动记录到 station/logs/sessions 下的独立会话目录。每次设备 TCP 连接建立新目录：

- raw.log：主机收包时间和收到的完整文本行，包含未通过解析的行；不是逐字节抓包。
- events.jsonl：会话建立、断开原因。
- ekf_日期_时间.csv、ctl_日期_时间.csv、worm_日期_时间.csv：复用旧 CLI 的列定义，
  相应主题首次到达时创建。host_iso 使用实际接收时的 UTC 时间。

CSV 可以交给原绘图工具，输入目录需指定到对应会话目录；旧默认目录不会自动递归查找 sessions。
记录由有界队列和单独磁盘线程完成。写入失败或队列溢出会报告 error/dropped_records，
不会伪装为完整记录。当前按行 flush，不保证操作系统或机器突然掉电时最后数据已物理落盘。
原始数据/CSV保留故障 nan，网页快照将非有限值转换成 null 并附无效原因。

可用环境变量 URGC_LOG_DIR 指定目录；URGC_RECORDING_ENABLED=0 可禁用记录。
禁用设备链路时也不会创建日志。网页尚未提供历史会话浏览、下载和回放入口。

## 数据质量

state.required_topics / missing_topics 说明本阶段期望的三个主题是否齐备。
fresh 仅表示接收时间满足阈值；quality 和 invalid_fields 表示字段有效性，
不能把任何一个标志单独当作允许控制的依据。数值有限但板端判为无效时，原数保留供诊断，
正常监视面板应显示不可用。网页另用本地时钟检测推送中断，不依赖下一条后台心跳才变灰。

后续实施和验收安排统一记录在仓库根目录的本地 `project_status.md`（不随 Git 分发）。

## 本地验证

在仓库根目录，使用已装 requirements 的 station/.venv：

    station\.venv\Scripts\python.exe -B -m unittest station.tools.test_groundstation_state station.tools.test_groundstation_quality station.tools.test_groundstation_link station.tools.test_groundstation_recording -v
    station\.venv\Scripts\python.exe -X utf8 -B station/tools/verify_groundstation_workbench.py

后一项需要先构建前端，默认使用 Windows 已安装的 Edge，可用 --edge 指定浏览器路径。
它只运行明确标记的本地测试夹具，验证三个视口的四个页面、纯黑/渐变/导航样式、
关键表格完整可见、长记录路径、缺失有效位、非有限数、遥测停流、恢复和核心断开；
另验证主页操作分组不发送HTTP写请求。截图输出到 station/plots/urgc-workbench-check。
不会启动设备 8800 监听，也不代表真实板端或控制功能验收通过。
