# UnderwaterRobotGroundControl Core

简称：URGC。

这是常驻地面站核心的只读遥测阶段。默认监听板端 TCP 客户端使用的
`192.168.57.100:8800`，复用 `contcp_t.py` 的解析规则接收 `$UGEKF`、
`$UGCTL`、`$UGWRM` 和 `$UGRDS`，再把最新状态发布到共享状态。

建立连接后仅发送 `MSG_STREAM_CONTROL`：先请求 `EKF | CTL | WRM`
（`0x0D`），再尝试加入 RDSS（`0x2D`）；旧固件拒绝 RDSS 位时仍保留基础遥测。
有效 UGRDS schema2 使用共享解析器，按2秒时效发布并记录到会话 `rdss.csv`；
网页专用北斗面板及 BAT 接入尚未完成。当前没有执行器、回零、PID、参数写入或任务控制接口。
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

## 网页手柄只读预览（G0）

工作台侧栏的“手柄输入”页复用现有 `gamepad_control.py`，由核心的
`GamepadPreview` 托管 `GamepadService`。手柄应连接到运行核心的 Windows 电脑，
并非任意浏览器所在电脑。启动核心时才加载 XInput 和启动采集，退出核心时停止采集；
缺少 XInput 或没有手柄不会阻止遥测页面工作。

仅检查手柄、不监听控制板的方式（在仓库根目录、新 PowerShell 窗口执行）：

```powershell
$env:URGC_DEVICE_LINK_ENABLED = '0'
Set-Location station
.\.venv\Scripts\python.exe -B -m groundstation
```

打开 `http://127.0.0.1:8000`，选择“手柄输入”和槽位 0～3。需先在
`station/frontend` 执行 `npm run build`，核心才会托管最新页面。
重新启用设备监听前关闭此核心，并删除该环境变量或在新终端按原方式启动；
不要在同一个设备端口上同时运行 URGC 与 `contcp_t.py`。

- `GET /api/v1/input/gamepad` 提供只读预览；同一份结构也附在 `/api/v1/state`
  与现有 WebSocket 状态的 `gamepad` 字段，不新增设备连接或写接口。
- 采集沿用约 50Hz，网页状态推送上限 10Hz。使用核心单调时钟判断输入年龄，
  网页继续累加年龄；默认超过 200ms、拔出、采集退出或核心失联时撤销有效预览。
- 选槽位只改变查看对象。推力百分比、pitch/roll RPM、尾舵目标变化率来自原映射函数，
  配置限幅沿用当前 `GamepadConfig`，不是实测推力、转速或舵角。
- Y/A 和十字键按住有对应意图，松开/同按为零；左摇杆只左右偏转不原地转。
  RT/LT 松开表示停止改变尾舵目标而非回中；B/X 分别显示回中/退出意图，但本页不执行。
- G0 不调用 `run_auto_sti_mode`，不发送 CONTROL、PID、校准、中立或释放指令，
  不积分舵角、不实现 B 后等待释放的控制会话逻辑，也不将手柄输入写入实验控制日志。
  后续共享控制会话及租约完成后再开放真实控制，不能用预览替代实机验收。

可设置 `$env:URGC_GAMEPAD_ENABLED = '0'` 完全禁用手柄采集；其余遥测功能保持可用。

## 自身 GPS 与 EKF 融合诊断（UGPS schema1）

配套新固件在 EKF 订阅打开时约 1Hz 推送独立 `$UGPS`；沿用原有 EKF 订阅位，
不改变 `$UGEKF` 字段，不新增 GPS 订阅命令。旧固件未上报时显示“未上报；旧固件可能不支持”，
UGPS 不列为必需主题。它来自本机 GNSS，地面目标坐标和 RDSS 缓存坐标不作为自身定位。

CLI 仍由 `contcp_t.receive_thread` 独占板端接收；`monitor.py` 读取现有 8801 快照。
面板分开显示自身经纬度、NE 测量、EKF 的 NE 估计、创新/NIS、接收年龄与最近融合年龄。
新诊断帧到达不表示新定位，也不表示 EKF 接受了该定位；判断接受需结合 `accept_count`、
`fuse_age_ms`、状态与有效位。`reference_verified=0` 明示“安装杆臂/真北未实测确认”，
`aiding_valid=1` 也不表示导航已经就绪。此诊断不发执行器命令或改变控制权限。

报文为 `$UGPS,<33个十进制整数>*HH\r\n`；HH 是 `$` 与 `*` 之间 ASCII 的逐字节异或。
严格解析定义在 `station/gps_telemetry.py`，字段顺序为：

```text
schema,seq,t_ms,state,reason,flags,sample_seq,rx_age_ms,fuse_age_ms,
utc_date,utc_ms,lat_e7,lon_e7,origin_id,origin_lat_e7,origin_lon_e7,
meas_n_mm,meas_e_mm,est_n_mm,est_e_mm,innov_n_mm,innov_e_mm,nis_milli,
r_var_cm2,pnn_cm2,pne_cm2,pee_cm2,accept_count,reject_count,reset_count,
duplicate_count,drop_count,input_reject_count
```

- `seq/t_ms` 标识诊断快照，`sample_seq` 标识定位样本；重复快照不刷新 CLI/Web 接收新鲜度。
  诊断主题超过 3000ms 未更新、设备断连或换会话后，不显示旧值为当前值。
- `rx_age_ms/fuse_age_ms` 为板端快照时的接收/融合年龄，`4294967295` 表示未知；
  `utc_date` 为 YYYYMMDD，`utc_ms` 为 UTC 日内毫秒，不当作 MCU 单调时钟。
- 经纬度除以 `1e7` 得到度，NE 位置/创新除以 `1000` 得到米，NIS 除以 `1000`；
  R/P 除以 `10000` 得到 m²。`pne_cm2` 是允许负数的交叉协方差。
  `r_var_cm2` 是配置的观测方差，不是接收机报告的 HACC 或实际定位精度。
- `state`：0 等待定位、1 质量观察、2 融合中、3 失锁推算、4 重新捕获、5 等待 EKF。
  `reason`：0 无、1 输入无效、2 重复、3 时间倒退、4 过期、5 未就绪、6 质量观察、
  7 跳点、8 创新超限、9 数值异常、10 水平重置、11 IMU 过期。
- `flags` bit0～7：地理原点有效、已锚定、辅助有效、参考经实测确认、协方差有效、
  测量有效、创新有效、曾融合。无效位下可能保留最后有限历史值，缺失或数值溢出时用 0 占位；
  面板均依有效位显示不可用，原始日志保留收到的整数，不能把历史值或占位零当作当前定位。
- `reject_count` 为导航门控/算法拒绝，`input_reject_count` 为输入解析拒绝；
  重复、丢样、重置另计，不能把收到一帧简单等同于完成一次融合。

CLI 默认实验目录新增 `telemetry_ugps.csv`，原 CSV 列顺序保留，GPS 列追加到尾部；
`row_json` 与 `events.jsonl` 保留原值，坏帧和断连残片保留原文及解析失败状态。
URGC 共用同一个严格解析器和字段有效性，状态 API 可读 `telemetry.ugps`，
会话目录增加 `gps_日期_时间.csv`；本阶段不新增网页 GPS 专页，也不新增设备 socket。

## contcp_t 双终端监控（旧主题无需更新固件）

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
| 尾舵上位机指令 | 最近一次 `sendall` 成功的发送角度（°）、来源/阶段、发送时间和记录年龄；不是板端执行确认或实测角度。失败发送不覆盖最近成功值；超过2.5s标“发送记录超龄”，断连保留历史标识，重连清空旧值 |
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
完整面板收到 GPS 诊断后会增加多行，建议终端高度至少45行；窗口过矮会显示截断提示。

### 尾舵 DS820MG：网线指令、记录与 Ozone 调试

当前标称总行程90°，以中位为0°，软件允许−45°～+45°，不是中位两侧各90°。
需要配套烧录已放开到±45°的固件；旧版板端仍可能把请求限制在±10°。
先确认机构两端无干涉，再扩大实际测试范围；这里是舵机轴的名义角度，连杆后的尾舵角需标定。

键盘 `auto` 中 R/F 按一次持续增大/减小目标，速度10°/s；V停止改变目标并保持，T回中，
Z清零所有目标，Q/ESC发中立和释放后退出。R/F松开不会自动停止，端点为±45°。

手柄 `auto_sti` 中 RT 增大尾舵目标、LT 减小，按压量越大变化越快，满按30°/s。
两扳机各有10%死区，死区外线性映射到0～30°/s；例如单扳机55%对应15°/s。
松开两扳机或两者同时超过死区时保持当前目标，端点为±45°。
B直接将目标设为0°，不加软件回中限速；按过B后，必须B、LT、RT全部松开才恢复扳机控制。
B只回中尾舵，推进、pitch和roll仍按原输入控制。X/ESC则发送全中立和释放并退出。
进入模式前须摇杆回中、Y/A/十字键左右/B松开、LT/RT分别处于死区内；准备阶段只发送中立。
控制循环以约50Hz积分目标，以10Hz发送绝对角度；B回中和扳机停止改变目标立即发送。
这是目标角度变化速度，三线舵机没有实际舵角/转速反馈。位置指令仍由舵机按自身能力跟随。
沿用现有CONTROL21协议及尾舵固件，本次手柄映射只需更新PC代码。
连接后输入 `auto_sti`，或 `auto_sti 0 20` 指定槽位0和推进上限20%（不改变尾舵30°/s）。
单独检查手柄输入可运行 `python -B station/tools/gamepad_monitor.py`，它不会连接控制板或发送指令。

monitor顶部显示实际成功发送的字段，不用板端PWM反推实测角度，也不新增设备连接。
“发送记录超龄”的2.5s只是PC显示阈值，不证明板端收包、租约超时或实际舵机位置。
旧19字节中立/释放包没有尾舵字段，按其撤销偏角语义显示0°；释放后其他来源可能接管。

每次连接的 `commands.csv` 保留 `rudder_angle_cdeg`（百分之一度），并追加易读列
`rudder_angle_deg`（度）；筛选 `success=True` 查看成功提交的指令，`False`表示失败/可能部分发送。
结合 `source`、`phase`、`seq`、`host_iso`、`tx_start_ns/tx_end_ns`定位动作；面板记录目录给出完整路径。
手柄模式来源为 `auto_sti`；`input_json`同时保存LT/RT、B及回中序号、积分使用的目标变化速度
`rudder_rate_deg_s`和等待松开的标志`rudder_wait_release`，角度以实际编码发送字段为准。
未下发控制包时不会凭空生成0°记录；旧CSV没有新增列时仍可将cdeg除以100。

Ozone调试使用新固件的 `g_rudder_debug.enabled` 和 `target_angle_cdeg`，默认都为0。
先退出AUTO、关闭PID并确认无校准；常规仲裁为OFF且泵未启用才允许调试。
遥控保持中立可以保留，运动命令或开泵会撤销调试使能。
先写 `target_angle_cdeg=1000`（+10°），再写 `enabled=1`，恢复程序运行；
`+4500/0/-4500`分别表示+45°/中位/−45°。观察 `g_rudder_state.source=3` 和
`applied_pulse_us`确认下发命令，不把它们当角度反馈；旧 `g_dbg_servo_pulse_us` 与
`g_rudder_state` 都是观察量，不用它们发指令。
写目标0回中，`enabled=0`交还正常控制；DIRECT、PID、SBUS手动或校准接管会清除enabled。
Ozone暂停时任务不会消费新目标，必须Resume；PWM可能继续保持旧目标，暂停不等于停舵。
Ozone直接改变的目标并非PC发送包，因此不会出现在monitor的“上位机指令”或commands.csv中。

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
- GET /api/v1/input/gamepad：核心手柄采样、质量及基础映射意图；
- GET /api/v1/recording：本次设备会话的记录目录、文件、行数及写入错误；
- WS /api/v1/ws/state：遥测与手柄共享推送，按 100ms 周期合并变更/发送新鲜度心跳；
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

    station\.venv\Scripts\python.exe -B -m unittest discover -s station/tools -p 'test_groundstation_*.py' -v
    python -B -m unittest discover -s station/tools -p 'test_frontend_gamepad.py' -v
    station\.venv\Scripts\python.exe -X utf8 -B station/tools/verify_groundstation_workbench.py

后一项需要先构建前端，默认使用 Windows 已安装的 Edge，可用 --edge 指定浏览器路径。
它只运行明确标记的本地测试夹具，验证三个视口的五个页面、纯黑/渐变/导航样式、
关键表格完整可见、长记录路径、缺失有效位、非有限数、遥测停流、恢复和核心断开；
另验证假手柄按住/松开、槽位选择保持、无板端预览、手柄独立过期及全程无HTTP写请求。
截图默认输出到仓库 `.buildtmp/urgc-workbench-check`，检查完清理本次产物。
不会启动设备 8800 监听，也不代表真实板端或控制功能验收通过。
