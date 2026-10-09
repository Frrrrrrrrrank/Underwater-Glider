# URGC 开发计划与验收

更新时间：2026-10-08。产品：UnderwaterRobotGroundControl（URGC）。

## 使用边界与本轮决策

- 当前优先单设备、单操作者、持续网线连接；默认围绕台架观测与后续调参组织。
- 用户使用普通笔记本或外接显示器；主视口验证 1280×720、1366×768、1920×1080。
- 运行页不依赖整页下拉；顶部连接/模式/数据质量常驻，长表与诊断区内部滚动。
- 保留 Vue + TypeScript、Python/FastAPI、UGCP，设备 socket 由常驻核心独占。
- 北斗短报文、视频、手柄、多机观察继续后置。控制仍需独立服务和实机验收。
- 参考项目的布局与分层不等于其 MAVLink 功能可以直接接入本项目的 UGCP。

## 参考项目与采用方式

| 项目 | 官方具体做法 | URGC 采用方式 |
| --- | --- | --- |
| QGroundControl | Vehicle/Fact/元数据关联状态、单位、枚举；Fly、Plan、配置、分析分工 | 字段定义与业务数据质量共享，运行/诊断/记录分视图 |
| Cockpit | Profile/View/Widget 组织，跨视图共享顶部状态，统一变量绑定 | 复用面板、任务切换；第一版固定布局，通用拖拽编辑器后置 |
| Mission Planner | Flight Data 内有 Quick/Status/Messages/Actions 等分区，支持实时调参曲线和遥测日志回放 | 目标/反馈/误差并排；数值表与曲线互补；先恢复实验记录，再做回放 |
| MAVProxy | 通信核心可加载图形/诊断模块，连接后自动保存遥测日志 | 记录归核心所有；网页生命周期不决定采集生命周期 |

官方依据：

- [QGC Fact System](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/fact_system.html)
- [Cockpit 工作区](https://blueos.cloud/cockpit/docs/latest/usage/advanced/)
- [Cockpit 前端车辆模型](https://github.com/bluerobotics/cockpit/blob/master/src/stores/mainVehicle.ts)
- [Mission Planner Flight Data](https://ardupilot.org/planner/docs/mission-planner-flight-data.html)
- [MAVProxy 模块](https://ardupilot.org/mavproxy/docs/modules/index.html)
- [MAVProxy 记录](https://ardupilot.org/mavproxy/docs/getting_started/logfiles.html)

Cockpit 的车辆语义和部分控制位于浏览器/Electron renderer；BlueOS 是另一个车端服务系统。
URGC 将业务与采集集中到 Python 核心是本项目的选择，并非完整复制 Cockpit。

## 里程碑

| 阶段 | 交付 | 验收关口 | 状态 |
| --- | --- | --- | --- |
| M1 实用只读工作台 | 固定主视口、字段定义、目标/反馈表、诊断与记录页；缺失/失效/断线处理；会话 CSV | 三档窗口无文档溢出，非有限故障帧不破坏 API，日志失败可见 | 软件实现和本地验收通过；布局待用户审阅 |
| M2 只读链路验收 | 订阅首帧/主题恢复、原始记录回放、系统快照适配、协议函数从 CLI 抽离 | 停流、重连、板端重启、坏帧、真机三主题；离线回放能复现同一状态 | 待实施/实机 |
| M3 控制和参数服务 | 提取旧 PidSession/直控/回零服务，统一发送与回执；两层租约 | 从请求到板端结果全程可观察，网页丢失、核心退出、板端失联各有验证 | 后续 |
| M4 网线完整工作流 | 参数编辑/校验/应用、操作记录、调参趋势、启动入口与恢复流程 | 用 URGC 完成已定义的台架实验并复核结果 | 后续 |
| M5 扩展 | 北斗任务消息与确认；按实际需求追加多机观察/手柄/视频 | 每条链路按能力、时效和断链语义独立验收 | 后续 |

M1 的软件测试通过后仍需 M2 真机验收，不能把本地模拟数据视为板端通过。
M2 不应阻挡用户继续审阅 M1 的工作台；但 M3 开放控制必须先满足数据与断链验收。

## 字段及显示契约

前端名称、单位、精度、枚举统一放在字段定义中，表格/趋势图共用。
后端只做数据质量与设备语义，不依赖 CSS 布局。未知、无效、过期分别显示，
无效值不显示为 0；故障枚举与诊断计数仍保留。

| 分组 | 首批来源 | 显示约束 |
| --- | --- | --- |
| 姿态/深度 | UGEKF.roll_deg/pitch_deg/yaw_deg/depth/vD，fl | 深度 m 向下为正；角度 deg；fl bit1 判估计有效 |
| 控制比较 | UGCTL.target_depth/depth/depth_error；pitch_sp_deg/pitch_deg/pitch_error_deg | 请求目标、板端设定、反馈明确区分，不能把请求当反馈 |
| 浮力机构 | UGCTL.target_displacement_mm/displacement_mm/displacement_error_mm/pump_enable/pump_direction | 单位 mm，显示泵方向与运行状态；后续补系统快照的独立传感器有效位 |
| 蜗杆 | UGWRM.stroke_percent/mm_from_front/pulse/cmd_rpm/fb_rpm | span_valid/data_fresh 决定哪些位置量可用；命令转速不等于反馈转速 |
| 模式/故障 | UGCTL.mode/phase/armed/output_valid/fault/core_fault；UGWRM.home_state/home_fail | 中文枚举，保留原值供诊断，解除 ARM 不等于通信失败 |
| 链路/记录 | device_link、各主题 age、recording | 浏览器离线后设备状态只可称最后已知，记录状态不可继续确认 |

CSV 保持旧列定义，目录改为独立会话；原始文本保留校验失败行和 nan，网页 JSON 将非有限数
转换为 null 并提供 invalid_fields。quality=invalid 表示至少部分字段无效，不是丢弃整帧。
缺失主题通过 missing_topics 表达，fresh 不等于控制就绪。

## 布局讨论项

本轮默认方案 A：表格为主，短时趋势辅助，适合当前台架观测。
备选方案 B：趋势为主，表格放侧边，适合之后频繁做阶跃/响应整定。
两者共享同一状态和面板，不需要两套通信代码。
用户已认可监控页基础布局；另要求独立遥控主页，具体操作交互待讨论。

页面范围扩展为主页、运行监控、遥测诊断、会话记录。主页目前为只读仪表和操作方案，
分组按钮仅切换说明，不发送控制请求。真正的参数和控制服务到 M3/M4 实现。
地图在具有可信定位数据及对应任务时再接入；
不能把惯导 pN/pE 直接当可靠的全球航迹。

## 遥控主页：已实现布局与待确认操作

用户明确要求：默认主页；侧栏标题“工作台侧栏”；导航无 01/02/03；选中项全宽直角、
无左右边框；Redbird 顶栏紫色到黑色渐变；其余整体纯黑。
已实现布局为左侧操作分组、中央姿态/航向、右侧状态与操作方案、底部关键遥测。
HomeView 只接收 App 的共享遥测；有效位缺失、过期、断线时不显示默认水平姿态或假航向。

主页暂适配 Glider，不代表已经通用支持其他水下机器人。后续根据车辆适配器报告的能力
选择面板和动作；蜗杆/油囊不能默认套给没有这些执行器的 ROV。多机观察与多类型适配是两回事。

水下机器人参考：

- [Cockpit 水下视图](https://blueos.cloud/cockpit/docs/latest/usage/getting-started/#rovs-submarines)：
  Video/HUD 服务第一人称操纵，Map 适合具备定位/自主能力的任务；当前 URGC 先用有效姿态和深度。
- [ArduSub 电机设置](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/motors_ardusub.html)：
  单电机测试与自动辨向属于设备设置，并有明确测试使能步骤；URGC 的维护动作不铺满日常操作页。
- [Neptus LAUV 布局](https://github.com/LSTS/neptus/blob/develop/conf/consoles/lauv.ncon)：
  地图/任务执行状态/计划控制分工，留作未来北斗任务视图参考，不映射为实时遥控指令。

| 现有码中的操作 | 迁移入口 | 拟呈现方式（待用户确认） |
| --- | --- | --- |
| 双推进总量/差动/归零 | contcp_t.run_auto_mode、_apply_thrust_direction | 主页直接遥控面板；目标与反馈分开，初期低限幅 |
| 蜗杆正反转/禁用 | contcp_t.run_auto_mode | 前移/停止/后移、转速、行程、限位；按住点动需要新增松开/失焦处理 |
| 油泵双向/停泵 | contcp_t.run_auto_mode、pump_driver.IoControlPump_Poll | 位移、方向、停泵及联锁原因；请求不等于实际驱动 |
| 横滚步进轴 | contcp_t.input_control_packet | 执行器展开面板；现有协议/pack支持，旧AUTO无快捷交互 |
| 接管/交还SBUS | contcp_t.send_control_packet、send_neutral_and_release | 常驻控制来源；实际来源需系统快照，不能从网页选项推断 |
| PID模式/目标/ARM/OFF | contcp_t.PidSession | 主页闭环面板；选择目标、发送、确认、使能分开 |
| 蜗杆回零开始/取消 | contcp_t.send_worm_calib、worm_axis状态机 | 主页快捷入口，展开条件/确认；运行时阶段与取消常驻，禁重复启动 |
| PID参数/磁标定 | PidSession.push_parameters、tools/mag_* | 独立调参/维护视图，参数显示编辑值与已确认值 |

接控制前必须解决：

1. 旧 AUTO 是持续手动遥控，并非自主模式；pack/LATCHED 不能直接作为网页默认策略。
2. 全零保留 DIRECT、退出 PID、交还 SBUS、取消回零是不同命令。当前无已验证的统一全局急停。
   取消回零或释放控制后可能由低优先级控制源接管，必须明确退出后的来源。
3. 回零按板端任务呈现，不能按按钮发送成功或倒计时显示完成；重复开始会重新进入 SEEK，不能盲重试。
   当前回零没有通信租约，网页离线不能声明“回零已停止”。需明确断链策略和常驻取消入口。
4. 现码 worm_axis.h 的回零宏为 350/35 RPM，旧注释/历史文档仍写 150/15；
   contcp_t 手动 W/S 实码 ±50 RPM，旧帮助仍写 ±200。接入前核实，不在UI复制旧数字；本轮未改运动参数。
5. 控制服务仍必须通过 M2/M3 的状态、回执、租约、互斥与实机验收；本轮只讨论呈现，不增加写接口。

## 验收清单

- [x] Edge 实测 1280×720、1366×768、1920×1080 三个视图：主文档无横/纵向溢出；长记录路径不撑宽，监控三个关键短表无需内部滚动。
- [x] 新主页在上述三档视口通过；四页导航、纯黑背景、紫黑渐变、直角无左右边框选中项均以浏览器样式验证。
- [x] 五个主页分组仅改变面板，浏览器检查无HTTP写请求；缺有效位、非有限姿态/深度、停流和后端断开时仪表/数值失效。
- [x] 浏览器实测后端关闭、遥测停流：在线/新鲜标识撤销，数值不再作为有效值显示；恢复遥测后重新显示。设备断链语义另有状态单测。
- [x] 部分主题未到、数值为 nan、fresh 但板端位无效：质量单测覆盖；有效位缺失与 nan 另经浏览器故障注入验证。
- [x] 严格 JSON 序列化通过；其他正常字段继续更新。
- [x] 三类 CSV、原始行、会话开始结束事件落盘；重连文件分离；磁盘错误可见。
- [x] 23 项地面站专项测试通过；上位机全量 53 项中通过 52 项、跳过系统 Python 缺 FastAPI 的 1 项（该项已在站端虚拟环境单独通过）；前端类型检查/生产构建通过。
- [ ] 实际板端验收：三主题数值、单位、频率、位标志与旧 CLI/板端一致。

浏览器验证入口：station/tools/verify_groundstation_workbench.py。只用回环随机端口、
测试夹具和隐藏 Edge，不连接板端；截图位于 station/plots/urgc-workbench-check。
屏幕尺寸指网页可用视口，而非显示器铭牌分辨率；系统缩放和浏览器工具栏会减少可用空间。

控制阶段单独增加验收：操作权限/租约、事务身份、发送失败、拒绝/超时/确认、急停/OFF语义。
浏览器只提交操作意图；实时续发归核心，但必须随操作者租约到期停止。
北斗阶段另定义任务版本、有效期、去重、确认、持久化；不缓存过期实时控制包等待重连补发。
