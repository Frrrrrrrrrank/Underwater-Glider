# 状态估计与 GNSS 水平恢复

本目录已实现自机 GNSS 位置、可选速度融合和上浮后的水平位置/速度恢复。
算法源工程为同级 `Glider_sim/src/glider_sim/ug_estimation`，`ug_estimation` 是板端集成副本。
内部采用 NED（北、东、下）/ FRD（前、右、下），位置/速度参考点为 IMU。
GNSS 观测属于天线相位中心，压力提供独立深度观测，RMC 不生成高度观测。

## 从硬件到软件

```text
自机天线/北斗模块 → UART7 中断 → 字节 ring → GliderCore_RunRxTask
  → process_line / 严格 GNRMC builder → GnssSample
  → main 回调 → EkfTask_PublishGnss → 8 项 FIFO
  → EKF task 每步最多 4 项 → GnssNavigation_Step
  → 固定原点投影 → 正常 p/v 更新或原子 p/v 恢复
  → StateEstimate 一致快照 → Ethernet task → $UGPS,2 → PC 唯一 recv
  → CLI monitor/实验 CSV，或 Web StateStore/SessionRecorder
```

串口任务只解析、发布，EKF task 是滤波器和导航管理器的唯一写入者。
FIFO 入队/出队只在临界区复制结构，矩阵计算在临界区外执行。
队列满时丢最旧项并增加 `drop_count`；每轮最多消费4项，避免突发无限占用任务。
事件唤醒之外还有20ms兜底，即使没有新GNSS，也会刷新年龄和失锁状态。

地面目标坐标、卫星短信的位置和180秒短信缓存均不进入自机观测链。
`$UGPS` 是控制板经以太网链路发给PC的诊断，不是卫星空口包。
CLI与Web是两种入口，只能选一个占用设备TCP端口；monitor读取CLI本机快照。

## 文件和函数入口

`Glider-Core`、`Core`、`Glider-App` 相对于 `software`，`station` 相对于仓库根。

| 文件/入口 | 输入 → 输出及职责 |
| --- | --- |
| `Glider-App/common/gnss_sample.h` | `GnssSample`：UTC、接收时间、经纬度、可选NE速度、来源和有效位 |
| `Glider-Core/glider_rdss_codec.c:GliderRdssCodec_TryMakeGnssSample` | 完整GNRMC → 合格位置样本或NO_FIX事件；坏报文不发布 |
| `Glider-Core/glider_core.c:process_line` | 记录完整行处理时刻、调用原始行hook和严格builder，与短信业务分流 |
| `Core/Src/main.c:GliderCore_OnGnssSample` | 样本转交 `EkfTask_PublishGnss`，不直接调用滤波器 |
| `Glider-App/tasks/ekf_task.c:EkfTask_PublishGnss/StartEkfTask` | 发布并唤醒EKF，IMU/GNSS事件与周期兜底共用单一任务 |
| `ekf_bridge.c:EkfBridge_SubmitGnss/takeGnssInput/EkfBridge_Step` | 8项FIFO/每步4项；先处理本轮IMU/深度，再逐项GNSS，发布一致快照 |
| `gnss_navigation.c:GnssNavigation_Step/refresh` | 去重、年龄、GPS自身跳点、原点、可信度和恢复选择 |
| `gnss_navigation.c:GnssNavigation_Project` | WGS84椭球面ECEF差 → 原点当地NE米数，double消差后转float |
| `gnss_navigation.c:GnssNavigation_OnFilterReset` | 全滤波重启时保留原点/已消费UTC，撤销当前融合可信度 |
| `ug_estimation/src/embedded/ug_ekf_glider.cpp` | 四个GPS C API，分开处理历元身份、接收时间、状态和gyro新鲜度 |
| `ug_estimation/include/ug_estimation/core/measurements/gps.h` | 天线p/v模型、姿态/gyro bias雅可比与gyro噪声项 |
| `ug_estimation/src/core/eskf.cpp` | `updateGps/updateGpsVelocity/reanchorHorizontal`、NIS/Joseph更新、原子恢复 |
| `gnss_diagnostics.h`、`state_estimate.h` | 诊断结构/状态/原因/flags，与 `UgEkfOutput` 一起发布 |
| `ekf_bridge.c:EkfBridge_CopyState` | 跨任务读取同一版本的滤波结果、诊断和年龄 |
| `Glider-Core/ethernet_uart_driver.c:EthernetUartDriver_SendGpsTelemetry` | 快照 → 定点UGPS2，EKF订阅下约1Hz独立发送 |
| `station/gps_telemetry.py:ugps_parse/ugps_invalid_fields` | schema1/2严格解析和字段有效性，CLI/Web共用 |
| `station/monitor.py:_gps_lines` | 原始GPS、当前估计、速度来源/可信度、最近恢复前后值和年龄分列 |

## 输入：位置、可选速度与无定位事件

严格builder只接 `GNRMC`：检查XOR、UTC/日历、A/V状态、方向/经纬度和可选模式。
两位年份按2000～2099解释，UTC支持到毫秒；闰秒和更细小数暂拒绝。
无mode的旧RMC可接受；有效位置模式支持A/D/F/R/P，显式无定位标志转成事件。

`lat_deg_e7/lon_deg_e7` 是十进制度乘1e7，南纬/西经为负。
`seq` 只是发布序号，`utc_date_yyyymmdd/utc_ms_of_day` 用于历元身份。
`GNSS_VALID_REQUIRED` 代表位置输入检查通过，不代表精度或已被EKF接受。

SOG/COG可选：`speed_mps=knots×1852/3600`，
`vN=speed×cos(COG)`、`vE=speed×sin(COG)`，COG从真北顺时针计算。
空值、坏格式、COG不在 `[0,360)` 或速度低于0.3m/s时不生成二维速度，位置仍可使用。
低速方向不可靠，因此接收机报告零速也不在这里变成零速约束；COG不当作艇体yaw。
RMC速度来源只表示接收机提供SOG/COG，尚无证据确认模块内部是否采用多普勒测速。

校验与UTC合法的V、mode=N或导航状态=V发布 `GNSS_VALID_NO_FIX` 事件，
坐标/速度清零但不置有效位。管理器增加 `no_fix_count`、撤测量有效位、更新失锁状态，
不刷新最后有效定位或融合年龄。坏校验不采信内容，只增加输入拒绝计数。

RMC没有HACC/GST精度信息，当前 `quality_source=0`，由配置提供观测方差。
结构预留 `quality_source=1` 和位置/速度方差，但尚未接入GGA/GST的历元关联。
`GliderCore_OnRawLine(line,t_rx_us)` 是默认空的weak hook；当前没有连续原始NMEA落盘器。
UGPS/CSV是诊断快照，不能代替完整原始报文来证明频率、设备质量和内部延迟。

## 身份、时间与固定原点

四个GPS C API的最后两个参数均为 `epoch_id, t_us`：

| API | 行为 |
| --- | --- |
| `UgEkf_PushGpsPosition` | 正常位置NIS/Joseph更新 |
| `UgEkf_PushGpsVelocity` | 正常速度更新，使用近时gyro、bias与天线杆臂 |
| `UgEkf_ReanchorHorizontal` | 一次验证、一次提交位置/速度及协方差，当前恢复策略使用 |
| `UgEkf_ResetHorizontalPosition` | 兼容的仅位置重置，保留速度均值；当前导航恢复使用上一项 |

`epoch_id` 是经校验的日期/UTC排序键，只比较身份和先后，不计算时间差。
位置、速度各有独立水位，同历元可各用一次；成功原子恢复同时消费两路，不能再补融合同历元。
普通位置NIS拒绝后可用同历元进入明确的恢复路径；已接受的观测不能重复应用。
同UTC即使换接收时间/发布序号仍重复；两个新UTC即使 `t_us` 相同也可分别处理。

`t_us` 是MCU处理完整行的单调时间，来自 `GliderTime_NowUs`，目前分辨率1ms。
任务临界区扩展HAL 32位毫秒回绕；core用double绝对秒相减后转FP32 dt。
同1ms收到不同历元不人为给时间加1；超龄与gyro匹配仍用真实接收时间。
UTC未映射MCU/PPS，也没有历史状态回滚；包含接收机/UART/ring延迟的近时更新需实测验证。
`max_sample_age_ms` 只约束解析到消费的年龄，不能检测接收机内部延迟。
已见过但陈旧的历元会推进去重水位，不会靠重复送入变成新观测。

首次合格新定位且EKF允许恢复时立即建立原点和位置锚点，不再要求先连续三帧。
原点在一次任务内固定，质量升降、失锁、重捕获均不移动；磁标定重启保留原点和UTC水位，
重新对齐后必须等待新定位。投影要求原点纬度绝对值小于85°、完整ECEF弦长不超过100km。

## 稀疏跟踪与上浮恢复

每条新定位先检查输入、UTC、年龄和GPS自身合理性，再选择更新或恢复：

1. 无锚点：首次定位或滤波器重启后，用原子p/v恢复立即建立锚点。
2. 已锚定且水平数值/速度不合理、速度协方差损坏，或距上次位置融合达到30s：
   新合格定位直接恢复，不等大残差先被NIS拒绝。
3. 短间歇且预测可用：正常位置更新保留Ppv，让第0/4/9秒这种稀疏观测逐渐约束速度。
   若普通位置更新被NIS拒绝且融合年龄超过5s，同历元允许转入恢复。
4. 位置通过后，有可用RMC速度就独立更新速度；速度被拒不撤销本次已接受的位置。

GPS自身突然跳点与旧滤波漂移分开处理：相对最近接受GPS的距离超过
`3×sqrt(2R)+max_speed×接收间隔` 时，先保存候选并拒绝该点；
后续另一新历元若与候选一致，可确认新的位置簇并恢复。候选和历史位置比较限于30s内。
这不要求新GPS靠近旧漂移估计，也不意味着首帧就能识别所有坏定位。

有可信RMC速度就用它重建IMU水平速度；没有速度时，
**一次性设置水平速度均值为0、方差为25I (m/s)²**，来源未知、速度可信位为0。
这是宽不确定性的先验，不是ZUPT，也不会周期性把艇速拉回零。
后续原始位置经Ppv学习速度，不把同一组位置差分后再作独立速度重复融合。

恢复需要匹配gyro才能补偿速度杆臂。若RMC位置仍在输入年龄界内，但该接收时刻与最新
gyro相差超过250ms，该次恢复降级为“位置＋未知速度先验”。原始RMC速度仍留诊断，
增加速度拒绝计数，不因速度缺少近时gyro而丢掉位置锚点。当前IMU本身过期仍禁止融合。

`reanchorHorizontal` 保留pD/vD、姿态、gyro/accel bias及其11维健康协方差块，
按新观测和杆臂重建水平块/交叉项；检查失败不部分改状态。
仅水平P非有限时，预测可隔离其NaN，继续传播保留11维；普通磁/深度等纠正会被健康检查
或原子纠正检查阻止，直到水平恢复。这不是另一个能独立完整运行的11态EKF。
保留块损坏时不能靠水平恢复掩盖故障。

位置/速度接收机噪声暂按互不相关建模，共同姿态/bias的杆臂相关性保留；
设备真实跨观测相关性尚无证据。缺DVL、声学或滑翔模型约束时，水下纯IMU水平推算会漂移。

## 参数与状态

`GnssNavigation_DefaultConfig` 默认值如下。数量/持续时间只用于提升可信度，
不再阻挡首帧锚定和后续每次位置更新。

| 参数 | 默认值 | 实际用途 |
| --- | --- | --- |
| `position_sigma_m` | 5m | 无设备精度时位置R=25I m² |
| `velocity_sigma_mps` | 1m/s | 无设备速度精度时速度R=1I (m/s)² |
| `unknown_velocity_sigma_mps` | 5m/s | 无速度恢复先验Pvv=25I (m/s)² |
| `max_speed_mps` | 5m/s | GPS跳点运动上界、输入/估计速度合理性 |
| `max_position_std_m` | 50m | 辅助有效的位置协方差上界 |
| `max_velocity_std_mps` | 2m/s | 速度可信的协方差上界，含交叉项行和 |
| `max_sample_age_ms` | 1500ms | 解析到消费的最大输入年龄 |
| `fix_timeout_ms` | 5000ms | 超过后重新捕获；此时位置NIS拒绝允许恢复 |
| `max_imu_age_ms` | 250ms | 参与融合的最新IMU年龄 |
| `velocity_confirm_ms` | 2000ms | 位置逐渐约束速度的最短时间；同时至少两次普通位置更新 |
| `long_loss_ms` | 30000ms | 达到后推算不可信，下一合格定位恢复 |
| `min_good_fixes` | 2 | 辅助/速度可信度提升的接受数量门限，不是融合入口门槛 |
| `reference_verified` | 0 | 安装杆臂/真北尚未实测确认 |

`UgEkf_DefaultConfig` 的 `nis_gate_gps/nis_gate_gps_velocity` 均为9.21034；
`gps_velocity_gyro_var=1e-4 (rad/s)²`、`gps_velocity_gyro_max_age_us=250000`。
旧 `gps_reset_velocity_var=4 (m/s)²` 仅用于兼容的仅位置reset，不用于当前未知速度先验。

| schema2 state | 含义 |
| --- | --- |
| 0 WAIT_FIX | 等待能建立锚点的新定位 |
| 1 POSITION_ONLY | 已有位置锚点，速度尚未达可信条件 |
| 2 TRACKING | 速度也已达到当前跟踪条件 |
| 3 DR_UNTRUSTED | 长失辅、水平数值或速度不合理 |
| 4 REACQUIRING | 无定位事件或短时辅助超时，保留可用学习状态 |
| 5 WAIT_EKF | IMU过期，或保留状态不满足更新/恢复条件 |

测量有效、已锚定、速度可信、辅助有效分别判断，TRACKING不授予航点控制权限。
`valid_for_control` 仍是原有姿态/数值/IMU健康入口，水平导航还要看GNSS状态/年龄和参考验证。
实际 `EkfBridge_Init` 的天线FRD杆臂为(0,0,0)，磁偏角也未完成实艇确认，
`reference_verified=0`；下文1m杆臂仅用于解释。

## 一个上浮恢复案例

假设已有固定地理原点，水下长时间缺GPS后，滤波位置漂移到北向500m，
水平速度错误地达到20m/s。艇头朝东，示例天线在IMU前方1m，此刻角速度为零。

1. 上浮收到合格新GNRMC。投影得到**天线**NE=(10,21)m，
   SOG≈1.943844节、COG=90°，转成天线NE速度=(0,1)m/s。
2. 样本带UTC身份和真实接收时间进入FIFO。EKF任务确认IMU/输入新鲜。
   已超过30s失辅，或20m/s估计超速度上界，因此直接选择原子恢复。
3. 艇头朝东时，FRD前方1m对应NED偏移(0,1)m；扣去杆臂，
   **IMU**位置恢复为(10,20)m。零角速时无旋转速度，IMU速度恢复为(0,1)m/s。
   深度、姿态和bias保留，旧500m/20m/s水平信息及相关不确定性被重新建立。
4. 若无可用速度或gyro无法近时匹配，位置仍恢复为(10,20)m，
   速度先验为(0,0)、Pvv=25I，速度显示未知；后续位置继续学习实际速度。
5. UGPS2保存恢复前后p/v、原因和年龄。PC分开显示天线测量、当前IMU估计、
   最近恢复后的历史锚点。IMU继续预测后当前位置可变化，历史恢复记录不跟着改写。
   恢复本身不等于连续跟踪，后续合格观测和协方差再决定可信位。

## UGPS schema2 与记录

格式为 `$UGPS,2,...*HH\r\n`，HH是 `$` 与 `*` 之间ASCII字节逐字节XOR。
共**59个十进制整数字段，不含UGPS前缀**；前33项原顺序不变，追加26项：

```text
schema,seq,t_ms,state,reason,flags,sample_seq,rx_age_ms,fuse_age_ms,
utc_date,utc_ms,lat_e7,lon_e7,origin_id,origin_lat_e7,origin_lon_e7,
meas_n_mm,meas_e_mm,est_n_mm,est_e_mm,innov_n_mm,innov_e_mm,nis_milli,
r_var_cm2,pnn_cm2,pne_cm2,pee_cm2,accept_count,reject_count,reset_count,
duplicate_count,drop_count,input_reject_count,
quality_source,velocity_source,vel_fuse_age_ms,no_fix_count,
meas_vn_mmps,meas_ve_mmps,est_vn_mmps,est_ve_mmps,vel_nis_milli,
vel_r_var_cm2ps2,pvn_cm2ps2,pvne_cm2ps2,pve_cm2ps2,vel_accept_count,
vel_reject_count,recovery_reason,reset_before_n_mm,reset_before_e_mm,
reset_before_vn_mmps,reset_before_ve_mmps,reset_after_n_mm,reset_after_e_mm,
reset_after_vn_mmps,reset_after_ve_mmps,reset_age_ms,reset_valid_mask
```

| flags位 | 含义 |
| --- | --- |
| 0～3 | 原点有效、已锚定、辅助有效、安装/真北参考已验证 |
| 4～7 | 位置协方差有效、原始位置测量有效、位置创新有效、本轮滤波曾融合 |
| 8 | 速度可信 |
| 9 | 当前输入样本有有效速度测量 |
| 10 | 有最近恢复记录 |
| 11 | 速度协方差有效 |
| 12 | 速度创新有效 |

`quality_source`：0配置/未知、1设备报告；当前RMC是0。
遥测 `velocity_source`：0未知、1原生RMC SOG/COG、2原始位置更新经滤波逐渐约束。
它与core `horizontal_velocity_source` 枚举不同，后者0无、1未知先验、2接收机速度，不要混用数值。
位置推得的可信速度不会伪造一次原生速度融合，`vel_fuse_age_ms` 保留真实速度观测年龄。

位置/创新除以1000为m，速度除以1000为m/s；位置R/P除以10000为m²，
速度R/P除以10000为(m/s)²，NIS除以1000；`pne/pvne` 为有符号交叉协方差。
年龄是快照时的板端年龄，UINT32_MAX表示从未有过/未知，不能当0。
原始测量、接收/融合/恢复年龄彼此独立，1Hz新快照不等于新GPS。

`recovery_reason`：0无、1首次定位、2失去辅助、3滤波器重启、4数值恢复、5拒绝后恢复。
`reset_valid_mask` 低8位依次对应before N/E/Vn/Ve、after N/E/Vn/Ve。
非有限/超定点范围只撤对应位，其他有限分量仍可展示；0占位不能读成恢复到0。
普通字段也按flags显示；日志允许保留有限历史值，失效时不把它当当前有效值。

管理器 `accept_count/reject_count` 记录位置处理，恢复另计 `reset_count`；
速度单列 `vel_accept_count/vel_reject_count`；重复、FIFO丢样、codec输入拒绝分别计数。
core正常更新计数不把原子恢复算成普通GPS更新，不能直接等同管理器计数。

发送缓冲及底层SendRaw上限已扩到1024字节，忙时不改正在异步发送的缓冲。
沿用EKF订阅，BAT/RDSS优先，不扩旧UGEKF或72B快照。
PC兼容schema1原有状态语义，新增字段缺失时显示未知，混合CSV的新列留空。
CLI实验记录和Web会话记录共用字段/质量规则；Web无GPS专页，不增加设备socket。
具体启动及PC字段解释见仓库根 `station/groundstation/README.md`。

## UART7 原始报文采集与离线分析

这一层用于测量真实接收情况。它旁路记录 UART7 按 LF 结束的行，保留 CR、NUL、非 ASCII 字节和错误的 NMEA 校验；不改变 GNRMC 到 EKF 的输入资格。每行最多保留前384字节，超过时保留原长度和截断标志。未出现 LF 的尾部尚不是完整行；UART ring 已经丢掉的字节无法重建。

采集使用现有 UGCP `MSG_STREAM_CONTROL=0x02` 的 `RAW=0x40` 位，默认关闭。订阅为完整 mask；首次开启先发送旧固件认识的基础订阅，再发送扩展订阅。持续采集每5秒续租，板端15秒没有续租就停止并清队列。正常遥测优先，每个1秒预算窗口最多发送2048个线传输字节，8槽队列满时丢最旧行并计数。

转发协议为十进制整数加十六进制原文，外层另做 XOR 校验：

```text
$UGRAW,1,capture_id,seq,rx_us_hi,rx_us_lo,tx_us_hi,tx_us_lo,queue_drop,uart_drop,line_drop,flags,src_len,kept_len,HEX*HH\r\n
```

| 字段 | 含义 |
| --- | --- |
| capture_id / seq | 本次采集编号 / 观测行序号；重新开启产生新编号，正常续租不重置；32位自然回绕 |
| rx_us_hi/lo | UART7任务组装到LF时的板端时间，高低32位拼成64位微秒，时钟实际分辨率1ms |
| tx_us_hi/lo | Ethernet任务准备转发时的板端时间；与rx差值表示板端排队等待，不是完整GNSS测量延迟 |
| queue_drop | 本次采集队列丢弃行数；随新capture清零 |
| uart_drop | UART7字节ring累计丢字节数；单位是字节 |
| line_drop | 原业务decoder累计超长行丢弃数；采集仍可能保存该行前384字节 |
| flags / src_len / kept_len | bit0表示截断；原行长度/保存长度，包含CR、不含结束LF |
| HEX | 原文字节的十六进制编码；内层NMEA的`*HH`保留原样，外层校验正确不代表内层定位有效 |

板端时间的重建公式为 `(hi << 32) | lo`。原始记录结构 `RawCaptureRecord` 定义在 `Glider-Core/raw_capture.c`，保存时刻、采集编号、序号、计数、长度和字节数组；PC字段顺序定义在 `station/raw_telemetry.py:UGRAW_WIRE_FIELDS`。

| 文件/函数 | 在链路中的职责 |
| --- | --- |
| `glider_core.c:process_rx_byte` | UART7 ring出队后独立组装原始行，LF时调用hook；同一时刻继续原业务解码 |
| `main.c:GliderCore_OnRawLine` | 将字节/长度/时刻及源丢弃计数交给采集队列 |
| `raw_capture.c:RawCapture_Observe/SetEnabled/Poll` | 有界入队、启停/续租和到期清队列；跨任务短临界区保护 |
| `raw_capture.c:RawCapture_TryTransmit/format_snapshot` | 静态快照格式化HEX包、检查预算、复核队头身份；发送成功才出队 |
| `ethernet_uart_driver.c:EthernetUartDriver_SendRawCapture` | 仅UART空闲时调用，保持在途静态缓冲不变，排在其他遥测之后 |
| `station/tools/analyze_gnss_capture.py` | `load_capture`选择日志并核验外包；`classify_payload`复核输入资格；`analyze_records`统计独立历元、间断、恢复和质量 |

序号缺口、queue_drop、uart_drop、line_drop的含义和单位不同，不能相加成总丢包数。记录开启后的首行也可能是已经开始接收的部分行；判断定位仍以完整NMEA校验和字段为准。

PC复用唯一设备接收者和异步落盘队列，CLI与Web二选一。离线分析将报文到达、校验正确、有效位置、NO_FIX、新UTC历元、重复和乱序分别统计。GGA/GST先记录出现情况和原始字段，当前固件仍只把严格合格的GNRMC送入EKF。仅显式声明静止时计算相对采样中心的NE分散与半径分位数，称为静止重复性；没有已知真值时不称绝对定位精度。

例：板端在1.0、1.1、1.2秒收到三条UTC相同的GNRMC，原始记录保留三条，到达间隔为0.1秒，但独立定位只有一个历元；在2.0秒收到下一UTC时，独立定位的间隔才是1秒。坏校验帧也会留下，便于区分接收机无定位、链路坏帧和EKF拒绝。

启动、输出路径与分析命令见 `station/groundstation/README.md` 的原始采集说明。

## 软件验证与现场边界

2026-11-06原始采集阶段：板端主机全量132项通过；站端全量401项中391通过、10项因Web依赖跳过，`station/.venv`补跑Web47项及RAW17项全部通过，覆盖跳过项。末次CSV残片边界修正后，分析专项19项和跨层流水线2项再次全部通过。

新增板端测试实际编译生产FIFO/formatter/发送函数，覆盖异步缓冲、队头被替换、坏字节、超长行、租约和带宽。`software/tests/test_raw_capture_pipeline.py`把真实sender输出送入CLI/Web异步记录器，再执行分析CLI，验证4条到达得到2个独立位置、1个重复、1个NO_FIX，输入定位恢复耗时4秒、板端排队10ms。这些是构造数据，未采到真实天线日志。

完整ARM构建通过，text466060/data10844/bss95720；交付在 `software/build/uart7_capture_20261106`（相对仓库根），含ELF/HEX/BIN/MAP和build_info/SHA256；此前GPS恢复固件保留在 `software/build/gps_recovery_20261106`。本阶段不改共享滤波算法；此前sim/板端×FP32/FP64检查通过、11共享文件一致的证据保留。

实际 `-Og` 静态分析：新增发送入口已知链120B（不含snprintf库栈），UART7任务已知链960B、Ethernet任务952B，各分配2KiB；EKF已知最深链仍9076B，分配16KiB。静态链不含未建模库/RTOS/中断边界，真实高水位仍需上板测量。

在 `D:\ForSTM32\xinjian` 工作区根执行：

```powershell
$env:PATH = 'D:/msys64/mingw64/bin;D:/msys64/usr/bin;' + $env:PATH
$env:TMP = 'D:/ForSTM32/xinjian/.buildtmp'
$env:TEMP = $env:TMP
$env:TMPDIR = $env:TMP
$env:PYTHONPATH = (Resolve-Path Underwater-Glider).Path
D:/xinjian/python.exe -B -m unittest discover -s Underwater-Glider/software/tests -p 'test_*.py'
D:/xinjian/python.exe -B -m unittest discover -s Underwater-Glider/station/tools -p 'test_*.py'
Underwater-Glider/station/.venv/Scripts/python.exe -B -m unittest discover `
  -s Underwater-Glider/station/tools -p 'test_groundstation_*.py'
Underwater-Glider/station/.venv/Scripts/python.exe -B -m unittest discover `
  -s Underwater-Glider/station/tools -p 'test_raw_telemetry.py'
D:/xinjian/python.exe -B Glider_sim/src/glider_sim/ug_estimation/test/host_replay/run_gps_checks.py `
  --source Glider_sim/src/glider_sim/ug_estimation `
  --source Underwater-Glider/software/Glider-App/estimation/ug_estimation `
  --eigen Underwater-Glider/software/Middlewares/Third_Party/Eigen `
  --build-dir .buildtmp/gps_ekf_checks
```

尚需实艇确认：杆臂/真北、模块原始频率和质量、延迟、任务耗时/栈高水位、烧录后的水面/水下行为。
现场依次验证开阔水面静止、已知路径运动、遮挡/恢复、稀疏露出水面、完整下潜上浮，
同时采集原始NMEA、UGPS、IMU/深度和参数。合成回放不能代替现场结果。
滑翔水动力模型、海流状态、DVL/声学融合仍是后续独立工作；当前未接航点或水平位置闭环。

## 外部实现依据

- **DUNE AUV，b6b8e0f**：`BasicNavigation::consume(GpsFix)` 先检查定位有效性和 HACC/HDOP；
  AUV `task()` 调用 `m_kal.update(0.0)`，`KalmanFilter::update()` 明确在门限非零时才做创新拒绝。
  这条路径允许合格 GPS 纠正很大的预测漂移；GPS/DVL 同时超时则使用 RPM 航速模型。
  [GPS 质量入口](https://github.com/LSTS/dune/blob/b6b8e0f84371dd9a4b5dbafb3667ced19bfbda0f/src/DUNE/Navigation/BasicNavigation.cpp#L537-L591)、
  [AUV 更新及速度来源](https://github.com/LSTS/dune/blob/b6b8e0f84371dd9a4b5dbafb3667ced19bfbda0f/src/Navigation/AUV/Navigation/Task.cpp#L603-L669)、
  [门限含义](https://github.com/LSTS/dune/blob/b6b8e0f84371dd9a4b5dbafb3667ced19bfbda0f/src/DUNE/Navigation/KalmanFilter.cpp#L156-L164)。
- **ArduPilot 共享 EKF3**：开始 GPS 绝对辅助时调用 `ResetVelocity` 和 `ResetPosition`；
  正常位置更新也允许超时后重置，并在速度超时时恢复速度。它把正常创新更新与重建参考分开。
  [模式转换](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp#L462-L520)、
  [位置/速度超时恢复](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp#L925-L958)。
- **Seaglider basestation3，fda58d22**：`compute_displacements` 用模型航速和航向积分位移；
  `compute_dac` 用起止 GPS 位移减去模型位移，再除以潜次时间，得到深度平均流。
  这是岸端潜次后处理，为本艇将来滑翔模型/海流估计提供参考，不是实时 EKF 重捕获代码。
  [模型位移](https://github.com/iop-apl-uw/basestation3/blob/fda58d2291eae2982518651543e83cb248efc136/MakeDiveProfiles.py#L1156-L1234)、
  [GPS 闭合与平均流](https://github.com/iop-apl-uw/basestation3/blob/fda58d2291eae2982518651543e83cb248efc136/MakeDiveProfiles.py#L1237-L1254)。

上述实现提供设计参考，不表示本工程具有它们的全部传感器、状态或任务能力。
DUNE的推进器RPM航速模型不能直接当作无动力滑翔段模型；Seaglider的潜次GPS闭合
可用于后续独立分析模型/平均海流，不是本次嵌入式实时滤波的一部分。
