# UnderwaterGlider Driver and Bus Inventory

## Scope
- This document records the intended driver/bus allocation for the current hardware iteration.
- Pump RS485 code is retained in the repository for later use, but `Bus3` is allocated to the linear displacement sensor in this configuration.
- Pump GPIO control remains active through `IoControlPump_*`.

## 1) Driver Modules and Hardware Mapping

| Module | Main function | Runtime entry point | Bus/Peripheral | Pins | Link settings | Device address / ID |
|---|---|---|---|---|---|---|
| `Glider-Core/glider_core.c` | RDSS AT/NMEA communication and short-message state machine | `StartUartRxTask` + `StartUartTxTask` | `UART7` | `PE8 TX`, `PE7 RX` | `115200`, `8N1` | Target platform card ID macro: `381127` |
| `Glider-Core/glider_rdss_codec.c` | RDSS/NMEA frame parser only | Called by `glider_core` | No direct hardware | N/A | N/A | N/A |
| `Glider-Core/imu_core.c` | IMU frame receive/parse/cache | `Start_IMU_Task` + `IMU_Core_Init(&huart5)` | `UART5` | `PB13 TX`, `PB12 RX` | `115200`, `8N1` | No bus address (point-to-point UART) |
| `Glider-Core/displacement_sensor.c` | Linear displacement sensor for oil-out volume estimation (MODBUS-RTU) | `StartUsart3Task -> DisplacementSensor_Poll` + `USART3 IRQ -> DisplacementSensor_OnRxComplete` | `USART3` RS485 / `RS485_Bus3` | `PD8 TX`, `PD9 RX`, `PD12 DE` | `115200`, `8N1`, RS485 DE high | MODBUS slave ID `0x02` |
| `Glider-Core/pump_driver.c` | Pump RS485 control logic retained for later reuse; GPIO pump control remains active | `IoControlPump_Init` + `IoControlPump_Poll` for GPIO only | GPIO pump control active; RS485 part disabled | `PB10` speed, `PD14` direction | GPIO level control via SBUS switches | N/A |
| `Glider-Core/motor_driver.c` | Pump actuator motor control (MODBUS-RTU closed-loop stepper) | `StartUsart2Task -> UMotor_StartTask` | `USART2` RS485 | `PA2 TX`, `PA3 RX`, `PA1 DE` | `115200`, `8N1`, RS485 DE high | Default active node ID `0x01` |
| `Glider-Core/stepper_driver.c` | Open-loop pulse/dir stepper control | Called by `StartServoTask` RC control loop (`Stepper_SetContinuousSignedRpm`) | `TIM1 CH2` + GPIO | `PE11` pulse, `PE13` dir, `PE10` PUL-, `PE12` DIR- | PWM pulse generation (timer-based) | No bus address |
| `Glider-Core/rudder_driver.c` | DS820MG 尾舵绝对位置命令 | `StartServoTask -> RudderDriver_Apply` | `TIM4 CH4` | `PD15 / J9 pin1` | 333 Hz，1500 us 中位，首期限幅 ±10° | 无总线地址 |

## 2) Bus-to-Device Mount Table

### UART/RS485 buses
- `USART2` RS485 (`PA1/PA2/PA3`): motor bus for `UMotor` nodes (`0x01`~`0x04`), handled by `StartUsart2Task -> UMotor_StartTask`.
- `USART3` RS485 (`PD12/PD8/PD9`, `RS485_Bus3`, external `J17/J18`): linear displacement sensor bus, handled by `StartUsart3Task -> DisplacementSensor_Poll`.
- `UART5` (`PB13/PB12`): IMU module.
- `UART7` (`PE8/PE7`): RDSS terminal module (`glider_core`).

### Configured but currently not used by Glider-Core modules
- `USART1` RS485 (`PA12/PA9/PA10`): initialized in HAL, no `Glider-Core` module bound.
- `UART5` (`PB13/PB12`): initialized in HAL, no `Glider-Core` module bound.
- `UART8` (`PE1/PE0`): initialized in HAL, no `Glider-Core` module bound.
- `SPI1` (`PA4/PA5/PA6/PA7`): configured master, no current device driver in `Glider-Core`.
- `FDCAN1` (`PD1/PD0`): configured, no current device driver in `Glider-Core`.
- `TIM2 CH3/CH4` (`PB10/PB11`): configured, no current driver logic attached.
- `ADC1` (`PC5/PB0/PB1` analog pins): configured with regular channel `ADC_CHANNEL_5`, no active acquisition task found.
- `Glider-Core/pump_driver.c` RS485 API: retained in repo intentionally, but not mounted by default because `Bus3` is reserved for the displacement sensor. GPIO pump control from this file remains mounted.

## 3) UART and RS485 Configuration Summary

Intended bus settings for this allocation:
- `USART2` motor bus: `115200`, `8N1`, RS485 DE high.
- `USART3` displacement-sensor bus (`Bus3`): `115200`, `8N1`, RS485 DE high.
- `USART1`: RS485-capable spare bus, no custom driver mounted by default.
- `UART4`, `UART7`, `UART8`: normal UART links as listed above.

## 4) Device Addressing and Protocol Details

### 4.1 Motor bus (`motor_driver` on USART2)
- Node count macro: `UMOTOR_NODE_COUNT = 4`.
- Default enabled node IDs in `g_umotor`: only `0x03` enabled by default, other slots disabled.
- MODBUS function codes used:
  - `0x06` write single register
  - `0x03` read holding registers
- Key control/status registers:
  - Speed command: `0x0033`
  - Start command: `0x0037`
  - Enable: `0x0039`
  - Stop: `0x0038`
  - Position low/high: `0x0034` / `0x0035`
  - Status word: `0x0004`
  - Feedback speed: `0x0006`
- Read windows used:
  - Verify block: start `0x0014`, count `10`
  - Status block: start `0x0000`, count `16`
  - Fault block: start `0x0194`, count `7`

### 4.2 Displacement sensor (`displacement_sensor` on USART3 / `RS485_Bus3`)
- MODBUS slave ID: `0x02`.
- Poll period: `100 ms`.
- Driver sends read frame for `0x0000` with count `0x0010` (16 regs).
- Before sending, the driver arms a persistent 37-byte buffer with `HAL_UART_Receive_IT`.
- The ISR only publishes completion/error events; `StartUsart3Task` parses CRC/data and owns the `100 ms` timeout/recovery.
- Response length is `1 address + 1 function + 1 byte-count + 32 data + 2 CRC = 37 bytes`.
- Parsed measurement source registers:
  - `0x0008` high word
  - `0x0009` low word
  - Combined as float displacement in mm.
- Physical RS485 output path: `USART3 -> U15 -> RS485_Bus3 -> J17/J18`.

### 4.3 Pump control reserve (`pump_driver` RS485 kept but not enabled)
- Keep `Glider-Core/pump_driver.c` in the repository for later pump work.
- Do not bind `PumpDriver_Init/PumpDriver_Poll` to `Bus3` in this configuration.
- Keep `IoControlPump_Init/IoControlPump_Poll` active for GPIO pump control.
- If pump RS485 control is re-enabled later, revisit UART assignment and protocol conflict with the displacement sensor.

### 4.4 RDSS terminal (`glider_core` on UART7)
- No MODBUS-style numeric device address.
- Uses AT/NMEA command channel.
- Target platform card ID macro: `GLIDER_TARGET_CARD_ID = "381127"`.
- Local card ID is queried at runtime (`$CCICA,0,00*7B`) and stored dynamically.

### 4.5 IMU (`imu_core` on UART5)
- No numeric bus address in current protocol implementation.
- Frame headers: `0x7E 0x23`.
- Function codes used by parser include `0x04`, `0x16`, `0x26`, `0x32`, `0x01`, `0x81`.

## 5) Timer/GPIO Function Pins (Motor/Servo/Stepper Related)

### TIM1
- `PE11` -> `TIM1_CH2` (used by `stepper_driver` pulse output)
- `PE14` -> `TIM1_CH4` (configured in HAL, currently not used by `stepper_driver`)
- Timer setup in `main.c`: prescaler `120-1`, period `1000-1`.

### TIM4
- `PD14` -> `TIM4_CH3`：MSP 遗留代码将其改为普通 GPIO；不启动该通道。
- `PD15` -> `TIM4_CH4`：DS820MG 尾舵，J9 pin1；pin2=共享7.4V，pin3=GND。
- 当前100MHz核时钟：PSC=99、ARR=3002，1us计数、3003us周期；CCR4=1500为中位。驱动启动前核验真实时钟。

### TIM2
- `PB10` -> `TIM2_CH3`
- `PB11` -> `TIM2_CH4`
- Configured but no active `Glider-Core` driver binding found.

### Extra GPIO used by stepper wiring
- `PE13` direction output
- `PE10` and `PE12` forced low for common-cathode PUL-/DIR- wiring

## 6) Other Peripheral Configuration Snapshot

### SPI1
- Pins: `PA4 NSS`, `PA5 SCK`, `PA6 MISO`, `PA7 MOSI`
- Mode: master, 2-line
- IOC calculated rate: about `40.0 Mbits/s`
- No mounted device driver found in current `Glider-Core` code.

### FDCAN1
- Pins: `PD0 RX`, `PD1 TX`
- IOC nominal bitrate calculation: `520833` bps
- No mounted device driver found in current `Glider-Core` code.

### ADC1
- Analog pins configured: `PC5 (INP8)`, `PB0 (INP9)`, `PB1 (INP5)`
- Regular conversion configured for `ADC_CHANNEL_5` (single conversion)
- No periodic ADC sampling task found in current application logic.

## 7) Intended Runtime Thread-to-Driver Binding

- `StartUartRxTask` -> `GliderCore_RunRxTask`
- `StartUartTxTask` -> `GliderCore_RunTxTask`
- `Start_IMU_Task` -> IMU data update loop + `IMU_Core_RunRxTask`
- `StartUsart3Task` -> displacement event parsing/timeout in `DisplacementSensor_Poll` + `IoControlPump_Poll` (GPIO pump control)
- `USART3 IRQ -> HAL_UART_RxCpltCallback -> DisplacementSensor_OnRxComplete` only publishes the 37-byte receive-complete event
- `StartUsart2Task` -> `UMotor_StartTask`
- `PumpDriver_*` RS485 code is retained but not mounted by default
- `StartServoTask` runs RC control state machine: OFF/all-up interlock + open-loop mode (CH5 mid), left stick Y controls `UMotor` node `0x01`, left stick X controls open-loop stepper.

## 8) Notes / To-Confirm Items

- 旧 `servo_driver` 源码保留但不再编译；尾舵使用独立 `rudder_driver`，由 `StartServoTask` 在网线直控分支仲裁，其他来源回中。
- `Bus3` (`USART3 -> U15 -> RS485_Bus3 -> J17/J18`) is reserved for the linear displacement sensor in this document.
- `PumpDriver_*` RS485 code is intentionally preserved but should remain disabled while `Bus3` is assigned to the displacement sensor; `IoControlPump_*` remains active for GPIO pump control.
- `main.h` defines `IMU_DRY_Pin` on `PC4`, while `MX_GPIO_Init` configures `IMU_DRY_Pin` plus `PC7` as inputs.
- `SDMMC1` init function exists but `MX_SDMMC1_MMC_Init()` call is commented out in startup path.
- `USART1`/`UART4`/`UART8` are initialized but not bound to current custom drivers.

## 9) Source Files Used for This Inventory

- The custom-driver list below reflects the intended module set for this bus allocation; some files may live on another branch or be reintroduced later.

- `Core/Src/main.c`
- `Core/Src/stm32h7xx_hal_msp.c`
- `Core/Inc/main.h`
- `software.ioc`
- `Glider-Core/pump_driver.h`
- `Glider-Core/pump_driver.c`
- `Glider-Core/motor_driver.h`
- `Glider-Core/motor_driver.c`
- `Glider-Core/displacement_sensor.h`
- `Glider-Core/displacement_sensor.c`
- `Glider-Core/imu_core.h`
- `Glider-Core/imu_core.c`
- `Glider-Core/stepper_driver.h`
- `Glider-Core/stepper_driver.c`
- `Glider-Core/rudder_driver.h`
- `Glider-Core/rudder_driver.c`
- `Glider-Core/glider_core.h`
- `Glider-Core/glider_core.c`
- `Glider-Core/glider_rdss_codec.h`

## 10) SBUS Channel Mapping (FS-iA10)

### 10.1 Value Convention

- Typical observed min: `240`
- Typical observed center: `1024`
- Typical observed max: `1807`
- Keep raw values as `uint16_t` in driver outputs and apply normalization in control layers.

### 10.2 Channel Semantics

| Channel | Code Index Constant | Named Field | Meaning | Direction/Range Notes |
|---|---|---|---|---|
| CH0 | `SBUS_CH_RIGHT_STICK_X` | `right_stick_x` | Right stick left/right | Center ~1024, left to ~240, right to ~1807 |
| CH1 | `SBUS_CH_RIGHT_STICK_Y` | `right_stick_y` | Right stick up/down | Center ~1024, up increase, down decrease |
| CH2 | `SBUS_CH_LEFT_STICK_Y` | `left_stick_y` | Left stick up/down | Same trend as CH1 |
| CH3 | `SBUS_CH_LEFT_STICK_X` | `left_stick_x` | Left stick left/right | Same trend as CH0 |
| CH4 | `SBUS_CH_SWITCH_LEFT_OUTER` | `switch_left_outer` | Left-most switch (2-pos) | Up ~240, down ~1807 |
| CH5 | `SBUS_CH_SWITCH_LEFT_MID_1` | `switch_left_mid_1` | Left side 2nd switch (3-pos) | Up ~240, mid ~1024, down ~1807 |
| CH6 | `SBUS_CH_SWITCH_LEFT_MID_2` | `switch_left_mid_2` | Left side 3rd switch (3-pos) | Same as CH5 |
| CH7 | `SBUS_CH_SWITCH_RIGHT_OUTER` | `switch_right_outer` | Right-most switch (2-pos) | Same as CH4 |
| CH8 | `SBUS_CH_WHEEL_LEFT` | `wheel_left` | Left wheel | Left roll decreases to ~240, center ~1024, right roll ~1807 |
| CH9 | `SBUS_CH_WHEEL_RIGHT` | `wheel_right` | Right wheel | Current record says left roll increases; recheck hardware trend before closed-loop use |

### 10.3 Suggested Thresholds for Switch Parsing

- Two-position switch:
  - UP: `< 640`
  - DOWN: `> 1400`
- Three-position switch:
  - UP: `< 640`
  - MID: `640..1400`
  - DOWN: `> 1400`

### 10.4 Debug Watch Targets

- Raw channels: `g_sbus_readout.channels[]`
- Named controls: `g_sbus_readout.controls.*`
- Link status: `g_sbus_readout.frame_lost`, `g_sbus_readout.failsafe`
- Update indicator: `g_sbus_readout.frame_update_count`
- RC control state: `g_rc_ctrl_mode`, `g_rc_ctrl_off_reason`
- RC command outputs: `g_rc_worm_cmd_rpm`, `g_rc_stepper_cmd_rpm`
- RC key inputs: `g_rc_sw_left_outer`, `g_rc_sw_left_mid_1`, `g_rc_sw_left_mid_2`, `g_rc_sw_right_outer`, `g_rc_left_stick_y`, `g_rc_left_stick_x`, `g_rc_right_stick_y`


## 11) DS820MG 尾舵网线 AUTO（2026-10-09）

- 启动新版 `station/contcp_t.py`，设备连接后输入 `auto`。必须同时使用新版板端固件；
  旧固件不识别消息0x07，不能据PC显示认定尾舵或其他AUTO执行器已响应。
- 按一次R/F以10°/s持续增减目标角；松键继续，V停止改变目标并保持，T回中。
  R/F同时按停止；V/T/Z按住优先。初期PC和板端共同限制±10°，即约1389～1611us。
- Z清零并回中，Q/ESC退出时尝试先中立再释放；Ctrl+C、接收线程断线或发送失败也进入清理。
  释放后沿用既有全机控制权分配，不能称为全局急停；尾舵本轮没有SBUS接管入口。
- AUTO每100ms发送绝对目标，尾舵键立即触发发送；消息0x07的payload为旧19B字段加
  int16小端角度（度×100），总帧35B。协议允许±4500cdeg，驱动机械限幅更窄。
  旧0x01/0x04仍是19B，收到旧包明确撤销尾舵偏角。无锁存尾舵消息。
- 尾舵只在有效DIRECT源下动作；2.5s租约超时回中，PID/回零/其他源获权时回中。
  若DIRECT后来重新获得控制权且PC仍发新鲜目标，会执行该目标。
- Ozone：`g_rudder_state`；`source`为0未启动/1中位保护/2网线命令。
  `requested_angle_cdeg/applied_angle_cdeg`是命令，`applied_pulse_us`是已写CCR脉宽；
  三线舵机没有角度返回。H请求的72B快照复用servo_pulse_us/servo_mode，长度不变。
- 标定修改 `rudder_driver.h` 的RUDDER_DIRECTION（±1）、RUDDER_CENTER_TRIM_US、
  RUDDER_LIMIT_CDEG；放宽角度时同步PC AUTO_RUDDER_LIMIT_DEG并先验证连杆行程。
  同TIM4的其他通道共用周期，不能同时按50Hz配置。
- 实机待验：确认板版本/J9脚序与供电；示波器核对333Hz/1500us后，先空载测试中位、
  R/V/F/T、Z/退出和拔网线，再接连杆。7.4V堵转电流标4.3A，检查共享电源瞬态压降。
  外供电时断开J9 pin2并共地；回中是带动力动作，停止PWM不等于断电。
- 离线验证入口：`station/tools/test_auto_rudder_control.py`、
  `software/tests/test_rudder_driver.py`、`software/tests/test_rudder_protocol.py`；
  后两者需要主机GCC，测试环境建议设置PYTHONDONTWRITEBYTECODE=1。
