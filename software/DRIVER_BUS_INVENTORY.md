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
| `Glider-Core/servo_driver.c` | DS3120 servo angle-to-PWM helper | `StartServoTask_Disabled` (not currently mounted) | `TIM4 CH3` | `PD14` | 50 Hz class PWM timing via TIM4 config | No bus address |

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
- `PD14` -> `TIM4_CH3` (servo PWM output pin in `servo_driver` design)
- `PD15` -> `TIM4_CH4` (configured in HAL)
- Timer setup: prescaler `119`, period `19999` (20 ms period, 50 Hz class PWM).

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

- `servo_driver` exists but is not mounted by default; `StartServoTask` is now used for SBUS-based motor/stepper RC control.
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
- `Glider-Core/servo_driver.h`
- `Glider-Core/servo_driver.c`
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
