# UnderwaterGlider Driver and Bus Inventory

## Scope
- This is a read-only snapshot of the current workspace configuration.
- No application source code was modified while creating this file.
- Peripheral parameters are taken from `Core/Src/main.c`, `Core/Src/stm32h7xx_hal_msp.c`, `software.ioc`, and `Glider-Core/*`.

## 1) Driver Modules and Hardware Mapping

| Module | Main function | Runtime entry point | Bus/Peripheral | Pins | Link settings | Device address / ID |
|---|---|---|---|---|---|---|
| `Glider-Core/glider_core.c` | RDSS AT/NMEA communication and short-message state machine | `StartUartRxTask` + `StartUartTxTask` | `UART7` | `PE8 TX`, `PE7 RX` | `115200`, `8N1` | Target platform card ID macro: `381127` |
| `Glider-Core/glider_rdss_codec.c` | RDSS/NMEA frame parser only | Called by `glider_core` | No direct hardware | N/A | N/A | N/A |
| `Glider-Core/imu_core.c` | IMU frame receive/parse/cache | `Start_IMU_Task` + `IMU_Core_Init(&huart4)` | `UART4` | `PB9 TX`, `PB8 RX`, `PB15 CTS` | `115200`, `8N1` | No bus address (point-to-point UART) |
| `Glider-Core/displacement_sensor.c` | Linear displacement sensor (MODBUS-RTU) | `StartUsart3Task -> Displacement_StartTask` | `USART3` RS485 | `PD8 TX`, `PD9 RX`, `PD12 DE` | `115200`, `8N1`, RS485 DE high | MODBUS slave ID `0x02` |
| `Glider-Core/motor_driver.c` | Closed-loop stepper multi-node control (MODBUS-RTU) | `StartUsart2Task -> UMotor_StartTask` | `USART2` RS485 | `PA2 TX`, `PA3 RX`, `PA1 DE` | `115200`, `8N1`, RS485 DE high | Node IDs `0x01`~`0x04` |
| `Glider-Core/stepper_driver.c` | Open-loop pulse/dir stepper control | Currently mounted in `StartServoTask` via `Stepper_TestTask` | `TIM1 CH2` + GPIO | `PE11` pulse, `PE13` dir, `PE10` PUL-, `PE12` DIR- | PWM pulse generation (timer-based) | No bus address |
| `Glider-Core/servo_driver.c` | DS3120 servo angle-to-PWM helper | `StartServoTask_Disabled` (not currently mounted) | `TIM4 CH3` | `PD14` | 50 Hz class PWM timing via TIM4 config | No bus address |

## 2) Bus-to-Device Mount Table

### UART/RS485 buses
- `USART2` RS485 (`PA1/PA2/PA3`): motor bus, 4 nodes (`0x01`~`0x04`).
- `USART3` RS485 (`PD12/PD8/PD9`): displacement sensor, slave `0x02`.
- `UART4` (`PB9/PB8`): IMU module.
- `UART7` (`PE8/PE7`): RDSS terminal module (`glider_core`).

### Configured but currently not used by Glider-Core modules
- `USART1` RS485 (`PA12/PA9/PA10`): initialized in HAL, no `Glider-Core` module bound.
- `UART5` (`PB13/PB12`): initialized in HAL, no `Glider-Core` module bound.
- `UART8` (`PE1/PE0`): initialized in HAL, no `Glider-Core` module bound.
- `SPI1` (`PA4/PA5/PA6/PA7`): configured master, no current device driver in `Glider-Core`.
- `FDCAN1` (`PD1/PD0`): configured, no current device driver in `Glider-Core`.
- `TIM2 CH3/CH4` (`PB10/PB11`): configured, no current driver logic attached.
- `ADC1` (`PC5/PB0/PB1` analog pins): configured with regular channel `ADC_CHANNEL_5`, no active acquisition task found.

## 3) UART and RS485 Configuration Summary

All UART/USART instances in `main.c` are configured as:
- Baud rate: `115200`
- Word length: `8 bits`
- Stop bits: `1`
- Parity: `None`
- Oversampling: `16`

RS485 instances:
- `USART1`, `USART2`, `USART3` use `HAL_RS485Ex_Init(..., UART_DE_POLARITY_HIGH, 0, 0)`.

Non-RS485 UART instances:
- `UART4`, `UART5`, `UART7`, `UART8` use `HAL_UART_Init`.

## 4) Device Addressing and Protocol Details

### 4.1 Motor bus (`motor_driver` on USART2)
- Node count macro: `UMOTOR_NODE_COUNT = 4`.
- Default enabled node IDs in `g_umotor`: `0x01`, `0x02`, `0x03`, `0x04`.
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

### 4.2 Displacement sensor (`displacement_sensor` on USART3)
- MODBUS slave ID: `0x02`.
- Poll period: `100 ms`.
- Driver sends read frame for `0x0000` with count `0x0010` (16 regs).
- Parsed measurement source registers:
  - `0x0008` high word
  - `0x0009` low word
  - Combined as float displacement in mm.

### 4.3 RDSS terminal (`glider_core` on UART7)
- No MODBUS-style numeric device address.
- Uses AT/NMEA command channel.
- Target platform card ID macro: `GLIDER_TARGET_CARD_ID = "381127"`.
- Local card ID is queried at runtime (`$CCICA,0,00*7B`) and stored dynamically.

### 4.4 IMU (`imu_core` on UART4)
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

## 7) Runtime Thread-to-Driver Binding (from `main.c`)

- `StartUartRxTask` -> `GliderCore_RunRxTask`
- `StartUartTxTask` -> `GliderCore_RunTxTask`
- `Start_IMU_Task` -> IMU data update loop + `IMU_Core_RunRxTask`
- `StartUsart3Task` -> `Displacement_StartTask`
- `StartUsart2Task` -> `UMotor_StartTask`
- `StartServoTask` currently calls `Stepper_TestTask` (not servo task)

## 8) Notes / To-Confirm Items

- `servo_driver` exists, but `StartServoTask` in `main.c` currently mounts stepper test logic instead.
- `main.h` defines `IMU_DRY_Pin` on `PC4`, while `MX_GPIO_Init` configures `IMU_DRY_Pin` plus `PC7` as inputs.
- `SDMMC1` init function exists but `MX_SDMMC1_MMC_Init()` call is commented out in startup path.
- `USART1`/`UART5`/`UART8` are initialized but not bound to current custom drivers.

## 9) Source Files Used for This Inventory

- `Core/Src/main.c`
- `Core/Src/stm32h7xx_hal_msp.c`
- `Core/Inc/main.h`
- `software.ioc`
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