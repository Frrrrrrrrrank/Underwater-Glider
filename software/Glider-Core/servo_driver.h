/*
 * servo_driver.h
 * 
 * Servo motor control driver for DS3120 270-degree servo
 * Provides PWM control interface through FreeRTOS task
 */

#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>

/* === Servo Debug Structure ===
 * Externally accessible control interface for servo motor
 * Allows runtime adjustment and monitoring through debugger
 */
typedef struct
{
  uint32_t pwm_started;           /* 1 when HAL_TIM_PWM_Start succeeded */
  uint32_t mode;                  /* 0: sweep test, 1: manual angle input */
  float cmd_angle_deg;            /* Input angle from debugger, unit: degree */
  float current_angle_deg;        /* Current applied angle, unit: degree */
  float min_angle_deg;            /* Sweep minimum angle in mode 0 */
  float max_angle_deg;            /* Sweep maximum angle in mode 0 */
  uint16_t current_pulse_us;      /* Current CCR pulse width, unit: us */
  uint16_t min_pulse_us;          /* Pulse at physical 0 degree */
  uint16_t center_pulse_us;       /* Mid reference pulse for debug */
  uint16_t max_pulse_us;          /* Pulse at physical 270 degree */
  float sweep_speed_deg_s;        /* Auto mode angular speed, unit: deg/s */
  uint16_t sweep_step_us;         /* Sweep delta pulse per update in auto mode */
  uint16_t update_period_ms;      /* Update period in ms */
  int32_t sweep_direction;        /* +1/-1 pulse direction in auto mode */
  uint32_t move_count;            /* Increment when output pulse changes */
  uint32_t step_count;            /* Task loop counter for servo task */
} ServoDebugView;

/* === Global Debug Variable ===
 * Can be read/written by debugger to control servo in real-time
 */
extern volatile ServoDebugView g_servo_debug;

/* === Public Functions ===
 */

/* Clamp angle to physical 0-270 degree range */
float servo_clamp_angle(float angle_deg);

/* Convert angle (in degrees) to PWM pulse width (in microseconds) */
uint16_t servo_angle_to_pulse(float angle_deg);

/* FreeRTOS task entry point for servo control */
void StartServoTask(void *argument);

#endif /* SERVO_DRIVER_H */
