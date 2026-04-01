/*
 * servo_driver.c
 * 
 * Servo motor control driver implementation
 * Controls DS3120 270-degree servo via PWM on TIM4 CH3
 */

#include <stddef.h>
#include "servo_driver.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

/* === Servo Debug Global Variable ===
 * Default configuration:
 * - Physical mapping: 0° -> 500µs, 270° -> 1315µs (calibrated from datasheet)
 * - Sweep range: full 0-270° for mode 0 operation
 * - Angular speed: 60 deg/s
 */
volatile ServoDebugView g_servo_debug = {
  0U,     /* pwm_started */
  0U,     /* mode: default sweep test */
  135.0f, /* cmd_angle_deg */
  135.0f, /* current_angle_deg */
  0.0f,   /* min_angle_deg (actual angle: 0°) */
  270.0f, /* max_angle_deg (actual angle: 270°) */
  908U,   /* current_pulse_us (actual 135° with calibrated range) */
  500U,   /* min_pulse_us (calibrated actual 0°) */
  908U,   /* center_pulse_us (display/reference only) */
  1315U,  /* max_pulse_us (calibrated actual 270° from test) */
  60.0f,  /* sweep_speed_deg_s */
  10U,    /* sweep_step_us */
  20U,    /* update_period_ms */
  1,      /* sweep_direction */
  0U,     /* move_count */
  0U      /* step_count */
};

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/*
 * Clamp angle to physical 0-270 degree range
 * Parameter: angle_deg - angle in degrees
 * Return: Clamped angle within [0.0, 270.0]
 */
float servo_clamp_angle(float angle_deg)
{
  const float k_map_min_deg = 0.0f;
  const float k_map_max_deg = 270.0f;

  if (angle_deg < k_map_min_deg)
  {
    return k_map_min_deg;
  }
  if (angle_deg > k_map_max_deg)
  {
    return k_map_max_deg;
  }
  return angle_deg;
}

/*
 * Convert angle (degrees) to PWM pulse width (microseconds)
 * Uses linear mapping from calibrated servo datashee:
 *   pulse_us = min_pulse_us + (angle - 0) * (max_pulse_us - min_pulse_us) / 270
 * 
 * Parameter: angle_deg - desired angle in degrees [0, 270]
 * Return: PWM compare value in microseconds [500, 1315]
 */
uint16_t servo_angle_to_pulse(float angle_deg)
{
  const float k_map_min_deg = 0.0f;
  const float k_map_max_deg = 270.0f;
  float clamped = servo_clamp_angle(angle_deg);
  float angle_range = k_map_max_deg - k_map_min_deg;
  if (angle_range < 0.1f) angle_range = 0.1f;
  
  float pulse_range = (float)(g_servo_debug.max_pulse_us - g_servo_debug.min_pulse_us);
  float scale = pulse_range / angle_range;
  float pulse_f = (float)g_servo_debug.min_pulse_us + (clamped - k_map_min_deg) * scale;

  if (pulse_f < (float)g_servo_debug.min_pulse_us)
  {
    pulse_f = (float)g_servo_debug.min_pulse_us;
  }
  if (pulse_f > (float)g_servo_debug.max_pulse_us)
  {
    pulse_f = (float)g_servo_debug.max_pulse_us;
  }

  return (uint16_t)(pulse_f + 0.5f);
}

/* ============================================================================
 * FreeRTOS Task Implementation
 * ============================================================================ */

/*
 * Servo Control Task (Disabled)
 * 
 * This task is currently disabled and managed by main.c's StartServoTask.
 * Helper functions servo_clamp_angle() and servo_angle_to_pulse() remain available.
 * To re-enable servo control, call StartServoTask from main.c or FreeRTOS.
 */
void StartServoTask_Disabled(void *argument)
{
  /* Task placeholder - disabled */
  (void)argument;
}
