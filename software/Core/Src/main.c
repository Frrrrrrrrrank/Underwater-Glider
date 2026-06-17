/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "glider_core.h"
#include "glider_rdss_codec.h"
#include "imu_core.h"
#include "servo_driver.h"
#include "motor_driver.h"
#include "pump_driver.h"
#include "pressure_sensor_driver.h"
#include "sbus_driver.h"
#include "stepper_driver.h"
#include "buzzer_driver.h"
#include "ethernet_uart_driver.h"
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  uint32_t boot_stage;            /* Boot marker from main() */
  uint32_t alive_counter;         /* defaultTask loop counter */
  uint32_t rtos_tick;             /* osKernelGetTickCount() snapshot */
  uint32_t rx_line_count;         /* Decoded RX line counter */
  uint32_t rx_overflow_count;     /* RX ring overflow counter */
  uint32_t tx_attempt_count;      /* UART TX attempts */
  uint32_t tx_ok_count;           /* UART TX success count */
  uint32_t tx_fail_count;         /* UART TX fail count */
  uint32_t tx_last_status;        /* Last HAL_UART_Transmit status */
  uint32_t tx_last_uart_error;    /* Last UART error code */
  char last_tx_text[32];          /* Last transmitted command text */
  char card_id[24];               /* Stored card id used by TESTTC send */
  uint32_t card_id_known;         /* 1 when card id was parsed/queried */
  uint32_t shortmsg_sent_count;   /* Number of sent coordinate short messages */
  char last_shortmsg_payload[96]; /* Last sent coordinate payload */
  uint32_t last_rx_update_count;  /* Incremented on each new RX callback update */
  uint32_t last_rx_text_len;      /* Length of last text frame */
  char last_rx_text[512];         /* Last text frame (null-terminated) */
  uint32_t last_rx_bin_len;       /* Length of last binary frame */
  uint8_t last_rx_bin[64];        /* Last binary frame bytes */
  uint32_t is_rd_mode;            /* 1 when last decoded text is RD Mode! */
  uint32_t is_bdpwi;              /* 1 when last decoded sentence is BDPWI */
  char bdpwi_utc[16];             /* BDPWI UTC field string */
  uint32_t bdpwi_field1;          /* BDPWI Field[2], compatibility mirror */
  uint32_t bdpwi_field2;          /* BDPWI Field[3], compatibility mirror */
  uint32_t bdpwi_field1_valid;    /* BDPWI Field[2] numeric-valid flag */
  uint32_t bdpwi_field2_valid;    /* BDPWI Field[3] numeric-valid flag */
  char bdpwi_latitude_raw[16];    /* BDPWI Field[4] raw text */
  char bdpwi_longitude_raw[16];   /* BDPWI Field[5] raw text */
  char bdpwi_heading_raw[16];     /* BDPWI Field[6] raw text */
  int32_t bdpwi_latitude_x100;    /* BDPWI Field[4] fixed-point x100 */
  int32_t bdpwi_longitude_x100;   /* BDPWI Field[5] fixed-point x100 */
  int32_t bdpwi_heading_x100;     /* BDPWI Field[6] fixed-point x100 */
  uint32_t bdpwi_latitude_valid;  /* BDPWI Field[4] decimal-valid flag */
  uint32_t bdpwi_longitude_valid; /* BDPWI Field[5] decimal-valid flag */
  uint32_t bdpwi_heading_valid;   /* BDPWI Field[6] decimal-valid flag */
  char bdpwi_latitude_dm[20];     /* Located BDPWI latitude candidate */
  char bdpwi_latitude_ns[4];      /* Located BDPWI latitude direction */
  char bdpwi_longitude_dm[20];    /* Located BDPWI longitude candidate */
  char bdpwi_longitude_ew[4];     /* Located BDPWI longitude direction */
  int32_t bdpwi_latitude_deg_e7;  /* Converted BDPWI latitude * 1e7 */
  int32_t bdpwi_longitude_deg_e7; /* Converted BDPWI longitude * 1e7 */
  float bdpwi_latitude_deg;       /* Map-ready BDPWI latitude */
  float bdpwi_longitude_deg;      /* Map-ready BDPWI longitude */
  uint32_t bdpwi_latitude_deg_valid;  /* Located latitude conversion valid */
  uint32_t bdpwi_longitude_deg_valid; /* Located longitude conversion valid */
  uint32_t bdpwi_old_beam_count;  /* Old-system locked beam count */
  uint32_t bdpwi_new_beam_count;  /* New-system locked beam count */
  uint32_t bdpwi_best_d_cnr;      /* Best D-branch CNR */
  uint32_t bdpwi_best_p_cnr;      /* Best P-branch CNR */
  uint32_t bdpwi_good_beam_count; /* Count of good beams */
  uint32_t rd_signal_good;        /* 1 when signal is good enough */
  uint32_t tx_mode;               /* 0 signal poll, 1 position test */
  uint32_t is_gnrmc;              /* 1 when last decoded sentence is GNRMC */
  char gnrmc_utc[16];             /* GNRMC UTC */
  char gnrmc_status[4];           /* GNRMC A/V */
  char gnrmc_date[16];            /* GNRMC date */
  char gnrmc_latitude_dm[20];     /* GNRMC raw latitude */
  char gnrmc_latitude_ns[4];      /* GNRMC latitude direction */
  char gnrmc_longitude_dm[20];    /* GNRMC raw longitude */
  char gnrmc_longitude_ew[4];     /* GNRMC longitude direction */
  int32_t gnrmc_latitude_deg_e7;  /* GNRMC latitude * 1e7 */
  int32_t gnrmc_longitude_deg_e7; /* GNRMC longitude * 1e7 */
  float gnrmc_latitude_deg;       /* GNRMC decimal latitude */
  float gnrmc_longitude_deg;      /* GNRMC decimal longitude */
  uint32_t gnrmc_status_valid;    /* 1 when status is A */
  uint32_t gnrmc_latitude_valid;  /* 1 when latitude is valid */
  uint32_t gnrmc_longitude_valid; /* 1 when longitude is valid */
  uint32_t is_bdepi;              /* 1 when last decoded sentence is BDEPI */
  char bdepi_sender_card[24];     /* BDEPI Field[1] */
  char bdepi_receiver_card[24];   /* BDEPI Field[2] */
  char bdepi_utc[16];             /* BDEPI Field[6] */
  uint32_t bdepi_freq_code;       /* BDEPI Field[3] */
  uint32_t bdepi_emergency_code;  /* BDEPI Field[4] */
  uint32_t bdepi_report_mode;     /* BDEPI Field[5] */
  uint32_t bdepi_freq_valid;      /* BDEPI Field[3] numeric-valid */
  uint32_t bdepi_emergency_valid; /* BDEPI Field[4] numeric-valid */
  uint32_t bdepi_report_mode_valid; /* BDEPI Field[5] numeric-valid */
  char bdepi_longitude_dm[20];    /* BDEPI Field[7] raw dddmm.mmmmmm */
  char bdepi_longitude_ew[4];     /* BDEPI Field[8] */
  char bdepi_latitude_dm[20];     /* BDEPI Field[9] raw ddmm.mmmmmm */
  char bdepi_latitude_ns[4];      /* BDEPI Field[10] */
  int32_t bdepi_longitude_deg_e7; /* Converted decimal degree * 1e7 */
  int32_t bdepi_latitude_deg_e7;  /* Converted decimal degree * 1e7 */
  float bdepi_longitude_deg;      /* Map-ready longitude as decimal degree */
  float bdepi_latitude_deg;       /* Map-ready latitude as decimal degree */
  uint32_t bdepi_longitude_valid; /* BDEPI longitude conversion valid */
  uint32_t bdepi_latitude_valid;  /* BDEPI latitude conversion valid */
  char bdepi_altitude_raw[16];    /* BDEPI Field[11] raw altitude */
  int32_t bdepi_altitude_x100;    /* BDEPI altitude * 100 */
  uint32_t bdepi_altitude_valid;  /* BDEPI altitude conversion valid */
  char bdepi_altitude_unit[4];    /* BDEPI Field[12] */
  char bdepi_altitude_abnormal[8]; /* BDEPI Field[13] */
  char bdepi_altitude_abn_unit[4]; /* BDEPI Field[14] */
  char bdepi_precision_indicator[8]; /* BDEPI Field[15] */
  char bdepi_multi_solution[8];   /* BDEPI Field[16] */
  char bdepi_rescue_subtype[8];   /* BDEPI Field[17] */
  char bdepi_rescue_center_type[8]; /* BDEPI Field[18] */
  uint32_t bdepi_data_len;        /* BDEPI Field[19] */
  uint32_t bdepi_data_len_valid;  /* BDEPI Field[19] numeric-valid */
  char bdepi_data_payload[48];    /* BDEPI Field[20] */
} GliderDebugView;

typedef struct
{
  uint32_t frame_count;           /* Total IMU frames received */
  uint32_t crc_error_count;       /* CRC checksum failures */
  uint32_t rx_overflow_count;     /* Ring buffer overflow count */
  uint32_t rx_irq_count;          /* UART4 RX IRQ callback count */
  uint32_t rx_byte_count;         /* Raw received byte count */
  uint32_t rx_poll_byte_count;    /* Bytes captured by polling fallback */
  uint32_t rx_rearm_ok_count;     /* RX IT re-arm successful count */
  uint32_t rx_rearm_fail_count;   /* RX IT re-arm failed count */
  uint32_t rx_init_status;        /* Initial RX IT arm status */
  uint32_t uart4_irq_count;       /* UART4 IRQ entry count */
  uint32_t uart4_error_count;     /* UART4 HAL error callback count */
  uint32_t uart4_last_error;      /* UART4 last HAL error code */
  uint32_t uart_rxne_flag;        /* Current RXNE flag state (1=data ready, 0=no data) */
  uint32_t uart_error_flags;      /* Current error flags (bit0=ORE, bit1=NE, bit2=FE, bit3=PE) */
  uint32_t recent_rx_len;         /* Number of valid bytes in recent_rx_bytes */
  uint8_t recent_rx_bytes[16];    /* Recent raw bytes from IMU RX stream */
  uint32_t task_loop_count;       /* IMU task loop count */
  uint32_t last_function_code;    /* Last received function code (0x40, 0x41, etc) */
  uint32_t last_rx_function;      /* Return-state frame field[0] */
  int32_t last_rx_state;          /* Return-state frame field[1] */
  
  /* Raw 9-axis data */
  float ax, ay, az;               /* Accelerometer (m/s²) */
  float gx, gy, gz;               /* Gyroscope (rad/s) */
  float mx, my, mz;               /* Magnetometer (µT) */
  uint32_t raw_updated;           /* Flag: raw data was updated */
  
  /* Euler angles */
  float roll, pitch, yaw;         /* Degrees */
  uint32_t euler_updated;         /* Flag: euler data was updated */
  
  /* Quaternion */
  float q0, q1, q2, q3;
  uint32_t quat_updated;          /* Flag: quaternion data was updated */
  
  /* Barometer */
  float height, temperature;
  float pressure, pressure_contrast;
  uint32_t baro_updated;          /* Flag: baro data was updated */
  
  /* Version */
  uint8_t version_high;
  uint8_t version_mid;
  uint8_t version_low;
} IMUDebugView;

typedef struct
{
  uint32_t update_count;          /* Increments on each decoded line */
  uint32_t frame_type;            /* GliderRdssFrameType */
  uint32_t checksum_valid;        /* 1 if checksum passed or absent */
  uint32_t is_bdpwi;              /* 1 if BDPWI */
  uint32_t is_bdepi;              /* 1 if BDEPI */
  uint32_t is_gnrmc;              /* 1 if GNRMC */
  char sentence[24];              /* Sentence id */
  char utc[16];                   /* UTC string from decoded frame */
  uint32_t mode_code;             /* BDPWI field1 or BDEPI report_mode */
  uint32_t status_code;           /* BDPWI field2 or BDEPI emergency */
  float longitude_deg;            /* Map-ready when available */
  float latitude_deg;             /* Map-ready when available */
  uint32_t position_valid;        /* 1 if longitude/latitude are valid */
  uint32_t signal_good;           /* 1 if BDPWI signal is good */
  uint32_t tx_mode;               /* 0 signal poll, 1 position test */
  uint32_t card_id_known;         /* 1 when local card id is known */
  uint32_t card_mismatch;         /* 1 when queried local card changed */
  uint32_t has_cached_fix;        /* 1 when last valid RN/GNRMC is cached */
  uint32_t shortmsg_pending;      /* 1 when payload queued for send */
  uint32_t ready_to_send;         /* 1 when all last-send conditions are met */
  char local_card_id[24];         /* Local queried card id */
  char target_card_id[24];        /* Destination platform card id */
  char last_tx_text[32];          /* Last transmitted command text */
  char last_text[96];             /* Last text line snapshot */
} GliderDebugQuickView;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

MMC_HandleTypeDef hmmc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uart7RxTask */
osThreadId_t uart7RxTaskHandle;
const osThreadAttr_t uart7RxTask_attributes = {
  .name = "uart7RxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for uartTxTask */
osThreadId_t uartTxTaskHandle;
const osThreadAttr_t uartTxTask_attributes = {
  .name = "uartTxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* Definitions for imuRxTask */
osThreadId_t imuRxTaskHandle;
const osThreadAttr_t imuRxTask_attributes = {
  .name = "imuRxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for servoTask */
osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTask_attributes = {
  .name = "servoTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for Usart3Task */
osThreadId_t Usart3TaskHandle;
const osThreadAttr_t Usart3Task_attributes = {
  .name = "Usart3Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Usart2Task */
osThreadId_t Usart2TaskHandle;
const osThreadAttr_t Usart2Task_attributes = {
  .name = "Usart2Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* USER CODE BEGIN PV */
volatile uint32_t g_dbg_alive_counter = 0U;
volatile uint32_t g_dbg_rtos_tick = 0U;
volatile uint32_t g_dbg_rx_line_count = 0U;
volatile uint32_t g_dbg_rx_overflow_count = 0U;
volatile uint32_t g_dbg_default_task_tick = 0U;
volatile uint32_t g_dbg_boot_stage = 0U;
volatile uint32_t g_dbg_systick_count = 0U;
volatile uint32_t g_dbg_fault_code = 0U;
volatile uint32_t g_dbg_fault_count = 0U;
volatile uint32_t g_dbg_assert_count = 0U;
volatile uint32_t g_dbg_assert_line = 0U;
volatile uint32_t g_dbg_rtos_hook_code = 0U;
volatile uint32_t g_dbg_tx_attempt_count = 0U;
volatile uint32_t g_dbg_tx_ok_count = 0U;
volatile uint32_t g_dbg_tx_fail_count = 0U;
volatile uint32_t g_dbg_tx_last_status = 0U;
volatile uint32_t g_dbg_tx_last_uart_error = 0U;
volatile uint32_t g_dbg_signal_good = 0U;
volatile uint32_t g_dbg_tx_mode = 0U;
volatile char g_dbg_last_tx_text[32] = {0};
volatile char g_dbg_card_id[24] = {0};
volatile uint32_t g_dbg_card_id_known = 0U;
volatile uint32_t g_dbg_shortmsg_sent_count = 0U;
volatile char g_dbg_last_shortmsg_payload[96] = {0};
volatile uint32_t g_dbg_last_rx_text_len = 0U;
volatile char g_dbg_last_rx_text[192] = {0};
volatile uint32_t g_dbg_last_rx_bin_len = 0U;
volatile uint8_t g_dbg_last_rx_bin[64] = {0};
volatile uint32_t g_dbg_last_rx_update_count = 0U;
volatile uint32_t g_dbg_is_rd_mode = 0U;
volatile uint32_t g_dbg_is_bdpwi = 0U;
volatile char g_dbg_bdpwi_utc[16] = {0};
volatile uint32_t g_dbg_bdpwi_field1 = 0U;
volatile uint32_t g_dbg_bdpwi_field2 = 0U;
volatile uint32_t g_dbg_bdpwi_field1_valid = 0U;
volatile uint32_t g_dbg_bdpwi_field2_valid = 0U;
volatile uint32_t g_dbg_uart4_irq_count = 0U;
volatile uint32_t g_dbg_uart3_irq_count = 0U;
volatile uint32_t g_dbg_uart3_rxne_count = 0U;
volatile uint32_t g_dbg_uart4_error_count = 0U;
volatile uint32_t g_dbg_uart4_last_error = 0U;
volatile uint32_t g_dbg_imu_task_loop_count = 0U;
volatile uint32_t g_dbg_servo_start_ok = 0U;
volatile uint32_t g_dbg_servo_step_count = 0U;
volatile uint32_t g_dbg_servo_pulse_us = 1500U;
volatile uint32_t g_rc_ctrl_mode = 0U;
volatile uint32_t g_rc_ctrl_off_reason = 0U;
volatile int32_t g_rc_worm_cmd_rpm = 0;
volatile int32_t g_rc_stepper_cmd_rpm = 0;
volatile uint16_t g_rc_sw_left_outer = 0U;
volatile uint16_t g_rc_sw_left_mid_1 = 0U;
volatile uint16_t g_rc_sw_left_mid_2 = 0U;
volatile uint16_t g_rc_sw_right_outer = 0U;
volatile uint16_t g_rc_left_stick_y = SBUS_VALUE_CENTER_OBSERVED;
volatile uint16_t g_rc_left_stick_x = SBUS_VALUE_CENTER_OBSERVED;
volatile uint16_t g_rc_right_stick_y = SBUS_VALUE_CENTER_OBSERVED;
volatile GliderDebugView g_dbg_view = {0};
volatile GliderDebugQuickView g_dbg_quick = {0};
volatile IMUDebugView g_imu_debug = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SDMMC1_MMC_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_UART8_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM15_Init(void);
void StartDefaultTask(void *argument);
void StartUartRxTask(void *argument);
void StartUartTxTask(void *argument);
void Start_IMU_Task(void *argument);
void StartServoTask(void *argument);
void StartUsart3Task(void *argument);
void StartUsart2Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define RC_SWITCH_UP_THRESHOLD     640U
#define RC_SWITCH_DOWN_THRESHOLD   1400U
#define RC_STICK_DEADBAND          40U
#define RC_CONTROL_PERIOD_MS       20U
#define RC_FRAME_TIMEOUT_MS        150U

#define RC_WORM_MOTOR_NODE_ID      0x01U
/* g_umotor 默认槽位 0x01~0x04 对应 slot 0~3；node_id 0x01 对应 slot 0 */
#define RC_WORM_MOTOR_SLOT         0U
#define RC_WORM_MOTOR_MAX_RPM      600U
#define RC_STEPPER_MAX_RPM         800U

#define RC_OFF_REASON_NONE                 0U
#define RC_OFF_REASON_FRAME_TIMEOUT        1U
#define RC_OFF_REASON_FAILSAFE_OR_LOST     2U
#define RC_OFF_REASON_ALL_SWITCH_UP        3U
#define RC_OFF_REASON_MODE_SWITCH_NOT_MID  4U

typedef enum
{
  RC_CTRL_MODE_OFF = 0U,
  RC_CTRL_MODE_OPEN_LOOP_SPEED = 1U
} RcControlMode;

static uint32_t RcControl_IsSwitchUp(uint16_t raw)
{
  return (raw < RC_SWITCH_UP_THRESHOLD) ? 1U : 0U;
}

static uint32_t RcControl_IsSwitchMid(uint16_t raw)
{
  return ((raw >= RC_SWITCH_UP_THRESHOLD) && (raw <= RC_SWITCH_DOWN_THRESHOLD)) ? 1U : 0U;
}

static int32_t RcControl_ClampSigned(int32_t value, int32_t limit)
{
  if (limit < 0)
  {
    limit = -limit;
  }

  if (value > limit)
  {
    return limit;
  }

  if (value < -limit)
  {
    return -limit;
  }

  return value;
}

/* Map SBUS stick raw value to signed RPM with a deadband around center. */
static int32_t RcControl_MapStickToSignedRpm(uint16_t raw, uint16_t max_rpm)
{
  const uint32_t center = SBUS_VALUE_CENTER_OBSERVED;
  const uint32_t min_v = SBUS_VALUE_MIN_OBSERVED;
  const uint32_t max_v = SBUS_VALUE_MAX_OBSERVED;
  const uint32_t threshold_high = center + RC_STICK_DEADBAND;
  const uint32_t threshold_low = center - RC_STICK_DEADBAND;
  int32_t signed_rpm = 0;

  if (max_rpm == 0U)
  {
    return 0;
  }

  if ((uint32_t)raw > threshold_high)
  {
    const uint32_t num = (uint32_t)raw - threshold_high;
    const uint32_t den = max_v - threshold_high;
    if (den != 0U)
    {
      signed_rpm = (int32_t)((num * (uint32_t)max_rpm) / den);
    }
  }
  else if ((uint32_t)raw < threshold_low)
  {
    const uint32_t num = threshold_low - (uint32_t)raw;
    const uint32_t den = threshold_low - min_v;
    if (den != 0U)
    {
      signed_rpm = -(int32_t)((num * (uint32_t)max_rpm) / den);
    }
  }

  return RcControl_ClampSigned(signed_rpm, (int32_t)max_rpm);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  g_dbg_boot_stage = 1U;

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  g_dbg_boot_stage = 2U;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  g_dbg_boot_stage = 3U;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  // MX_SDMMC1_MMC_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_LPUART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  g_dbg_boot_stage = 46U;
  GliderCore_Init(&huart7);
  g_dbg_boot_stage = 4U;

  /* Initialize Ethernet UART driver on LPUART1 */
  EthernetUartDriver_Init(&hlpuart1);
  
  /* Initialize IMU module on UART4 */
  IMU_Core_Init(&huart4);

  /* Initialize pressure sensor non-blocking RX driver */
  PressureSensorDriver_Init(&huart1);

  /* Initialize SBUS non-blocking RX driver on UART8 */
  SbusDriver_Init(&huart8);

  /* Initialize buzzer PWM driver on TIM2 CH1 / PA0 */
  BuzzerDriver_Init(&htim2);
  {
    static const BuzzerTone init_done_tone[] = {
      {1200U, 90U},
      {1600U, 90U},
      {2000U, 90U},
      {2600U, 140U}
    };
    uint32_t tone_start_tick;

    BuzzerDriver_PlaySequence(init_done_tone,
                              (uint32_t)(sizeof(init_done_tone) / sizeof(init_done_tone[0])));
    tone_start_tick = HAL_GetTick();
    while ((g_buzzer_diag.active != 0U) && ((HAL_GetTick() - tone_start_tick) < 700U))
    {
      BuzzerDriver_Poll(HAL_GetTick());
      HAL_Delay(5U);
    }
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  PumpDriver_Init(&huart3);
  IoControlPump_Init();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of uart7RxTask */
  uart7RxTaskHandle = osThreadNew(StartUartRxTask, NULL, &uart7RxTask_attributes);

  /* creation of uartTxTask */
  uartTxTaskHandle = osThreadNew(StartUartTxTask, NULL, &uartTxTask_attributes);

  /* creation of imuRxTask */
  imuRxTaskHandle = osThreadNew(Start_IMU_Task, NULL, &imuRxTask_attributes);

  /* creation of servoTask */
  servoTaskHandle = osThreadNew(StartServoTask, NULL, &servoTask_attributes);

  /* creation of Usart3Task */
  Usart3TaskHandle = osThreadNew(StartUsart3Task, NULL, &Usart3Task_attributes);

  /* creation of Usart2Task */
  Usart2TaskHandle = osThreadNew(StartUsart2Task, NULL, &Usart2Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  (void)osThreadNew(EthernetUartDriver_RunTask, NULL, &Usart2Task_attributes);
  g_dbg_boot_stage = 6U;
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  g_dbg_boot_stage = 7U;
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 48;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV4;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 1;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x007074AF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 100000;
  huart8.Init.WordLength = UART_WORDLENGTH_9B;
  huart8.Init.StopBits = UART_STOPBITS_2;
  huart8.Init.Parity = UART_PARITY_EVEN;
  huart8.Init.Mode = UART_MODE_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT|UART_ADVFEATURE_SWAP_INIT;
  huart8.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  huart8.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart3, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_MMC_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hmmc1.Instance = SDMMC1;
  hmmc1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hmmc1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hmmc1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hmmc1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hmmc1.Init.ClockDiv = 0;
  if (HAL_MMC_Init(&hmmc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 120-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 119;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */

/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PUMP_DIR_Pin|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PUMP_SPEED_B0_Pin|PUMP_SPEED_B1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PUMP_STP_GPIO_Port, PUMP_STP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PUMP_DIR_Pin */
  GPIO_InitStruct.Pin = PUMP_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PUMP_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PUMP_SPEED_B0_Pin PUMP_SPEED_B1_Pin PUMP_STP_Pin */
  GPIO_InitStruct.Pin = PUMP_SPEED_B0_Pin|PUMP_SPEED_B1_Pin|PUMP_STP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_DRY_Pin PC7 */
  GPIO_InitStruct.Pin = IMU_DRY_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = LED_Pin|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE12 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void GliderCore_OnTextFrame(const char *line)
{
  size_t i = 0U;
  size_t len = 0U;

  if (line == NULL)
  {
    g_dbg_last_rx_text_len = 0U;
    g_dbg_last_rx_text[0] = '\0';
    g_dbg_last_rx_update_count++;
    g_dbg_view.last_rx_text_len = 0U;
    g_dbg_view.last_rx_text[0] = '\0';
    g_dbg_view.last_rx_update_count = g_dbg_last_rx_update_count;
    return;
  }

  while (line[len] != '\0' && len < (sizeof(g_dbg_last_rx_text) - 1U))
  {
    len++;
  }

  for (i = 0U; i < len; i++)
  {
    g_dbg_last_rx_text[i] = line[i];
  }
  g_dbg_last_rx_text[len] = '\0';

  for (i = len + 1U; i < sizeof(g_dbg_last_rx_text); i++)
  {
    g_dbg_last_rx_text[i] = '\0';
  }

  g_dbg_last_rx_text_len = (uint32_t)len;
  g_dbg_last_rx_update_count++;
  g_dbg_view.last_rx_text_len = g_dbg_last_rx_text_len;
  for (i = 0U; i < sizeof(g_dbg_view.last_rx_text); i++)
  {
    g_dbg_view.last_rx_text[i] = g_dbg_last_rx_text[i];
  }
  g_dbg_view.last_rx_update_count = g_dbg_last_rx_update_count;

  for (i = 0U; i < sizeof(g_dbg_quick.last_text); i++)
  {
    if (i < len)
    {
      g_dbg_quick.last_text[i] = line[i];
    }
    else
    {
      g_dbg_quick.last_text[i] = '\0';
    }
  }
}

void GliderCore_OnBinaryFrame(const uint8_t *payload, size_t payload_len)
{
  size_t i = 0U;
  size_t copy_len = 0U;

  if (payload == NULL)
  {
    g_dbg_last_rx_bin_len = 0U;
    for (i = 0U; i < sizeof(g_dbg_last_rx_bin); i++)
    {
      g_dbg_last_rx_bin[i] = 0U;
    }
    g_dbg_last_rx_update_count++;
    g_dbg_view.last_rx_bin_len = 0U;
    for (i = 0U; i < sizeof(g_dbg_view.last_rx_bin); i++)
    {
      g_dbg_view.last_rx_bin[i] = 0U;
    }
    g_dbg_view.last_rx_update_count = g_dbg_last_rx_update_count;
    return;
  }

  copy_len = (payload_len < sizeof(g_dbg_last_rx_bin)) ? payload_len : sizeof(g_dbg_last_rx_bin);
  for (i = 0U; i < copy_len; i++)
  {
    g_dbg_last_rx_bin[i] = payload[i];
  }
  for (; i < sizeof(g_dbg_last_rx_bin); i++)
  {
    g_dbg_last_rx_bin[i] = 0U;
  }

  g_dbg_last_rx_bin_len = (uint32_t)copy_len;
  g_dbg_last_rx_update_count++;
  g_dbg_view.last_rx_bin_len = g_dbg_last_rx_bin_len;
  for (i = 0U; i < sizeof(g_dbg_view.last_rx_bin); i++)
  {
    g_dbg_view.last_rx_bin[i] = g_dbg_last_rx_bin[i];
  }
  g_dbg_view.last_rx_update_count = g_dbg_last_rx_update_count;
}

void GliderCore_OnDecodedFrame(const GliderRdssDecodedFrame *frame)
{
  size_t i = 0U;
  size_t len = 0U;

  if (frame == NULL)
  {
    g_dbg_is_rd_mode = 0U;
    g_dbg_is_bdpwi = 0U;
    g_dbg_card_id[0] = '\0';
    g_dbg_card_id_known = 0U;
    g_dbg_bdpwi_utc[0] = '\0';
    g_dbg_bdpwi_field1 = 0U;
    g_dbg_bdpwi_field2 = 0U;
    g_dbg_bdpwi_field1_valid = 0U;
    g_dbg_bdpwi_field2_valid = 0U;
    g_dbg_view.is_rd_mode = 0U;
    g_dbg_view.is_bdpwi = 0U;
    g_dbg_view.bdpwi_utc[0] = '\0';
    g_dbg_view.bdpwi_field1 = 0U;
    g_dbg_view.bdpwi_field2 = 0U;
    g_dbg_view.bdpwi_field1_valid = 0U;
    g_dbg_view.bdpwi_field2_valid = 0U;
    g_dbg_view.bdpwi_latitude_raw[0] = '\0';
    g_dbg_view.bdpwi_longitude_raw[0] = '\0';
    g_dbg_view.bdpwi_heading_raw[0] = '\0';
    g_dbg_view.bdpwi_latitude_dm[0] = '\0';
    g_dbg_view.bdpwi_latitude_ns[0] = '\0';
    g_dbg_view.bdpwi_longitude_dm[0] = '\0';
    g_dbg_view.bdpwi_longitude_ew[0] = '\0';
    g_dbg_view.bdpwi_latitude_x100 = 0;
    g_dbg_view.bdpwi_longitude_x100 = 0;
    g_dbg_view.bdpwi_heading_x100 = 0;
    g_dbg_view.bdpwi_latitude_deg_e7 = 0;
    g_dbg_view.bdpwi_longitude_deg_e7 = 0;
    g_dbg_view.bdpwi_latitude_deg = 0.0f;
    g_dbg_view.bdpwi_longitude_deg = 0.0f;
    g_dbg_view.bdpwi_latitude_valid = 0U;
    g_dbg_view.bdpwi_longitude_valid = 0U;
    g_dbg_view.bdpwi_heading_valid = 0U;
    g_dbg_view.bdpwi_latitude_deg_valid = 0U;
    g_dbg_view.bdpwi_longitude_deg_valid = 0U;
    g_dbg_view.bdpwi_old_beam_count = 0U;
    g_dbg_view.bdpwi_new_beam_count = 0U;
    g_dbg_view.bdpwi_best_d_cnr = 0U;
    g_dbg_view.bdpwi_best_p_cnr = 0U;
    g_dbg_view.bdpwi_good_beam_count = 0U;
    g_dbg_view.rd_signal_good = 0U;
    g_dbg_view.tx_mode = 0U;
    g_dbg_view.last_tx_text[0] = '\0';
    g_dbg_view.card_id[0] = '\0';
    g_dbg_view.card_id_known = 0U;
    g_dbg_view.shortmsg_sent_count = 0U;
    g_dbg_view.last_shortmsg_payload[0] = '\0';
    g_dbg_view.is_gnrmc = 0U;
    g_dbg_view.gnrmc_utc[0] = '\0';
    g_dbg_view.gnrmc_status[0] = '\0';
    g_dbg_view.gnrmc_date[0] = '\0';
    g_dbg_view.gnrmc_latitude_dm[0] = '\0';
    g_dbg_view.gnrmc_latitude_ns[0] = '\0';
    g_dbg_view.gnrmc_longitude_dm[0] = '\0';
    g_dbg_view.gnrmc_longitude_ew[0] = '\0';
    g_dbg_view.gnrmc_latitude_deg_e7 = 0;
    g_dbg_view.gnrmc_longitude_deg_e7 = 0;
    g_dbg_view.gnrmc_latitude_deg = 0.0f;
    g_dbg_view.gnrmc_longitude_deg = 0.0f;
    g_dbg_view.gnrmc_status_valid = 0U;
    g_dbg_view.gnrmc_latitude_valid = 0U;
    g_dbg_view.gnrmc_longitude_valid = 0U;
    g_dbg_view.is_bdepi = 0U;
    g_dbg_view.bdepi_sender_card[0] = '\0';
    g_dbg_view.bdepi_receiver_card[0] = '\0';
    g_dbg_view.bdepi_utc[0] = '\0';
    g_dbg_view.bdepi_freq_code = 0U;
    g_dbg_view.bdepi_emergency_code = 0U;
    g_dbg_view.bdepi_report_mode = 0U;
    g_dbg_view.bdepi_freq_valid = 0U;
    g_dbg_view.bdepi_emergency_valid = 0U;
    g_dbg_view.bdepi_report_mode_valid = 0U;
    g_dbg_view.bdepi_longitude_dm[0] = '\0';
    g_dbg_view.bdepi_longitude_ew[0] = '\0';
    g_dbg_view.bdepi_latitude_dm[0] = '\0';
    g_dbg_view.bdepi_latitude_ns[0] = '\0';
    g_dbg_view.bdepi_longitude_deg_e7 = 0;
    g_dbg_view.bdepi_latitude_deg_e7 = 0;
    g_dbg_view.bdepi_longitude_deg = 0.0f;
    g_dbg_view.bdepi_latitude_deg = 0.0f;
    g_dbg_view.bdepi_longitude_valid = 0U;
    g_dbg_view.bdepi_latitude_valid = 0U;
    g_dbg_view.bdepi_altitude_raw[0] = '\0';
    g_dbg_view.bdepi_altitude_x100 = 0;
    g_dbg_view.bdepi_altitude_valid = 0U;
    g_dbg_view.bdepi_altitude_unit[0] = '\0';
    g_dbg_view.bdepi_altitude_abnormal[0] = '\0';
    g_dbg_view.bdepi_altitude_abn_unit[0] = '\0';
    g_dbg_view.bdepi_precision_indicator[0] = '\0';
    g_dbg_view.bdepi_multi_solution[0] = '\0';
    g_dbg_view.bdepi_rescue_subtype[0] = '\0';
    g_dbg_view.bdepi_rescue_center_type[0] = '\0';
    g_dbg_view.bdepi_data_len = 0U;
    g_dbg_view.bdepi_data_len_valid = 0U;
    g_dbg_view.bdepi_data_payload[0] = '\0';

    g_dbg_quick.frame_type = 0U;
    g_dbg_quick.checksum_valid = 0U;
    g_dbg_quick.is_bdpwi = 0U;
    g_dbg_quick.is_bdepi = 0U;
    g_dbg_quick.is_gnrmc = 0U;
    g_dbg_quick.sentence[0] = '\0';
    g_dbg_quick.utc[0] = '\0';
    g_dbg_quick.mode_code = 0U;
    g_dbg_quick.status_code = 0U;
    g_dbg_quick.longitude_deg = 0.0f;
    g_dbg_quick.latitude_deg = 0.0f;
    g_dbg_quick.position_valid = 0U;
    g_dbg_quick.signal_good = 0U;
    g_dbg_quick.tx_mode = 0U;
    g_dbg_quick.card_id_known = 0U;
    g_dbg_quick.card_mismatch = 0U;
    g_dbg_quick.has_cached_fix = 0U;
    g_dbg_quick.shortmsg_pending = 0U;
    g_dbg_quick.ready_to_send = 0U;
    g_dbg_quick.local_card_id[0] = '\0';
    g_dbg_quick.target_card_id[0] = '\0';
    g_dbg_quick.last_tx_text[0] = '\0';
    return;
  }

  g_dbg_quick.update_count++;
  g_dbg_quick.frame_type = (uint32_t)frame->type;
  g_dbg_quick.checksum_valid = frame->checksum_valid ? 1U : 0U;
  g_dbg_quick.is_bdpwi = frame->is_bdpwi ? 1U : 0U;
  g_dbg_quick.is_bdepi = frame->is_bdepi ? 1U : 0U;
  g_dbg_quick.is_gnrmc = frame->is_gnrmc ? 1U : 0U;
  for (i = 0U; i < sizeof(g_dbg_quick.sentence); i++)
  {
    g_dbg_quick.sentence[i] = frame->sentence[i];
  }
  for (i = 0U; i < sizeof(g_dbg_quick.last_tx_text); i++)
  {
    g_dbg_quick.last_tx_text[i] = g_dbg_last_tx_text[i];
  }

  g_dbg_is_rd_mode = frame->is_rd_mode ? 1U : 0U;
  g_dbg_is_bdpwi = frame->is_bdpwi ? 1U : 0U;

  while (frame->bdpwi_utc[len] != '\0' && len < (sizeof(g_dbg_bdpwi_utc) - 1U))
  {
    len++;
  }
  for (i = 0U; i < len; i++)
  {
    g_dbg_bdpwi_utc[i] = frame->bdpwi_utc[i];
  }
  g_dbg_bdpwi_utc[len] = '\0';
  for (i = len + 1U; i < sizeof(g_dbg_bdpwi_utc); i++)
  {
    g_dbg_bdpwi_utc[i] = '\0';
  }

  g_dbg_bdpwi_field1 = frame->bdpwi_field1;
  g_dbg_bdpwi_field2 = frame->bdpwi_field2;
  g_dbg_bdpwi_field1_valid = frame->bdpwi_field1_valid ? 1U : 0U;
  g_dbg_bdpwi_field2_valid = frame->bdpwi_field2_valid ? 1U : 0U;
  g_dbg_view.is_rd_mode = g_dbg_is_rd_mode;
  g_dbg_view.is_bdpwi = g_dbg_is_bdpwi;
  for (i = 0U; i < sizeof(g_dbg_view.bdpwi_utc); i++)
  {
    g_dbg_view.bdpwi_utc[i] = g_dbg_bdpwi_utc[i];
  }
  g_dbg_view.bdpwi_field1 = g_dbg_bdpwi_field1;
  g_dbg_view.bdpwi_field2 = g_dbg_bdpwi_field2;
  g_dbg_view.bdpwi_field1_valid = g_dbg_bdpwi_field1_valid;
  g_dbg_view.bdpwi_field2_valid = g_dbg_bdpwi_field2_valid;

  for (i = 0U; i < sizeof(g_dbg_view.bdpwi_latitude_raw); i++)
  {
    g_dbg_view.bdpwi_latitude_raw[i] = frame->bdpwi.latitude_raw[i];
    g_dbg_view.bdpwi_longitude_raw[i] = frame->bdpwi.longitude_raw[i];
    g_dbg_view.bdpwi_heading_raw[i] = frame->bdpwi.heading_raw[i];
  }
  for (i = 0U; i < sizeof(g_dbg_view.bdpwi_latitude_dm); i++)
  {
    g_dbg_view.bdpwi_latitude_dm[i] = frame->bdpwi.latitude_dm[i];
    g_dbg_view.bdpwi_longitude_dm[i] = frame->bdpwi.longitude_dm[i];
  }
  for (i = 0U; i < sizeof(g_dbg_view.bdpwi_latitude_ns); i++)
  {
    g_dbg_view.bdpwi_latitude_ns[i] = frame->bdpwi.latitude_ns[i];
    g_dbg_view.bdpwi_longitude_ew[i] = frame->bdpwi.longitude_ew[i];
  }
  g_dbg_view.bdpwi_latitude_x100 = frame->bdpwi.latitude_x100;
  g_dbg_view.bdpwi_longitude_x100 = frame->bdpwi.longitude_x100;
  g_dbg_view.bdpwi_heading_x100 = frame->bdpwi.heading_x100;
  g_dbg_view.bdpwi_latitude_deg_e7 = frame->bdpwi.latitude_deg_e7;
  g_dbg_view.bdpwi_longitude_deg_e7 = frame->bdpwi.longitude_deg_e7;
  g_dbg_view.bdpwi_latitude_valid = frame->bdpwi.latitude_valid ? 1U : 0U;
  g_dbg_view.bdpwi_longitude_valid = frame->bdpwi.longitude_valid ? 1U : 0U;
  g_dbg_view.bdpwi_heading_valid = frame->bdpwi.heading_valid ? 1U : 0U;
  if (frame->bdpwi.latitude_deg_valid)
  {
    g_dbg_view.bdpwi_latitude_deg = ((float)frame->bdpwi.latitude_deg_e7) / 10000000.0f;
  }
  else
  {
    g_dbg_view.bdpwi_latitude_deg = 0.0f;
  }
  if (frame->bdpwi.longitude_deg_valid)
  {
    g_dbg_view.bdpwi_longitude_deg = ((float)frame->bdpwi.longitude_deg_e7) / 10000000.0f;
  }
  else
  {
    g_dbg_view.bdpwi_longitude_deg = 0.0f;
  }
  g_dbg_view.bdpwi_latitude_deg_valid = frame->bdpwi.latitude_deg_valid ? 1U : 0U;
  g_dbg_view.bdpwi_longitude_deg_valid = frame->bdpwi.longitude_deg_valid ? 1U : 0U;
  g_dbg_view.bdpwi_old_beam_count = frame->bdpwi.old_beam_count;
  g_dbg_view.bdpwi_new_beam_count = frame->bdpwi.new_beam_count;
  g_dbg_view.bdpwi_best_d_cnr = frame->bdpwi.best_d_cnr;
  g_dbg_view.bdpwi_best_p_cnr = frame->bdpwi.best_p_cnr;
  g_dbg_view.bdpwi_good_beam_count = frame->bdpwi.good_beam_count;
  g_dbg_view.rd_signal_good = frame->bdpwi.signal_good ? 1U : 0U;
  g_dbg_view.tx_mode = g_dbg_tx_mode;
  g_dbg_view.is_gnrmc = frame->is_gnrmc ? 1U : 0U;
  for (i = 0U; i < sizeof(g_dbg_view.gnrmc_utc); i++)
  {
    g_dbg_view.gnrmc_utc[i] = frame->gnrmc.utc[i];
    g_dbg_view.gnrmc_date[i] = frame->gnrmc.date[i];
  }
  for (i = 0U; i < sizeof(g_dbg_view.gnrmc_status); i++)
  {
    g_dbg_view.gnrmc_status[i] = frame->gnrmc.status[i];
  }
  for (i = 0U; i < sizeof(g_dbg_view.gnrmc_latitude_dm); i++)
  {
    g_dbg_view.gnrmc_latitude_dm[i] = frame->gnrmc.latitude_dm[i];
    g_dbg_view.gnrmc_longitude_dm[i] = frame->gnrmc.longitude_dm[i];
  }
  for (i = 0U; i < sizeof(g_dbg_view.gnrmc_latitude_ns); i++)
  {
    g_dbg_view.gnrmc_latitude_ns[i] = frame->gnrmc.latitude_ns[i];
    g_dbg_view.gnrmc_longitude_ew[i] = frame->gnrmc.longitude_ew[i];
  }
  g_dbg_view.gnrmc_latitude_deg_e7 = frame->gnrmc.latitude_deg_e7;
  g_dbg_view.gnrmc_longitude_deg_e7 = frame->gnrmc.longitude_deg_e7;
  g_dbg_view.gnrmc_status_valid = frame->gnrmc.status_valid ? 1U : 0U;
  g_dbg_view.gnrmc_latitude_valid = frame->gnrmc.latitude_valid ? 1U : 0U;
  g_dbg_view.gnrmc_longitude_valid = frame->gnrmc.longitude_valid ? 1U : 0U;
  g_dbg_view.gnrmc_latitude_deg = frame->gnrmc.latitude_valid ? ((float)frame->gnrmc.latitude_deg_e7) / 10000000.0f : 0.0f;
  g_dbg_view.gnrmc_longitude_deg = frame->gnrmc.longitude_valid ? ((float)frame->gnrmc.longitude_deg_e7) / 10000000.0f : 0.0f;

  g_dbg_view.is_bdepi = frame->is_bdepi ? 1U : 0U;
  for (i = 0U; i < sizeof(g_dbg_view.bdepi_sender_card); i++)
  {
    g_dbg_view.bdepi_sender_card[i] = frame->bdepi.sender_card[i];
    g_dbg_view.bdepi_receiver_card[i] = frame->bdepi.receiver_card[i];
  }
  for (i = 0U; i < sizeof(g_dbg_view.bdepi_utc); i++)
  {
    g_dbg_view.bdepi_utc[i] = frame->bdepi.utc[i];
    g_dbg_view.bdepi_altitude_raw[i] = frame->bdepi.altitude_raw[i];
  }
  for (i = 0U; i < sizeof(g_dbg_view.bdepi_longitude_dm); i++)
  {
    g_dbg_view.bdepi_longitude_dm[i] = frame->bdepi.longitude_dm[i];
    g_dbg_view.bdepi_latitude_dm[i] = frame->bdepi.latitude_dm[i];
  }
  for (i = 0U; i < sizeof(g_dbg_view.bdepi_longitude_ew); i++)
  {
    g_dbg_view.bdepi_longitude_ew[i] = frame->bdepi.longitude_ew[i];
    g_dbg_view.bdepi_latitude_ns[i] = frame->bdepi.latitude_ns[i];
  }

  g_dbg_view.bdepi_freq_code = frame->bdepi.freq_code;
  g_dbg_view.bdepi_emergency_code = frame->bdepi.emergency_code;
  g_dbg_view.bdepi_report_mode = frame->bdepi.report_mode_code;
  g_dbg_view.bdepi_freq_valid = frame->bdepi.freq_code_valid ? 1U : 0U;
  g_dbg_view.bdepi_emergency_valid = frame->bdepi.emergency_code_valid ? 1U : 0U;
  g_dbg_view.bdepi_report_mode_valid = frame->bdepi.report_mode_valid ? 1U : 0U;
  g_dbg_view.bdepi_longitude_deg_e7 = frame->bdepi.longitude_deg_e7;
  g_dbg_view.bdepi_latitude_deg_e7 = frame->bdepi.latitude_deg_e7;
  if (frame->bdepi.longitude_valid)
  {
    g_dbg_view.bdepi_longitude_deg = ((float)frame->bdepi.longitude_deg_e7) / 10000000.0f;
  }
  else
  {
    g_dbg_view.bdepi_longitude_deg = 0.0f;
  }

  if (frame->bdepi.latitude_valid)
  {
    g_dbg_view.bdepi_latitude_deg = ((float)frame->bdepi.latitude_deg_e7) / 10000000.0f;
  }
  else
  {
    g_dbg_view.bdepi_latitude_deg = 0.0f;
  }
  g_dbg_view.bdepi_longitude_valid = frame->bdepi.longitude_valid ? 1U : 0U;
  g_dbg_view.bdepi_latitude_valid = frame->bdepi.latitude_valid ? 1U : 0U;
  g_dbg_view.bdepi_altitude_x100 = frame->bdepi.altitude_x100;
  g_dbg_view.bdepi_altitude_valid = frame->bdepi.altitude_valid ? 1U : 0U;
  g_dbg_view.bdepi_data_len = frame->bdepi.data_len;
  g_dbg_view.bdepi_data_len_valid = frame->bdepi.data_len_valid ? 1U : 0U;

  for (i = 0U; i < sizeof(g_dbg_view.bdepi_altitude_unit); i++)
  {
    g_dbg_view.bdepi_altitude_unit[i] = frame->bdepi.altitude_unit[i];
    g_dbg_view.bdepi_altitude_abn_unit[i] = frame->bdepi.altitude_abnormal_unit[i];
  }
  for (i = 0U; i < sizeof(g_dbg_view.bdepi_altitude_abnormal); i++)
  {
    g_dbg_view.bdepi_altitude_abnormal[i] = frame->bdepi.altitude_abnormal[i];
    g_dbg_view.bdepi_precision_indicator[i] = frame->bdepi.precision_indicator[i];
    g_dbg_view.bdepi_multi_solution[i] = frame->bdepi.multi_solution[i];
    g_dbg_view.bdepi_rescue_subtype[i] = frame->bdepi.rescue_subtype[i];
    g_dbg_view.bdepi_rescue_center_type[i] = frame->bdepi.rescue_center_type[i];
  }
  for (i = 0U; i < sizeof(g_dbg_view.bdepi_data_payload); i++)
  {
    g_dbg_view.bdepi_data_payload[i] = frame->bdepi.data_payload[i];
  }

  if (frame->is_bdepi)
  {
    for (i = 0U; i < sizeof(g_dbg_quick.utc); i++)
    {
      g_dbg_quick.utc[i] = frame->bdepi.utc[i];
    }
    g_dbg_quick.mode_code = frame->bdepi.report_mode_code;
    g_dbg_quick.status_code = frame->bdepi.emergency_code;
    if (frame->bdepi.longitude_valid && frame->bdepi.latitude_valid)
    {
      g_dbg_quick.longitude_deg = g_dbg_view.bdepi_longitude_deg;
      g_dbg_quick.latitude_deg = g_dbg_view.bdepi_latitude_deg;
      g_dbg_quick.position_valid = 1U;
    }
    g_dbg_quick.tx_mode = g_dbg_tx_mode;
  }
  else if (frame->is_gnrmc)
  {
    for (i = 0U; i < sizeof(g_dbg_quick.utc); i++)
    {
      g_dbg_quick.utc[i] = frame->gnrmc.utc[i];
    }
    g_dbg_quick.mode_code = 0U;
    g_dbg_quick.status_code = frame->gnrmc.status_valid ? 1U : 0U;
    if (frame->gnrmc.status_valid && frame->gnrmc.longitude_valid && frame->gnrmc.latitude_valid)
    {
      g_dbg_quick.longitude_deg = g_dbg_view.gnrmc_longitude_deg;
      g_dbg_quick.latitude_deg = g_dbg_view.gnrmc_latitude_deg;
      g_dbg_quick.position_valid = 1U;
    }
    g_dbg_quick.tx_mode = g_dbg_tx_mode;
  }
  else if (frame->is_bdpwi)
  {
    for (i = 0U; i < sizeof(g_dbg_quick.utc); i++)
    {
      g_dbg_quick.utc[i] = frame->bdpwi.utc[i];
    }
    g_dbg_quick.mode_code = frame->bdpwi.mode_code;
    g_dbg_quick.status_code = frame->bdpwi.status_code;
    if (frame->bdpwi.longitude_deg_valid && frame->bdpwi.latitude_deg_valid)
    {
      g_dbg_quick.longitude_deg = g_dbg_view.bdpwi_longitude_deg;
      g_dbg_quick.latitude_deg = g_dbg_view.bdpwi_latitude_deg;
      g_dbg_quick.position_valid = 1U;
    }
    g_dbg_quick.tx_mode = g_dbg_tx_mode;
  }
  else
  {
    g_dbg_quick.utc[0] = '\0';
    g_dbg_quick.mode_code = 0U;
    g_dbg_quick.status_code = 0U;
    g_dbg_quick.tx_mode = g_dbg_tx_mode;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == UART4)
  {
    g_dbg_uart4_error_count++;
    g_dbg_uart4_last_error = huart->ErrorCode;
    
    /* Clear error flags so UART can resume receiving */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    
    /* Optionally clear the DMA/reception state to allow recovery */
    huart->RxState = HAL_UART_STATE_READY;
  }

  PressureSensorDriver_OnError(huart);
  SbusDriver_OnError(huart);
  EthernetUartDriver_OnError(huart);
}

void vAssertCalled(const char *file, int line)
{
  (void)file;
  g_dbg_assert_count++;
  g_dbg_assert_line = (uint32_t)line;
  g_dbg_rtos_hook_code = 1U;
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

void vApplicationMallocFailedHook(void)
{
  g_dbg_rtos_hook_code = 2U;
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  g_dbg_rtos_hook_code = 3U;
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t led_last_toggle_tick = 0U;
  (void)argument;
  g_dbg_boot_stage = 100U;

  /* Infinite loop */
  for(;;)
  {
    g_dbg_alive_counter++;
    g_dbg_rtos_tick = osKernelGetTickCount();
    g_dbg_rx_line_count = GliderCore_GetRxLineCount();
    g_dbg_rx_overflow_count = GliderCore_GetRxOverflowCount();
    g_dbg_tx_attempt_count = GliderCore_GetTxAttemptCount();
    g_dbg_tx_ok_count = GliderCore_GetTxOkCount();
    g_dbg_tx_fail_count = GliderCore_GetTxFailCount();
    g_dbg_tx_last_status = GliderCore_GetLastTxHalStatus();
    g_dbg_tx_last_uart_error = GliderCore_GetLastUartErrorCode();
    g_dbg_signal_good = GliderCore_GetSignalGood();
    g_dbg_tx_mode = GliderCore_GetTxMode();
    g_dbg_shortmsg_sent_count = GliderCore_GetShortMsgSentCount();
    g_dbg_card_id_known = GliderCore_GetCardIdKnown();
    g_dbg_default_task_tick = g_dbg_rtos_tick;

    if ((g_dbg_rtos_tick - led_last_toggle_tick) >= 500U)
    {
      led_last_toggle_tick = g_dbg_rtos_tick;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    BuzzerDriver_Poll(g_dbg_rtos_tick);

    g_dbg_view.boot_stage = g_dbg_boot_stage;
    g_dbg_view.alive_counter = g_dbg_alive_counter;
    g_dbg_view.rtos_tick = g_dbg_rtos_tick;
    g_dbg_view.rx_line_count = g_dbg_rx_line_count;
    g_dbg_view.rx_overflow_count = g_dbg_rx_overflow_count;
    g_dbg_view.tx_attempt_count = g_dbg_tx_attempt_count;
    g_dbg_view.tx_ok_count = g_dbg_tx_ok_count;
    g_dbg_view.tx_fail_count = g_dbg_tx_fail_count;
    g_dbg_view.tx_last_status = g_dbg_tx_last_status;
    g_dbg_view.tx_last_uart_error = g_dbg_tx_last_uart_error;
    g_dbg_view.rd_signal_good = g_dbg_signal_good;
    g_dbg_view.tx_mode = g_dbg_tx_mode;
    for (size_t tx_i = 0U; tx_i < sizeof(g_dbg_last_tx_text); tx_i++)
    {
      g_dbg_last_tx_text[tx_i] = '\0';
    }
    {
      const char *last_tx = GliderCore_GetLastTxText();
      size_t tx_i = 0U;
      while (last_tx != NULL && last_tx[tx_i] != '\0' && tx_i < (sizeof(g_dbg_last_tx_text) - 1U))
      {
        g_dbg_last_tx_text[tx_i] = last_tx[tx_i];
        tx_i++;
      }
      g_dbg_last_tx_text[tx_i] = '\0';
    }
    for (size_t tx_i = 0U; tx_i < sizeof(g_dbg_view.last_tx_text); tx_i++)
    {
      g_dbg_view.last_tx_text[tx_i] = g_dbg_last_tx_text[tx_i];
    }
    for (size_t card_i = 0U; card_i < sizeof(g_dbg_card_id); card_i++)
    {
      g_dbg_card_id[card_i] = '\0';
    }
    {
      const char *card_id = GliderCore_GetCardId();
      size_t card_i = 0U;
      while (card_id != NULL && card_id[card_i] != '\0' && card_i < (sizeof(g_dbg_card_id) - 1U))
      {
        g_dbg_card_id[card_i] = card_id[card_i];
        card_i++;
      }
      g_dbg_card_id[card_i] = '\0';
    }
    for (size_t card_i = 0U; card_i < sizeof(g_dbg_view.card_id); card_i++)
    {
      g_dbg_view.card_id[card_i] = g_dbg_card_id[card_i];
    }
    g_dbg_view.card_id_known = g_dbg_card_id_known;
    for (size_t sm_i = 0U; sm_i < sizeof(g_dbg_last_shortmsg_payload); sm_i++)
    {
      g_dbg_last_shortmsg_payload[sm_i] = '\0';
    }
    {
      const char *last_sm = GliderCore_GetLastShortMsgPayload();
      size_t sm_i = 0U;
      while (last_sm != NULL && last_sm[sm_i] != '\0' && sm_i < (sizeof(g_dbg_last_shortmsg_payload) - 1U))
      {
        g_dbg_last_shortmsg_payload[sm_i] = last_sm[sm_i];
        sm_i++;
      }
      g_dbg_last_shortmsg_payload[sm_i] = '\0';
    }
    g_dbg_view.shortmsg_sent_count = g_dbg_shortmsg_sent_count;
    for (size_t sm_i = 0U; sm_i < sizeof(g_dbg_view.last_shortmsg_payload); sm_i++)
    {
      g_dbg_view.last_shortmsg_payload[sm_i] = g_dbg_last_shortmsg_payload[sm_i];
    }

    g_dbg_quick.signal_good = g_dbg_signal_good;
    g_dbg_quick.tx_mode = g_dbg_tx_mode;
    g_dbg_quick.card_id_known = g_dbg_card_id_known;
    g_dbg_quick.card_mismatch = GliderCore_GetCardMismatch();
    g_dbg_quick.has_cached_fix = GliderCore_GetHasCachedFix();
    g_dbg_quick.shortmsg_pending = GliderCore_GetShortMsgPending();
    g_dbg_quick.ready_to_send = GliderCore_GetReadyToSend();

    for (size_t card_i = 0U; card_i < sizeof(g_dbg_quick.local_card_id); card_i++)
    {
      g_dbg_quick.local_card_id[card_i] = g_dbg_card_id[card_i];
    }
    {
      const char *target_card = GliderCore_GetPlatformCardId();
      size_t card_i = 0U;
      for (card_i = 0U; card_i < sizeof(g_dbg_quick.target_card_id); card_i++)
      {
        g_dbg_quick.target_card_id[card_i] = '\0';
      }
      card_i = 0U;
      while (target_card != NULL && target_card[card_i] != '\0' && card_i < (sizeof(g_dbg_quick.target_card_id) - 1U))
      {
        g_dbg_quick.target_card_id[card_i] = target_card[card_i];
        card_i++;
      }
      g_dbg_quick.target_card_id[card_i] = '\0';
    }

    PressureSensorDriver_Poll(g_dbg_rtos_tick);

    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUartRxTask */
/**
* @brief Function implementing the uart7RxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartRxTask */
void StartUartRxTask(void *argument)
{
  /* USER CODE BEGIN StartUartRxTask */
  (void)argument;
  GliderCore_RunRxTask();
  /* USER CODE END StartUartRxTask */
}

/* USER CODE BEGIN Header_StartUartTxTask */
/**
* @brief Function implementing the uartTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTxTask */
void StartUartTxTask(void *argument)
{
  /* USER CODE BEGIN StartUartTxTask */
  (void)argument;
  GliderCore_RunTxTask();
  /* USER CODE END StartUartTxTask */
}

/* USER CODE BEGIN Header_Start_IMU_Task */
/**
* @brief Function implementing the imuRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_IMU_Task */
void Start_IMU_Task(void *argument)
{
  /* USER CODE BEGIN Start_IMU_Task */
  uint32_t i;
  uint8_t recent_buf[16];
  IMU_RawData raw;
  IMU_EulerAngle euler;
  IMU_Quaternion quat;
  IMU_BaroData baro;
  IMU_Version ver;

  /* Infinite loop */
  for(;;)
  {
    g_dbg_imu_task_loop_count++;

    /* Update statistics */
    g_imu_debug.frame_count = IMU_Core_GetFrameCount();
    g_imu_debug.crc_error_count = IMU_Core_GetCrcErrorCount();
    g_imu_debug.rx_overflow_count = IMU_Core_GetRxOverflowCount();
    g_imu_debug.rx_irq_count = IMU_Core_GetRxIrqCount();
    g_imu_debug.rx_byte_count = IMU_Core_GetRxByteCount();
    g_imu_debug.rx_poll_byte_count = IMU_Core_GetRxPollByteCount();
    g_imu_debug.rx_rearm_ok_count = IMU_Core_GetRxRearmOkCount();
    g_imu_debug.rx_rearm_fail_count = IMU_Core_GetRxRearmFailCount();
    g_imu_debug.rx_init_status = IMU_Core_GetRxInitStatus();
    g_imu_debug.uart4_irq_count = g_dbg_uart4_irq_count;
    g_imu_debug.uart4_error_count = g_dbg_uart4_error_count;
    g_imu_debug.uart4_last_error = g_dbg_uart4_last_error;
    g_imu_debug.uart_rxne_flag = IMU_Core_GetUartRxneFlag();
    g_imu_debug.uart_error_flags = IMU_Core_GetUartErrorFlags();
    g_imu_debug.recent_rx_len = IMU_Core_GetRecentRxBytes(recent_buf, sizeof(recent_buf));
    for (i = 0U; i < sizeof(g_imu_debug.recent_rx_bytes); i++)
    {
      g_imu_debug.recent_rx_bytes[i] = (i < g_imu_debug.recent_rx_len) ? recent_buf[i] : 0U;
    }
    g_imu_debug.task_loop_count = g_dbg_imu_task_loop_count;
    g_imu_debug.last_function_code = IMU_Core_GetLastFunctionCode();
    g_imu_debug.last_rx_function = IMU_Core_GetLastRxFunction();
    g_imu_debug.last_rx_state = IMU_Core_GetLastRxState();

    /* Read raw 9-axis data if available */
    if (IMU_Core_GetRawData(&raw))
    {
      g_imu_debug.ax = raw.accel.ax;
      g_imu_debug.ay = raw.accel.ay;
      g_imu_debug.az = raw.accel.az;
      g_imu_debug.gx = raw.gyro.gx;
      g_imu_debug.gy = raw.gyro.gy;
      g_imu_debug.gz = raw.gyro.gz;
      g_imu_debug.mx = raw.mag.mx;
      g_imu_debug.my = raw.mag.my;
      g_imu_debug.mz = raw.mag.mz;
      g_imu_debug.raw_updated = raw.updated;
    }

    /* Read Euler angles if available */
    if (IMU_Core_GetEulerAngle(&euler))
    {
      g_imu_debug.roll = euler.roll;
      g_imu_debug.pitch = euler.pitch;
      g_imu_debug.yaw = euler.yaw;
      g_imu_debug.euler_updated = euler.updated;
    }

    /* Read quaternion if available */
    if (IMU_Core_GetQuaternion(&quat))
    {
      g_imu_debug.q0 = quat.q0;
      g_imu_debug.q1 = quat.q1;
      g_imu_debug.q2 = quat.q2;
      g_imu_debug.q3 = quat.q3;
      g_imu_debug.quat_updated = quat.updated;
    }

    /* Read barometer data if available */
    if (IMU_Core_GetBaroData(&baro))
    {
      g_imu_debug.height = baro.height;
      g_imu_debug.temperature = baro.temperature;
      g_imu_debug.pressure = baro.pressure;
      g_imu_debug.pressure_contrast = baro.pressure_contrast;
      g_imu_debug.baro_updated = baro.updated;
    }

    /* Read version info if available */
    if (IMU_Core_GetVersion(&ver))
    {
      g_imu_debug.version_high = ver.version_high;
      g_imu_debug.version_mid = ver.version_mid;
      g_imu_debug.version_low = ver.version_low;
    }

    /* Process RX data (runs the frame parser state machine) */
    IMU_Core_RunRxTask();

    /* Yield CPU so other tasks (e.g. servo task) can run reliably. */
    osDelay(1);
  }
  /* USER CODE END Start_IMU_Task */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
* @brief Function implementing the servoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoTask */
void StartServoTask(void *argument)
{
  /* USER CODE BEGIN StartServoTask */
  uint16_t channels[SBUS_CHANNEL_COUNT] = {0U};
  uint32_t frame_lost = 0U;
  uint32_t failsafe = 0U;
  uint32_t mode = RC_CTRL_MODE_OFF;
  uint32_t off_reason = RC_OFF_REASON_NONE;
  uint32_t now_tick;
  uint32_t channel_count;
  uint8_t motor_enabled = 0U;
  uint16_t sw_left_outer;
  uint16_t sw_left_mid_1;
  uint16_t sw_left_mid_2;
  uint16_t sw_right_outer;
  int32_t worm_cmd_rpm;
  int32_t stepper_cmd_rpm;

  (void)argument;

  Stepper_Init();
  Stepper_SetContinuousSignedRpm(0);
  UMotor_SetTargetSpeedById(RC_WORM_MOTOR_NODE_ID, 0);
  UMotor_SetMotorEnable(RC_WORM_MOTOR_SLOT, 0U);

  for(;;)
  {
    now_tick = HAL_GetTick();
    channel_count = SbusDriver_GetChannels(channels, SBUS_CHANNEL_COUNT,
                                           &frame_lost, &failsafe, NULL);

    worm_cmd_rpm = 0;
    stepper_cmd_rpm = 0;
    mode = RC_CTRL_MODE_OFF;
    off_reason = RC_OFF_REASON_NONE;

    if ((channel_count < SBUS_CHANNEL_COUNT) ||
        (SbusDriver_IsFrameFresh(now_tick, RC_FRAME_TIMEOUT_MS) == 0U))
    {
      off_reason = RC_OFF_REASON_FRAME_TIMEOUT;
    }
    else if ((failsafe != 0U) || (frame_lost != 0U))
    {
      off_reason = RC_OFF_REASON_FAILSAFE_OR_LOST;
    }
    else
    {
      sw_left_outer = channels[SBUS_CH_SWITCH_LEFT_OUTER];
      sw_left_mid_1 = channels[SBUS_CH_SWITCH_LEFT_MID_1];
      sw_left_mid_2 = channels[SBUS_CH_SWITCH_LEFT_MID_2];
      sw_right_outer = channels[SBUS_CH_SWITCH_RIGHT_OUTER];

      g_rc_sw_left_outer = sw_left_outer;
      g_rc_sw_left_mid_1 = sw_left_mid_1;
      g_rc_sw_left_mid_2 = sw_left_mid_2;
      g_rc_sw_right_outer = sw_right_outer;

      if ((RcControl_IsSwitchUp(sw_left_outer) != 0U) &&
          (RcControl_IsSwitchUp(sw_left_mid_1) != 0U) &&
          (RcControl_IsSwitchUp(sw_left_mid_2) != 0U) &&
          (RcControl_IsSwitchUp(sw_right_outer) != 0U))
      {
        off_reason = RC_OFF_REASON_ALL_SWITCH_UP;
      }
      else if (RcControl_IsSwitchMid(sw_left_mid_1) != 0U)
      {
        mode = RC_CTRL_MODE_OPEN_LOOP_SPEED;
      }
      else
      {
        off_reason = RC_OFF_REASON_MODE_SWITCH_NOT_MID;
      }
    }

    if (mode == RC_CTRL_MODE_OPEN_LOOP_SPEED)
    {
      g_rc_left_stick_y = channels[SBUS_CH_LEFT_STICK_Y];
      g_rc_left_stick_x = channels[SBUS_CH_LEFT_STICK_X];
      g_rc_right_stick_y = channels[SBUS_CH_RIGHT_STICK_Y];

      worm_cmd_rpm = RcControl_MapStickToSignedRpm(g_rc_left_stick_y, RC_WORM_MOTOR_MAX_RPM);
      stepper_cmd_rpm = RcControl_MapStickToSignedRpm(g_rc_left_stick_x, RC_STEPPER_MAX_RPM);

      if (motor_enabled == 0U)
      {
        UMotor_SetMotorEnable(RC_WORM_MOTOR_SLOT, 1U);
        motor_enabled = 1U;
      }

      UMotor_SetTargetSpeedById(RC_WORM_MOTOR_NODE_ID, worm_cmd_rpm);
      Stepper_SetContinuousSignedRpm(stepper_cmd_rpm);
    }
    else
    {
      UMotor_SetTargetSpeedById(RC_WORM_MOTOR_NODE_ID, 0);

      if (motor_enabled != 0U)
      {
        UMotor_SetMotorEnable(RC_WORM_MOTOR_SLOT, 0U);
        motor_enabled = 0U;
      }

      Stepper_SetContinuousSignedRpm(0);
    }

    g_rc_ctrl_mode = mode;
    g_rc_ctrl_off_reason = off_reason;
    g_rc_worm_cmd_rpm = worm_cmd_rpm;
    g_rc_stepper_cmd_rpm = stepper_cmd_rpm;



    osDelay(RC_CONTROL_PERIOD_MS);
  }
  /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_StartUsart3Task */
/**
* @brief Function implementing the Usart3Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsart3Task */
void StartUsart3Task(void *argument)
{
  /* USER CODE BEGIN StartUsart3Task */
  (void)argument;

  for (;;)
  {
    uint32_t now = osKernelGetTickCount();
    IoControlPump_Poll(now);
    osDelay(10);
  }
  /* USER CODE END StartUsart3Task */
}

/* USER CODE BEGIN Header_StartUsart2Task */
/**
* @brief Function implementing the Usart2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsart2Task */
void StartUsart2Task(void *argument)
{
  /* USER CODE BEGIN StartUsart2Task */
  UMotor_StartTask(argument);
  /* USER CODE END StartUsart2Task */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  g_dbg_rtos_hook_code = 4U;
  __disable_irq();
  while (1)
  {
    g_dbg_fault_count++;
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
