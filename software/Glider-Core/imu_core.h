#ifndef IMU_CORE_H
#define IMU_CORE_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * IMU Frame Protocol Definitions
 * ============================================================
 */
#define IMU_FRAME_HEAD1  0x7EU
#define IMU_FRAME_HEAD2  0x23U

/* Function codes for IMU data types (per protocol specification) */
#define IMU_FUNC_RAW_DATA       0x04U  /* Return raw 9-axis data (accel+gyro+mag) */
#define IMU_FUNC_QUATERNION     0x16U  /* Return quaternion (q0,q1,q2,q3) */
#define IMU_FUNC_EULER          0x26U  /* Return Euler angles (roll,pitch,yaw) */
#define IMU_FUNC_BAROMETER      0x32U  /* Return barometer data (height,temp,pressure,pressure_ref) */
#define IMU_FUNC_VERSION        0x01U  /* Return firmware version (major,minor,patch) */
#define IMU_FUNC_RETURN_STATUS  0x81U  /* Return status response */

/* Command function codes (for future implementation) */
#define IMU_CMD_REQUEST_VERSION 0x80U  /* Request firmware version */
#define IMU_CMD_CALIBRATE_IMU   0x70U  /* Calibrate IMU */
#define IMU_CMD_CALIBRATE_MAG   0x71U  /* Calibrate magnetometer */
#define IMU_CMD_CALIBRATE_TEMP  0x73U  /* Calibrate temperature */
#define IMU_CMD_SET_OUTPUT_FREQ 0x60U  /* Set output frequency */
#define IMU_CMD_SET_ALGO_TYPE   0x61U  /* Set algorithm type */
#define IMU_CMD_RESET_USER_DATA 0xA0U  /* Reset user data */

/* Frame structure constants */
#define IMU_FRAME_MIN_LENGTH  5U   /* HEAD1 + HEAD2 + LENGTH + FUNCTION + CHECKSUM */
#define IMU_RX_BUFFER_SIZE   256U
#define IMU_FRAME_DATA_MAX    64U
#define IMU_RX_RECENT_DEBUG_SIZE 32U

/* ============================================================
 * IMU Data Structures (Raw sensor values and processed data)
 * ============================================================
 */

/* Accelerometer data (m/s^2) */
typedef struct
{
    float ax;
    float ay;
    float az;
} IMU_AccelData;

/* Gyroscope data (rad/s) */
typedef struct
{
    float gx;
    float gy;
    float gz;
} IMU_GyroData;

/* Magnetometer data (uT) */
typedef struct
{
    float mx;
    float my;
    float mz;
} IMU_MagData;

/* Raw IMU data (9-axis or 10-axis depending on configuration) */
typedef struct
{
    IMU_AccelData accel;
    IMU_GyroData gyro;
    IMU_MagData mag;
    uint32_t updated;  /* Flag indicating data was updated */
} IMU_RawData;

/* Euler angles (degrees) */
typedef struct
{
    float roll;   /* X-axis rotation */
    float pitch;  /* Y-axis rotation */
    float yaw;    /* Z-axis rotation */
    uint32_t updated;
} IMU_EulerAngle;

/* Quaternion representation */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
    uint32_t updated;
} IMU_Quaternion;

/* Barometer data (altitude, temperature, pressure) */
typedef struct
{
    float height;
    float temperature;
    float pressure;
    float pressure_contrast;
    uint32_t updated;
} IMU_BaroData;

/* Version information */
typedef struct
{
    uint8_t version_high;
    uint8_t version_mid;
    uint8_t version_low;
} IMU_Version;

/* ============================================================
 * Module Initialization and Task Functions
 * ============================================================
 */

/**
 * @brief Initialize IMU module with UART4 handle
 * @param uart_handle UART4 handle from STM32 HAL
 */
void IMU_Core_Init(UART_HandleTypeDef *uart_handle);

/**
 * @brief RX task routine (should be called in FreeRTOS task)
 * Processes incoming UART data from IMU module
 */
void IMU_Core_RunRxTask(void);

/**
 * @brief TX task routine (optional, for querying IMU)
 * Sends configuration or query commands to IMU
 */
void IMU_Core_RunTxTask(void);

/* ============================================================
 * Data Getter Functions - Access sensor readings
 * ============================================================
 */

/**
 * @brief Get raw 9-axis IMU data (accel, gyro, mag)
 * @param data Pointer to IMU_RawData structure (output)
 * @return 1 if data is valid/updated, 0 otherwise
 */
uint32_t IMU_Core_GetRawData(IMU_RawData *data);

/**
 * @brief Get Euler angles
 * @param euler Pointer to IMU_EulerAngle structure (output)
 * @return 1 if data is valid/updated, 0 otherwise
 */
uint32_t IMU_Core_GetEulerAngle(IMU_EulerAngle *euler);

/**
 * @brief Get quaternion
 * @param quat Pointer to IMU_Quaternion structure (output)
 * @return 1 if data is valid/updated, 0 otherwise
 */
uint32_t IMU_Core_GetQuaternion(IMU_Quaternion *quat);

/**
 * @brief Get barometer data
 * @param baro Pointer to IMU_BaroData structure (output)
 * @return 1 if data is valid/updated, 0 otherwise
 */
uint32_t IMU_Core_GetBaroData(IMU_BaroData *baro);

/**
 * @brief Get IMU version information
 * @param version Pointer to IMU_Version structure (output)
 * @return 1 if version has been read, 0 otherwise
 */
uint32_t IMU_Core_GetVersion(IMU_Version *version);

/* ============================================================
 * Diagnostic and State Functions
 * ============================================================
 */

/**
 * @brief Get number of complete frames received
 * @return Frame count
 */
uint32_t IMU_Core_GetFrameCount(void);

/**
 * @brief Get RX buffer overflow count
 * @return Overflow count
 */
uint32_t IMU_Core_GetRxOverflowCount(void);

/**
 * @brief Get number of CRC error frames
 * @return CRC error count
 */
uint32_t IMU_Core_GetCrcErrorCount(void);

/**
 * @brief Get last received function code
 * @return Function code
 */
uint8_t IMU_Core_GetLastFunctionCode(void);

/**
 * @brief Get return-state payload field: last received function code on IMU side.
 * @return Function code from IMU return-state frame.
 */
uint8_t IMU_Core_GetLastRxFunction(void);

/**
 * @brief Get return-state payload field: last execution state on IMU side.
 * @return State code from IMU return-state frame.
 */
int16_t IMU_Core_GetLastRxState(void);

/**
 * @brief Get UART RX interrupt callback count for IMU UART.
 * @return Number of times IMU UART RX complete callback fired.
 */
uint32_t IMU_Core_GetRxIrqCount(void);

/**
 * @brief Get raw byte count pushed into IMU ring buffer.
 * @return Total received bytes seen by IMU UART callback.
 */
uint32_t IMU_Core_GetRxByteCount(void);

/**
 * @brief Get byte count received by polling fallback path.
 * @return Number of bytes captured via RXNE polling (non-IRQ path).
 */
uint32_t IMU_Core_GetRxPollByteCount(void);

/**
 * @brief Copy recent RX bytes for protocol diagnostics.
 * @param out Destination buffer.
 * @param max_len Maximum bytes to copy.
 * @return Number of bytes copied.
 */
uint32_t IMU_Core_GetRecentRxBytes(uint8_t *out, uint32_t max_len);

/**
 * @brief Get successful RX interrupt re-arm count.
 * @return Number of HAL_OK returns from HAL_UART_Receive_IT re-arm attempts.
 */
uint32_t IMU_Core_GetRxRearmOkCount(void);

/**
 * @brief Get failed RX interrupt re-arm count.
 * @return Number of non-HAL_BUSY and non-HAL_OK re-arm failures.
 */
uint32_t IMU_Core_GetRxRearmFailCount(void);

/**
 * @brief Get initial RX interrupt arm status code.
 * @return HAL_StatusTypeDef value captured during IMU_Core_Init.
 */
uint32_t IMU_Core_GetRxInitStatus(void);

/**
 * @brief Reset all statistics counters
 */
void IMU_Core_ResetStatistics(void);

/* ============================================================
 * Optional: Weak callbacks for custom frame processing
 * ============================================================
 */

/**
 * @brief Get current state of UART RXNE flag
 * @return 1 if RXNE flag is set (data available), 0 otherwise
 */
uint32_t IMU_Core_GetUartRxneFlag(void);

/**
 * @brief Get current UART error flags
 * @return Bitmask: bit0=ORE, bit1=NE, bit2=FE, bit3=PE
 */
uint32_t IMU_Core_GetUartErrorFlags(void);

/**
 * @brief Optional: Weak callbacks for custom frame processing
 * User can override this to add custom processing
 */
void IMU_Core_OnFrameReceived(uint8_t function_code);

/**
 * @brief Callback for raw data updates
 * User can override to react immediately on data arrival
 */
void IMU_Core_OnRawDataUpdate(const IMU_RawData *data);

/**
 * @brief Internal UART RX callback
 * Called by HAL_UART_RxCpltCallback in glider_core.c
 * 
 * @param huart UART handle
 */
void IMU_OnUART_RxComplete(UART_HandleTypeDef *huart);

#endif /* IMU_CORE_H */
