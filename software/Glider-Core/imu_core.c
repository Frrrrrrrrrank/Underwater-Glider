#include "imu_core.h"
#include "cmsis_os.h"
#include <string.h>

/* ============================================================
 * Ring Buffer Management (IMU-specific)
 * ============================================================
 */

static UART_HandleTypeDef *imu_uart = NULL;
static volatile uint8_t imu_rx_ring[IMU_RX_BUFFER_SIZE];
static volatile uint16_t imu_rx_head = 0;
static volatile uint16_t imu_rx_tail = 0;
static volatile uint32_t imu_rx_overflow_count = 0;
static volatile uint8_t imu_rx_byte = 0;  /* Temporary buffer for HAL interrupt */
static volatile uint32_t imu_rx_irq_count = 0;
static volatile uint32_t imu_rx_byte_count = 0;
static volatile uint32_t imu_rx_poll_byte_count = 0;
static volatile uint32_t imu_rx_rearm_ok_count = 0;
static volatile uint32_t imu_rx_rearm_fail_count = 0;
static volatile uint32_t imu_rx_init_status = 0;
static volatile uint8_t imu_recent_rx[IMU_RX_RECENT_DEBUG_SIZE];
static volatile uint8_t imu_recent_rx_pos = 0;
static volatile uint8_t imu_recent_rx_count = 0;

#define IMU_RX_IRQ_BUDGET_BYTES 64U

/* ============================================================
 * IMU Data Cache (Static storage for sensor readings)
 * ============================================================
 */

typedef struct
{
    /* Raw sensor data */
    float ax, ay, az;           /* Accelerometer (m/s^2) */
    float gx, gy, gz;           /* Gyroscope (rad/s) */
    float mx, my, mz;           /* Magnetometer (uT) */
    uint32_t raw_data_updated;

    /* Euler angles */
    float roll, pitch, yaw;     /* Degrees */
    uint32_t euler_updated;

    /* Quaternion */
    float q0, q1, q2, q3;
    uint32_t quat_updated;

    /* Barometer */
    float height;
    float temperature;
    float pressure;
    float pressure_contrast;
    uint32_t baro_updated;

    /* Version */
    uint8_t version_high;
    uint8_t version_mid;
    uint8_t version_low;
    uint32_t version_known;

    /* Statistics */
    uint32_t frame_count;
    uint32_t crc_error_count;
    uint8_t last_function_code;
    uint8_t last_rx_function;
    int16_t last_rx_state;
} IMU_DataCache;

static IMU_DataCache imu_cache = {0};

/* ============================================================
 * Helper Functions for Binary Data Conversion
 * ============================================================
 */

/**
 * @brief Convert 2 bytes (little-endian) to signed 16-bit integer
 * Protocol specifies little-endian: low byte first, high byte second
 */
static int16_t to_int16(const uint8_t *data)
{
    if (data == NULL)
        return 0;
    /* Little-endian: data[0] is low byte, data[1] is high byte */
    return (int16_t)((data[1] << 8) | data[0]);
}

/**
 * @brief Convert 4 bytes (little-endian) to IEEE 754 float
 * Protocol specifies little-endian: low byte first, high byte last
 */
static float to_float(const uint8_t *data)
{
    union
    {
        uint32_t u32;
        float f32;
    } converter;

    if (data == NULL)
        return 0.0f;

    /* Little-endian: data[0] is low byte, data[3] is high byte */
    converter.u32 = ((uint32_t)data[3] << 24) |
                    ((uint32_t)data[2] << 16) |
                    ((uint32_t)data[1] << 8) |
                    ((uint32_t)data[0]);

    return converter.f32;
}

/* ============================================================
 * Ring Buffer Operations
 * ============================================================
 */

/**
 * @brief Pop one byte from ring buffer
 * @param out Pointer to output byte
 * @return 0 on success, -1 on empty
 */
static int ring_pop(uint8_t *out)
{
    uint16_t tail = 0;

    if (out == NULL)
        return -1;

    if (imu_rx_tail == imu_rx_head)
        return -1;  /* Buffer empty */

    tail = imu_rx_tail;
    *out = (uint8_t)imu_rx_ring[tail];
    imu_rx_tail = (uint16_t)((tail + 1U) % IMU_RX_BUFFER_SIZE);
    return 0;
}

/**
 * @brief Push one byte to ring buffer
 */
static void ring_push(uint8_t data)
{
    uint16_t next = (uint16_t)((imu_rx_head + 1U) % IMU_RX_BUFFER_SIZE);

    if (next == imu_rx_tail)
    {
        /* Buffer overflow - discard oldest data */
        imu_rx_overflow_count++;
        imu_rx_tail = (uint16_t)((imu_rx_tail + 1U) % IMU_RX_BUFFER_SIZE);
    }

    imu_rx_ring[imu_rx_head] = data;
    imu_rx_head = next;

    imu_recent_rx[imu_recent_rx_pos] = data;
    imu_recent_rx_pos = (uint8_t)((imu_recent_rx_pos + 1U) % IMU_RX_RECENT_DEBUG_SIZE);
    if (imu_recent_rx_count < IMU_RX_RECENT_DEBUG_SIZE)
    {
        imu_recent_rx_count++;
    }
}

/* ============================================================
 * Frame Parsing and Data Extraction
 * ============================================================
 */

/**
 * @brief Parse raw accelerometer/gyro/magnetometer frame
 * Frame contains 18 bytes of data: ax(2) + ay(2) + az(2) + gx(2) + gy(2) + gz(2) + mx(2) + my(2) + mz(2)
 */
static void parse_raw_accel_frame(const uint8_t *frame_data, uint16_t data_length)
{
    if (data_length < 19)  /* 18 data bytes + 1 checksum */
        return;

    float accel_ratio = 16.0f * 9.80665f / 32767.0f;//针对EKF修改，算法的输入是m/s^2
    imu_cache.ax = to_int16(&frame_data[0]) * accel_ratio;
    imu_cache.ay = to_int16(&frame_data[2]) * accel_ratio;
    imu_cache.az = to_int16(&frame_data[4]) * accel_ratio;

    float deg_to_rad = 3.14159265358979323846f / 180.0f;
    float gyro_ratio = (2000.0f / 32767.0f) * deg_to_rad;
    imu_cache.gx = to_int16(&frame_data[6]) * gyro_ratio;
    imu_cache.gy = to_int16(&frame_data[8]) * gyro_ratio;
    imu_cache.gz = to_int16(&frame_data[10]) * gyro_ratio;

    float mag_ratio = 800.0f / 32767.0f;
    imu_cache.mx = to_int16(&frame_data[12]) * mag_ratio;
    imu_cache.my = to_int16(&frame_data[14]) * mag_ratio;
    imu_cache.mz = to_int16(&frame_data[16]) * mag_ratio;

    imu_cache.raw_data_updated = 1U;
}

/**
 * @brief Parse Euler angles frame (roll, pitch, yaw - 12 bytes)
 */
static void parse_euler_frame(const uint8_t *frame_data, uint16_t data_length)
{
    if (data_length < 13)  /* 12 data bytes + 1 checksum */
        return;

    imu_cache.roll = to_float(&frame_data[0]);
    imu_cache.pitch = to_float(&frame_data[4]);
    imu_cache.yaw = to_float(&frame_data[8]);
    imu_cache.euler_updated = 1U;
}

/**
 * @brief Parse quaternion frame (q0, q1, q2, q3 - 16 bytes)
 */
static void parse_quat_frame(const uint8_t *frame_data, uint16_t data_length)
{
    if (data_length < 17)  /* 16 data bytes + 1 checksum */
        return;

    imu_cache.q0 = to_float(&frame_data[0]);
    imu_cache.q1 = to_float(&frame_data[4]);
    imu_cache.q2 = to_float(&frame_data[8]);
    imu_cache.q3 = to_float(&frame_data[12]);
    imu_cache.quat_updated = 1U;
}

/**
 * @brief Parse barometer frame (altitude, temp, pressure, pressure_contrast - 16 bytes)
 */
static void parse_baro_frame(const uint8_t *frame_data, uint16_t data_length)
{
    if (data_length < 17)  /* 16 data bytes + 1 checksum */
        return;

    imu_cache.height = to_float(&frame_data[0]);
    imu_cache.temperature = to_float(&frame_data[4]);
    imu_cache.pressure = to_float(&frame_data[8]);
    imu_cache.pressure_contrast = to_float(&frame_data[12]);
    imu_cache.baro_updated = 1U;
}

/**
 * @brief Parse version frame (3 bytes)
 */
static void parse_version_frame(const uint8_t *frame_data, uint16_t data_length)
{
    if (data_length < 4)  /* 3 data bytes + 1 checksum */
        return;

    imu_cache.version_high = frame_data[0];
    imu_cache.version_mid = frame_data[1];
    imu_cache.version_low = frame_data[2];
    imu_cache.version_known = 1U;
}

/**
 * @brief Parse return state frame
 */
static void parse_return_state_frame(const uint8_t *frame_data, uint16_t data_length)
{
    if (data_length < 3)  /* 2 data bytes + 1 checksum */
        return;

    imu_cache.last_rx_function = frame_data[0];
    imu_cache.last_rx_state = (int16_t)frame_data[1];
}

/**
 * @brief Main frame data parser - dispatches to appropriate handler
 */
static void parse_frame_data(uint8_t frame_function, const uint8_t *frame_data, uint16_t data_length)
{
    if (frame_data == NULL)
        return;

    imu_cache.last_function_code = frame_function;

    /* Dispatch to appropriate frame parser based on protocol function code */
    if (frame_function == IMU_FUNC_RAW_DATA) {
        parse_raw_accel_frame(frame_data, data_length);
    } else if (frame_function == IMU_FUNC_EULER) {
        parse_euler_frame(frame_data, data_length);
    } else if (frame_function == IMU_FUNC_QUATERNION) {
        parse_quat_frame(frame_data, data_length);
    } else if (frame_function == IMU_FUNC_BAROMETER) {
        parse_baro_frame(frame_data, data_length);
    } else if (frame_function == IMU_FUNC_VERSION) {
        parse_version_frame(frame_data, data_length);
    } else if (frame_function == IMU_FUNC_RETURN_STATUS) {
        parse_return_state_frame(frame_data, data_length);
    }
    /* For unknown function codes: no action needed */

    /* Call optional user callback */
    IMU_Core_OnFrameReceived(frame_function);
}

/* Try validate one candidate frame length and parse on checksum match. */
static uint32_t try_parse_candidate(uint8_t frame_length,
                                    uint8_t frame_function,
                                    const uint8_t *frame_buffer,
                                    uint16_t candidate_len)
{
    uint8_t calculated_checksum;
    uint8_t received_checksum;
    uint16_t i;

    if (frame_buffer == NULL)
        return 0U;

    if (candidate_len == 0U || candidate_len > IMU_FRAME_DATA_MAX)
        return 0U;

    calculated_checksum = (uint8_t)(IMU_FRAME_HEAD1 + IMU_FRAME_HEAD2 + frame_length + frame_function);
    for (i = 0U; i < (candidate_len - 1U); i++)
    {
        calculated_checksum = (uint8_t)(calculated_checksum + frame_buffer[i]);
    }

    received_checksum = frame_buffer[candidate_len - 1U];
    if (calculated_checksum != received_checksum)
        return 0U;

    parse_frame_data(frame_function, frame_buffer, candidate_len);
    imu_cache.frame_count++;
    return 1U;
}

/* ============================================================
 * Frame Processor (State Machine)
 * ============================================================
 */

/**
 * @brief Process incoming byte stream and extract complete frames
 * Implements a state machine for IMU protocol frame parsing
 */
static void process_imu_frames(void)
{
    enum
    {
        RX_STATE_EXPECT_HEAD1 = 0,
        RX_STATE_EXPECT_HEAD2,
        RX_STATE_EXPECT_LENGTH,
        RX_STATE_EXPECT_FUNCTION,
        RX_STATE_COLLECT_DATA
    };

    static uint8_t rx_state = RX_STATE_EXPECT_HEAD1;
    static uint8_t frame_length = 0;
    static uint8_t frame_function = 0;
    static uint8_t frame_buffer[IMU_FRAME_DATA_MAX];
    static uint16_t frame_index = 0;
    uint8_t current_byte = 0;

    while (ring_pop(&current_byte) == 0)
    {
        switch (rx_state)
        {
        case RX_STATE_EXPECT_HEAD1:
            if (current_byte == IMU_FRAME_HEAD1)
                rx_state = RX_STATE_EXPECT_HEAD2;
            break;

        case RX_STATE_EXPECT_HEAD2:
            if (current_byte == IMU_FRAME_HEAD2)
                rx_state = RX_STATE_EXPECT_LENGTH;
            else
                rx_state = RX_STATE_EXPECT_HEAD1;
            break;

        case RX_STATE_EXPECT_LENGTH:
            frame_length = current_byte;
            rx_state = RX_STATE_EXPECT_FUNCTION;
            break;

        case RX_STATE_EXPECT_FUNCTION:
            frame_function = current_byte;
            frame_index = 0;
            rx_state = RX_STATE_COLLECT_DATA;
            break;

        case RX_STATE_COLLECT_DATA:
        {
            uint16_t data_length = (frame_length >= 4U) ? (uint16_t)(frame_length - 4U) : 0U;

            if (data_length == 0U || data_length > sizeof(frame_buffer))
            {
                rx_state = RX_STATE_EXPECT_HEAD1;
                break;
            }

            frame_buffer[frame_index++] = current_byte;

            if (frame_index >= data_length)
            {
                if (try_parse_candidate(frame_length, frame_function, frame_buffer, data_length) == 0U)
                {
                    imu_cache.crc_error_count++;
                }
                rx_state = RX_STATE_EXPECT_HEAD1;
            }
        }
        break;

        default:
            rx_state = RX_STATE_EXPECT_HEAD1;
            break;
        }
    }
}

/* ============================================================
 * Module Initialization
 * ============================================================
 */

void IMU_Core_Init(UART_HandleTypeDef *uart_handle)
{
    imu_uart = uart_handle;
    imu_rx_head = 0;
    imu_rx_tail = 0;
    imu_rx_overflow_count = 0;
    imu_rx_byte = 0;
    imu_rx_irq_count = 0;
    imu_rx_byte_count = 0;
    imu_rx_poll_byte_count = 0;
    imu_rx_rearm_ok_count = 0;
    imu_rx_rearm_fail_count = 0;
    imu_rx_init_status = (uint32_t)HAL_ERROR;

    /* Initialize data cache */
    memset(&imu_cache, 0, sizeof(imu_cache));

    if (imu_uart != NULL)
    {
        __HAL_UART_DISABLE_IT(imu_uart, UART_IT_RXNE);
        __HAL_UART_DISABLE_IT(imu_uart, UART_IT_ERR);
        __HAL_UART_DISABLE_IT(imu_uart, UART_IT_PE);

        __HAL_UART_CLEAR_OREFLAG(imu_uart);
        __HAL_UART_CLEAR_NEFLAG(imu_uart);
        __HAL_UART_CLEAR_FEFLAG(imu_uart);
        __HAL_UART_CLEAR_PEFLAG(imu_uart);
        imu_uart->ErrorCode = HAL_UART_ERROR_NONE;
        imu_uart->RxState = HAL_UART_STATE_READY;
        imu_rx_init_status = (uint32_t)HAL_OK;

        if (imu_uart->Instance == UART5)
        {
            HAL_NVIC_SetPriority(UART5_IRQn, 10, 0);
            HAL_NVIC_EnableIRQ(UART5_IRQn);
        }
        __HAL_UART_ENABLE_IT(imu_uart, UART_IT_RXNE);
    }
}

/* ============================================================
 * RX Task - Main data reception and processing loop
 * ============================================================
 */

void IMU_Core_RunRxTask(void)
{
    uint32_t guard = 0U;

    if (imu_uart != NULL)
    {
        if (imu_uart->Instance != UART5)
        {
            while (((imu_uart->Instance->ISR & USART_ISR_RXNE_RXFNE) != 0U) && (guard < IMU_RX_BUFFER_SIZE))
            {
                uint8_t polled_byte = (uint8_t)(imu_uart->Instance->RDR & 0xFFU);
                ring_push(polled_byte);
                imu_rx_byte_count++;
                imu_rx_poll_byte_count++;
                guard++;
            }
        }

        if ((imu_uart->Instance->ISR & USART_ISR_ORE) != 0U)
        {
            __HAL_UART_CLEAR_OREFLAG(imu_uart);
            imu_uart->ErrorCode = HAL_UART_ERROR_NONE;
            imu_uart->RxState = HAL_UART_STATE_READY;
        }
        if ((imu_uart->Instance->ISR & USART_ISR_NE) != 0U)
        {
            __HAL_UART_CLEAR_NEFLAG(imu_uart);
        }
        if ((imu_uart->Instance->ISR & USART_ISR_FE) != 0U)
        {
            __HAL_UART_CLEAR_FEFLAG(imu_uart);
        }
        if ((imu_uart->Instance->ISR & USART_ISR_PE) != 0U)
        {
            __HAL_UART_CLEAR_PEFLAG(imu_uart);
        }
    }

    process_imu_frames();
}

/* ============================================================
 * TX Task - Optional for sending configuration commands
 * ============================================================
 */

void IMU_Core_RunTxTask(void)
{
    /* This task can be used to send configuration commands or requests to the IMU */
    /* For example: version query, sensor reset, sampling rate change, etc. */
    
    for (;;)
    {
        /* Implement any periodic TX operations here */
        osDelay(1000);  /* 1 second delay */
    }
}

/* ============================================================
 * Getter Functions - Access Latest Sensor Data
 * ============================================================
 */

uint32_t IMU_Core_GetRawData(IMU_RawData *data)
{
    if (data == NULL)
        return 0;

    data->accel.ax = imu_cache.ax;
    data->accel.ay = imu_cache.ay;
    data->accel.az = imu_cache.az;

    data->gyro.gx = imu_cache.gx;
    data->gyro.gy = imu_cache.gy;
    data->gyro.gz = imu_cache.gz;

    data->mag.mx = imu_cache.mx;
    data->mag.my = imu_cache.my;
    data->mag.mz = imu_cache.mz;

    data->updated = imu_cache.raw_data_updated;

    /* Clear the update flag after reading */
    imu_cache.raw_data_updated = 0;

    return (imu_cache.frame_count > 0) ? 1 : 0;
}

uint32_t IMU_Core_GetEulerAngle(IMU_EulerAngle *euler)
{
    if (euler == NULL)
        return 0;

    euler->roll = imu_cache.roll;
    euler->pitch = imu_cache.pitch;
    euler->yaw = imu_cache.yaw;
    euler->updated = imu_cache.euler_updated;

    /* Clear the update flag after reading */
    imu_cache.euler_updated = 0;

    return (imu_cache.frame_count > 0) ? 1 : 0;
}

uint32_t IMU_Core_GetQuaternion(IMU_Quaternion *quat)
{
    if (quat == NULL)
        return 0;

    quat->q0 = imu_cache.q0;
    quat->q1 = imu_cache.q1;
    quat->q2 = imu_cache.q2;
    quat->q3 = imu_cache.q3;
    quat->updated = imu_cache.quat_updated;

    /* Clear the update flag after reading */
    imu_cache.quat_updated = 0;

    return (imu_cache.frame_count > 0) ? 1 : 0;
}

uint32_t IMU_Core_GetBaroData(IMU_BaroData *baro)
{
    if (baro == NULL)
        return 0;

    baro->height = imu_cache.height;
    baro->temperature = imu_cache.temperature;
    baro->pressure = imu_cache.pressure;
    baro->pressure_contrast = imu_cache.pressure_contrast;
    baro->updated = imu_cache.baro_updated;

    /* Clear the update flag after reading */
    imu_cache.baro_updated = 0;

    return (imu_cache.frame_count > 0) ? 1 : 0;
}

uint32_t IMU_Core_GetVersion(IMU_Version *version)
{
    if (version == NULL)
        return 0;

    version->version_high = imu_cache.version_high;
    version->version_mid = imu_cache.version_mid;
    version->version_low = imu_cache.version_low;

    return imu_cache.version_known;
}

/* ============================================================
 * Diagnostic Functions
 * ============================================================
 */

uint32_t IMU_Core_GetFrameCount(void)
{
    return imu_cache.frame_count;
}

uint32_t IMU_Core_GetRxOverflowCount(void)
{
    return imu_rx_overflow_count;
}

uint32_t IMU_Core_GetRxIrqCount(void)
{
    return imu_rx_irq_count;
}

uint32_t IMU_Core_GetRxByteCount(void)
{
    return imu_rx_byte_count;
}

uint32_t IMU_Core_GetRxPollByteCount(void)
{
    return imu_rx_poll_byte_count;
}

uint32_t IMU_Core_GetRecentRxBytes(uint8_t *out, uint32_t max_len)
{
    uint32_t count = 0;
    uint8_t start = 0;
    uint32_t i = 0;

    if (out == NULL || max_len == 0U)
        return 0U;

    count = (uint32_t)imu_recent_rx_count;
    if (count > max_len)
        count = max_len;

    if (count == 0U)
        return 0U;

    start = (uint8_t)((imu_recent_rx_pos + IMU_RX_RECENT_DEBUG_SIZE - count) % IMU_RX_RECENT_DEBUG_SIZE);
    for (i = 0U; i < count; i++)
    {
        out[i] = (uint8_t)imu_recent_rx[(start + i) % IMU_RX_RECENT_DEBUG_SIZE];
    }

    return count;
}

uint32_t IMU_Core_GetRxRearmOkCount(void)
{
    return imu_rx_rearm_ok_count;
}

uint32_t IMU_Core_GetRxRearmFailCount(void)
{
    return imu_rx_rearm_fail_count;
}

uint32_t IMU_Core_GetRxInitStatus(void)
{
    return imu_rx_init_status;
}

uint32_t IMU_Core_GetCrcErrorCount(void)
{
    return imu_cache.crc_error_count;
}

uint8_t IMU_Core_GetLastFunctionCode(void)
{
    return imu_cache.last_function_code;
}

uint8_t IMU_Core_GetLastRxFunction(void)
{
    return imu_cache.last_rx_function;
}

int16_t IMU_Core_GetLastRxState(void)
{
    return imu_cache.last_rx_state;
}

/**
 * @brief Get current UART RXNE flag status
 * @return 1 if RXNE flag is set (data available), 0 otherwise
 */
uint32_t IMU_Core_GetUartRxneFlag(void)
{
    if (imu_uart == NULL)
        return 0;
    return ((imu_uart->Instance->ISR & (1UL << 5)) != 0U) ? 1U : 0U;
}

/**
 * @brief Get current UART error flags status
 * @return Bitmask: bit0=ORE, bit1=NE, bit2=FE, bit3=PE
 */
uint32_t IMU_Core_GetUartErrorFlags(void)
{
    if (imu_uart == NULL)
        return 0;
    
    uint32_t flags = 0;
    if ((imu_uart->Instance->ISR & USART_ISR_ORE) != 0U)
        flags |= 0x01;
    if ((imu_uart->Instance->ISR & USART_ISR_NE) != 0U)
        flags |= 0x02;
    if ((imu_uart->Instance->ISR & USART_ISR_FE) != 0U)
        flags |= 0x04;
    if ((imu_uart->Instance->ISR & USART_ISR_PE) != 0U)
        flags |= 0x08;
    
    return flags;
}

void IMU_Core_ResetStatistics(void)
{
    imu_cache.frame_count = 0;
    imu_cache.crc_error_count = 0;
    imu_rx_overflow_count = 0;
    imu_rx_irq_count = 0;
    imu_rx_byte_count = 0;
    imu_rx_poll_byte_count = 0;
    imu_rx_rearm_ok_count = 0;
    imu_rx_rearm_fail_count = 0;
    imu_recent_rx_pos = 0;
    imu_recent_rx_count = 0;
}

/* ============================================================
 * Weak Callback Functions - Override in user code as needed
 * ============================================================
 */

__weak void IMU_Core_OnFrameReceived(uint8_t function_code)
{
    (void)function_code;
    /* User can override this to perform custom actions on frame arrival */
}

__weak void IMU_Core_OnRawDataUpdate(const IMU_RawData *data)
{
    (void)data;
    /* User can override this to react immediately on raw data arrival */
}

/* ============================================================
 * UART Interrupt Callback - HAL integration
 * ============================================================
 */

/**
 * @brief Internal UART RX callback for IMU module
 * Called by HAL_UART_RxCpltCallback from glider_core.c
 * 
 * @param huart UART handle
 */
void IMU_OnUART_RxComplete(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
        return;

    if (imu_uart != NULL && huart->Instance == imu_uart->Instance)
    {
        if (imu_uart->Instance != UART5)
        {
            imu_rx_irq_count++;
            ring_push(imu_rx_byte);
            imu_rx_byte_count++;
            __HAL_UART_DISABLE_IT(imu_uart, UART_IT_RXNE);
            __HAL_UART_DISABLE_IT(imu_uart, UART_IT_ERR);
            __HAL_UART_DISABLE_IT(imu_uart, UART_IT_PE);
            imu_uart->RxState = HAL_UART_STATE_READY;
        }
    }
}

void IMU_OnUART_Irq(UART_HandleTypeDef *huart)
{
    uint32_t guard = 0U;

    if (huart == NULL)
        return;

    if (imu_uart == NULL || huart->Instance != imu_uart->Instance)
        return;

    imu_rx_irq_count++;

    while (((huart->Instance->ISR & USART_ISR_RXNE_RXFNE) != 0U) && (guard < IMU_RX_IRQ_BUDGET_BYTES))
    {
        uint8_t irq_byte = (uint8_t)(huart->Instance->RDR & 0xFFU);
        ring_push(irq_byte);
        imu_rx_byte_count++;
        guard++;
    }

    if ((huart->Instance->ISR & USART_ISR_ORE) != 0U)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }
    if ((huart->Instance->ISR & USART_ISR_NE) != 0U)
    {
        __HAL_UART_CLEAR_NEFLAG(huart);
    }
    if ((huart->Instance->ISR & USART_ISR_FE) != 0U)
    {
        __HAL_UART_CLEAR_FEFLAG(huart);
    }
    if ((huart->Instance->ISR & USART_ISR_PE) != 0U)
    {
        __HAL_UART_CLEAR_PEFLAG(huart);
    }

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_READY;
}

void IMU_OnUART_Error(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
        return;

    if (imu_uart != NULL && huart->Instance == imu_uart->Instance)
    {
        if (imu_uart->Instance != UART5)
        {
            __HAL_UART_DISABLE_IT(imu_uart, UART_IT_RXNE);
            __HAL_UART_DISABLE_IT(imu_uart, UART_IT_ERR);
            __HAL_UART_DISABLE_IT(imu_uart, UART_IT_PE);
        }

        __HAL_UART_CLEAR_OREFLAG(imu_uart);
        __HAL_UART_CLEAR_NEFLAG(imu_uart);
        __HAL_UART_CLEAR_FEFLAG(imu_uart);
        __HAL_UART_CLEAR_PEFLAG(imu_uart);
        if ((imu_uart->Instance->ISR & USART_ISR_RXNE_RXFNE) != 0U)
        {
            (void)(imu_uart->Instance->RDR);
        }
        imu_uart->ErrorCode = HAL_UART_ERROR_NONE;
        imu_uart->RxState = HAL_UART_STATE_READY;
    }
}
