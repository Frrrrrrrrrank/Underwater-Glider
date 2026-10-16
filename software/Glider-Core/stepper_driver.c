#include "stepper_driver.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "sbus_driver.h"
#include "limit_switch.h"
#include "FreeRTOS.h"
#include "task.h"
#include <limits.h>

/* 
 * 硬件资源映射：
 * PE11 (TIM1_CH2)  -> PUL+  (脉冲发送端)
 * PE13 (GPIO_Out)  -> DIR+  (方向控制端)
 * 
 * 共阴极接法，负极永远在STM32这边拉低：
 * PE10 (GPIO_Out)  -> PUL-  (接地处理)
 * PE12 (GPIO_Out)  -> DIR-  (接地处理)
 */

extern TIM_HandleTypeDef htim1;

#define STEPPER_TIM            (&htim1)
#define STEPPER_TIM_CH         TIM_CHANNEL_2

#define STEPPER_DIR_PORT       GPIOE
#define STEPPER_DIR_PIN        GPIO_PIN_13

#define STEPPER_PUL_N_PORT     GPIOE
#define STEPPER_PUL_N_PIN      GPIO_PIN_10

#define STEPPER_DIR_N_PORT     GPIOE
#define STEPPER_DIR_N_PIN      GPIO_PIN_12

/* 电机驱动器细分：改成你现在的 1600 */
#define STEPPER_PULSES_PER_REV 1600U
/* TIM1 的基准计数频率：你在 IOC 里配了 Prescaler 使得它变为 1 MHz (1微秒一次) */
#define STEPPER_TIM_FREQ_HZ    1000000U

/* 步进电机起步/停止时的最小转速 (防止定时器溢出或堵转) */
#define STEPPER_START_RPM      30U
/* 默认梯形加减速段所消耗的脉冲数 (比如 400个脉冲相当于1/4圈用来加速) */
#define STEPPER_ACCEL_PULSES   400U

#define STEPPER_TEST_MAX_RPM            800U
#define STEPPER_TEST_STICK_DEADBAND     40U
#define STEPPER_TEST_POLL_MS            20U
#define STEPPER_TEST_FRAME_TIMEOUT_MS   150U
#define STEPPER_TEST_CONTINUOUS_PULSES  ((int32_t)1000000000)

typedef struct {
    volatile int32_t current_pos;
    volatile int32_t target_pos;
    volatile uint32_t pulses_remaining;
    volatile uint8_t is_running;
    volatile int8_t direction;
    volatile uint32_t target_rpm;
    volatile uint32_t current_rpm;
    volatile uint32_t step_count;    // 当前已触发的脉冲数
    volatile uint32_t total_pulses;  // 本次运动设定的总脉冲数
    volatile int32_t requested_rpm;
    volatile uint32_t limit_stop_count;
    volatile uint8_t raw_limit_mask;
    volatile uint8_t blocked_mask;
    volatile uint8_t release_pending_mask;
    volatile uint8_t last_stop_reason;
} StepperCtrl_t;

static StepperCtrl_t g_stepper = {0};
/* 只有任务服务写时间戳；中断只置禁止位、取消对应释放计时。 */
static uint32_t s_release_since_ms[2] = {0U, 0U};

static void Stepper_StartMove(int32_t pulses, uint16_t rpm);

/* 以下三个内部函数仅在 TIM1 中断内或任务短临界区内调用。 */
static void Stepper_LatchLimits(uint8_t raw_mask)
{
    raw_mask &= ROLL_LIMIT_ALL_ACTIVE;
    g_stepper.raw_limit_mask = raw_mask;
    g_stepper.blocked_mask |= raw_mask;
    g_stepper.release_pending_mask &= (uint8_t)~raw_mask;
}

static uint8_t Stepper_IsDirectionBlocked(int8_t direction)
{
    uint8_t mask = (direction > 0) ? ROLL_LIMIT_POS_ACTIVE :
                   ((direction < 0) ? ROLL_LIMIT_NEG_ACTIVE : 0U);
    return ((g_stepper.blocked_mask & mask) != 0U) ? 1U : 0U;
}

static void Stepper_StopInternal(StepperStopReason reason)
{
    uint8_t was_running = g_stepper.is_running;

    __HAL_TIM_DISABLE_IT(STEPPER_TIM, TIM_IT_UPDATE);
    (void)HAL_TIM_PWM_Stop(STEPPER_TIM, STEPPER_TIM_CH);
    (void)HAL_TIM_Base_Stop(STEPPER_TIM);
    __HAL_TIM_CLEAR_FLAG(STEPPER_TIM, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(STEPPER_TIM, 0U);
    g_stepper.is_running = 0U;
    g_stepper.pulses_remaining = 0U;
    g_stepper.current_rpm = 0U;
    g_stepper.target_rpm = 0U;
    /* 重复零命令不抹掉限位原因；拒绝启动不累计成真实运动中的限位停机。 */
    if ((was_running != 0U) || (reason != STEPPER_STOP_COMMAND))
    {
        g_stepper.last_stop_reason = (uint8_t)reason;
    }
    if ((was_running != 0U) && (reason == STEPPER_STOP_LIMIT))
    {
        g_stepper.limit_stop_count++;
    }
    /* 保留请求、目标位置与限位状态，便于区分中止运动和到达目标。 */
}

void Stepper_ServiceLimits(uint32_t now_ms)
{
    uint8_t i;
    taskENTER_CRITICAL();
    Stepper_LatchLimits(LimitSwitch_ReadRollActiveMask());
    for (i = 0U; i < 2U; ++i)
    {
        uint8_t bit = (uint8_t)(1U << i);
        if (((g_stepper.blocked_mask & bit) != 0U) &&
            ((g_stepper.raw_limit_mask & bit) == 0U))
        {
            if ((g_stepper.release_pending_mask & bit) == 0U)
            {
                s_release_since_ms[i] = now_ms;
                g_stepper.release_pending_mask |= bit;
            }
            else if ((uint32_t)(now_ms - s_release_since_ms[i]) >= STEPPER_LIMIT_RELEASE_MS)
            {
                g_stepper.blocked_mask &= (uint8_t)~bit;
                g_stepper.release_pending_mask &= (uint8_t)~bit;
            }
        }
    }
    if ((g_stepper.is_running != 0U) &&
        (Stepper_IsDirectionBlocked(g_stepper.direction) != 0U))
    {
        Stepper_StopInternal(STEPPER_STOP_LIMIT);
    }
    taskEXIT_CRITICAL();
}

uint8_t Stepper_CopyState(StepperState *out_state)
{
    if (out_state == NULL) return 0U;
    taskENTER_CRITICAL();
    out_state->requested_rpm = g_stepper.requested_rpm;
    out_state->pulse_rpm = (g_stepper.is_running != 0U) ?
        ((int32_t)g_stepper.current_rpm * g_stepper.direction) : 0;
    out_state->current_pos = g_stepper.current_pos;
    out_state->target_pos = g_stepper.target_pos;
    out_state->pulses_remaining = g_stepper.pulses_remaining;
    out_state->limit_stop_count = g_stepper.limit_stop_count;
    out_state->running = g_stepper.is_running;
    out_state->raw_limit_mask = g_stepper.raw_limit_mask;
    out_state->blocked_mask = g_stepper.blocked_mask;
    out_state->last_stop_reason = g_stepper.last_stop_reason;
    taskEXIT_CRITICAL();
    return 1U;
}

/* Debug watch helpers for SBUS-controlled test */
static volatile uint16_t g_stepper_test_raw_left_x = SBUS_VALUE_CENTER_OBSERVED;
static volatile int32_t g_stepper_test_cmd_rpm = 0;

static int32_t Stepper_ClampSignedRpm(int32_t value)
{
    if (value > (int32_t)STEPPER_TEST_MAX_RPM)
    {
        return (int32_t)STEPPER_TEST_MAX_RPM;
    }
    if (value < -(int32_t)STEPPER_TEST_MAX_RPM)
    {
        return -(int32_t)STEPPER_TEST_MAX_RPM;
    }
    return value;
}

static int32_t Stepper_MapLeftStickToSignedRpm(uint16_t raw)
{
    const uint32_t center = SBUS_VALUE_CENTER_OBSERVED;
    const uint32_t min_v = SBUS_VALUE_MIN_OBSERVED;
    const uint32_t max_v = SBUS_VALUE_MAX_OBSERVED;
    const uint32_t deadband = STEPPER_TEST_STICK_DEADBAND;
    uint32_t threshold_high;
    uint32_t threshold_low;

    threshold_high = center + deadband;
    threshold_low = center - deadband;

    if ((uint32_t)raw > threshold_high)
    {
        uint32_t num = (uint32_t)raw - threshold_high;
        uint32_t den = max_v - threshold_high;
        uint32_t rpm = 0U;

        if (den != 0U)
        {
            rpm = (num * STEPPER_TEST_MAX_RPM) / den;
        }
        if (rpm > 0U && rpm < STEPPER_START_RPM)
        {
            rpm = STEPPER_START_RPM;
        }
        return Stepper_ClampSignedRpm((int32_t)rpm);
    }

    if ((uint32_t)raw < threshold_low)
    {
        uint32_t num = threshold_low - (uint32_t)raw;
        uint32_t den = threshold_low - min_v;
        uint32_t rpm = 0U;

        if (den != 0U)
        {
            rpm = (num * STEPPER_TEST_MAX_RPM) / den;
        }
        if (rpm > 0U && rpm < STEPPER_START_RPM)
        {
            rpm = STEPPER_START_RPM;
        }
        return Stepper_ClampSignedRpm(-(int32_t)rpm);
    }

    return 0;
}

void Stepper_SetContinuousSignedRpm(int32_t signed_rpm)
{
    int8_t desired_dir = 0;
    uint16_t abs_rpm = 0U;

    signed_rpm = Stepper_ClampSignedRpm(signed_rpm);

    if (signed_rpm > 0)
    {
        desired_dir = 1;
        abs_rpm = (uint16_t)signed_rpm;
    }
    else if (signed_rpm < 0)
    {
        desired_dir = -1;
        abs_rpm = (uint16_t)(-signed_rpm);
    }

    if (abs_rpm < STEPPER_START_RPM)
    {
        abs_rpm = STEPPER_START_RPM;
    }
    if (abs_rpm > STEPPER_TEST_MAX_RPM)
    {
        abs_rpm = STEPPER_TEST_MAX_RPM;
    }

    /* 相同方向的持续命令也必须采样，不能绕过限位检查。 */
    taskENTER_CRITICAL();
    g_stepper.requested_rpm = signed_rpm;
    Stepper_LatchLimits(LimitSwitch_ReadRollActiveMask());
    if (desired_dir == 0)
    {
        Stepper_StopInternal(STEPPER_STOP_COMMAND);
        taskEXIT_CRITICAL();
        return;
    }
    if (Stepper_IsDirectionBlocked(desired_dir) != 0U)
    {
        Stepper_StopInternal(STEPPER_STOP_LIMIT);
        taskEXIT_CRITICAL();
        return;
    }
    if ((g_stepper.is_running != 0U) && (g_stepper.direction == desired_dir))
    {
        g_stepper.target_rpm = abs_rpm;
        taskEXIT_CRITICAL();
        return;
    }
    taskEXIT_CRITICAL();
    Stepper_StartMove((desired_dir > 0) ? STEPPER_TEST_CONTINUOUS_PULSES :
                     -STEPPER_TEST_CONTINUOUS_PULSES, abs_rpm);
}

void Stepper_Init(void) 
{
    taskENTER_CRITICAL();
    Stepper_StopInternal(STEPPER_STOP_COMMAND);
    g_stepper = (StepperCtrl_t){0};
    s_release_since_ms[0] = 0U;
    s_release_since_ms[1] = 0U;
    Stepper_LatchLimits(LimitSwitch_ReadRollActiveMask());

    /* 初始化负极端恒定为低电平，模拟 GND */
    HAL_GPIO_WritePin(STEPPER_PUL_N_PORT, STEPPER_PUL_N_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STEPPER_DIR_N_PORT, STEPPER_DIR_N_PIN, GPIO_PIN_RESET);

    taskEXIT_CRITICAL();
}

/* 内部辅助函数：根据转速实时设置定时器频率以实现加减速 (直接写寄存器避免 HAL 库延迟) */
static void Stepper_UpdateARR(uint32_t rpm) 
{
    if (rpm == 0) rpm = 1;
    
    /* 算出每秒需要发多少个脉冲 */
    uint32_t pulses_per_sec = rpm * STEPPER_PULSES_PER_REV / 60U;
    if (pulses_per_sec == 0) pulses_per_sec = 1;
    
    /* 重装载值 ARR 计算 */
    uint32_t arr = STEPPER_TIM_FREQ_HZ / pulses_per_sec;
    if (arr < 2) arr = 2;
    
    /* 直接修改定时器寄存器生效。这比调用 `__HAL_TIM_SET_AUTORELOAD` 更快更纯粹 */
    STEPPER_TIM->Instance->ARR = arr - 1;
    STEPPER_TIM->Instance->CCR2 = arr / 2; // 维持 50% 的脉宽占空比
}

static void Stepper_StartMove(int32_t pulses, uint16_t rpm)
{
    int8_t direction = (pulses > 0) ? 1 : -1;
    uint32_t total_pulses = (pulses > 0) ? (uint32_t)pulses : 0U - (uint32_t)pulses;
    int64_t target;
    taskENTER_CRITICAL();
    Stepper_StopInternal(STEPPER_STOP_COMMAND);
    Stepper_LatchLimits(LimitSwitch_ReadRollActiveMask());
    if (Stepper_IsDirectionBlocked(direction) != 0U)
    {
        Stepper_StopInternal(STEPPER_STOP_LIMIT);
        taskEXIT_CRITICAL();
        return;
    }
    target = (int64_t)g_stepper.current_pos + (int64_t)pulses;
    if ((target < INT32_MIN) || (target > INT32_MAX))
    {
        Stepper_StopInternal(STEPPER_STOP_RANGE);
        taskEXIT_CRITICAL();
        return;
    }
    
    HAL_GPIO_WritePin(STEPPER_DIR_PORT, STEPPER_DIR_PIN,
                      (direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    taskEXIT_CRITICAL();

    /* 忠于手册 3.3.4 时序图：DR至少提前PU下降沿50μs确定状态。
     * 此处加一个简易的软件延时(远大于50us)，确保方向信号被驱动器光耦稳定锁别后再出脉冲！ */
    for (volatile uint32_t delay_cnt = 0; delay_cnt < 20000; delay_cnt++) {
        __NOP();
    }
    
    /* 等待期间限位可能改变；最后检查与启动合在短临界区里。 */
    taskENTER_CRITICAL();
    Stepper_LatchLimits(LimitSwitch_ReadRollActiveMask());
    if (Stepper_IsDirectionBlocked(direction) != 0U)
    {
        Stepper_StopInternal(STEPPER_STOP_LIMIT);
        taskEXIT_CRITICAL();
        return;
    }
    /* 最后才发布运动参数，等待方向建立期间快照仍是完整的停机状态。
     * 脉冲幅值用无符号运算得到，INT32_MIN 也不会产生取负溢出。 */
    g_stepper.direction = direction;
    g_stepper.total_pulses = total_pulses;
    g_stepper.pulses_remaining = total_pulses;
    g_stepper.step_count = 0U;
    g_stepper.target_rpm = (rpm < STEPPER_START_RPM) ? STEPPER_START_RPM : rpm;
    g_stepper.current_rpm = STEPPER_START_RPM;
    g_stepper.target_pos = (int32_t)target;
    Stepper_UpdateARR(g_stepper.current_rpm);
    __HAL_TIM_CLEAR_FLAG(STEPPER_TIM, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(STEPPER_TIM, TIM_IT_UPDATE);
    if (HAL_TIM_PWM_Start(STEPPER_TIM, STEPPER_TIM_CH) == HAL_OK)
    {
        g_stepper.is_running = 1U;
        g_stepper.last_stop_reason = (uint8_t)STEPPER_STOP_NONE;
    }
    else
    {
        Stepper_StopInternal(STEPPER_STOP_START_FAILED);
    }
    taskEXIT_CRITICAL();
}

void Stepper_MoveRelative(int32_t pulses, uint16_t rpm)
{
    uint16_t limited_rpm = (rpm > STEPPER_TEST_MAX_RPM) ? STEPPER_TEST_MAX_RPM : rpm;
    taskENTER_CRITICAL();
    g_stepper.requested_rpm = (pulses > 0) ? (int32_t)limited_rpm : -(int32_t)limited_rpm;
    if ((pulses == 0) || (rpm == 0U))
    {
        g_stepper.requested_rpm = 0;
        Stepper_StopInternal(STEPPER_STOP_COMMAND);
        taskEXIT_CRITICAL();
        return;
    }
    taskEXIT_CRITICAL();
    Stepper_StartMove(pulses, limited_rpm);
}

void Stepper_MoveAbsolute(int32_t target_pos, uint16_t rpm) 
{
    int64_t relative_pulses;
    uint16_t limited_rpm = (rpm > STEPPER_TEST_MAX_RPM) ? STEPPER_TEST_MAX_RPM : rpm;
    /* 先停止并固定当前位置，再计算距离，避免中断在计算后又推进一步。 */
    taskENTER_CRITICAL();
    Stepper_StopInternal(STEPPER_STOP_COMMAND);
    relative_pulses = (int64_t)target_pos - (int64_t)g_stepper.current_pos;
    g_stepper.requested_rpm = (relative_pulses > 0) ? (int32_t)limited_rpm : -(int32_t)limited_rpm;
    if ((relative_pulses == 0) || (rpm == 0U))
    {
        g_stepper.requested_rpm = 0;
        taskEXIT_CRITICAL();
        return;
    }
    if ((relative_pulses < INT32_MIN) || (relative_pulses > INT32_MAX))
    {
        Stepper_StopInternal(STEPPER_STOP_RANGE);
        taskEXIT_CRITICAL();
        return;
    }
    taskEXIT_CRITICAL();
    Stepper_StartMove((int32_t)relative_pulses, limited_rpm);
}

void Stepper_Stop(void) 
{
    taskENTER_CRITICAL();
    g_stepper.requested_rpm = 0;
    Stepper_StopInternal(STEPPER_STOP_COMMAND);
    taskEXIT_CRITICAL();
}

uint8_t Stepper_IsRunning(void) 
{
    return g_stepper.is_running;
}

int32_t Stepper_GetCurrentPosition(void) 
{
    return g_stepper.current_pos;
}

void Stepper_TIM_IRQHandler(void) 
{
    if (g_stepper.is_running != 0U) 
    {
        Stepper_LatchLimits(LimitSwitch_ReadRollActiveMask());
        if (g_stepper.pulses_remaining > 0U) 
        {
            if (((g_stepper.direction > 0) && (g_stepper.current_pos == INT32_MAX)) ||
                ((g_stepper.direction < 0) && (g_stepper.current_pos == INT32_MIN)))
            {
                Stepper_StopInternal(STEPPER_STOP_RANGE);
                return;
            }
            /* 更新中断先计入已结束的周期；位置只是脉冲估算，不是机械反馈。 */
            g_stepper.step_count++;
            g_stepper.pulses_remaining--;
            g_stepper.current_pos += g_stepper.direction;
            if (Stepper_IsDirectionBlocked(g_stepper.direction) != 0U)
            {
                Stepper_StopInternal(STEPPER_STOP_LIMIT);
                return;
            }
            
            if (g_stepper.pulses_remaining == 0U) 
            {
                Stepper_StopInternal(STEPPER_STOP_COMPLETED);
            }
            else
            {
                /* =================平滑加减速(S型/梯形曲线逼近)================ */
                uint32_t calc_rpm = g_stepper.target_rpm;
                
                if (g_stepper.total_pulses > STEPPER_ACCEL_PULSES * 2) {
                    /* 有足够的空间形成完整的梯形：起步加速 -> 匀速巡航 -> 减速停靠 */
                    if (g_stepper.step_count < STEPPER_ACCEL_PULSES) {
                        /* 1. 加速段：从最小起跑转速插值爬升到目标速度 */
                        calc_rpm = STEPPER_START_RPM + (g_stepper.target_rpm - STEPPER_START_RPM) * g_stepper.step_count / STEPPER_ACCEL_PULSES;
                    } 
                    else if (g_stepper.pulses_remaining <= STEPPER_ACCEL_PULSES) {
                        /* 3. 减速段：从目标速度插值衰减回最小转速 */
                        calc_rpm = STEPPER_START_RPM + (g_stepper.target_rpm - STEPPER_START_RPM) * g_stepper.pulses_remaining / STEPPER_ACCEL_PULSES;
                    }
                    /* 2. 匀速段：啥也不做，维持 calc_rpm = target_rpm */
                } 
                else {
                    /* 所设脉冲数太短不足以跑满加减速过程，只能退化为尖顶三角形行驶曲线 */
                    uint32_t half = g_stepper.total_pulses / 2;
                    if (half == 0) half = 1;
                    if (g_stepper.step_count < half) {
                        calc_rpm = STEPPER_START_RPM + (g_stepper.target_rpm - STEPPER_START_RPM) * g_stepper.step_count / half;
                    } else {
                        calc_rpm = STEPPER_START_RPM + (g_stepper.target_rpm - STEPPER_START_RPM) * g_stepper.pulses_remaining / half;
                    }
                }
                
                /* 为了优化 CPU 的运算及打断资源，只有侦测到速度变更才去查表(或重算)定时器装载值 */
                if (calc_rpm != g_stepper.current_rpm) {
                    g_stepper.current_rpm = calc_rpm;
                    Stepper_UpdateARR(calc_rpm);
                }
                /* ========================================================= */
            }
        } 
        else 
        {
            Stepper_StopInternal(STEPPER_STOP_COMPLETED);
        }
    }
}

/* 拦截 HAL 的定时器溢出回调，这里是由于不需要再修改 main 或者 stm32h_it.c 因此做的隐式封装 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        Stepper_TIM_IRQHandler(); 
    }
}

/* 在 StartServoTask 中被挂载的测试本体 */
void Stepper_TestTask(void *argument)
{
    (void)argument;

    Stepper_Init();

    /* 等待接收机和驱动稳定 */
    osDelay(1000);

    for (;;)
    {
        Stepper_ServiceLimits(HAL_GetTick());
        if (SbusDriver_IsFrameFresh(osKernelGetTickCount(), STEPPER_TEST_FRAME_TIMEOUT_MS) == 0U ||
            g_sbus_readout.failsafe != 0U)
        {
            g_stepper_test_cmd_rpm = 0;
            Stepper_SetContinuousSignedRpm(0);
            osDelay(STEPPER_TEST_POLL_MS);
            continue;
        }

        g_stepper_test_raw_left_x = g_sbus_readout.controls.left_stick_x;
        g_stepper_test_cmd_rpm = Stepper_MapLeftStickToSignedRpm(g_stepper_test_raw_left_x);

        Stepper_SetContinuousSignedRpm(g_stepper_test_cmd_rpm);
        osDelay(STEPPER_TEST_POLL_MS);
    }
}
