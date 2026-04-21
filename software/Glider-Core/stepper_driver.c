#include "stepper_driver.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "sbus_driver.h"

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
} StepperCtrl_t;

static StepperCtrl_t g_stepper = {0};

/* Debug watch helpers for SBUS-controlled test */
static volatile uint16_t g_stepper_test_raw_left_y = SBUS_VALUE_CENTER_OBSERVED;
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

static void Stepper_ApplySignedRpmCommand(int32_t signed_rpm)
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

    if (desired_dir == 0)
    {
        if (Stepper_IsRunning() != 0U)
        {
            Stepper_Stop();
        }
        g_stepper.target_rpm = 0U;
        return;
    }

    if (abs_rpm < STEPPER_START_RPM)
    {
        abs_rpm = STEPPER_START_RPM;
    }
    if (abs_rpm > STEPPER_TEST_MAX_RPM)
    {
        abs_rpm = STEPPER_TEST_MAX_RPM;
    }

    if (g_stepper.is_running == 0U)
    {
        Stepper_MoveRelative((desired_dir > 0) ? STEPPER_TEST_CONTINUOUS_PULSES : -STEPPER_TEST_CONTINUOUS_PULSES, abs_rpm);
        return;
    }

    if (g_stepper.direction != desired_dir)
    {
        Stepper_Stop();
        Stepper_MoveRelative((desired_dir > 0) ? STEPPER_TEST_CONTINUOUS_PULSES : -STEPPER_TEST_CONTINUOUS_PULSES, abs_rpm);
        return;
    }

    g_stepper.target_rpm = abs_rpm;
}

void Stepper_Init(void) 
{
    g_stepper.current_pos = 0;
    g_stepper.target_pos = 0;
    g_stepper.pulses_remaining = 0;
    g_stepper.is_running = 0;

    /* 初始化负极端恒定为低电平，模拟 GND */
    HAL_GPIO_WritePin(STEPPER_PUL_N_PORT, STEPPER_PUL_N_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STEPPER_DIR_N_PORT, STEPPER_DIR_N_PIN, GPIO_PIN_RESET);

    __HAL_TIM_DISABLE_IT(STEPPER_TIM, TIM_IT_UPDATE);
    HAL_TIM_PWM_Stop(STEPPER_TIM, STEPPER_TIM_CH);
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

void Stepper_MoveRelative(int32_t pulses, uint16_t rpm) 
{
    if (pulses == 0 || rpm == 0) return;
    
    Stepper_Stop(); // 停止当前正在进行的其他任务
    
    if (pulses > 0) 
    {
        g_stepper.direction = 1;
        HAL_GPIO_WritePin(STEPPER_DIR_PORT, STEPPER_DIR_PIN, GPIO_PIN_SET);
        g_stepper.total_pulses = (uint32_t)pulses;
    } 
    else 
    {
        g_stepper.direction = -1;
        HAL_GPIO_WritePin(STEPPER_DIR_PORT, STEPPER_DIR_PIN, GPIO_PIN_RESET);
        g_stepper.total_pulses = (uint32_t)(-pulses);
    }
    
    g_stepper.pulses_remaining = g_stepper.total_pulses;
    g_stepper.step_count = 0;
    
    /* 速度逻辑与基础限制起跑值 */
    g_stepper.target_rpm = rpm;
    if (g_stepper.target_rpm < STEPPER_START_RPM) g_stepper.target_rpm = STEPPER_START_RPM;
    g_stepper.current_rpm = STEPPER_START_RPM;

    /* 忠于手册 3.3.4 时序图：DR至少提前PU下降沿50μs确定状态。
     * 此处加一个简易的软件延时(远大于50us)，确保方向信号被驱动器光耦稳定锁别后再出脉冲！ */
    for (volatile uint32_t delay_cnt = 0; delay_cnt < 20000; delay_cnt++) {
        __NOP();
    }
    
    g_stepper.target_pos = g_stepper.current_pos + pulses;
    g_stepper.is_running = 1;

    Stepper_UpdateARR(g_stepper.current_rpm);
    
    __HAL_TIM_CLEAR_FLAG(STEPPER_TIM, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(STEPPER_TIM, TIM_IT_UPDATE);
    HAL_TIM_PWM_Start(STEPPER_TIM, STEPPER_TIM_CH);
}

void Stepper_MoveAbsolute(int32_t target_pos, uint16_t rpm) 
{
    int32_t relative_pulses = target_pos - g_stepper.current_pos;
    Stepper_MoveRelative(relative_pulses, rpm);
}

void Stepper_Stop(void) 
{
    __HAL_TIM_DISABLE_IT(STEPPER_TIM, TIM_IT_UPDATE);
    HAL_TIM_PWM_Stop(STEPPER_TIM, STEPPER_TIM_CH);
    HAL_TIM_Base_Stop(STEPPER_TIM);
    __HAL_TIM_SET_COUNTER(STEPPER_TIM, 0); // 清零计数值，防止下一次启动存在死区脉冲或毛刺
    g_stepper.is_running = 0;
    g_stepper.pulses_remaining = 0;
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
        if (g_stepper.pulses_remaining > 0U) 
        {
            g_stepper.step_count++;
            g_stepper.pulses_remaining--;
            g_stepper.current_pos += g_stepper.direction;
            
            if (g_stepper.pulses_remaining == 0U) 
            {
                Stepper_Stop();
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
            Stepper_Stop();
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
        if (SbusDriver_IsFrameFresh(osKernelGetTickCount(), STEPPER_TEST_FRAME_TIMEOUT_MS) == 0U ||
            g_sbus_readout.failsafe != 0U)
        {
            g_stepper_test_cmd_rpm = 0;
            Stepper_ApplySignedRpmCommand(0);
            osDelay(STEPPER_TEST_POLL_MS);
            continue;
        }

        g_stepper_test_raw_left_y = g_sbus_readout.controls.left_stick_y;
        g_stepper_test_cmd_rpm = Stepper_MapLeftStickToSignedRpm(g_stepper_test_raw_left_y);

        Stepper_ApplySignedRpmCommand(g_stepper_test_cmd_rpm);
        osDelay(STEPPER_TEST_POLL_MS);
    }
}