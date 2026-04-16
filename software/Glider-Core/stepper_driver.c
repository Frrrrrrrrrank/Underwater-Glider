#include "stepper_driver.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

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
    
    /* 停顿两秒等待驱动器上电准备好 */
    osDelay(2000);
    
    for (;;)
    {
        /* 1600细分下，发 16000 脉冲就是 10 圈 */
        /* 加了平滑加减速后，我们可以安全地飙到 600 RPM 甚至更快，体验如丝般顺滑 */
        Stepper_MoveRelative(16000, 800);
        
        while (Stepper_IsRunning()) 
        {
            osDelay(50);
        }
        
        /* 走到目标后停顿 2 秒 */
        osDelay(2000);
        
        /* 反转：返回0位置(回到刚才起点), 速度同样 600RPM */
        Stepper_MoveAbsolute(0, 800);
        
        while (Stepper_IsRunning()) 
        {
            osDelay(50);
        }
        
        /* 停顿 2 秒，接着再正转，如此往复... */
        osDelay(2000);
    }
}