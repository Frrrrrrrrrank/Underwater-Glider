/*
*********************************************************************************************************
*
*	ģ������ : ��ʱ��ģ��
*	�ļ����� : bsp_timer.c
*	��    �� : V1.5
*	˵    �� : ����systick��ʱ����Ϊϵͳ�δ�ʱ����ȱʡ��ʱ����Ϊ1ms��
*
*				ʵ���˶��������ʱ����������ʹ��(����1ms)�� ����ͨ���޸� TMR_COUNT ������ʱ������
*				ʵ����ms�����ӳٺ���������1ms�� ��us���ӳٺ���
*				ʵ����ϵͳ����ʱ�亯����1ms��λ��
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2013-02-01 armfly  ��ʽ����
*		V1.1    2013-06-21 armfly  ����us���ӳٺ��� bsp_DelayUS
*		V1.2    2014-09-07 armfly  ����TIM4 Ӳ����ʱ�жϣ�ʵ��us����ʱ.20us - 16��
*		V1.3    2015-04-06 armfly  ���� bsp_CheckRunTime(int32_t _LastTime) ��������ʱ���ֵ
*		V1.4	2015-05-22 armfly  ���� bsp_InitHardTimer() ��������������ѡ��TIM2-5
*		V1.5	2018-11-26 armfly  s_tTmr����ֵ0; ����g_ucEnableSystickISR��������HAL��ǰ��systick�ж�
*								   ������쳣��
*
*	Copyright (C), 2015-2030, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include "bsp.h"
#include "bsp_timer.h"

/*
	��������Ӳ����ʱ����TIM�� ����ʹ TIM2 - TIM5
*/
#define USE_TIM2
//#define USE_TIM3
//#define USE_TIM4
//#define USE_TIM5

#ifdef USE_TIM2
	#define TIM_HARD					TIM2
	#define	RCC_TIM_HARD_CLK_ENABLE()	__HAL_RCC_TIM2_CLK_ENABLE()
	#define TIM_HARD_IRQn				TIM2_IRQn
	#define TIM_HARD_IRQHandler			TIM2_IRQHandler
#endif

#ifdef USE_TIM3
	#define TIM_HARD					TIM3
	#define	RCC_TIM_HARD_CLK_ENABLE()	__HAL_RCC_TIM3_CLK_ENABLE()	
	#define TIM_HARD_IRQn				TIM3_IRQn
	#define TIM_HARD_IRQHandler			TIM3_IRQHandler
#endif

#ifdef USE_TIM4
	#define TIM_HARD					TIM4
	#define	RCC_TIM_HARD_CLK_ENABLE()	__HAL_RCC_TIM4_CLK_ENABLE()
	#define TIM_HARD_IRQn				TIM4_IRQn
	#define TIM_HARD_IRQHandler			TIM4_IRQHandler
#endif

#ifdef USE_TIM5
	#define TIM_HARD					TIM5
	#define	RCC_TIM_HARD_CLK_ENABLE()	__HAL_RCC_TIM5_CLK_ENABLE()
	#define TIM_HARD_IRQn				TIM5_IRQn
	#define TIM_HARD_IRQHandler			TIM5_IRQHandler
#endif

/* ���� TIM��ʱ�жϵ���ִ�еĻص�����ָ�� */
static void (*s_TIM_CallBack1)(void);
static void (*s_TIM_CallBack2)(void);
static void (*s_TIM_CallBack3)(void);
static void (*s_TIM_CallBack4)(void);


/* ��2��ȫ�ֱ���ת���� bsp_DelayMS() ���� */
static volatile uint32_t s_uiDelayCount = 0;
static volatile uint8_t s_ucTimeOutFlag = 0;

/* ����������ʱ���ṹ����� */
static SOFT_TMR s_tTmr[TMR_COUNT] = {0};

/*
	ȫ������ʱ�䣬��λ1ms
	����Ա�ʾ 24.85�죬�����Ĳ�Ʒ��������ʱ�䳬�������������뿼���������
*/
__IO int32_t g_iRunTime = 0;

static __IO uint8_t g_ucEnableSystickISR = 0;	/* �ȴ�������ʼ�� */

static void bsp_SoftTimerDec(SOFT_TMR *_tmr);


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitTimer
*	����˵��: ����systick�жϣ�����ʼ��������ʱ������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitTimer(void)
{
	uint8_t i;

	/* �������е�������ʱ�� */
	for (i = 0; i < TMR_COUNT; i++)
	{
		s_tTmr[i].Count = 0;
		s_tTmr[i].PreLoad = 0;
		s_tTmr[i].Flag = 0;
		s_tTmr[i].Mode = TMR_ONCE_MODE;	/* ȱʡ��1���Թ���ģʽ */
	}

	/*
		����systic�ж�����Ϊ1ms��������systick�жϡ�

    	SystemCoreClock �ǹ̼��ж����ϵͳ�ں�ʱ�ӣ�����STM32H7,һ��Ϊ400MHz

    	SysTick_Config() �������βα�ʾ�ں�ʱ�Ӷ��ٸ����ں󴥷�һ��Systick��ʱ�ж�.
	    	-- SystemCoreClock / 1000  ��ʾ��ʱƵ��Ϊ 1000Hz�� Ҳ���Ƕ�ʱ����Ϊ  1ms
	    	-- SystemCoreClock / 500   ��ʾ��ʱƵ��Ϊ 500Hz��  Ҳ���Ƕ�ʱ����Ϊ  2ms
	    	-- SystemCoreClock / 2000  ��ʾ��ʱƵ��Ϊ 2000Hz�� Ҳ���Ƕ�ʱ����Ϊ  500us

    	���ڳ����Ӧ�ã�����һ��ȡ��ʱ����1ms�����ڵ���CPU���ߵ͹���Ӧ�ã��������ö�ʱ����Ϊ 10ms
    */
	SysTick_Config(SystemCoreClock / 1000);
	
	g_ucEnableSystickISR = 1;		/* 1��ʾִ��systick�ж� */
	
	bsp_InitHardTimer();
}

/*
*********************************************************************************************************
*	�� �� ��: SysTick_ISR
*	����˵��: SysTick�жϷ������ÿ��1ms����1��
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
extern void bsp_RunPer1ms(void);
extern void bsp_RunPer10ms(void);
void SysTick_ISR(void)
{
	static uint8_t s_count = 0;
	uint8_t i;
	
	/* ÿ��1ms����1�� �������� bsp_DelayMS�� */
	if (s_uiDelayCount > 0)
	{
		if (--s_uiDelayCount == 0)
		{
			s_ucTimeOutFlag = 1;
		}
	}

	/* ÿ��1ms����������ʱ���ļ��������м�һ���� */
	for (i = 0; i < TMR_COUNT; i++)
	{
		bsp_SoftTimerDec(&s_tTmr[i]);
	}

	/* ȫ������ʱ��ÿ1ms��1 */
	g_iRunTime++;
	if (g_iRunTime == 0x7FFFFFFF)	/* ��������� int32_t ���ͣ������Ϊ 0x7FFFFFFF */
	{
		g_iRunTime = 0;
	}

	bsp_RunPer1ms();		/* ÿ��1ms����һ�δ˺������˺����� bsp.c */

	if (++s_count >= 10)
	{
		s_count = 0;

		bsp_RunPer10ms();	/* ÿ��10ms����һ�δ˺������˺����� bsp.c */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SoftTimerDec
*	����˵��: ÿ��1ms�����ж�ʱ��������1�����뱻SysTick_ISR�����Ե��á�
*	��    ��:  _tmr : ��ʱ������ָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SoftTimerDec(SOFT_TMR *_tmr)
{
	if (_tmr->Count > 0)
	{
		/* �����ʱ����������1�����ö�ʱ�������־ */
		if (--_tmr->Count == 0)
		{
			_tmr->Flag = 1;

			/* ������Զ�ģʽ�����Զ���װ������ */
			if(_tmr->Mode == TMR_AUTO_MODE)
			{
				_tmr->Count = _tmr->PreLoad;
			}
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_DelayMS
*	����˵��: ms���ӳ٣��ӳپ���Ϊ����1ms
*	��    ��:  n : �ӳٳ��ȣ���λ1 ms�� n Ӧ����2
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_DelayMS(uint32_t n)
{
	if (n == 0)
	{
		return;
	}
	else if (n == 1)
	{
		n = 2;
	}

	DISABLE_INT();  			/* ���ж� */

	s_uiDelayCount = n;
	s_ucTimeOutFlag = 0;

	ENABLE_INT();  				/* ���ж� */

	while (1)
	{
		bsp_Idle();				/* CPU����ִ�еĲ����� �� bsp.c �� bsp.h �ļ� */

		/*
			�ȴ��ӳ�ʱ�䵽
			ע�⣺��������Ϊ s_ucTimeOutFlag = 0�����Կ����Ż�������� s_ucTimeOutFlag ������������Ϊ volatile
		*/
		if (s_ucTimeOutFlag == 1)
		{
			break;
		}
	}
}

/*
*********************************************************************************************************
*    �� �� ��: bsp_DelayUS
*    ����˵��: us���ӳ١� ������systick��ʱ����������ܵ��ô˺�����
*    ��    ��:  n : �ӳٳ��ȣ���λ1 us
*    �� �� ֵ: ��
*********************************************************************************************************
*/
__weak void bsp_DelayUS(uint32_t n)
{
    uint32_t ticks;
    uint32_t told;
    uint32_t tnow;
    uint32_t tcnt = 0;
    uint32_t reload;
       
	reload = SysTick->LOAD;                
    ticks = n * (SystemCoreClock / 1000000);	 /* ��Ҫ�Ľ����� */  
    
    tcnt = 0;
    told = SysTick->VAL;             /* �ս���ʱ�ļ�����ֵ */

    while (1)
    {
        tnow = SysTick->VAL;    
        if (tnow != told)
        {    
            /* SYSTICK��һ���ݼ��ļ����� */    
            if (tnow < told)
            {
                tcnt += told - tnow;    
            }
            /* ����װ�صݼ� */
            else
            {
                tcnt += reload - tnow + told;    
            }        
            told = tnow;

            /* ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳� */
            if (tcnt >= ticks)
            {
            	break;
            }
        }  
    }
} 

/*
*********************************************************************************************************
*	�� �� ��: bsp_StartTimer
*	����˵��: ����һ����ʱ���������ö�ʱ���ڡ�
*	��    ��:  	_id     : ��ʱ��ID��ֵ��0,TMR_COUNT-1�����û���������ά����ʱ��ID���Ա��ⶨʱ��ID��ͻ��
*				_period : ��ʱ���ڣ���λ1ms
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StartTimer(uint8_t _id, uint32_t _period)
{
	if (_id >= TMR_COUNT)
	{
		/* ��ӡ������Դ�����ļ������������� */
		printf("Error: file %s, function %s()\r\n", __FILE__, __FUNCTION__);
		while(1); /* �����쳣�������ȴ����Ź���λ */
	}

	DISABLE_INT();  			/* ���ж� */

	s_tTmr[_id].Count = _period;		/* ʵʱ��������ֵ */
	s_tTmr[_id].PreLoad = _period;		/* �������Զ���װֵ�����Զ�ģʽ������ */
	s_tTmr[_id].Flag = 0;				/* ��ʱʱ�䵽��־ */
	s_tTmr[_id].Mode = TMR_ONCE_MODE;	/* 1���Թ���ģʽ */

	ENABLE_INT();  				/* ���ж� */
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_StartAutoTimer
*	����˵��: ����һ���Զ���ʱ���������ö�ʱ���ڡ�
*	��    ��:  	_id     : ��ʱ��ID��ֵ��0,TMR_COUNT-1�����û���������ά����ʱ��ID���Ա��ⶨʱ��ID��ͻ��
*				_period : ��ʱ���ڣ���λ10ms
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StartAutoTimer(uint8_t _id, uint32_t _period)
{
	if (_id >= TMR_COUNT)
	{
		/* ��ӡ������Դ�����ļ������������� */
		printf("Error: file %s, function %s()\r\n", __FILE__, __FUNCTION__);
		while(1); /* �����쳣�������ȴ����Ź���λ */
	}

	DISABLE_INT();  		/* ���ж� */

	s_tTmr[_id].Count = _period;			/* ʵʱ��������ֵ */
	s_tTmr[_id].PreLoad = _period;		/* �������Զ���װֵ�����Զ�ģʽ������ */
	s_tTmr[_id].Flag = 0;				/* ��ʱʱ�䵽��־ */
	s_tTmr[_id].Mode = TMR_AUTO_MODE;	/* �Զ�����ģʽ */

	ENABLE_INT();  			/* ���ж� */
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_StopTimer
*	����˵��: ֹͣһ����ʱ��
*	��    ��:  	_id     : ��ʱ��ID��ֵ��0,TMR_COUNT-1�����û���������ά����ʱ��ID���Ա��ⶨʱ��ID��ͻ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StopTimer(uint8_t _id)
{
	if (_id >= TMR_COUNT)
	{
		/* ��ӡ������Դ�����ļ������������� */
		printf("Error: file %s, function %s()\r\n", __FILE__, __FUNCTION__);
		while(1); /* �����쳣�������ȴ����Ź���λ */
	}

	DISABLE_INT();  	/* ���ж� */

	s_tTmr[_id].Count = 0;				/* ʵʱ��������ֵ */
	s_tTmr[_id].Flag = 0;				/* ��ʱʱ�䵽��־ */
	s_tTmr[_id].Mode = TMR_ONCE_MODE;	/* �Զ�����ģʽ */

	ENABLE_INT();  		/* ���ж� */
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_CheckTimer
*	����˵��: ��ⶨʱ���Ƿ�ʱ
*	��    ��:  	_id     : ��ʱ��ID��ֵ��0,TMR_COUNT-1�����û���������ά����ʱ��ID���Ա��ⶨʱ��ID��ͻ��
*				_period : ��ʱ���ڣ���λ1ms
*	�� �� ֵ: ���� 0 ��ʾ��ʱδ���� 1��ʾ��ʱ��
*********************************************************************************************************
*/
uint8_t bsp_CheckTimer(uint8_t _id)
{
	if (_id >= TMR_COUNT)
	{
		return 0;
	}

	if (s_tTmr[_id].Flag == 1)
	{
		s_tTmr[_id].Flag = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetRunTime
*	����˵��: ��ȡCPU����ʱ�䣬��λ1ms������Ա�ʾ 24.85�죬�����Ĳ�Ʒ��������ʱ�䳬�������������뿼���������
*	��    ��:  ��
*	�� �� ֵ: CPU����ʱ�䣬��λ1ms
*********************************************************************************************************
*/
int32_t bsp_GetRunTime(void)
{
	int32_t runtime;

	DISABLE_INT();  	/* ���ж� */

	runtime = g_iRunTime;	/* ���������Systick�ж��б���д�������Ҫ���жϽ��б��� */

	ENABLE_INT();  		/* ���ж� */

	return runtime;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_CheckRunTime
*	����˵��: ���㵱ǰ����ʱ��͸���ʱ��֮��Ĳ�ֵ�������˼�����ѭ����
*	��    ��:  _LastTime �ϸ�ʱ��
*	�� �� ֵ: ��ǰʱ��͹�ȥʱ��Ĳ�ֵ����λ1ms
*********************************************************************************************************
*/
int32_t bsp_CheckRunTime(int32_t _LastTime)
{
	int32_t now_time;
	int32_t time_diff;

	DISABLE_INT();  	/* ���ж� */

	now_time = g_iRunTime;	/* ���������Systick�ж��б���д�������Ҫ���жϽ��б��� */

	ENABLE_INT();  		/* ���ж� */
	
	if (now_time >= _LastTime)
	{
		time_diff = now_time - _LastTime;
	}
	else
	{
		time_diff = 0x7FFFFFFF - _LastTime + now_time;
	}

	return time_diff;
}

/*
*********************************************************************************************************
*	�� �� ��: SysTick_Handler
*	����˵��: ϵͳ��શ�ʱ���жϷ�����������ļ��������˸ú�����
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
__weak void SysTick_Handler(void)
{
	HAL_IncTick();	/* ST HAL��ĵδ�ʱ�жϷ������ */
	
	if (g_ucEnableSystickISR == 0)
	{
		return;
	}
	
	SysTick_ISR();	/* ������bsp��ĵδ�ʱ�жϷ������ */
}

/*
*********************************************************************************************************
*	����Ĵ���ʹ��һ��TIM��4�������жϣ�ʵ��4��Ӳ����ʱ��
*********************************************************************************************************
*/

#ifdef TIM_HARD

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitHardTimer
*	����˵��: ���� TIMx������us����Ӳ����ʱ��TIMx���������У�����ֹͣ.
*			TIMx������TIM2 - TIM5 ֮���TIM, ��ЩTIM��4��ͨ��, ���� APB1 �ϣ�����ʱ��=SystemCoreClock / 2
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitHardTimer(void)
{
    TIM_HandleTypeDef  TimHandle = {0};
	uint32_t usPeriod;
	uint16_t usPrescaler;
	uint32_t uiTIMxCLK;
	TIM_TypeDef* TIMx = TIM_HARD;
	
	RCC_TIM_HARD_CLK_ENABLE();		/* ʹ��TIMʱ�� */
	
    /*-----------------------------------------------------------------------
		bsp.c �ļ��� void SystemClock_Config(void) ������ʱ�ӵ���������: 

        System Clock source       = PLL (HSE)
        SYSCLK(Hz)                = 400000000 (CPU Clock)
        HCLK(Hz)                  = 200000000 (AXI and AHBs Clock)
        AHB Prescaler             = 2
        D1 APB3 Prescaler         = 2 (APB3 Clock  100MHz)
        D2 APB1 Prescaler         = 2 (APB1 Clock  100MHz)
        D2 APB2 Prescaler         = 2 (APB2 Clock  100MHz)
        D3 APB4 Prescaler         = 2 (APB4 Clock  100MHz)

        ��ΪAPB1 prescaler != 1, ���� APB1�ϵ�TIMxCLK = APB1 x 2 = 200MHz;
        ��ΪAPB2 prescaler != 1, ���� APB2�ϵ�TIMxCLK = APB2 x 2 = 200MHz;
        APB4�����TIMxCLKû�з�Ƶ�����Ծ���100MHz;

        APB1 ��ʱ���� TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM7, TIM12, TIM13, TIM14��LPTIM1
        APB2 ��ʱ���� TIM1, TIM8 , TIM15, TIM16��TIM17

        APB4 ��ʱ���� LPTIM2��LPTIM3��LPTIM4��LPTIM5

	----------------------------------------------------------------------- */
	if ((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM15) || (TIMx == TIM16) || (TIMx == TIM17))
	{
		/* APB2 ��ʱ��ʱ�� = 200M */
		uiTIMxCLK = SystemCoreClock / 2;
	}
	else	
	{
		/* APB1 ��ʱ�� = 200M */
		uiTIMxCLK = SystemCoreClock / 2;
	}

	usPrescaler = uiTIMxCLK / 1000000 - 1;	/* ��Ƶ�� = 1 */
	
	if (TIMx == TIM2 || TIMx == TIM5)
	{
		usPeriod = 0xFFFFFFFF;
	}
	else
	{
		usPeriod = 0xFFFF;
	}

	/* 
       ���÷�ƵΪusPrescaler����ô��ʱ����������1�ξ���1us
       ������usPeriod��ֵ�Ǿ�������������
       usPeriod = 0xFFFF ��ʾ���0xFFFF΢�롣
       usPeriod = 0xFFFFFFFF ��ʾ���0xFFFFFFFF΢�롣
    */
	TimHandle.Instance = TIMx;
	TimHandle.Init.Prescaler         = usPrescaler;
	TimHandle.Init.Period            = usPeriod;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;
    TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	
	if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}

	/* ���ö�ʱ���жϣ���CC����Ƚ��ж�ʹ�� */
	{
		HAL_NVIC_SetPriority(TIM_HARD_IRQn, 0, 2);
		HAL_NVIC_EnableIRQ(TIM_HARD_IRQn);	
	}
    
    /* ������ʱ�� */
	HAL_TIM_Base_Start(&TimHandle);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_StartHardTimer
*	����˵��: ʹ��TIM2-5�����ζ�ʱ��ʹ��, ��ʱʱ�䵽��ִ�лص�����������ͬʱ����4����ʱ��ͨ�����������š�
*             ��ʱ��������1us ����Ҫ�ķ��ڵ��ñ�������ִ��ʱ�䣩
*			  TIM2��TIM5 ��32λ��ʱ������ʱ��Χ�ܴ�
*			  TIM3��TIM4 ��16λ��ʱ����
*	��    ��: _CC : ����Ƚ�ͨ������1��2��3, 4
*             _uiTimeOut : ��ʱʱ��, ��λ 1us. ����16λ��ʱ������� 65.5ms; ����32λ��ʱ������� 4294��
*             _pCallBack : ��ʱʱ�䵽�󣬱�ִ�еĺ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StartHardTimer(uint8_t _CC, uint32_t _uiTimeOut, void * _pCallBack)
{
    uint32_t cnt_now;
    uint32_t cnt_tar;
	TIM_TypeDef* TIMx = TIM_HARD;
	
    /* H743�ٶȽϿ죬���貹���ӳ٣�ʵ�⾫������1us */
    
    cnt_now = TIMx->CNT; 
    cnt_tar = cnt_now + _uiTimeOut;			/* ���㲶��ļ�����ֵ */
    if (_CC == 1)
    {
        s_TIM_CallBack1 = (void (*)(void))_pCallBack;

		TIMx->CCR1 = cnt_tar; 			    /* ���ò���Ƚϼ�����CC1 */
        TIMx->SR = (uint16_t)~TIM_IT_CC1;   /* ���CC1�жϱ�־ */
		TIMx->DIER |= TIM_IT_CC1;			/* ʹ��CC1�ж� */
	}
    else if (_CC == 2)
    {
		s_TIM_CallBack2 = (void (*)(void))_pCallBack;

		TIMx->CCR2 = cnt_tar;				/* ���ò���Ƚϼ�����CC2 */
        TIMx->SR = (uint16_t)~TIM_IT_CC2;	/* ���CC2�жϱ�־ */
		TIMx->DIER |= TIM_IT_CC2;			/* ʹ��CC2�ж� */
    }
    else if (_CC == 3)
    {
        s_TIM_CallBack3 = (void (*)(void))_pCallBack;

		TIMx->CCR3 = cnt_tar;				/* ���ò���Ƚϼ�����CC3 */
        TIMx->SR = (uint16_t)~TIM_IT_CC3;	/* ���CC3�жϱ�־ */
		TIMx->DIER |= TIM_IT_CC3;			/* ʹ��CC3�ж� */
    }
    else if (_CC == 4)
    {
        s_TIM_CallBack4 = (void (*)(void))_pCallBack;

		TIMx->CCR4 = cnt_tar;				/* ���ò���Ƚϼ�����CC4 */
        TIMx->SR = (uint16_t)~TIM_IT_CC4;	/* ���CC4�жϱ�־ */
		TIMx->DIER |= TIM_IT_CC4;			/* ʹ��CC4�ж� */
    }
	else
    {
        return;
    }
}

/*
*********************************************************************************************************
*	�� �� ��: TIMx_IRQHandler
*	����˵��: TIM �жϷ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void TIM_HARD_IRQHandler(void)
{
	uint16_t itstatus = 0x0, itenable = 0x0;
	TIM_TypeDef* TIMx = TIM_HARD;
	
    
  	itstatus = TIMx->SR & TIM_IT_CC1;
	itenable = TIMx->DIER & TIM_IT_CC1;
    
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~TIM_IT_CC1;
		TIMx->DIER &= (uint16_t)~TIM_IT_CC1;		/* ����CC1�ж� */	

        /* �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� */
        s_TIM_CallBack1();
    }

	itstatus = TIMx->SR & TIM_IT_CC2;
	itenable = TIMx->DIER & TIM_IT_CC2;
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~TIM_IT_CC2;
		TIMx->DIER &= (uint16_t)~TIM_IT_CC2;		/* ����CC2�ж� */	

        /* �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� */
        s_TIM_CallBack2();
    }

	itstatus = TIMx->SR & TIM_IT_CC3;
	itenable = TIMx->DIER & TIM_IT_CC3;
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~TIM_IT_CC3;
		TIMx->DIER &= (uint16_t)~TIM_IT_CC3;		/* ����CC2�ж� */	

        /* �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� */
        s_TIM_CallBack3();
    }

	itstatus = TIMx->SR & TIM_IT_CC4;
	itenable = TIMx->DIER & TIM_IT_CC4;
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~TIM_IT_CC4;
		TIMx->DIER &= (uint16_t)~TIM_IT_CC4;		/* ����CC4�ж� */	

        /* �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� */
        s_TIM_CallBack4();
    }	
}

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
