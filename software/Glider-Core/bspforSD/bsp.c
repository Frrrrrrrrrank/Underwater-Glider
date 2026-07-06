/*
*********************************************************************************************************
*
*	Ä£¿éÃû³Æ : BSPÄ£¿é(For STM32H7)
*	ÎÄ¼þÃû³Æ : bsp.c
*	°æ    ±¾ : V1.0
*	Ëµ    Ã÷ : ÕâÊÇÓ²¼þµ×²ãÇý¶¯³ÌÐòµÄÖ÷ÎÄ¼þ¡£Ã¿¸öcÎÄ¼þ¿ÉÒÔ #include "bsp.h" À´°üº¬ËùÓÐµÄÍâÉèÇý¶¯Ä£¿é¡£
*			   bsp = Borad surport packet °å¼¶Ö§³Ö°ü
*	ÐÞ¸Ä¼ÇÂ¼ :
*		°æ±¾ºÅ  ÈÕÆÚ         ×÷Õß       ËµÃ÷
*		V1.0    2018-07-29  Eric2013   ÕýÊ½·¢²¼
*
*	Copyright (C), 2018-2030, °²¸»À³µç×Ó www.armfly.com
*
*********************************************************************************************************
*/
#include "bsp.h"



/*
*********************************************************************************************************
*	                                   º¯ÊýÉùÃ÷
*********************************************************************************************************
*/
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void MPU_Config(void);


/*
*********************************************************************************************************
*	º¯ Êý Ãû: bsp_Init
*	¹¦ÄÜËµÃ÷: ³õÊ¼»¯ËùÓÐµÄÓ²¼þÉè±¸¡£¸Ãº¯ÊýÅäÖÃCPU¼Ä´æÆ÷ºÍÍâÉèµÄ¼Ä´æÆ÷²¢³õÊ¼»¯Ò»Ð©È«¾Ö±äÁ¿¡£Ö»ÐèÒªµ÷ÓÃÒ»´Î
*	ÐÎ    ²Î£ºÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void bsp_Init(void)
{
    /* ÅäÖÃMPU */
	MPU_Config();
	
	/* Ê¹ÄÜL1 Cache */
	CPU_CACHE_Enable();

	/* 
       STM32H7xx HAL ¿â³õÊ¼»¯£¬´ËÊ±ÏµÍ³ÓÃµÄ»¹ÊÇH7×Ô´øµÄ64MHz£¬HSIÊ±ÖÓ:
	   - µ÷ÓÃº¯ÊýHAL_InitTick£¬³õÊ¼»¯µÎ´ðÊ±ÖÓÖÐ¶Ï1ms¡£
	   - ÉèÖÃNVIVÓÅÏÈ¼¶·Ö×éÎª4¡£
	 */
	HAL_Init();

	/* 
       ÅäÖÃÏµÍ³Ê±ÖÓµ½400MHz
       - ÇÐ»»Ê¹ÓÃHSE¡£
       - ´Ëº¯Êý»á¸üÐÂÈ«¾Ö±äÁ¿SystemCoreClock£¬²¢ÖØÐÂÅäÖÃHAL_InitTick¡£
    */
	SystemClock_Config();

	/* 
	   Event Recorder£º
	   - ¿ÉÓÃÓÚ´úÂëÖ´ÐÐÊ±¼ä²âÁ¿£¬MDK5.25¼°ÆäÒÔÉÏ°æ±¾²ÅÖ§³Ö£¬IAR²»Ö§³Ö¡£
	   - Ä¬ÈÏ²»¿ªÆô£¬Èç¹ûÒªÊ¹ÄÜ´ËÑ¡Ïî£¬Îñ±Ø¿´V7¿ª·¢°åÓÃ»§ÊÖ²áµÚ8ÕÂ
	*/	
//#if Enable_EventRecorder == 1  
//	/* ³õÊ¼»¯EventRecorder²¢¿ªÆô */
//	EventRecorderInitialize(EventRecordAll, 1U);
//	EventRecorderStart();
//#endif
	
//	bsp_InitKey();    	/* °´¼ü³õÊ¼»¯£¬Òª·ÅÔÚµÎ´ð¶¨Ê±Æ÷Ö®Ç°£¬ÒòÎª°´Å¥¼ì²âÊÇÍ¨¹ýµÎ´ð¶¨Ê±Æ÷É¨Ãè */
	bsp_InitTimer();  	/* ³õÊ¼»¯µÎ´ð¶¨Ê±Æ÷ */
//	bsp_InitUart();		/* ³õÊ¼»¯´®¿Ú */
//	bsp_InitExtIO();	/* ³õÊ¼»¯FMC×ÜÏß74HC574À©Õ¹IO. ±ØÐëÔÚ bsp_InitLed()Ç°Ö´ÐÐ */	
//	bsp_InitLed();    	/* ³õÊ¼»¯LED */	
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: SystemClock_Config
*	¹¦ÄÜËµÃ÷: ³õÊ¼»¯ÏµÍ³Ê±ÖÓ
*            	System Clock source            = PLL (HSE)
*            	SYSCLK(Hz)                     = 400000000 (CPU Clock)
*           	HCLK(Hz)                       = 200000000 (AXI and AHBs Clock)
*            	AHB Prescaler                  = 2
*            	D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
*            	D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
*            	D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
*            	D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
*            	HSE Frequency(Hz)              = 25000000
*           	PLL_M                          = 5
*            	PLL_N                          = 160
*            	PLL_P                          = 2
*            	PLL_Q                          = 4
*            	PLL_R                          = 2
*            	VDD(V)                         = 3.3
*            	Flash Latency(WS)              = 4
*	ÐÎ    ²Î: ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	HAL_StatusTypeDef ret = HAL_OK;

	/* Ëø×¡SCU(Supply configuration update) */
	MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

	/* 
      1¡¢Ð¾Æ¬ÄÚ²¿µÄLDOÎÈÑ¹Æ÷Êä³öµÄµçÑ¹·¶Î§£¬¿ÉÑ¡VOS1£¬VOS2ºÍVOS3£¬²»Í¬·¶Î§¶ÔÓ¦²»Í¬µÄFlash¶ÁËÙ¶È£¬
         ÏêÇé¿´²Î¿¼ÊÖ²áµÄTable 12µÄ±í¸ñ¡£
      2¡¢ÕâÀïÑ¡ÔñÊ¹ÓÃVOS1£¬µçÑ¹·¶Î§1.15V - 1.26V¡£
    */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/* Ê¹ÄÜHSE£¬²¢Ñ¡ÔñHSE×÷ÎªPLLÊ±ÖÓÔ´£¬Èç¹ûUSBÊ¹ÓÃHSI48µÄ»°£¬ÕâÀïÒª¿ªÆô*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
		
	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 160;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;		
		
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;	
	ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
	if(ret != HAL_OK)
	{
        Error_Handler_bsp(__FILE__, __LINE__);
	}

	/* 
       Ñ¡ÔñPLLµÄÊä³ö×÷ÎªÏµÍ³Ê±ÖÓ
       ÅäÖÃRCC_CLOCKTYPE_SYSCLKÏµÍ³Ê±ÖÓ
       ÅäÖÃRCC_CLOCKTYPE_HCLK Ê±ÖÓ£¬¶ÔÓ¦AHB1£¬AHB2£¬AHB3ºÍAHB4×ÜÏß
       ÅäÖÃRCC_CLOCKTYPE_PCLK1Ê±ÖÓ£¬¶ÔÓ¦APB1×ÜÏß
       ÅäÖÃRCC_CLOCKTYPE_PCLK2Ê±ÖÓ£¬¶ÔÓ¦APB2×ÜÏß
       ÅäÖÃRCC_CLOCKTYPE_D1PCLK1Ê±ÖÓ£¬¶ÔÓ¦APB3×ÜÏß
       ÅäÖÃRCC_CLOCKTYPE_D3PCLK1Ê±ÖÓ£¬¶ÔÓ¦APB4×ÜÏß     
    */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
								 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;  
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; 
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; 
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2; 
	
	/* ´Ëº¯Êý»á¸üÐÂSystemCoreClock£¬²¢ÖØÐÂÅäÖÃHAL_InitTick */
	ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
	if(ret != HAL_OK)
	{
        Error_Handler_bsp(__FILE__, __LINE__);
	}

    /*
      Ê¹ÓÃIOµÄ¸ßËÙÄ£Ê½£¬ÒªÊ¹ÄÜIO²¹³¥£¬¼´µ÷ÓÃÏÂÃæÈý¸öº¯Êý 
      £¨1£©Ê¹ÄÜCSI clock
      £¨2£©Ê¹ÄÜSYSCFG clock
      £¨3£©Ê¹ÄÜI/O²¹³¥µ¥Ôª£¬ ÉèÖÃSYSCFG_CCCSR¼Ä´æÆ÷µÄbit0
    */
	__HAL_RCC_CSI_ENABLE() ;

	__HAL_RCC_SYSCFG_CLK_ENABLE() ;

	HAL_EnableCompensationCell();

   /* AXI SRAMµÄÊ±ÖÓÊÇÉÏµç×Ô¶¯Ê¹ÄÜµÄ£¬¶øD2ÓòµÄSRAM1£¬SRAM2ºÍSRAM3Òªµ¥¶ÀÊ¹ÄÜ */	
#if 0
	__HAL_RCC_D2SRAM1_CLK_ENABLE();
	__HAL_RCC_D2SRAM2_CLK_ENABLE();
	__HAL_RCC_D2SRAM3_CLK_ENABLE();
#endif
	
	{
		/*
			USB¹¤×÷ÐèÒª48MHzµÄÊ±ÖÓ£¬¿ÉÒÔÓÉPLL1Q£¬PLL3QºÍHSI48Ìá¹©
			PLL1QÓÃÓÚ¸øSDMMCÌá¹©Ê±ÖÓ
		    PLL3Q¸øLTDCÌá¹©Ê±ÖÓ£¬Ò²¿ÉÒÔ¸úUSB¹²ÓÃ£¬²»¹ýµÃ¸üUSBÉèÖÃÏàÍ¬µÄÆµÂÊ²Å¿ÉÒ»ÆðÓÃ¡£
		    HSI48¿ÉÒÔ¹©USB¶ÀÏí£¬¾ÍÊÇ¾«¶È²»ÊÇºÜ¸ß¡£
		*/
		RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
#if 0
		/* PLL3-Q for USB Clock = 48M */
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
		PeriphClkInitStruct.PLL3.PLL3M = 5;
		PeriphClkInitStruct.PLL3.PLL3N = 48;
		PeriphClkInitStruct.PLL3.PLL3P = 2;
		PeriphClkInitStruct.PLL3.PLL3Q = 5;
		PeriphClkInitStruct.PLL3.PLL3R = 2;
		PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
		PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
		PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
		PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct)!= HAL_OK)
		{
			Error_Handler_bsp(__FILE__, __LINE__);
		}	
#else
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
		PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
		{
			Error_Handler_bsp(__FILE__, __LINE__);
		}	
#endif
	}	

}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: Error_Handler_bsp
*	ÐÎ    ²Î: file : Ô´´úÂëÎÄ¼þÃû³Æ¡£¹Ø¼ü×Ö __FILE__ ±íÊ¾Ô´´úÂëÎÄ¼þÃû¡£
*			  line £º´úÂëÐÐºÅ¡£¹Ø¼ü×Ö __LINE__ ±íÊ¾Ô´´úÂëÐÐºÅ
*	·µ »Ø Öµ: ÎÞ
*		Error_Handler_bsp(__FILE__, __LINE__);
*********************************************************************************************************
*/
void Error_Handler_bsp(char *file, uint32_t line)
{
	/* 
		ÓÃ»§¿ÉÒÔÌí¼Ó×Ô¼ºµÄ´úÂë±¨¸æÔ´´úÂëÎÄ¼þÃûºÍ´úÂëÐÐºÅ£¬±ÈÈç½«´íÎóÎÄ¼þºÍÐÐºÅ´òÓ¡µ½´®¿Ú
		printf("Wrong parameters value: file %s on line %d\r\n", file, line) 
	*/
	
	/* ÕâÊÇÒ»¸öËÀÑ­»·£¬¶ÏÑÔÊ§°ÜÊ±³ÌÐò»áÔÚ´Ë´¦ËÀ»ú£¬ÒÔ±ãÓÚÓÃ»§²é´í */
	if (line == 0)
	{
		return;
	}
	
	while(1)
	{
	}
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: MPU_Config
*	¹¦ÄÜËµÃ÷: ÅäÖÃMPU
*	ÐÎ    ²Î: ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
static void MPU_Config( void )
{
	MPU_Region_InitTypeDef MPU_InitStruct;

	/* ½ûÖ¹ MPU */
	HAL_MPU_Disable();

	/* ÅäÖÃAXI SRAMµÄMPUÊôÐÔÎªWrite through, read allocate£¬no write allocate */
	MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
	MPU_InitStruct.BaseAddress      = 0x24000000;
	MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
	MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
	MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
	MPU_InitStruct.SubRegionDisable = 0x00;
	MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	
	
	/* ÅäÖÃFMCÀ©Õ¹IOµÄMPUÊôÐÔÎªDevice»òÕßStrongly Ordered */
	MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
	MPU_InitStruct.BaseAddress      = 0x60000000;
	MPU_InitStruct.Size             = ARM_MPU_REGION_SIZE_64KB;	
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
	MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;	/* ²»ÄÜÓÃMPU_ACCESS_CACHEABLE;»á³öÏÖ2´ÎCS¡¢WEÐÅºÅ */
	MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
	MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
	MPU_InitStruct.SubRegionDisable = 0x00;
	MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
	
	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	
	/* ÅäÖÃSRAM4µÄÊôÐÔÎªWrite through, read allocate£¬no write allocate */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = 0x38000000;
    MPU_InitStruct.Size             = ARM_MPU_REGION_SIZE_64KB;	
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

	/*Ê¹ÄÜ MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: CPU_CACHE_Enable
*	¹¦ÄÜËµÃ÷: Ê¹ÄÜL1 Cache
*	ÐÎ    ²Î: ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
static void CPU_CACHE_Enable(void)
{
	/* Ê¹ÄÜ I-Cache */
	SCB_EnableICache();

	/* Ê¹ÄÜ D-Cache */
	SCB_EnableDCache();
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: bsp_RunPer10ms
*	¹¦ÄÜËµÃ÷: ¸Ãº¯ÊýÃ¿¸ô10ms±»SystickÖÐ¶Ïµ÷ÓÃ1´Î¡£Ïê¼û bsp_timer.cµÄ¶¨Ê±ÖÐ¶Ï·þÎñ³ÌÐò¡£Ò»Ð©´¦ÀíÊ±¼äÒªÇó²»ÑÏ¸ñµÄ
*			ÈÎÎñ¿ÉÒÔ·ÅÔÚ´Ëº¯Êý¡£±ÈÈç£º°´¼üÉ¨Ãè¡¢·äÃùÆ÷Ãù½Ð¿ØÖÆµÈ¡£
*	ÐÎ    ²Î: ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void bsp_RunPer10ms(void)
{
	bsp_KeyScan10ms();
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: bsp_RunPer1ms
*	¹¦ÄÜËµÃ÷: ¸Ãº¯ÊýÃ¿¸ô1ms±»SystickÖÐ¶Ïµ÷ÓÃ1´Î¡£Ïê¼û bsp_timer.cµÄ¶¨Ê±ÖÐ¶Ï·þÎñ³ÌÐò¡£Ò»Ð©ÐèÒªÖÜÆÚÐÔ´¦ÀíµÄÊÂÎñ
*			 ¿ÉÒÔ·ÅÔÚ´Ëº¯Êý¡£±ÈÈç£º´¥Ãþ×ø±êÉ¨Ãè¡£
*	ÐÎ    ²Î: ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void bsp_RunPer1ms(void)
{
	
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: bsp_Idle
*	¹¦ÄÜËµÃ÷: ¿ÕÏÐÊ±Ö´ÐÐµÄº¯Êý¡£Ò»°ãÖ÷³ÌÐòÔÚforºÍwhileÑ­»·³ÌÐòÌåÖÐÐèÒª²åÈë CPU_IDLE() ºêÀ´µ÷ÓÃ±¾º¯Êý¡£
*			 ±¾º¯ÊýÈ±Ê¡Îª¿Õ²Ù×÷¡£ÓÃ»§¿ÉÒÔÌí¼ÓÎ¹¹·¡¢ÉèÖÃCPU½øÈëÐÝÃßÄ£Ê½µÄ¹¦ÄÜ¡£
*	ÐÎ    ²Î: ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void bsp_Idle(void)
{
	/* --- Î¹¹· */

	/* --- ÈÃCPU½øÈëÐÝÃß£¬ÓÉSystick¶¨Ê±ÖÐ¶Ï»½ÐÑ»òÕßÆäËûÖÐ¶Ï»½ÐÑ */

	/* ÀýÈç emWin Í¼ÐÎ¿â£¬¿ÉÒÔ²åÈëÍ¼ÐÎ¿âÐèÒªµÄÂÖÑ¯º¯Êý */
	//GUI_Exec();

	/* ÀýÈç uIP Ð­Òé£¬¿ÉÒÔ²åÈëuipÂÖÑ¯º¯Êý */
	//TOUCH_CapScan();
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: HAL_Delay
*	¹¦ÄÜËµÃ÷: ÖØ¶¨ÏòºÁÃëÑÓ³Ùº¯Êý¡£Ìæ»»HALÖÐµÄº¯Êý¡£ÒòÎªHALÖÐµÄÈ±Ê¡º¯ÊýÒÀÀµÓÚSystickÖÐ¶Ï£¬Èç¹ûÔÚUSB¡¢SD¿¨
*             ÖÐ¶ÏÖÐÓÐÑÓ³Ùº¯Êý£¬Ôò»áËøËÀ¡£Ò²¿ÉÒÔÍ¨¹ýº¯ÊýHAL_NVIC_SetPriorityÌáÉýSystickÖÐ¶Ï
*	ÐÎ    ²Î: ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
#if 1
void HAL_Delay(uint32_t Delay)
{
	bsp_DelayUS(Delay * 1000);
}
#endif

/***************************** °²¸»À³µç×Ó www.armfly.com (END OF FILE) *********************************/
