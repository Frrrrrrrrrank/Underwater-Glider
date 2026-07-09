/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fatfs.c
  * @brief   FatFS project wrapper.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "fatfs.h"

uint8_t retSD;
char SDPath[4];
FATFS SDFatFS;
FIL SDFile;

void MX_FATFS_Init(void)
{
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);
}
