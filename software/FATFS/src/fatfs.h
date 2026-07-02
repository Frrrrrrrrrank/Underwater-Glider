/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fatfs.h
  * @brief   FatFS wrapper interface for project-level initialization.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __fatfs_H
#define __fatfs_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio_dma.h"

extern uint8_t retSD;
extern char SDPath[4];
extern FATFS SDFatFS;
extern FIL SDFile;

void MX_FATFS_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __fatfs_H */
