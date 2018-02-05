#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "stm32f4xx.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "task_control.h"
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
 
#define FLASH_USER_START_ADDR   (uint32_t)0x080E0000

#define FLASH_USER_END_ADDR     (uint32_t)0x080FFFFF




u8 Data_Save(void);
u8 Data_Read(void);

#endif









