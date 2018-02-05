#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "mpu6050_i2c.h"
#include "mpu6050_interrupt.h"
#include "mpu6050_driver.h"
#include "mpu6050_process.h"
#include "mpu6050.h"
#include "bsp.h"
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "HMC5883L.h"
void start_task(void *pdata);	
#endif
