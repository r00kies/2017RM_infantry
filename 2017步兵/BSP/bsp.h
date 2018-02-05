#ifndef __BSP_H__
#define __BSP_H__

#include <stm32f4xx.h>
#include "can1.h"
#include "can2.h"
#include "led.h"
#include "key.h"
#include "usart3.h"
#include "usart2.h"
#include "usart1.h"
#include "delay.h"
#include "pwm.h"
#include "laser.h"
#include "RELAY.h"
#include "tim6.h"
#include "iwdg.h"
#include "buzzer.h"
#include "stmflash.h"
#include "encoder.h"
void BSP_Init(void);
#endif 
