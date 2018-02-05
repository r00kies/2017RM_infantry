#ifndef __TASK_LED_H__
#define __TASK_LED_H__
#include <stm32f4xx.h>
#include "led.h"
#include "delay.h"
void Spark_LEDs(void);
void Spark_GreenLED(void);
void Turn_On_LEDs(void);
void Turn_Off_LEDs(void);
void led_task(void *pdata);
#define  LED_GREEN_ON()      GPIO_ResetBits(GPIOA, GPIO_Pin_6)
#define  LED_GREEN_OFF()     GPIO_SetBits(GPIOA, GPIO_Pin_6)
#define  LED_GREEN_TOGGLE()  GPIO_ToggleBits(GPIOA, GPIO_Pin_6)

#define  LED_RED_ON()        GPIO_ResetBits(GPIOA, GPIO_Pin_7)
#define  LED_RED_OFF()       GPIO_SetBits(GPIOA, GPIO_Pin_7)
#define  LED_RED_TOGGLE()    GPIO_ToggleBits(GPIOA, GPIO_Pin_7)

#endif
