#ifndef __TASK_DOOR_H__
#define __TASK_DOOR_H__
#include <stm32f4xx.h>
#include "pwm.h"
#include "usart2.h"
#include "task_led.h"
#include "TASK_key_board.H"
void door_task(void *pdata);
void DOOR_control();
#endif