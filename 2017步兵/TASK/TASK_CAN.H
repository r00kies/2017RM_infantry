#ifndef __TASK_CAN_H__
#define __TASK_CAN_H__
#include <stm32f4xx.h>
#include "task_led.h"
#include "can1.h"
#include "can2.h"
#include "task_control.h"

#define h8b(num) ((num) >> 8)
#define l8b(num) ((num)&0xff)
extern Chassis RM3510_DATA;
extern s16 current_out[6];
extern s16 PITCH_speed;
void can_task(void *pdata);
void RM_Driver_Cmd(CAN_TypeDef *CANx,s16 c205,s16 c206);
void RM3510_Cmd(CAN_TypeDef *CANx,s16 c201,s16 c202,s16 c203,s16 c204);
void ALL_can_tx();
#endif 