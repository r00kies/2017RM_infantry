#ifndef __LASER_H__
#define __LASER_H__

#include <stm32f4xx.h>

void LASER_Configuration(void);
void RELAY_init(void);
#define LASER_on() GPIO_SetBits(GPIOC,GPIO_Pin_5)
#define LASER_off() GPIO_ResetBits(GPIOC,GPIO_Pin_5)
#endif 
