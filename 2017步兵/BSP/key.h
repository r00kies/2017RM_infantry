#ifndef __KEY_H__
#define __KEY_H__
#include <stm32f4xx.h>
#define KEY0 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6) //PE4
#define KEY1 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)	//PE3 
void Key_Configuration(void);

#endif 