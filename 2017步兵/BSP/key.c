#include "key.h"

void Key_Configuration(void)
{
	GPIO_InitTypeDef gpio;   

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
		
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	
  gpio.GPIO_Mode = GPIO_Mode_IN;//
  gpio.GPIO_Speed = GPIO_Speed_100MHz;//
  gpio.GPIO_PuPd = GPIO_PuPd_UP;//
	GPIO_Init(GPIOC, &gpio);

}