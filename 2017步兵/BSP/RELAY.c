#include "RELAY.h"
//¼ÌµçÆ÷
//PC0
//PC1
//PC2
void RELAY_init(void)
{
  GPIO_InitTypeDef gpio;   
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
		
	gpio.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;	
  gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &gpio);
	GPIO_ResetBits(GPIOC,GPIO_Pin_0);
	GPIO_ResetBits(GPIOC,GPIO_Pin_1);
	GPIO_ResetBits(GPIOC,GPIO_Pin_2);
}