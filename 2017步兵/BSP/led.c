#include "led.h"
#include "delay.h"

void LED_Configuration(void)
{
	GPIO_InitTypeDef gpio;   

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
		
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &gpio);
	LED_GREEN_OFF();
	LED_RED_OFF();
}
