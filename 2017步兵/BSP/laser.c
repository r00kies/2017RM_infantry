#include "laser.h"



//¼¤¹â
void LASER_Configuration(void)//cyq
{
	GPIO_InitTypeDef gpio;   

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
		
	gpio.GPIO_Pin = GPIO_Pin_5;	
    gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &gpio);
	GPIO_SetBits(GPIOC,GPIO_Pin_5);//cyq
}
