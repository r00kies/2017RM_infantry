#include "usart3.h"
#include "led.h"
/*-----USART3_TX-----PA2----*/
/*-----USART3_RX-----PA3----*/
Cam_data cam_date;
unsigned char Uart3_Rx[Uart3_Rx_LEN]={0};

void USART3_Configuration(void)
{
			USART_InitTypeDef usart3;
			GPIO_InitTypeDef  gpio;
			NVIC_InitTypeDef  nvic;
			DMA_InitTypeDef   dma;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource10 ,GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource11 ,GPIO_AF_USART3);
		
		gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10 ;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB,&gpio);
	
		USART_DeInit(USART3);
		usart3.USART_BaudRate = 115200;
		usart3.USART_WordLength = USART_WordLength_8b;
		usart3.USART_StopBits = USART_StopBits_1;
		usart3.USART_Parity = USART_Parity_No;
		usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
		usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3,&usart3);
		USART_Cmd(USART3,ENABLE);
    
		
//		USART_SetPrescaler(USART3, 0x1);
//		USART_IrDAConfig(USART3, USART_IrDAMode_Normal);
//		USART_IrDACmd(USART3, ENABLE);
//		
		USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);
		USART_ITConfig(USART3, USART_IT_RXNE ,DISABLE);
		USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
		USART_Cmd(USART3,ENABLE);

    nvic.NVIC_IRQChannel = DMA1_Stream1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    DMA_DeInit(DMA1_Stream1);
    dma.DMA_Channel= DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)&(Uart3_Rx);
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = Uart3_Rx_LEN;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_Mode_Normal;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1,&dma);

    DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA1_Stream1,ENABLE);
		
}

void USART3_IRQHandler(void)                               
{   
	uint32_t temp = 0;
	uint16_t i = 0;
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
    	//USART_ClearFlag(USART1,USART_IT_IDLE);
    	temp = USART3->SR;
    	temp = USART3->DR; //?USART_IT_IDLE??
    	DMA_Cmd(DMA1_Stream1,DISABLE);
		  temp = Uart3_Rx_LEN - DMA_GetCurrDataCounter(DMA1_Stream1);
 			serialcom();
			DMA_SetCurrDataCounter(DMA1_Stream1,Uart3_Rx_LEN);
			DMA_Cmd(DMA1_Stream1,ENABLE);
    }
	__nop(); 
} 
void DMA1_Stream1_IRQHandler(void)
{   
	 if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
	 {
		 DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
		 DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1); 
	 }
}
u8 serialcom()
{
	  if((Uart3_Rx[0] == 0xaa)  && (Uart3_Rx[10] == 0xee)) 
  {
		
		if(Uart3_Rx[9]==((Uart3_Rx[1]+Uart3_Rx[2]+Uart3_Rx[3]+Uart3_Rx[4]+Uart3_Rx[5]+Uart3_Rx[6]+Uart3_Rx[7]+Uart3_Rx[8])&0x00ff))
		{
		 cam_date.X=((u16)Uart3_Rx[1]<<8)+(u16)Uart3_Rx[2];
		 cam_date.Y=((u16)Uart3_Rx[3]<<8)+(u16)Uart3_Rx[4];
		 cam_date.Width=((u16)Uart3_Rx[5]<<8)+(u16)Uart3_Rx[6];
		 cam_date.Len=((u16)Uart3_Rx[7]<<8)+(u16)Uart3_Rx[8];
		 LED_GREEN_TOGGLE();
		}
		else
		{
			char i;
            for(i=0;i<11;i++)
			Uart3_Rx[i]=0;
//            cam_date.X=320;
//            cam_date.Y=240;
			return 0;
		}
	}
	else
		return 0;
	
	return 1;
}
