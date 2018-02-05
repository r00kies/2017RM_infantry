#include "usart1.h"
#define Uart1_Rx_LEN		256
uint8_t Uart1_Rx[Uart1_Rx_LEN];
extern int Kp,Ki,Kd,Kt;
__IO uint8_t RxCounter=0;
/*-----USART1_TX-----PA9-----*/
/*-----USART1_RX-----PA10----*/
//cyq: for test

void USART1_Configuration(void)
{
    USART_InitTypeDef usart1;
	  GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
	  DMA_InitTypeDef   dma;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 ,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);   
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9 ,GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 
	
    gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA,&gpio);

    usart1.USART_BaudRate = 115200;
    usart1.USART_WordLength = USART_WordLength_8b;
    usart1.USART_StopBits = USART_StopBits_1;
    usart1.USART_Parity = USART_Parity_No;
    usart1.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1,&usart1);
    
		 USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);   
		 USART_ITConfig(USART1, USART_IT_RXNE ,DISABLE);
     USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	   USART_Cmd(USART1,ENABLE);
    
//		  /* Set the USARTy prescaler */
//  USART_SetPrescaler(USART1, 0x1);
//  /* Configure the USARTy IrDA mode */
//  USART_IrDAConfig(USART1, USART_IrDAMode_Normal);

//  /* Enable the USARTy IrDA mode */
//  USART_IrDACmd(USART1, ENABLE);
		
    nvic.NVIC_IRQChannel = DMA2_Stream5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		DMA_DeInit(DMA2_Stream5);
    dma.DMA_Channel= DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)Uart1_Rx;
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = Uart1_Rx_LEN;
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
    DMA_Init(DMA2_Stream5,&dma);

    DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA2_Stream5,ENABLE);
}

void USART1_SendChar(unsigned char b)
{
    while (USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
    USART_SendData(USART1,b);
}

int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
    USART_SendData(USART1, (uint8_t)ch);    
    return ch;
}
int fgetc(FILE *f)

{

  while(!(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET))

  {

  }

 return (USART_ReceiveData(USART1));

}


static u16 RS232_VisualScope_CRC16( u8 *Array, u16 Len )
{
	u16 USART_IX, USART_IY, USART_CRC;

	USART_CRC = 0xffff;
	for(USART_IX=0; USART_IX<Len; USART_IX++) {
		USART_CRC = USART_CRC^(u16)(Array[USART_IX]);
		for(USART_IY=0; USART_IY<=7; USART_IY++) {
			if((USART_CRC&1)!=0)
				USART_CRC = (USART_CRC>>1)^0xA001;
			else
				USART_CRC = USART_CRC>>1;
		}
	}
	return(USART_CRC);
}



void RS232_VisualScope( USART_TypeDef* USARTx, u8 *pWord, u16 Len )
{
	u8 i = 0;
	u16 Temp = 0;
	Temp = RS232_VisualScope_CRC16(pWord, Len);
	pWord[8] = Temp&0x00ff;
	pWord[9] = (Temp&0xff00)>>8;

	for(i=0; i<10; i++) {
		USART_SendData(USARTx, (uint8_t)*pWord);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
		pWord++;
	}
}


void send_str( USART_TypeDef* USARTx, u8 *pWord, u16 Len )
{
	u8 i = 0;
	u16 Temp = 0;
	for(i=0; i<Len; i++) {
		USART_SendData(USARTx, (uint8_t)*pWord);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
		pWord++;
	}
}

void USART1_IRQHandler(void)                               
{   
	
	uint32_t temp = 0;
	uint16_t i = 0;
	
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
    	//USART_ClearFlag(USART1,USART_IT_IDLE);
    	temp = USART1->SR;
    	temp = USART1->DR; //?USART_IT_IDLE??
    	DMA_Cmd(DMA2_Stream5,DISABLE);

		temp = Uart1_Rx_LEN - DMA_GetCurrDataCounter(DMA2_Stream5);
//    RS232_VisualScope(USART1,Uart1_Rx,temp);
    for(i=0;i<temp;i++)
    ANO_DT_Data_Receive_Prepare(Uart1_Rx[i]);
		DMA_SetCurrDataCounter(DMA2_Stream5,Uart1_Rx_LEN);
		DMA_Cmd(DMA2_Stream5,ENABLE);
    } 
	
	__nop(); 
} 

void DMA2_Stream5_IRQHandler(void)
{   
     if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5))
     {
			 
       DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
			 DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5); 
//			 printf("%s\r\n",Receive_data);
			 
		 }
}

