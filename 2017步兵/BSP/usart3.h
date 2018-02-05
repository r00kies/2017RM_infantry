#ifndef __USART3_H__
#define __USART3_H__

#include <stm32f4xx.h>
typedef struct
{
u16 X;
u16 Y;
u16 Len;
u16 Width;
} Cam_data;
extern Cam_data cam_date;
void USART3_Configuration(void);
u8 serialcom();
#define Uart3_Rx_LEN		128
extern unsigned char Uart3_Rx[Uart3_Rx_LEN];
#endif
