#ifndef __ENCODER_H
#define __ENCODER_H
#include <sys.h>	 
#include "usart.h"


/* STM32F407的定时器2和5是32位的，其他的定时器都是16位的  */
#define ENCODER_TIM_PERIOD (u16)(65535)   //不可大于65535 因为F407的定时器是16位的。

void LEFT_ENCODER_TIM_Init(void);
void RIGHT_ENCODER_TIM_Init(void);
int Read_Encoder(u8 TIMX);
#endif
