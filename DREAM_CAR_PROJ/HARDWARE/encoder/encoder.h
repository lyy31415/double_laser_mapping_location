#ifndef __ENCODER_H
#define __ENCODER_H
#include <sys.h>	 
#include "usart.h"


/* STM32F407�Ķ�ʱ��2��5��32λ�ģ������Ķ�ʱ������16λ��  */
#define ENCODER_TIM_PERIOD (u16)(65535)   //���ɴ���65535 ��ΪF407�Ķ�ʱ����16λ�ġ�

void LEFT_ENCODER_TIM_Init(void);
void RIGHT_ENCODER_TIM_Init(void);
int Read_Encoder(u8 TIMX);
#endif
