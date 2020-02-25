#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "my_queue.h"

void ULTRASONIC_TIM11_Cap_Init(u16 arr,u16 psc);
void ULTRASONIC_TIM12_Cap_Init(u16 arr,u16 psc);
void ULTRASONIC_TIM8_Cap_Init(u16 arr,u16 psc);
void ULTRASONIC_TRIG_Config(void);
void StartMeasureTIM12(void);
void StartMeasureTIM11(void);
void StartMeasureTIM8CH3(void);
void StartMeasureTIM8CH4(void);
void ULTRASONIC_START(void);
void ULTRA_OBSTAC_AVOID(void);
//float slide_window(float current);

typedef struct TIM2_CAP 
{
	
	//����״̬
	//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
	//[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
	//[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
	

	u8  TIM11CH1_CAPTURE_STA;	//���벶��״̬
	u32	TIM11CH1_CAPTURE_VAL_START;
	u32	TIM11CH1_CAPTURE_VAL;	//���벶��ֵ(TIM11��16λ)
	s32 TIM11CH1_DELTA_VAL;
	
	u8  TIM12CH2_CAPTURE_STA;	//���벶��״̬	
	u32	TIM12CH2_CAPTURE_VAL_START;
	u32	TIM12CH2_CAPTURE_VAL;	//���벶��ֵ(TIM12��16λ)
	s32 TIM12CH2_DELTA_VAL;
	
	u8  TIM8CH3_CAPTURE_STA;	//���벶��״̬		    
	u32	TIM8CH3_CAPTURE_VAL_START;
	u32	TIM8CH3_CAPTURE_VAL;	//���벶��ֵ(TIM8��16λ)
	s32 TIM8CH3_DELTA_VAL;

	u8  TIM8CH4_CAPTURE_STA;	//���벶��״̬	
	u32	TIM8CH4_CAPTURE_VAL_START;
	u32	TIM8CH4_CAPTURE_VAL;	//���벶��ֵ(TIM8��16λ)
	s32 TIM8CH4_DELTA_VAL;
	
	
	u32 TIM8CH3_CURRENT_CNT;
	u32 TIM8CH4_CURRENT_CNT;
	u32 TIM11CH1_CURRENT_CNT;
	u32 TIM12CH2_CURRENT_CNT;
	
} ultrasonic_tim_cap;






#endif
