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
	
	//捕获状态
	//[7]:0,没有成功的捕获;1,成功捕获到一次.
	//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
	//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
	

	u8  TIM11CH1_CAPTURE_STA;	//输入捕获状态
	u32	TIM11CH1_CAPTURE_VAL_START;
	u32	TIM11CH1_CAPTURE_VAL;	//输入捕获值(TIM11是16位)
	s32 TIM11CH1_DELTA_VAL;
	
	u8  TIM12CH2_CAPTURE_STA;	//输入捕获状态	
	u32	TIM12CH2_CAPTURE_VAL_START;
	u32	TIM12CH2_CAPTURE_VAL;	//输入捕获值(TIM12是16位)
	s32 TIM12CH2_DELTA_VAL;
	
	u8  TIM8CH3_CAPTURE_STA;	//输入捕获状态		    
	u32	TIM8CH3_CAPTURE_VAL_START;
	u32	TIM8CH3_CAPTURE_VAL;	//输入捕获值(TIM8是16位)
	s32 TIM8CH3_DELTA_VAL;

	u8  TIM8CH4_CAPTURE_STA;	//输入捕获状态	
	u32	TIM8CH4_CAPTURE_VAL_START;
	u32	TIM8CH4_CAPTURE_VAL;	//输入捕获值(TIM8是16位)
	s32 TIM8CH4_DELTA_VAL;
	
	
	u32 TIM8CH3_CURRENT_CNT;
	u32 TIM8CH4_CURRENT_CNT;
	u32 TIM11CH1_CURRENT_CNT;
	u32 TIM12CH2_CURRENT_CNT;
	
} ultrasonic_tim_cap;






#endif
