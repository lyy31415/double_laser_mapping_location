#include "timer.h"
#include "led.h"

u8 timeout_triple=0;//��ʱ���������


//ͨ�ö�ʱ��2�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��2!
void TIM6_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);  ///ʹ��TIM6ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(ENCODER_OMEGA_TIM,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(ENCODER_OMEGA_TIM,TIM_IT_Update,ENABLE); //����ʱ��6�����ж�
	TIM_Cmd(ENCODER_OMEGA_TIM,ENABLE); //ʹ�ܶ�ʱ��2
	
	NVIC_InitStructure.NVIC_IRQChannel = ENCODER_OMEGA_IRQn; //��ʱ��6�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ENCODER_OMEGA_TIM_PreemptionPriority; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = ENCODER_OMEGA_TIM_SubPriority; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//��ʱ��6�жϷ�����
void ENCODER_OMEGA_IRQHandler(void)
{
	if(TIM_GetITStatus(ENCODER_OMEGA_TIM,TIM_IT_Update)==SET) //����ж�
	{
		//LED1=!LED1;
			timeout_triple++;
	}
	TIM_ClearITPendingBit(ENCODER_OMEGA_TIM,TIM_IT_Update);  //����жϱ�־λ
}
