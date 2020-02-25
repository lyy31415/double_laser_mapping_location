#include "encoder.h"
#include "stm32f4xx_gpio.h"

//u8 timeout_left=0,timeout_right=0;//�����������������
static u8 ENCODER_L_DIR=0, ENCODER_R_DIR=0; // 0����������������
                                        // 1���������ݼ�����

/**************************************************************************
�������ܣ���TIM4��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void LEFT_ENCODER_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//ʹ�ܶ�ʱ��4��ʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��PA�˿�ʱ��
	
  GPIO_InitStructure.GPIO_Pin  = LEFT_ENCODER_A_Pin|LEFT_ENCODER_B_Pin;	//�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(LEFT_ENCODER_A_Port, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA
	
	GPIO_PinAFConfig(LEFT_ENCODER_A_Port,LEFT_ENCODER_A_PinSource,LEFT_ENCODER_A_AF); //GPIOD12����ΪTIM4
	GPIO_PinAFConfig(LEFT_ENCODER_B_Port,LEFT_ENCODER_B_PinSource,LEFT_ENCODER_B_AF);//GPIOD13����ΪTIM4
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ      ���ڲ��� By lyy
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
  TIM_TimeBaseInit(LEFT_ENCODER_TIM, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(LEFT_ENCODER_TIM, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);//ʹ�ñ�����ģʽ1
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(LEFT_ENCODER_TIM, &TIM_ICInitStructure);
//  TIM_ClearFlag(LEFT_ENCODER_TIM, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
//  TIM_ITConfig(LEFT_ENCODER_TIM, TIM_IT_Update, ENABLE);
  //Reset counter
//  TIM_SetCounter(LEFT_ENCODER_TIM,0);
  TIM_Cmd(LEFT_ENCODER_TIM, ENABLE); 
	
//	NVIC_InitStructure_L.NVIC_IRQChannel = LEFT_ENCODER_IRQn; //��ʱ��2�ж�
//	NVIC_InitStructure_L.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�1
//	NVIC_InitStructure_L.NVIC_IRQChannelSubPriority = 0; //�����ȼ�0
//	NVIC_InitStructure_L.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure_L);
}
/**************************************************************************
�������ܣ���TIM3��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void RIGHT_ENCODER_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//ʹ�ܶ�ʱ��3��ʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PA�˿�ʱ��
	
  GPIO_InitStructure.GPIO_Pin = RIGHT_ENCODER_A_Pin|RIGHT_ENCODER_B_Pin;	//�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(RIGHT_ENCODER_A_Port, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA
	
	GPIO_PinAFConfig(RIGHT_ENCODER_A_Port,RIGHT_ENCODER_A_PinSource,RIGHT_ENCODER_A_AF); //GPIOA6����ΪTIM3
	GPIO_PinAFConfig(RIGHT_ENCODER_B_Port,RIGHT_ENCODER_B_PinSource,RIGHT_ENCODER_B_AF);//GPIOA7����ΪTIM3
	
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ      ���ڲ��� By lyy
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
  TIM_TimeBaseInit(RIGHT_ENCODER_TIM, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(RIGHT_ENCODER_TIM, TIM_EncoderMode_TI1, TIM_ICPolarity_Falling, TIM_ICPolarity_Rising);// ʹ�ñ�����ģʽ1
	                                         //  TIM_EncoderMode_TI12��ʾ4��Ƶ��TIM_ICPolarity_Rising �� TIM_ICPolarity_Falling ��ʾ����  
                                           //  TIM_ICPolarity_BothEdge
	
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(RIGHT_ENCODER_TIM, &TIM_ICInitStructure);
//  TIM_ClearFlag(RIGHT_ENCODER_TIM, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
//  TIM_ITConfig(RIGHT_ENCODER_TIM, TIM_IT_Update, ENABLE);
  //Reset counter
//  TIM_SetCounter(RIGHT_ENCODER_TIM,0);
  TIM_Cmd(RIGHT_ENCODER_TIM, ENABLE); 
	
	
//	NVIC_InitStructure_R.NVIC_IRQChannel = RIGHT_ENCODER_IRQn; //��ʱ��2�ж�
//	NVIC_InitStructure_R.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�1
//	NVIC_InitStructure_R.NVIC_IRQChannelSubPriority = 1; //�����ȼ�1
//	NVIC_InitStructure_R.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure_R);

}

/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
	int Encoder_TIM=0, Encoder_L_TIM=0, Encoder_R_TIM=0;
  ENCODER_L_DIR = (LEFT_ENCODER_TIM->CR1 & 0x10) >> 4;
  ENCODER_R_DIR = (RIGHT_ENCODER_TIM->CR1 & 0x10) >> 4;
	switch(TIMX)
	{
	 case 4:  
	   Encoder_L_TIM= (uint16_t)LEFT_ENCODER_TIM->CNT;
		 if(ENCODER_L_DIR == 0 && Encoder_L_TIM < 60000)// ��������
		 {
			 if(Encoder_L_TIM < 32767)
					Encoder_TIM= (int16_t)LEFT_ENCODER_TIM->CNT;
			 else if(Encoder_L_TIM >= 32767)
				  Encoder_TIM= (uint16_t)LEFT_ENCODER_TIM->CNT;
//			 printf("�����\r\n");
		 }
		 if(ENCODER_L_DIR == 1 && Encoder_L_TIM > 5000)// �ݼ�����
		 {
			 if(Encoder_L_TIM > 32768)
					Encoder_TIM= (int16_t)LEFT_ENCODER_TIM->CNT;
			 else if(Encoder_L_TIM <= 32768)
				  Encoder_TIM= (uint16_t)LEFT_ENCODER_TIM->CNT - 65535;
//			 printf("��ݼ�\r\n");
		 }
			 
		 LEFT_ENCODER_TIM->CNT=0;
	//		   timeout_left=0;
		 break;
	 case 3:  
	   Encoder_R_TIM= (uint16_t)RIGHT_ENCODER_TIM->CNT;
		 if(ENCODER_R_DIR == 0 && Encoder_R_TIM < 60000)// ��������
		 {
			 if(Encoder_R_TIM < 32767)
					Encoder_TIM= (int16_t)RIGHT_ENCODER_TIM->CNT;
			 else if(Encoder_R_TIM >= 32767)
				  Encoder_TIM= (uint16_t)RIGHT_ENCODER_TIM->CNT;
//			 printf("�ҵ���\r\n");
		 }
		 if(ENCODER_R_DIR == 1 && Encoder_R_TIM > 5000)// �ݼ�����
		 {
			 if(Encoder_R_TIM > 32768)
					Encoder_TIM= (int16_t)RIGHT_ENCODER_TIM->CNT;
			 else if(Encoder_R_TIM <= 32768)
				  Encoder_TIM= (uint16_t)RIGHT_ENCODER_TIM->CNT - 65535;
//			 printf("�ҵݼ�\r\n");
		 }
				 
		 RIGHT_ENCODER_TIM->CNT=0;
	//		   timeout_right=0;
		 break;	
	 default:  
		 Encoder_TIM=0;
	}
//	 printf("ENCODER_L_DIR = %d\r\n",ENCODER_L_DIR);
//	 printf("ENCODER_R_DIR = %d\r\n",ENCODER_R_DIR);
	
	return Encoder_TIM;
}
/**************************************************************************
�������ܣ�TIM5�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
//void LEFT_ENCODER_IRQHandler(void)
//{ 		    		  			     	   
//  if(TIM_GetITStatus(LEFT_ENCODER_TIM,TIM_IT_Update)==SET) //����ж�
//	{
//			timeout_left++;
//		printf("left encoder interruput!");
//	}
//	TIM_ClearITPendingBit(LEFT_ENCODER_TIM,TIM_IT_Update);  //����жϱ�־λ

//	
//}
/**************************************************************************
�������ܣ�TIM3�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
//void RIGHT_ENCODER_IRQHandler(void)
//{ 		    		  			    
////	if(RIGHT_ENCODER_TIM->SR&0X0001)//����ж�
////	{    				   				     	    	
////	}				   
////	RIGHT_ENCODER_TIM->SR&=~(1<<0);//����жϱ�־λ 	 

//  if(TIM_GetITStatus(RIGHT_ENCODER_TIM,TIM_IT_Update)==SET) //����ж�
//	{
//			timeout_right++;
//		printf("right encoder interruput!");
//	}
//	TIM_ClearITPendingBit(RIGHT_ENCODER_TIM,TIM_IT_Update);  //����жϱ�־λ
//	
//}

