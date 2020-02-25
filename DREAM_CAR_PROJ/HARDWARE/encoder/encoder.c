#include "encoder.h"
#include "stm32f4xx_gpio.h"

//u8 timeout_left=0,timeout_right=0;//编码器计数溢出次数
static u8 ENCODER_L_DIR=0, ENCODER_R_DIR=0; // 0：计数器递增计数
                                        // 1：计数器递减计数

/**************************************************************************
函数功能：把TIM4初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void LEFT_ENCODER_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4的时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能PA端口时钟
	
  GPIO_InitStructure.GPIO_Pin  = LEFT_ENCODER_A_Pin|LEFT_ENCODER_B_Pin;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //复用
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//浮空
  GPIO_Init(LEFT_ENCODER_A_Port, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA
	
	GPIO_PinAFConfig(LEFT_ENCODER_A_Port,LEFT_ENCODER_A_PinSource,LEFT_ENCODER_A_AF); //GPIOD12复用为TIM4
	GPIO_PinAFConfig(LEFT_ENCODER_B_Port,LEFT_ENCODER_B_PinSource,LEFT_ENCODER_B_AF);//GPIOD13复用为TIM4
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频      用于采样 By lyy
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
  TIM_TimeBaseInit(LEFT_ENCODER_TIM, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(LEFT_ENCODER_TIM, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);//使用编码器模式1
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(LEFT_ENCODER_TIM, &TIM_ICInitStructure);
//  TIM_ClearFlag(LEFT_ENCODER_TIM, TIM_FLAG_Update);//清除TIM的更新标志位
//  TIM_ITConfig(LEFT_ENCODER_TIM, TIM_IT_Update, ENABLE);
  //Reset counter
//  TIM_SetCounter(LEFT_ENCODER_TIM,0);
  TIM_Cmd(LEFT_ENCODER_TIM, ENABLE); 
	
//	NVIC_InitStructure_L.NVIC_IRQChannel = LEFT_ENCODER_IRQn; //定时器2中断
//	NVIC_InitStructure_L.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级1
//	NVIC_InitStructure_L.NVIC_IRQChannelSubPriority = 0; //子优先级0
//	NVIC_InitStructure_L.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure_L);
}
/**************************************************************************
函数功能：把TIM3初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void RIGHT_ENCODER_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//使能定时器3的时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PA端口时钟
	
  GPIO_InitStructure.GPIO_Pin = RIGHT_ENCODER_A_Pin|RIGHT_ENCODER_B_Pin;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //复用
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//浮空
  GPIO_Init(RIGHT_ENCODER_A_Port, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA
	
	GPIO_PinAFConfig(RIGHT_ENCODER_A_Port,RIGHT_ENCODER_A_PinSource,RIGHT_ENCODER_A_AF); //GPIOA6复用为TIM3
	GPIO_PinAFConfig(RIGHT_ENCODER_B_Port,RIGHT_ENCODER_B_PinSource,RIGHT_ENCODER_B_AF);//GPIOA7复用为TIM3
	
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频      用于采样 By lyy
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
  TIM_TimeBaseInit(RIGHT_ENCODER_TIM, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(RIGHT_ENCODER_TIM, TIM_EncoderMode_TI1, TIM_ICPolarity_Falling, TIM_ICPolarity_Rising);// 使用编码器模式1
	                                         //  TIM_EncoderMode_TI12表示4倍频，TIM_ICPolarity_Rising 和 TIM_ICPolarity_Falling 表示方向  
                                           //  TIM_ICPolarity_BothEdge
	
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(RIGHT_ENCODER_TIM, &TIM_ICInitStructure);
//  TIM_ClearFlag(RIGHT_ENCODER_TIM, TIM_FLAG_Update);//清除TIM的更新标志位
//  TIM_ITConfig(RIGHT_ENCODER_TIM, TIM_IT_Update, ENABLE);
  //Reset counter
//  TIM_SetCounter(RIGHT_ENCODER_TIM,0);
  TIM_Cmd(RIGHT_ENCODER_TIM, ENABLE); 
	
	
//	NVIC_InitStructure_R.NVIC_IRQChannel = RIGHT_ENCODER_IRQn; //定时器2中断
//	NVIC_InitStructure_R.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级1
//	NVIC_InitStructure_R.NVIC_IRQChannelSubPriority = 1; //子优先级1
//	NVIC_InitStructure_R.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure_R);

}

/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
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
		 if(ENCODER_L_DIR == 0 && Encoder_L_TIM < 60000)// 递增计数
		 {
			 if(Encoder_L_TIM < 32767)
					Encoder_TIM= (int16_t)LEFT_ENCODER_TIM->CNT;
			 else if(Encoder_L_TIM >= 32767)
				  Encoder_TIM= (uint16_t)LEFT_ENCODER_TIM->CNT;
//			 printf("左递增\r\n");
		 }
		 if(ENCODER_L_DIR == 1 && Encoder_L_TIM > 5000)// 递减计数
		 {
			 if(Encoder_L_TIM > 32768)
					Encoder_TIM= (int16_t)LEFT_ENCODER_TIM->CNT;
			 else if(Encoder_L_TIM <= 32768)
				  Encoder_TIM= (uint16_t)LEFT_ENCODER_TIM->CNT - 65535;
//			 printf("左递减\r\n");
		 }
			 
		 LEFT_ENCODER_TIM->CNT=0;
	//		   timeout_left=0;
		 break;
	 case 3:  
	   Encoder_R_TIM= (uint16_t)RIGHT_ENCODER_TIM->CNT;
		 if(ENCODER_R_DIR == 0 && Encoder_R_TIM < 60000)// 递增计数
		 {
			 if(Encoder_R_TIM < 32767)
					Encoder_TIM= (int16_t)RIGHT_ENCODER_TIM->CNT;
			 else if(Encoder_R_TIM >= 32767)
				  Encoder_TIM= (uint16_t)RIGHT_ENCODER_TIM->CNT;
//			 printf("右递增\r\n");
		 }
		 if(ENCODER_R_DIR == 1 && Encoder_R_TIM > 5000)// 递减计数
		 {
			 if(Encoder_R_TIM > 32768)
					Encoder_TIM= (int16_t)RIGHT_ENCODER_TIM->CNT;
			 else if(Encoder_R_TIM <= 32768)
				  Encoder_TIM= (uint16_t)RIGHT_ENCODER_TIM->CNT - 65535;
//			 printf("右递减\r\n");
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
函数功能：TIM5中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
//void LEFT_ENCODER_IRQHandler(void)
//{ 		    		  			     	   
//  if(TIM_GetITStatus(LEFT_ENCODER_TIM,TIM_IT_Update)==SET) //溢出中断
//	{
//			timeout_left++;
//		printf("left encoder interruput!");
//	}
//	TIM_ClearITPendingBit(LEFT_ENCODER_TIM,TIM_IT_Update);  //清除中断标志位

//	
//}
/**************************************************************************
函数功能：TIM3中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
//void RIGHT_ENCODER_IRQHandler(void)
//{ 		    		  			    
////	if(RIGHT_ENCODER_TIM->SR&0X0001)//溢出中断
////	{    				   				     	    	
////	}				   
////	RIGHT_ENCODER_TIM->SR&=~(1<<0);//清除中断标志位 	 

//  if(TIM_GetITStatus(RIGHT_ENCODER_TIM,TIM_IT_Update)==SET) //溢出中断
//	{
//			timeout_right++;
//		printf("right encoder interruput!");
//	}
//	TIM_ClearITPendingBit(RIGHT_ENCODER_TIM,TIM_IT_Update);  //清除中断标志位
//	
//}

