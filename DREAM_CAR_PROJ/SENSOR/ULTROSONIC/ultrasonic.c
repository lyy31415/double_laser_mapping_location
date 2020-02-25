#include "ultrasonic.h"

ultrasonic_tim_cap capture;
float ultrasonic_dist1, ultrasonic_dist2, ultrasonic_dist3, ultrasonic_dist4;//超声波测距距离
u8 ultra_stop_status;
static float ultra_dist_avg1=0.0f, ultra_dist_avg2=0.0f, ultra_dist_avg3=0.0f, ultra_dist_avg4=0.0f;
static SqQueue ultra_que1, ultra_que2, ultra_que3, ultra_que4;

// 超声波用定时器 TIM8 CH3 PC8 CH4 PC9/TIM11 CH1 PF7/TIM12 CH2 PB15 
//定时器通道输入捕获配置
//arr：自动重装值(TIM8 TIM12 TIM11是16位的!!)
//psc：时钟预分频数
void ULTRASONIC_TIM11_Cap_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef 	 NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM11_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);  	//TIM11时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//使能PF时钟	
	
	// TIM11 CH1 PF7
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_TIM11_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(ULTRASONIC_TIM11_Port,&GPIO_InitStructure); //初始化PF7
	GPIO_PinAFConfig(ULTRASONIC_TIM11_Port,ULTRASONIC_TIM11_PinSource,ULTRASONIC_TIM11_AF); // PF7复用为 定时器11
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(ULTRASONIC_TIM11,&TIM_TimeBaseStructure);

	//初始化TIM11 CH1输入捕获参数
	TIM11_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM11_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM11_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM11_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM11_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(ULTRASONIC_TIM11, &TIM11_ICInitStructure);	
		
	TIM_ITConfig(ULTRASONIC_TIM11,TIM_IT_Update|TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断
	
	TIM_Cmd(ULTRASONIC_TIM11,ENABLE ); 	//使能定时器11

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM11_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=ULTRASONIC_TIM11_1_PreptPrio;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =ULTRASONIC_TIM11_1_SubPrio;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化NVIC寄存器

	InitQueue(&ultra_que1);
}




void ULTRASONIC_TIM12_Cap_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef 	 NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM12_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  	//TIM12时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PB时钟	
	
	// TIM12 CH2 PB15
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_TIM12_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(ULTRASONIC_TIM12_Port,&GPIO_InitStructure);
	GPIO_PinAFConfig(ULTRASONIC_TIM12_Port,ULTRASONIC_TIM12_PinSource,ULTRASONIC_TIM12_AF);// PB15复用为 定时器12

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(ULTRASONIC_TIM12,&TIM_TimeBaseStructure);

	//初始化TIM12 CH2输入捕获参数
	TIM12_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM12_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM12_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM12_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM12_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(ULTRASONIC_TIM12, &TIM12_ICInitStructure);	
		
	TIM_ITConfig(ULTRASONIC_TIM12,TIM_IT_Update|TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC1IE捕获中断
	
	TIM_Cmd(ULTRASONIC_TIM12,ENABLE ); 	//使能定时器12

  NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=ULTRASONIC_TIM12_2_PreptPrio;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =ULTRASONIC_TIM12_2_SubPrio;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化NVIC寄存器

	InitQueue(&ultra_que2);
}



void ULTRASONIC_TIM8_Cap_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef 	 NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM8_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM8时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PC时钟		
	
	// TIM8 CH3 PC8
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_TIM8_3_Pin; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(ULTRASONIC_TIM8_3_Port,&GPIO_InitStructure); //初始化PC8
	GPIO_PinAFConfig(ULTRASONIC_TIM8_3_Port,ULTRASONIC_TIM8_3_PinSource,ULTRASONIC_TIM8_3_AF); //PC8复用为 定时器8
	
	// TIM8 CH4 PC9
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_TIM8_4_Pin;
	GPIO_Init(ULTRASONIC_TIM8_4_Port,&GPIO_InitStructure);
	GPIO_PinAFConfig(ULTRASONIC_TIM8_4_Port,ULTRASONIC_TIM8_4_PinSource,ULTRASONIC_TIM8_4_AF);// PC9复用为 定时器8
		
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(ULTRASONIC_TIM8,&TIM_TimeBaseStructure);

	//初始化TIM8 CH1输入捕获参数
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM8_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(ULTRASONIC_TIM8, &TIM8_ICInitStructure);
	
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM_ICInit(ULTRASONIC_TIM8, &TIM8_ICInitStructure);	
			
	TIM_ITConfig(ULTRASONIC_TIM8,TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4,ENABLE);//允许更新中断 ,允许CC3IE  CC4IE捕获中断

  TIM_Cmd(ULTRASONIC_TIM8,ENABLE ); 	//使能定时器8

  NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=ULTRASONIC_TIM8_34_PreptPrio;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =ULTRASONIC_TIM8_34_SubPrio;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化NVIC寄存器
	
	InitQueue(&ultra_que3);
	InitQueue(&ultra_que4);
}




// 超声波 TRIG IO 口
void ULTRASONIC_TRIG_Config(void)  
{  
  GPIO_InitTypeDef  GPIO_InitStructure; 
    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
   
  /* 1# 2# 3# 4#超声波 trig口 */  
  GPIO_InitStructure.GPIO_Pin   = ULTRASONIC_TRIG_Pin1|ULTRASONIC_TRIG_Pin2|ULTRASONIC_TRIG_Pin3|ULTRASONIC_TRIG_Pin4;  	
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;   
  GPIO_Init(ULTRASONIC_TRIG_PORT, &GPIO_InitStructure);   
} 


// 超声波避障
void ULTRA_OBSTAC_AVOID(void)
{
	static u8 ultra_left=0,ultra_mid=0,ultra_right=0;
	static u8 ultra_avoid_status = 0, count = 0;
	static u8 vel_indicate=0, omega_indicate=0;
	static float target_omega_before_acc_backup = 0.0f;
//	static float target_v_before_acc_backup = 0.0f;
	
	if(++count >29)
	{
		count = 0;
//		if(ultra_stop_status == 0 && acc_complete_status == 1)
		if(ultra_stop_status == 0)
		{
			// 用于避障旋转时判断方向
			target_omega_before_acc_backup = target_omega_before_acc;
//			target_v_before_acc_backup = target_v_before_acc;
		}
			
		// 滑动窗口保持最新的10个数据，减小超声波噪声影响
		ultra_dist_avg1 = process_que(ultrasonic_dist1, &ultra_que1);
		ultra_dist_avg2 = process_que(ultrasonic_dist2, &ultra_que2);
		ultra_dist_avg3 = process_que(ultrasonic_dist3, &ultra_que3);
		ultra_dist_avg4 = process_que(ultrasonic_dist4, &ultra_que4);
//		ultra_dist_avg1 = ultrasonic_dist1;
//		ultra_dist_avg2 = ultrasonic_dist2;
//		ultra_dist_avg3 = ultrasonic_dist3;
//		ultra_dist_avg4 = ultrasonic_dist4;
		//	printf("ultra_dist_avg1= %f\r\n",ultra_dist_avg1);
		//	printf("ultra_dist_avg2= %f\r\n",ultra_dist_avg2);
		//	printf("ultra_dist_avg3= %f\r\n",ultra_dist_avg3);
		//	printf("ultra_dist_avg4= %f\r\n\r\n",ultra_dist_avg4);
		
		if(ultra_dist_avg1 < 120)  ultra_left = 1;
		else                       ultra_left = 0;
		
		if(ultra_dist_avg2 <260 || ultra_dist_avg3 <260)  ultra_mid = 1;
		else                                              ultra_mid = 0;
		
		if(ultra_dist_avg4 < 120)  ultra_right = 1;
		else                       ultra_right = 0;		
		
		if(ultra_left == 1 || ultra_mid == 1 || ultra_right == 1)
		{
			// 1.先躲避障碍
			if(ultra_avoid_status == 0)
			{
				if(target_omega_before_acc > 0.1f)//机器人角速度大于0，逆时针旋转
				{
					omega_indicate=1;// 角速度不为0，角速度指示为1
					if(target_v_before_acc > 0.0f)//机器人线速度大于0
					{
						vel_indicate = 1;// 线速度不为0，线速度指示为1
						if(ultra_left == 1)// 左前方有障碍物
						{
							ultra_stop_status = 1;
							//acc_complete_status = 0;
							target_omega_before_acc = 0;//角速度为0		
							omega_indicate=0;								
						}
						
						if(ultra_mid == 1)// 前方有障碍物
						{
							ultra_stop_status = 1;
							//acc_complete_status = 0;
							target_v_before_acc = 0;//线速度为0
							vel_indicate = 0;
						}
						
						if(ultra_right == 1)// 右方有障碍物
						{
							ultra_stop_status = 0;
							//acc_complete_status = 1;//不限制速度							
						}
					}
					else//机器人线速度不大于0
					{
						ultra_stop_status = 0;
						//acc_complete_status = 1;//不限制速度
					}
				}
				else if(target_omega_before_acc < -0.1f)//机器人角速度小于0，顺时针旋转
				{
					omega_indicate=1;// 角速度不为0，角速度指示为1
					if(target_v_before_acc > 0.0f)//机器人线速度大于0
					{
						vel_indicate = 1;// 线速度不为0，线速度指示为1
						if(ultra_left == 1)// 左前方有障碍物
						{
							ultra_stop_status = 0;
							//acc_complete_status = 1;//不限制速度							
						}
						
						if(ultra_mid == 1)// 前方有障碍物
						{
							ultra_stop_status = 1;
							//acc_complete_status = 0;
							target_v_before_acc = 0;//线速度为0
							vel_indicate = 0;							
						}
						
						if(ultra_right == 1)// 右方有障碍物
						{
							ultra_stop_status = 1;
							//acc_complete_status = 0;
							target_omega_before_acc = 0;//角速度为0
							omega_indicate=0;
						}						
					}
					else//机器人线速度不大于0
					{
						ultra_stop_status = 0;
						//acc_complete_status = 1;//不限制速度					
					}						
				}
				else//机器人角速度等于0，不旋转
				{
					omega_indicate=0;// 角速度为0，角速度指示为0
					if(target_v_before_acc > 0.0f)//机器人线速度大于0
					{
						if(ultra_left == 1 || ultra_mid == 1 || ultra_right == 1)// 周围有障碍物
						{
							ultra_stop_status = 1;
							//acc_complete_status = 0;
							vel_indicate = 0;// 需要线速度为0，线速度指示为0						
						}				
					}
					else//机器人线速度不大于0
					{
						ultra_stop_status = 0;
						//acc_complete_status = 1;//不限制速度				
					}						
				}
				
				
				// 2.被超声波逼停后，开始绕障，一旦开始绕障，直到逃逸到完全空闲区域，否则不会回到第一步的避障状态
				// 这里不让机器人线速度和角速度同时为0，而是让相应标志变量为0，是为了避免继电器频繁抖动
				if(vel_indicate == 0 && omega_indicate==0 && ultra_stop_status == 1)
				{
					ultra_avoid_status = 1;
					//printf("开始绕障\r\n");
				}				
			}
			else
			{
				// 3.正在绕障
				if(ultra_left == 0 && ultra_mid == 0)//优先逆时针旋转
				{
					ultra_stop_status = 1;
					//acc_complete_status = 0;
					
					target_v_before_acc = 0.0f;// 逆时针原地旋转，寻找空闲空间
					target_omega_before_acc = 20.0f;						
				}
				else if(ultra_right == 0 && ultra_mid == 0)//其次顺时针旋转
				{
					ultra_stop_status = 1;
					//acc_complete_status = 0;
					
					target_v_before_acc = 0.0f;// 顺时针原地旋转，寻找空闲空间
					target_omega_before_acc = -20.0f;						
				}
				else
				{  //左右都有障碍物时，根据目标角速度方向旋转，如目标角速度为0，则逆时针旋转
					if(target_omega_before_acc_backup < 0)
					{
						ultra_stop_status = 1;
						//acc_complete_status = 0;
						
						target_v_before_acc = 0.0f;// 顺时针原地旋转，寻找空闲空间
						target_omega_before_acc = -20.0f;									
					}
					else
					{
						ultra_stop_status = 1;
						//acc_complete_status = 0;
						
						target_v_before_acc = 0.0f;// 逆时针原地旋转，寻找空闲空间
						target_omega_before_acc = 20.0f;									
					}					
				}
					//printf("正在旋转绕障\r\n");
			}
		}
		else
		{
			// 4.已到达空闲区域，结束绕障，重新接收速度指令
			ultra_stop_status = 0;//四个超声波都没探测到障碍物，继续接受目标速度
			//acc_complete_status = 1;
			
			ultra_avoid_status = 0;
		}
	}
}


void ULTRASONIC_START(void)
{
	static u8 count = 0;
	if(++count > 20)
	{
		count = 0;
		StartMeasureTIM11();// 第一路超声波触发信号
		StartMeasureTIM12();// 第二路超声波触发信号
		StartMeasureTIM8CH3();// 第三路超声波触发信号
		StartMeasureTIM8CH4();// 第四路超声波触发信号				
	}

}


// 1# 超声波触发函数
void StartMeasureTIM11(void)
{
	ultrasonic_dist1 = capture.TIM11CH1_DELTA_VAL * 17.0f;
	
  GPIO_SetBits(ULTRASONIC_TRIG_PORT,ULTRASONIC_TRIG_Pin1); 		 
  delay_us(20);		                    
  GPIO_ResetBits(ULTRASONIC_TRIG_PORT,ULTRASONIC_TRIG_Pin1);
	
	if(ultrasonic_dist1 < 0)
	{
		ultrasonic_dist1 = (capture.TIM11CH1_DELTA_VAL + capture.TIM11CH1_CURRENT_CNT) * 17.0f;
	}
	
	if(capture.TIM11CH1_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
	{
		capture.TIM11CH1_CAPTURE_STA=0;			     //开启下一次捕获
//		printf("1# distance: %f mm\r\n", ultrasonic_dist1);
	}
	
}


// 2# 超声波触发函数
void StartMeasureTIM12(void)
{
	
	ultrasonic_dist2 = capture.TIM12CH2_DELTA_VAL * 17.0f;
	
  GPIO_SetBits(ULTRASONIC_TRIG_PORT,ULTRASONIC_TRIG_Pin2); 		 
  delay_us(20);		                    
  GPIO_ResetBits(ULTRASONIC_TRIG_PORT,ULTRASONIC_TRIG_Pin2);
	
	// 如果 ultrasonic_dist2 小于0，说明是中断里面计数器清零导致的
	if(ultrasonic_dist2 < 0)
	{
		ultrasonic_dist2 = (capture.TIM12CH2_DELTA_VAL + capture.TIM12CH2_CURRENT_CNT) * 17.0f;
	}
	
	if(capture.TIM12CH2_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
	{
		capture.TIM12CH2_CAPTURE_STA=0;			     //开启下一次捕获
//		printf("2# distance: %f mm\r\n", ultrasonic_dist2);
	}
}


// 3# 超声波触发函数
void StartMeasureTIM8CH3(void)
{
	ultrasonic_dist3 = capture.TIM8CH3_DELTA_VAL * 17.0f;
	
  GPIO_SetBits(ULTRASONIC_TRIG_PORT,ULTRASONIC_TRIG_Pin3); 		 
  delay_us(20);		                    
  GPIO_ResetBits(ULTRASONIC_TRIG_PORT,ULTRASONIC_TRIG_Pin3);
	
	if(ultrasonic_dist3 < 0)
	{
		ultrasonic_dist3 = (capture.TIM8CH3_DELTA_VAL + capture.TIM8CH4_CURRENT_CNT) * 17.0f;
	}
	
	if(capture.TIM8CH3_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
	{
		capture.TIM8CH3_CAPTURE_STA=0;			     //开启下一次捕获
//		printf("3# distance: %f mm\r\n", ultrasonic_dist3);
	}
	
}


// 4# 超声波触发函数
void StartMeasureTIM8CH4(void)
{
	ultrasonic_dist4 = capture.TIM8CH4_DELTA_VAL * 17.0f;
	
  GPIO_SetBits(ULTRASONIC_TRIG_PORT,ULTRASONIC_TRIG_Pin4); 		 
  delay_us(20);	
	GPIO_ResetBits(ULTRASONIC_TRIG_PORT,ULTRASONIC_TRIG_Pin4);
	
	if(ultrasonic_dist4 < 0)
	{
		ultrasonic_dist4 = (capture.TIM8CH4_DELTA_VAL + capture.TIM8CH4_CURRENT_CNT) * 17.0f;
	}
	
	if(capture.TIM8CH4_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
	{
		capture.TIM8CH4_CAPTURE_STA=0;			     //开启下一次捕获
//		printf("4# distance: %f mm\r\n", ultrasonic_dist4);
//		printf("4# distance: %d mm\r\n", capture.CH4_DELTA_VAL);
	}
	
}



void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	// CH1 捕获中断
 	if((capture.TIM11CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if(TIM_GetITStatus(ULTRASONIC_TIM11, TIM_IT_Update) != RESET)//溢出
		{	     
			if(capture.TIM11CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((capture.TIM11CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					capture.TIM11CH1_CAPTURE_STA|=0X80;		//标记成功捕获了一次
					capture.TIM11CH1_CAPTURE_VAL=0XFFFF;
				}else capture.TIM11CH1_CAPTURE_STA++;
			}	 
		}
		
		if(TIM_GetITStatus(ULTRASONIC_TIM11, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		{	
			if(capture.TIM11CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				capture.TIM11CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  capture.TIM11CH1_CAPTURE_VAL=TIM_GetCapture1(ULTRASONIC_TIM11);//捕获下降沿时 TIM11->CNT 的值
	 			TIM_OC1PolarityConfig(ULTRASONIC_TIM11,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
				
				capture.TIM11CH1_DELTA_VAL = capture.TIM11CH1_CAPTURE_VAL - capture.TIM11CH1_CAPTURE_VAL_START;//获取CH1捕获高电平计数值
				
				capture.TIM11CH1_CURRENT_CNT = TIM_GetCapture1(ULTRASONIC_TIM11);// 记录清零之前TIM11->CNT的值
				TIM_SetCounter(ULTRASONIC_TIM11,0);// 定时器11 清零
				
			}else  								//还未开始,第一次捕获上升沿
			{
				capture.TIM11CH1_CAPTURE_STA=0;			//清空
				capture.TIM11CH1_CAPTURE_VAL=0;
				capture.TIM11CH1_CAPTURE_VAL_START = 0;
				capture.TIM11CH1_DELTA_VAL = 0;
				capture.TIM11CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				capture.TIM11CH1_CAPTURE_VAL_START=TIM_GetCapture1(ULTRASONIC_TIM11);// 捕获上升沿时 TIM11->CNT 的值
	 			TIM_OC1PolarityConfig(ULTRASONIC_TIM11,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		    
//			printf("CH1 中断\r\n");
		}    					   
 	}
	
	TIM_ClearITPendingBit(ULTRASONIC_TIM11, TIM_IT_CC1|TIM_IT_Update); //清除中断标志位	   
}



void TIM8_BRK_TIM12_IRQHandler(void)
{
	// CH1 捕获中断
 	if((capture.TIM12CH2_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if(TIM_GetITStatus(ULTRASONIC_TIM12, TIM_IT_Update) != RESET)//溢出
		{	     
			if(capture.TIM12CH2_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((capture.TIM12CH2_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					capture.TIM12CH2_CAPTURE_STA|=0X80;		//标记成功捕获了一次
					capture.TIM12CH2_CAPTURE_VAL=0XFFFF;
				}else capture.TIM12CH2_CAPTURE_STA++;
			}	 
		}
		
		if(TIM_GetITStatus(ULTRASONIC_TIM12, TIM_IT_CC2) != RESET)//捕获1发生捕获事件
		{	
			if(capture.TIM12CH2_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				capture.TIM12CH2_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  capture.TIM12CH2_CAPTURE_VAL=TIM_GetCapture2(ULTRASONIC_TIM12);//捕获下降沿时 TIM12->CNT 的值
	 			TIM_OC2PolarityConfig(ULTRASONIC_TIM12,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
				
				capture.TIM12CH2_DELTA_VAL = capture.TIM12CH2_CAPTURE_VAL - capture.TIM12CH2_CAPTURE_VAL_START;//获取CH2捕获高电平计数值
				
				capture.TIM12CH2_CURRENT_CNT = TIM_GetCapture2(ULTRASONIC_TIM12);// 记录清零之前TIM12->CNT的值
				TIM_SetCounter(ULTRASONIC_TIM12,0);// 定时器12 清零
				
			}else  								//还未开始,第一次捕获上升沿
			{
				capture.TIM12CH2_CAPTURE_STA=0;			//清空
				capture.TIM12CH2_CAPTURE_VAL=0;
				capture.TIM12CH2_CAPTURE_VAL_START = 0;
				capture.TIM12CH2_DELTA_VAL = 0;
				capture.TIM12CH2_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				capture.TIM12CH2_CAPTURE_VAL_START=TIM_GetCapture2(ULTRASONIC_TIM12);// 捕获上升沿时 TIM12->CNT 的值
	 			TIM_OC2PolarityConfig(ULTRASONIC_TIM12,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		    
//			printf("CH1 中断\r\n");
		} 					   
 	}
	
	TIM_ClearITPendingBit(ULTRASONIC_TIM12, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位
}



void TIM8_CC_IRQHandler(void)
{
	// CH3 捕获中断
	if((capture.TIM8CH3_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if(TIM_GetITStatus(ULTRASONIC_TIM8, TIM_IT_Update) != RESET)//溢出
		{	     
			if(capture.TIM8CH3_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((capture.TIM8CH3_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					capture.TIM8CH3_CAPTURE_STA|=0X80;		//标记成功捕获了一次
					capture.TIM8CH3_CAPTURE_VAL=0XFFFF;
				}else capture.TIM8CH3_CAPTURE_STA++;
			}	 
		}
		
		if(TIM_GetITStatus(ULTRASONIC_TIM8, TIM_IT_CC3) != RESET)//捕获1发生捕获事件
		{	
			if(capture.TIM8CH3_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				capture.TIM8CH3_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  capture.TIM8CH3_CAPTURE_VAL=TIM_GetCapture3(ULTRASONIC_TIM8);//捕获下降沿时 TIM8->CNT 的值
	 			TIM_OC3PolarityConfig(ULTRASONIC_TIM8,TIM_ICPolarity_Rising); //CC3P=0 设置为上升沿捕获
				
				capture.TIM8CH3_DELTA_VAL = capture.TIM8CH3_CAPTURE_VAL - capture.TIM8CH3_CAPTURE_VAL_START;//获取CH3捕获高电平计数值
				
				
			}else  								//还未开始,第一次捕获上升沿
			{
				capture.TIM8CH3_CAPTURE_STA=0;			//清空
				capture.TIM8CH3_CAPTURE_VAL=0;
				capture.TIM8CH3_CAPTURE_VAL_START = 0;
				capture.TIM8CH3_DELTA_VAL = 0;
				capture.TIM8CH3_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				capture.TIM8CH3_CAPTURE_VAL_START=TIM_GetCapture3(ULTRASONIC_TIM8);// 捕获上升沿时 TIM8->CNT 的值
	 			TIM_OC3PolarityConfig(ULTRASONIC_TIM8,TIM_ICPolarity_Falling);		//CC3P=1 设置为下降沿捕获
			}	
//			printf("CH3 中断\r\n");	    
		}				   
 	}
 	    
			    
	
	
	// CH4 捕获中断
	if((capture.TIM8CH4_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if(TIM_GetITStatus(ULTRASONIC_TIM8, TIM_IT_Update) != RESET)//溢出
		{	     
			if(capture.TIM8CH4_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((capture.TIM8CH4_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					capture.TIM8CH4_CAPTURE_STA|=0X80;		//标记成功捕获了一次
					capture.TIM8CH4_CAPTURE_VAL=0XFFFF;
				}else capture.TIM8CH4_CAPTURE_STA++;
			}	 
		}
		
		if(TIM_GetITStatus(ULTRASONIC_TIM8, TIM_IT_CC4) != RESET)//捕获1发生捕获事件
		{	
			if(capture.TIM8CH4_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				capture.TIM8CH4_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  capture.TIM8CH4_CAPTURE_VAL=TIM_GetCapture4(ULTRASONIC_TIM8);//捕获下降沿时 TIM8->CNT 的值
	 			TIM_OC4PolarityConfig(ULTRASONIC_TIM8,TIM_ICPolarity_Rising); //CC4P=0 设置为上升沿捕获
				
				capture.TIM8CH4_DELTA_VAL = capture.TIM8CH4_CAPTURE_VAL - capture.TIM8CH4_CAPTURE_VAL_START;//获取CH4捕获高电平计数值
				
				capture.TIM8CH4_CURRENT_CNT = TIM_GetCapture4(ULTRASONIC_TIM8);// 记录清零之前TIM8->CNT的值
				TIM_SetCounter(ULTRASONIC_TIM8,0);// 定时器8 清零

				
			}else  								//还未开始,第一次捕获上升沿
			{
				capture.TIM8CH4_CAPTURE_STA=0;			//清空
				capture.TIM8CH4_CAPTURE_VAL=0;
				capture.TIM8CH4_CAPTURE_VAL_START = 0;
				capture.TIM8CH4_DELTA_VAL = 0;
				capture.TIM8CH4_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				capture.TIM8CH4_CAPTURE_VAL_START=TIM_GetCapture4(ULTRASONIC_TIM8);// 捕获上升沿时 TIM8->CNT 的值
	 			TIM_OC4PolarityConfig(ULTRASONIC_TIM8,TIM_ICPolarity_Falling);		//CC4P=1 设置为下降沿捕获
//				printf("CH4 上升沿\r\n");
			}		    
//			printf("CH4 中断\r\n");
		}	  	    					   
 	}
	TIM_ClearITPendingBit(ULTRASONIC_TIM8, TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update); //清除中断标志位	   
}

