#include "ultrasonic.h"

ultrasonic_tim_cap capture;
float ultrasonic_dist1, ultrasonic_dist2, ultrasonic_dist3, ultrasonic_dist4;//������������
u8 ultra_stop_status;
static float ultra_dist_avg1=0.0f, ultra_dist_avg2=0.0f, ultra_dist_avg3=0.0f, ultra_dist_avg4=0.0f;
static SqQueue ultra_que1, ultra_que2, ultra_que3, ultra_que4;

// �������ö�ʱ�� TIM8 CH3 PC8 CH4 PC9/TIM11 CH1 PF7/TIM12 CH2 PB15 
//��ʱ��ͨ�����벶������
//arr���Զ���װֵ(TIM8 TIM12 TIM11��16λ��!!)
//psc��ʱ��Ԥ��Ƶ��
void ULTRASONIC_TIM11_Cap_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef 	 NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM11_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);  	//TIM11ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//ʹ��PFʱ��	
	
	// TIM11 CH1 PF7
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_TIM11_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(ULTRASONIC_TIM11_Port,&GPIO_InitStructure); //��ʼ��PF7
	GPIO_PinAFConfig(ULTRASONIC_TIM11_Port,ULTRASONIC_TIM11_PinSource,ULTRASONIC_TIM11_AF); // PF7����Ϊ ��ʱ��11
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(ULTRASONIC_TIM11,&TIM_TimeBaseStructure);

	//��ʼ��TIM11 CH1���벶�����
	TIM11_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM11_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM11_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM11_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM11_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(ULTRASONIC_TIM11, &TIM11_ICInitStructure);	
		
	TIM_ITConfig(ULTRASONIC_TIM11,TIM_IT_Update|TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�
	
	TIM_Cmd(ULTRASONIC_TIM11,ENABLE ); 	//ʹ�ܶ�ʱ��11

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM11_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=ULTRASONIC_TIM11_1_PreptPrio;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =ULTRASONIC_TIM11_1_SubPrio;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

	InitQueue(&ultra_que1);
}




void ULTRASONIC_TIM12_Cap_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef 	 NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM12_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  	//TIM12ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��PBʱ��	
	
	// TIM12 CH2 PB15
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_TIM12_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(ULTRASONIC_TIM12_Port,&GPIO_InitStructure);
	GPIO_PinAFConfig(ULTRASONIC_TIM12_Port,ULTRASONIC_TIM12_PinSource,ULTRASONIC_TIM12_AF);// PB15����Ϊ ��ʱ��12

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(ULTRASONIC_TIM12,&TIM_TimeBaseStructure);

	//��ʼ��TIM12 CH2���벶�����
	TIM12_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM12_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM12_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM12_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM12_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(ULTRASONIC_TIM12, &TIM12_ICInitStructure);	
		
	TIM_ITConfig(ULTRASONIC_TIM12,TIM_IT_Update|TIM_IT_CC2,ENABLE);//��������ж� ,����CC1IE�����ж�
	
	TIM_Cmd(ULTRASONIC_TIM12,ENABLE ); 	//ʹ�ܶ�ʱ��12

  NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=ULTRASONIC_TIM12_2_PreptPrio;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =ULTRASONIC_TIM12_2_SubPrio;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

	InitQueue(&ultra_que2);
}



void ULTRASONIC_TIM8_Cap_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef 	 NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM8_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM8ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PCʱ��		
	
	// TIM8 CH3 PC8
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_TIM8_3_Pin; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(ULTRASONIC_TIM8_3_Port,&GPIO_InitStructure); //��ʼ��PC8
	GPIO_PinAFConfig(ULTRASONIC_TIM8_3_Port,ULTRASONIC_TIM8_3_PinSource,ULTRASONIC_TIM8_3_AF); //PC8����Ϊ ��ʱ��8
	
	// TIM8 CH4 PC9
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_TIM8_4_Pin;
	GPIO_Init(ULTRASONIC_TIM8_4_Port,&GPIO_InitStructure);
	GPIO_PinAFConfig(ULTRASONIC_TIM8_4_Port,ULTRASONIC_TIM8_4_PinSource,ULTRASONIC_TIM8_4_AF);// PC9����Ϊ ��ʱ��8
		
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(ULTRASONIC_TIM8,&TIM_TimeBaseStructure);

	//��ʼ��TIM8 CH1���벶�����
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM8_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(ULTRASONIC_TIM8, &TIM8_ICInitStructure);
	
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM_ICInit(ULTRASONIC_TIM8, &TIM8_ICInitStructure);	
			
	TIM_ITConfig(ULTRASONIC_TIM8,TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4,ENABLE);//��������ж� ,����CC3IE  CC4IE�����ж�

  TIM_Cmd(ULTRASONIC_TIM8,ENABLE ); 	//ʹ�ܶ�ʱ��8

  NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=ULTRASONIC_TIM8_34_PreptPrio;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =ULTRASONIC_TIM8_34_SubPrio;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���
	
	InitQueue(&ultra_que3);
	InitQueue(&ultra_que4);
}




// ������ TRIG IO ��
void ULTRASONIC_TRIG_Config(void)  
{  
  GPIO_InitTypeDef  GPIO_InitStructure; 
    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
   
  /* 1# 2# 3# 4#������ trig�� */  
  GPIO_InitStructure.GPIO_Pin   = ULTRASONIC_TRIG_Pin1|ULTRASONIC_TRIG_Pin2|ULTRASONIC_TRIG_Pin3|ULTRASONIC_TRIG_Pin4;  	
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;   
  GPIO_Init(ULTRASONIC_TRIG_PORT, &GPIO_InitStructure);   
} 


// ����������
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
			// ���ڱ�����תʱ�жϷ���
			target_omega_before_acc_backup = target_omega_before_acc;
//			target_v_before_acc_backup = target_v_before_acc;
		}
			
		// �������ڱ������µ�10�����ݣ���С����������Ӱ��
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
			// 1.�ȶ���ϰ�
			if(ultra_avoid_status == 0)
			{
				if(target_omega_before_acc > 0.1f)//�����˽��ٶȴ���0����ʱ����ת
				{
					omega_indicate=1;// ���ٶȲ�Ϊ0�����ٶ�ָʾΪ1
					if(target_v_before_acc > 0.0f)//���������ٶȴ���0
					{
						vel_indicate = 1;// ���ٶȲ�Ϊ0�����ٶ�ָʾΪ1
						if(ultra_left == 1)// ��ǰ�����ϰ���
						{
							ultra_stop_status = 1;
							//acc_complete_status = 0;
							target_omega_before_acc = 0;//���ٶ�Ϊ0		
							omega_indicate=0;								
						}
						
						if(ultra_mid == 1)// ǰ�����ϰ���
						{
							ultra_stop_status = 1;
							//acc_complete_status = 0;
							target_v_before_acc = 0;//���ٶ�Ϊ0
							vel_indicate = 0;
						}
						
						if(ultra_right == 1)// �ҷ����ϰ���
						{
							ultra_stop_status = 0;
							//acc_complete_status = 1;//�������ٶ�							
						}
					}
					else//���������ٶȲ�����0
					{
						ultra_stop_status = 0;
						//acc_complete_status = 1;//�������ٶ�
					}
				}
				else if(target_omega_before_acc < -0.1f)//�����˽��ٶ�С��0��˳ʱ����ת
				{
					omega_indicate=1;// ���ٶȲ�Ϊ0�����ٶ�ָʾΪ1
					if(target_v_before_acc > 0.0f)//���������ٶȴ���0
					{
						vel_indicate = 1;// ���ٶȲ�Ϊ0�����ٶ�ָʾΪ1
						if(ultra_left == 1)// ��ǰ�����ϰ���
						{
							ultra_stop_status = 0;
							//acc_complete_status = 1;//�������ٶ�							
						}
						
						if(ultra_mid == 1)// ǰ�����ϰ���
						{
							ultra_stop_status = 1;
							//acc_complete_status = 0;
							target_v_before_acc = 0;//���ٶ�Ϊ0
							vel_indicate = 0;							
						}
						
						if(ultra_right == 1)// �ҷ����ϰ���
						{
							ultra_stop_status = 1;
							//acc_complete_status = 0;
							target_omega_before_acc = 0;//���ٶ�Ϊ0
							omega_indicate=0;
						}						
					}
					else//���������ٶȲ�����0
					{
						ultra_stop_status = 0;
						//acc_complete_status = 1;//�������ٶ�					
					}						
				}
				else//�����˽��ٶȵ���0������ת
				{
					omega_indicate=0;// ���ٶ�Ϊ0�����ٶ�ָʾΪ0
					if(target_v_before_acc > 0.0f)//���������ٶȴ���0
					{
						if(ultra_left == 1 || ultra_mid == 1 || ultra_right == 1)// ��Χ���ϰ���
						{
							ultra_stop_status = 1;
							//acc_complete_status = 0;
							vel_indicate = 0;// ��Ҫ���ٶ�Ϊ0�����ٶ�ָʾΪ0						
						}				
					}
					else//���������ٶȲ�����0
					{
						ultra_stop_status = 0;
						//acc_complete_status = 1;//�������ٶ�				
					}						
				}
				
				
				// 2.����������ͣ�󣬿�ʼ���ϣ�һ����ʼ���ϣ�ֱ�����ݵ���ȫ�������򣬷��򲻻�ص���һ���ı���״̬
				// ���ﲻ�û��������ٶȺͽ��ٶ�ͬʱΪ0����������Ӧ��־����Ϊ0����Ϊ�˱���̵���Ƶ������
				if(vel_indicate == 0 && omega_indicate==0 && ultra_stop_status == 1)
				{
					ultra_avoid_status = 1;
					//printf("��ʼ����\r\n");
				}				
			}
			else
			{
				// 3.��������
				if(ultra_left == 0 && ultra_mid == 0)//������ʱ����ת
				{
					ultra_stop_status = 1;
					//acc_complete_status = 0;
					
					target_v_before_acc = 0.0f;// ��ʱ��ԭ����ת��Ѱ�ҿ��пռ�
					target_omega_before_acc = 20.0f;						
				}
				else if(ultra_right == 0 && ultra_mid == 0)//���˳ʱ����ת
				{
					ultra_stop_status = 1;
					//acc_complete_status = 0;
					
					target_v_before_acc = 0.0f;// ˳ʱ��ԭ����ת��Ѱ�ҿ��пռ�
					target_omega_before_acc = -20.0f;						
				}
				else
				{  //���Ҷ����ϰ���ʱ������Ŀ����ٶȷ�����ת����Ŀ����ٶ�Ϊ0������ʱ����ת
					if(target_omega_before_acc_backup < 0)
					{
						ultra_stop_status = 1;
						//acc_complete_status = 0;
						
						target_v_before_acc = 0.0f;// ˳ʱ��ԭ����ת��Ѱ�ҿ��пռ�
						target_omega_before_acc = -20.0f;									
					}
					else
					{
						ultra_stop_status = 1;
						//acc_complete_status = 0;
						
						target_v_before_acc = 0.0f;// ��ʱ��ԭ����ת��Ѱ�ҿ��пռ�
						target_omega_before_acc = 20.0f;									
					}					
				}
					//printf("������ת����\r\n");
			}
		}
		else
		{
			// 4.�ѵ���������򣬽������ϣ����½����ٶ�ָ��
			ultra_stop_status = 0;//�ĸ���������û̽�⵽�ϰ����������Ŀ���ٶ�
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
		StartMeasureTIM11();// ��һ·�����������ź�
		StartMeasureTIM12();// �ڶ�·�����������ź�
		StartMeasureTIM8CH3();// ����·�����������ź�
		StartMeasureTIM8CH4();// ����·�����������ź�				
	}

}


// 1# ��������������
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
	
	if(capture.TIM11CH1_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
	{
		capture.TIM11CH1_CAPTURE_STA=0;			     //������һ�β���
//		printf("1# distance: %f mm\r\n", ultrasonic_dist1);
	}
	
}


// 2# ��������������
void StartMeasureTIM12(void)
{
	
	ultrasonic_dist2 = capture.TIM12CH2_DELTA_VAL * 17.0f;
	
  GPIO_SetBits(ULTRASONIC_TRIG_PORT,ULTRASONIC_TRIG_Pin2); 		 
  delay_us(20);		                    
  GPIO_ResetBits(ULTRASONIC_TRIG_PORT,ULTRASONIC_TRIG_Pin2);
	
	// ��� ultrasonic_dist2 С��0��˵�����ж�������������㵼�µ�
	if(ultrasonic_dist2 < 0)
	{
		ultrasonic_dist2 = (capture.TIM12CH2_DELTA_VAL + capture.TIM12CH2_CURRENT_CNT) * 17.0f;
	}
	
	if(capture.TIM12CH2_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
	{
		capture.TIM12CH2_CAPTURE_STA=0;			     //������һ�β���
//		printf("2# distance: %f mm\r\n", ultrasonic_dist2);
	}
}


// 3# ��������������
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
	
	if(capture.TIM8CH3_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
	{
		capture.TIM8CH3_CAPTURE_STA=0;			     //������һ�β���
//		printf("3# distance: %f mm\r\n", ultrasonic_dist3);
	}
	
}


// 4# ��������������
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
	
	if(capture.TIM8CH4_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
	{
		capture.TIM8CH4_CAPTURE_STA=0;			     //������һ�β���
//		printf("4# distance: %f mm\r\n", ultrasonic_dist4);
//		printf("4# distance: %d mm\r\n", capture.CH4_DELTA_VAL);
	}
	
}



void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	// CH1 �����ж�
 	if((capture.TIM11CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(ULTRASONIC_TIM11, TIM_IT_Update) != RESET)//���
		{	     
			if(capture.TIM11CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((capture.TIM11CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					capture.TIM11CH1_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
					capture.TIM11CH1_CAPTURE_VAL=0XFFFF;
				}else capture.TIM11CH1_CAPTURE_STA++;
			}	 
		}
		
		if(TIM_GetITStatus(ULTRASONIC_TIM11, TIM_IT_CC1) != RESET)//����1���������¼�
		{	
			if(capture.TIM11CH1_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				capture.TIM11CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  capture.TIM11CH1_CAPTURE_VAL=TIM_GetCapture1(ULTRASONIC_TIM11);//�����½���ʱ TIM11->CNT ��ֵ
	 			TIM_OC1PolarityConfig(ULTRASONIC_TIM11,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
				
				capture.TIM11CH1_DELTA_VAL = capture.TIM11CH1_CAPTURE_VAL - capture.TIM11CH1_CAPTURE_VAL_START;//��ȡCH1����ߵ�ƽ����ֵ
				
				capture.TIM11CH1_CURRENT_CNT = TIM_GetCapture1(ULTRASONIC_TIM11);// ��¼����֮ǰTIM11->CNT��ֵ
				TIM_SetCounter(ULTRASONIC_TIM11,0);// ��ʱ��11 ����
				
			}else  								//��δ��ʼ,��һ�β���������
			{
				capture.TIM11CH1_CAPTURE_STA=0;			//���
				capture.TIM11CH1_CAPTURE_VAL=0;
				capture.TIM11CH1_CAPTURE_VAL_START = 0;
				capture.TIM11CH1_DELTA_VAL = 0;
				capture.TIM11CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
				capture.TIM11CH1_CAPTURE_VAL_START=TIM_GetCapture1(ULTRASONIC_TIM11);// ����������ʱ TIM11->CNT ��ֵ
	 			TIM_OC1PolarityConfig(ULTRASONIC_TIM11,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
			}		    
//			printf("CH1 �ж�\r\n");
		}    					   
 	}
	
	TIM_ClearITPendingBit(ULTRASONIC_TIM11, TIM_IT_CC1|TIM_IT_Update); //����жϱ�־λ	   
}



void TIM8_BRK_TIM12_IRQHandler(void)
{
	// CH1 �����ж�
 	if((capture.TIM12CH2_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(ULTRASONIC_TIM12, TIM_IT_Update) != RESET)//���
		{	     
			if(capture.TIM12CH2_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((capture.TIM12CH2_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					capture.TIM12CH2_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
					capture.TIM12CH2_CAPTURE_VAL=0XFFFF;
				}else capture.TIM12CH2_CAPTURE_STA++;
			}	 
		}
		
		if(TIM_GetITStatus(ULTRASONIC_TIM12, TIM_IT_CC2) != RESET)//����1���������¼�
		{	
			if(capture.TIM12CH2_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				capture.TIM12CH2_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  capture.TIM12CH2_CAPTURE_VAL=TIM_GetCapture2(ULTRASONIC_TIM12);//�����½���ʱ TIM12->CNT ��ֵ
	 			TIM_OC2PolarityConfig(ULTRASONIC_TIM12,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
				
				capture.TIM12CH2_DELTA_VAL = capture.TIM12CH2_CAPTURE_VAL - capture.TIM12CH2_CAPTURE_VAL_START;//��ȡCH2����ߵ�ƽ����ֵ
				
				capture.TIM12CH2_CURRENT_CNT = TIM_GetCapture2(ULTRASONIC_TIM12);// ��¼����֮ǰTIM12->CNT��ֵ
				TIM_SetCounter(ULTRASONIC_TIM12,0);// ��ʱ��12 ����
				
			}else  								//��δ��ʼ,��һ�β���������
			{
				capture.TIM12CH2_CAPTURE_STA=0;			//���
				capture.TIM12CH2_CAPTURE_VAL=0;
				capture.TIM12CH2_CAPTURE_VAL_START = 0;
				capture.TIM12CH2_DELTA_VAL = 0;
				capture.TIM12CH2_CAPTURE_STA|=0X40;		//��ǲ�����������
				capture.TIM12CH2_CAPTURE_VAL_START=TIM_GetCapture2(ULTRASONIC_TIM12);// ����������ʱ TIM12->CNT ��ֵ
	 			TIM_OC2PolarityConfig(ULTRASONIC_TIM12,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
			}		    
//			printf("CH1 �ж�\r\n");
		} 					   
 	}
	
	TIM_ClearITPendingBit(ULTRASONIC_TIM12, TIM_IT_CC2|TIM_IT_Update); //����жϱ�־λ
}



void TIM8_CC_IRQHandler(void)
{
	// CH3 �����ж�
	if((capture.TIM8CH3_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(ULTRASONIC_TIM8, TIM_IT_Update) != RESET)//���
		{	     
			if(capture.TIM8CH3_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((capture.TIM8CH3_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					capture.TIM8CH3_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
					capture.TIM8CH3_CAPTURE_VAL=0XFFFF;
				}else capture.TIM8CH3_CAPTURE_STA++;
			}	 
		}
		
		if(TIM_GetITStatus(ULTRASONIC_TIM8, TIM_IT_CC3) != RESET)//����1���������¼�
		{	
			if(capture.TIM8CH3_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				capture.TIM8CH3_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  capture.TIM8CH3_CAPTURE_VAL=TIM_GetCapture3(ULTRASONIC_TIM8);//�����½���ʱ TIM8->CNT ��ֵ
	 			TIM_OC3PolarityConfig(ULTRASONIC_TIM8,TIM_ICPolarity_Rising); //CC3P=0 ����Ϊ�����ز���
				
				capture.TIM8CH3_DELTA_VAL = capture.TIM8CH3_CAPTURE_VAL - capture.TIM8CH3_CAPTURE_VAL_START;//��ȡCH3����ߵ�ƽ����ֵ
				
				
			}else  								//��δ��ʼ,��һ�β���������
			{
				capture.TIM8CH3_CAPTURE_STA=0;			//���
				capture.TIM8CH3_CAPTURE_VAL=0;
				capture.TIM8CH3_CAPTURE_VAL_START = 0;
				capture.TIM8CH3_DELTA_VAL = 0;
				capture.TIM8CH3_CAPTURE_STA|=0X40;		//��ǲ�����������
				capture.TIM8CH3_CAPTURE_VAL_START=TIM_GetCapture3(ULTRASONIC_TIM8);// ����������ʱ TIM8->CNT ��ֵ
	 			TIM_OC3PolarityConfig(ULTRASONIC_TIM8,TIM_ICPolarity_Falling);		//CC3P=1 ����Ϊ�½��ز���
			}	
//			printf("CH3 �ж�\r\n");	    
		}				   
 	}
 	    
			    
	
	
	// CH4 �����ж�
	if((capture.TIM8CH4_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(ULTRASONIC_TIM8, TIM_IT_Update) != RESET)//���
		{	     
			if(capture.TIM8CH4_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((capture.TIM8CH4_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					capture.TIM8CH4_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
					capture.TIM8CH4_CAPTURE_VAL=0XFFFF;
				}else capture.TIM8CH4_CAPTURE_STA++;
			}	 
		}
		
		if(TIM_GetITStatus(ULTRASONIC_TIM8, TIM_IT_CC4) != RESET)//����1���������¼�
		{	
			if(capture.TIM8CH4_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				capture.TIM8CH4_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  capture.TIM8CH4_CAPTURE_VAL=TIM_GetCapture4(ULTRASONIC_TIM8);//�����½���ʱ TIM8->CNT ��ֵ
	 			TIM_OC4PolarityConfig(ULTRASONIC_TIM8,TIM_ICPolarity_Rising); //CC4P=0 ����Ϊ�����ز���
				
				capture.TIM8CH4_DELTA_VAL = capture.TIM8CH4_CAPTURE_VAL - capture.TIM8CH4_CAPTURE_VAL_START;//��ȡCH4����ߵ�ƽ����ֵ
				
				capture.TIM8CH4_CURRENT_CNT = TIM_GetCapture4(ULTRASONIC_TIM8);// ��¼����֮ǰTIM8->CNT��ֵ
				TIM_SetCounter(ULTRASONIC_TIM8,0);// ��ʱ��8 ����

				
			}else  								//��δ��ʼ,��һ�β���������
			{
				capture.TIM8CH4_CAPTURE_STA=0;			//���
				capture.TIM8CH4_CAPTURE_VAL=0;
				capture.TIM8CH4_CAPTURE_VAL_START = 0;
				capture.TIM8CH4_DELTA_VAL = 0;
				capture.TIM8CH4_CAPTURE_STA|=0X40;		//��ǲ�����������
				capture.TIM8CH4_CAPTURE_VAL_START=TIM_GetCapture4(ULTRASONIC_TIM8);// ����������ʱ TIM8->CNT ��ֵ
	 			TIM_OC4PolarityConfig(ULTRASONIC_TIM8,TIM_ICPolarity_Falling);		//CC4P=1 ����Ϊ�½��ز���
//				printf("CH4 ������\r\n");
			}		    
//			printf("CH4 �ж�\r\n");
		}	  	    					   
 	}
	TIM_ClearITPendingBit(ULTRASONIC_TIM8, TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update); //����жϱ�־λ	   
}

