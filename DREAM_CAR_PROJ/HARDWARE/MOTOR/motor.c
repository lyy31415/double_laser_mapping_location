#include "motor.h"

int Moto1=0;
int Moto2=0;

void DreamCar_Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //ʹ��PB�˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = LEFT_MOTOR_DIR1_Pin|LEFT_MOTOR_DIR2_Pin;	//�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(LEFT_MOTOR_DIR1_Port, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB 
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //ʹ��PB�˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = RIGHT_MOTOR_DIR1_Pin|RIGHT_MOTOR_DIR2_Pin;	//�˿�����
	GPIO_Init(RIGHT_MOTOR_DIR1_Port, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB
}
void DreamCar_PWM_Init(u16 arr,u16 psc)
{		 		
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  DreamCar_Motor_Init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);// ʹ�ܶ�ʱ��ʱ��
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);  //ʹ��GPIO����ʱ��
	
	
   //���ø�����Ϊ�����������,���TIM9 CH1 CH2��PWM���岨��
	GPIO_InitStructure.GPIO_Pin = LEFT_MOTOR_SPEED_Pin|RIGHT_MOTOR_SPEED_Pin; //TIM9_CH1 //TIM9_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LEFT_MOTOR_SPEED_Port, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(LEFT_MOTOR_SPEED_Port,LEFT_MOTOR_SPEED_PinSource,LEFT_MOTOR_SPEED_AF); //GPIOE5����ΪTIM9
	GPIO_PinAFConfig(RIGHT_MOTOR_SPEED_Port,RIGHT_MOTOR_SPEED_PinSource,RIGHT_MOTOR_SPEED_AF);//GPIOE6����ΪTIM9
	
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(LEFT_MOTOR_TIM, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(LEFT_MOTOR_TIM, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC2Init(RIGHT_MOTOR_TIM, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

//  TIM_CtrlPWMOutputs(LEFT_MOTOR_TIM,ENABLE);	//MOE �����ʹ��	

	TIM_OC1PreloadConfig(LEFT_MOTOR_TIM, TIM_OCPreload_Enable);  //CH1Ԥװ��ʹ��	 
	TIM_OC2PreloadConfig(RIGHT_MOTOR_TIM, TIM_OCPreload_Enable);  //CH2Ԥװ��ʹ��	 
	
	TIM_ARRPreloadConfig(LEFT_MOTOR_TIM, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	
	TIM_Cmd(LEFT_MOTOR_TIM, ENABLE);  //ʹ��TIM9
 
} 



void Set_Pwm(int moto1,int moto2)
{
//	    int siqu=PWM_Base;//������ѹ���������������������ͬ������ռ�ձ���1000/8400ʱ������ſ�ʼת�����ҵ��ռ�ձ���1200/8400ʱ������ſ�ʼת����
			if(moto2<0)			BIN1=1,			BIN2=0;
			else 	          BIN1=0,			BIN2=1;
			PWMB=myabs(moto2);
		  if(moto1<0)	AIN1=0,			AIN2=1;
			else        AIN1=1,			AIN2=0;
			PWMA=myabs(moto1);	
}


void Xianfu_Pwm(void)
{	
	  int Amplitude=PWM_Max;    // PWMռ�ձ�������8400
    if(Moto1<-Amplitude) Moto1=-Amplitude;	
		if(Moto1>Amplitude)  Moto1=Amplitude;	
	  if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;		
}


int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}


void motor_setup(void)
{
	static u8 ss=0;
//	static u32 tt=0;
	
	Xianfu_Pwm();//����PWM����ռ�ձȣ��ﲻ��100%ռ�ձ�
	
		if(key==0)// Ϊ�˲�������
		{
			ss = 1;
		}
		
		if((key==98) && (ss==1)) // ����ң���� UP ����
		{
			beep_on(1,100);
			ss = 0;
		
			target_v+=0.1f;// ��/��
		}
		
		
		if((key==168)  && (ss==1)) // ����ң���� DOWN ����
		{
			beep_on(1,100);
			ss = 0;
		
			target_v-=0.1f;// ��/��
		}
		
		if((key==34) && (ss==1)) // ����ң���� LEFT ����
		{
			beep_on(1,100);
			ss = 0;
			
			if(motor_pid_debug == 0)
			{
				target_omega+=30.0f;// ��/��
			}
			else if(motor_pid_debug == 1)
			{
				enc_omega[1] += 3.0f;//���ֽ��ٶȼ� 3 ����
				enc_omega[0] += 3.0f;//���ֽ��ٶȼ� 3 ����
			}
		}
		
		
		
//		if( key == 176 )// ���ⰴ��3 ��������ת90��
//		{
//			beep_on(1,100);
//			Moto1-=800;//ÿ������500
//			Moto2+=800;//ÿ�μ���500
//		}
		
		if((key==194) && (ss==1)) // ����ң���� RIGHT ����
		{
			beep_on(1,100);
			ss = 0;
			
			if(motor_pid_debug == 0)
			{
				target_omega-=30.0f;// ��/��
			}
			else if(motor_pid_debug == 1)
			{
				enc_omega[1] -= 3.0f;//���ֽ��ٶȼ� 3 ����
				enc_omega[0] -= 3.0f;//���ֽ��ٶȼ� 3 ����
			}
		}
//		printf("���ֽ��ٶ��趨ֵenc_omega[1]��%f  rad/s\r\n",enc_omega[1]);
//		printf("���ֽ��ٶ��趨ֵenc_omega[0]��%f  rad/s\r\n",enc_omega[0]);

		
//		if( key == 48 )//���ⰴ��4��������ת90��
//		{
//			beep_on(1,100);
//			Moto1+=800;//ÿ������1500
//			Moto2-=800;//ÿ�μ���1500
//		}
		
		if(key==66) // ����ң���� 0 ����
		{
			beep_on(1,100);
			
			Moto1=0;//����
			Moto2=0;//����
			
//			LEFT_ENCODER_TIM->CNT=0;//��������������
//			RIGHT_ENCODER_TIM->CNT=0;
		}
		
//		if(key==104) // ����ң���� 1 ����
//		{
//			beep_on(1,100);
//			
//			Moto1=1500;
//			Moto2=1500;
//		}
		
//		if(key==152) // ����ң���� 2 ����
//		{
//			BEEP=1;//����������
//			delay_ms(100);
//      BEEP=0;//�������ر�
//			
//			Moto1=-1500;
//			Moto2=-1500;
//		}
		
		if(key==2) // ����ң���� ��ͣ ����
		{
			beep_on(1,100);
			
			Moto1=0;//����
			Moto2=0;//����
		}
		
//		printf("Moto1:%d\r\n",Moto1);
//		printf("Moto2:%d\r\n",Moto2);
		
		Set_Pwm(Moto1,Moto2);//����ռ�ձ�
//		Set_Pwm(0,Moto2);
//		Moto2-=50;
//		printf("PWMB[%d] = %d\r\n",tt++,PWMB);
}
