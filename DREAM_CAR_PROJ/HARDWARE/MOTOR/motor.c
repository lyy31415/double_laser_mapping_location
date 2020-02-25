#include "motor.h"

int Moto1=0;
int Moto2=0;

void DreamCar_Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //使能PB端口时钟
  GPIO_InitStructure.GPIO_Pin = LEFT_MOTOR_DIR1_Pin|LEFT_MOTOR_DIR2_Pin;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(LEFT_MOTOR_DIR1_Port, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB 
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //使能PB端口时钟
  GPIO_InitStructure.GPIO_Pin = RIGHT_MOTOR_DIR1_Pin|RIGHT_MOTOR_DIR2_Pin;	//端口配置
	GPIO_Init(RIGHT_MOTOR_DIR1_Port, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB
}
void DreamCar_PWM_Init(u16 arr,u16 psc)
{		 		
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  DreamCar_Motor_Init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);// 使能定时器时钟
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);  //使能GPIO外设时钟
	
	
   //设置该引脚为复用输出功能,输出TIM9 CH1 CH2的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = LEFT_MOTOR_SPEED_Pin|RIGHT_MOTOR_SPEED_Pin; //TIM9_CH1 //TIM9_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LEFT_MOTOR_SPEED_Port, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(LEFT_MOTOR_SPEED_Port,LEFT_MOTOR_SPEED_PinSource,LEFT_MOTOR_SPEED_AF); //GPIOE5复用为TIM9
	GPIO_PinAFConfig(RIGHT_MOTOR_SPEED_Port,RIGHT_MOTOR_SPEED_PinSource,RIGHT_MOTOR_SPEED_AF);//GPIOE6复用为TIM9
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(LEFT_MOTOR_TIM, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
	TIM_OC1Init(LEFT_MOTOR_TIM, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2Init(RIGHT_MOTOR_TIM, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx

//  TIM_CtrlPWMOutputs(LEFT_MOTOR_TIM,ENABLE);	//MOE 主输出使能	

	TIM_OC1PreloadConfig(LEFT_MOTOR_TIM, TIM_OCPreload_Enable);  //CH1预装载使能	 
	TIM_OC2PreloadConfig(RIGHT_MOTOR_TIM, TIM_OCPreload_Enable);  //CH2预装载使能	 
	
	TIM_ARRPreloadConfig(LEFT_MOTOR_TIM, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
	TIM_Cmd(LEFT_MOTOR_TIM, ENABLE);  //使能TIM9
 
} 



void Set_Pwm(int moto1,int moto2)
{
//	    int siqu=PWM_Base;//死区电压补偿，左右两个电机不相同；左电机占空比在1000/8400时，电机才开始转动；右电机占空比在1200/8400时，电机才开始转动；
			if(moto2<0)			BIN1=1,			BIN2=0;
			else 	          BIN1=0,			BIN2=1;
			PWMB=myabs(moto2);
		  if(moto1<0)	AIN1=0,			AIN2=1;
			else        AIN1=1,			AIN2=0;
			PWMA=myabs(moto1);	
}


void Xianfu_Pwm(void)
{	
	  int Amplitude=PWM_Max;    // PWM占空比满幅是8400
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
	
	Xianfu_Pwm();//限制PWM波形占空比，达不到100%占空比
	
		if(key==0)// 为了不连续按
		{
			ss = 1;
		}
		
		if((key==98) && (ss==1)) // 红外遥控器 UP 按键
		{
			beep_on(1,100);
			ss = 0;
		
			target_v+=0.1f;// 米/秒
		}
		
		
		if((key==168)  && (ss==1)) // 红外遥控器 DOWN 按键
		{
			beep_on(1,100);
			ss = 0;
		
			target_v-=0.1f;// 米/秒
		}
		
		if((key==34) && (ss==1)) // 红外遥控器 LEFT 按键
		{
			beep_on(1,100);
			ss = 0;
			
			if(motor_pid_debug == 0)
			{
				target_omega+=30.0f;// 度/秒
			}
			else if(motor_pid_debug == 1)
			{
				enc_omega[1] += 3.0f;//左轮角速度加 3 弧度
				enc_omega[0] += 3.0f;//右轮角速度加 3 弧度
			}
		}
		
		
		
//		if( key == 176 )// 红外按键3 ，向左旋转90度
//		{
//			beep_on(1,100);
//			Moto1-=800;//每次增加500
//			Moto2+=800;//每次减少500
//		}
		
		if((key==194) && (ss==1)) // 红外遥控器 RIGHT 按键
		{
			beep_on(1,100);
			ss = 0;
			
			if(motor_pid_debug == 0)
			{
				target_omega-=30.0f;// 度/秒
			}
			else if(motor_pid_debug == 1)
			{
				enc_omega[1] -= 3.0f;//左轮角速度减 3 弧度
				enc_omega[0] -= 3.0f;//右轮角速度减 3 弧度
			}
		}
//		printf("左轮角速度设定值enc_omega[1]：%f  rad/s\r\n",enc_omega[1]);
//		printf("右轮角速度设定值enc_omega[0]：%f  rad/s\r\n",enc_omega[0]);

		
//		if( key == 48 )//红外按键4，向右旋转90度
//		{
//			beep_on(1,100);
//			Moto1+=800;//每次增加1500
//			Moto2-=800;//每次减少1500
//		}
		
		if(key==66) // 红外遥控器 0 按键
		{
			beep_on(1,100);
			
			Moto1=0;//置零
			Moto2=0;//置零
			
//			LEFT_ENCODER_TIM->CNT=0;//编码器读数清零
//			RIGHT_ENCODER_TIM->CNT=0;
		}
		
//		if(key==104) // 红外遥控器 1 按键
//		{
//			beep_on(1,100);
//			
//			Moto1=1500;
//			Moto2=1500;
//		}
		
//		if(key==152) // 红外遥控器 2 按键
//		{
//			BEEP=1;//蜂鸣器开启
//			delay_ms(100);
//      BEEP=0;//蜂鸣器关闭
//			
//			Moto1=-1500;
//			Moto2=-1500;
//		}
		
		if(key==2) // 红外遥控器 暂停 按键
		{
			beep_on(1,100);
			
			Moto1=0;//置零
			Moto2=0;//置零
		}
		
//		printf("Moto1:%d\r\n",Moto1);
//		printf("Moto2:%d\r\n",Moto2);
		
		Set_Pwm(Moto1,Moto2);//设置占空比
//		Set_Pwm(0,Moto2);
//		Moto2-=50;
//		printf("PWMB[%d] = %d\r\n",tt++,PWMB);
}
