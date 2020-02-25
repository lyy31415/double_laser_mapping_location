#include "main.h"

u8 key,key_local;


int main(void)
{		
	
	/******************* 初始化 *********************/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);                               //初始化延时函数
	uart_init(115200);                             //串口初始化
	KEY_Init();					//初始化按键
	TIM6_Init(0XFFFF,8400-1);//10Khz计数频率,最大计时6.5秒超出   TIM6时钟频率为APB1两倍，为42MHz * 2 =84MHz
	
	#if __PID_TIME__
	TIM7_Init(0XFFFF,8400-1);//PID时间定时器初始化，10Khz计数频率,最大计时6.5秒超出   TIM7时钟频率为APB1两倍，为42MHz * 2 =84MHz
	#endif
	
	LEFT_ENCODER_TIM_Init();
	RIGHT_ENCODER_TIM_Init();
//	GYRODMA_Config(DMA2_Stream7,DMA2_Stream1);//配置陀螺仪发送/接收DMA
	ROS_DMA_Config(Ros_TX_DMA_Stream,Ros_RX_DMA_Stream);//配置ROS通信发送/接收DMA
	Remote_Init();//红外遥控初始化
	Adc_Init();//初始化ADC转换
	OLED_Init();
	DreamCar_PWM_Init(8400-1,1-1);//初始化PWM通道，用于控制电机  20KHz的PWM波   TIM9时钟频率为APB2两倍，为84MHz * 2 =168MHz
	matrix_Init();//矩阵初始化
	BEEP_Init();//蜂鸣器初始化
	PID_Math_Init();
	DataScope_USART2();
	ULTRASONIC_TIM11_Cap_Init(0XFFFF,16800-1);//以10khz的频率计数，溢出时间 6.5 秒   TIM11时钟频率为APB2两倍，为84MHz * 2 =168MHz
	ULTRASONIC_TIM12_Cap_Init(0XFFFF,8400-1);//以10khz的频率计数，溢出时间 6.5 秒   TIM12时钟频率为APB1两倍，为42MHz * 2 =84MHz
	ULTRASONIC_TIM8_Cap_Init(0XFFFF,16800-1);//以10khz的频率计数，溢出时间 6.5 秒    TIM8时钟频率为APB2两倍，为84MHz * 2 =168MHz
	ULTRASONIC_TRIG_Config();//配置超声波触发IO参数
	relay_gpio_init();// 继电器IO配置
	MPU_Init();				//初始化MPU6050
	mpu_dmp_init();   //mpu6050,dmp初始化
	printf("\r\nF407：等待用户按键!\r\n");
	
	TIM_SetCounter(CAL_POS_TIM,0);//重设TIM6定时器的计数器值
	timeout_triple = 0;
	
	#if __PID_TIME__
	TIM_SetCounter(PID_TIM,0);
	timeout_pid = 0;
	#endif
	
		
	while(1)
	{
		key=Remote_Scan(); //红外按键
    key_local = KEY_Scan(0);//本地按键
		
		cal_sensor_angle();		
		cal_agv_pos();
		oled_show();//oled显示
		MOTION_CONTROL();// 运动控制
		Get_Adc_Average(ADC_Channel_5,1);//ADC采样获取电压值
		handle_data();//处理和 ros 之间的通信
		ULTRASONIC_START();//开启超声波测距模块

//		delay_ms(1);
  }
}



