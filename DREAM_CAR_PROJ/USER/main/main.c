#include "main.h"

u8 key,key_local;


int main(void)
{		
	
	/******************* ��ʼ�� *********************/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);                               //��ʼ����ʱ����
	uart_init(115200);                             //���ڳ�ʼ��
	KEY_Init();					//��ʼ������
	TIM6_Init(0XFFFF,8400-1);//10Khz����Ƶ��,����ʱ6.5�볬��   TIM6ʱ��Ƶ��ΪAPB1������Ϊ42MHz * 2 =84MHz
	
	#if __PID_TIME__
	TIM7_Init(0XFFFF,8400-1);//PIDʱ�䶨ʱ����ʼ����10Khz����Ƶ��,����ʱ6.5�볬��   TIM7ʱ��Ƶ��ΪAPB1������Ϊ42MHz * 2 =84MHz
	#endif
	
	LEFT_ENCODER_TIM_Init();
	RIGHT_ENCODER_TIM_Init();
//	GYRODMA_Config(DMA2_Stream7,DMA2_Stream1);//���������Ƿ���/����DMA
	ROS_DMA_Config(Ros_TX_DMA_Stream,Ros_RX_DMA_Stream);//����ROSͨ�ŷ���/����DMA
	Remote_Init();//����ң�س�ʼ��
	Adc_Init();//��ʼ��ADCת��
	OLED_Init();
	DreamCar_PWM_Init(8400-1,1-1);//��ʼ��PWMͨ�������ڿ��Ƶ��  20KHz��PWM��   TIM9ʱ��Ƶ��ΪAPB2������Ϊ84MHz * 2 =168MHz
	matrix_Init();//�����ʼ��
	BEEP_Init();//��������ʼ��
	PID_Math_Init();
	DataScope_USART2();
	ULTRASONIC_TIM11_Cap_Init(0XFFFF,16800-1);//��10khz��Ƶ�ʼ��������ʱ�� 6.5 ��   TIM11ʱ��Ƶ��ΪAPB2������Ϊ84MHz * 2 =168MHz
	ULTRASONIC_TIM12_Cap_Init(0XFFFF,8400-1);//��10khz��Ƶ�ʼ��������ʱ�� 6.5 ��   TIM12ʱ��Ƶ��ΪAPB1������Ϊ42MHz * 2 =84MHz
	ULTRASONIC_TIM8_Cap_Init(0XFFFF,16800-1);//��10khz��Ƶ�ʼ��������ʱ�� 6.5 ��    TIM8ʱ��Ƶ��ΪAPB2������Ϊ84MHz * 2 =168MHz
	ULTRASONIC_TRIG_Config();//���ó���������IO����
	relay_gpio_init();// �̵���IO����
	MPU_Init();				//��ʼ��MPU6050
	mpu_dmp_init();   //mpu6050,dmp��ʼ��
	printf("\r\nF407���ȴ��û�����!\r\n");
	
	TIM_SetCounter(CAL_POS_TIM,0);//����TIM6��ʱ���ļ�����ֵ
	timeout_triple = 0;
	
	#if __PID_TIME__
	TIM_SetCounter(PID_TIM,0);
	timeout_pid = 0;
	#endif
	
		
	while(1)
	{
		key=Remote_Scan(); //���ⰴ��
    key_local = KEY_Scan(0);//���ذ���
		
		cal_sensor_angle();		
		cal_agv_pos();
		oled_show();//oled��ʾ
		MOTION_CONTROL();// �˶�����
		Get_Adc_Average(ADC_Channel_5,1);//ADC������ȡ��ѹֵ
		handle_data();//����� ros ֮���ͨ��
		ULTRASONIC_START();//�������������ģ��

//		delay_ms(1);
  }
}



