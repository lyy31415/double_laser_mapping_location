#ifndef __VERSION_1_0_H
#define __VERSION_1_0_H   

#define CONTROL_PERIOD_TIME  0.0008f  // �������ڴ���� 0.0008��
#define MY_PI       3.14159f
#define MY_PI_INV   0.3183f
#define _180_div_PI 57.2957795f
#define PI_div_180  0.01745f


//�������ô���  By lyy
#define Gyro_Uart_Port				USART6
#define Gyro_Uart_TX_Port			GPIOG
#define Gyro_Uart_TX_PinSource		GPIO_PinSource14
#define Gyro_Uart_TX_Pin			((uint16_t)(1<<Gyro_Uart_TX_PinSource))
#define Gyro_Uart_TX_AF				GPIO_AF_USART6
#define Gyro_Uart_RX_Port			GPIOG
#define Gyro_Uart_RX_PinSource		GPIO_PinSource9
#define Gyro_Uart_RX_Pin			((uint16_t)(1<<Gyro_Uart_RX_PinSource))
#define Gyro_Uart_RX_AF				GPIO_AF_USART6
#define Gyro_Uart_IRQn				USART6_IRQn
#define Gyro_Uart_IRQHandler		USART6_IRQHandler

#define Gyro_TX_DMA_Channel			USART6_TX_DMA_Channel
#define Gyro_TX_DMA_Stream			USART6_TX_DMA_Stream
#define Gyro_RX_DMA_Channel			USART6_RX_DMA_Channel
#define Gyro_RX_DMA_Stream			USART6_RX_DMA_Stream


//USART2 PA2 PA3 ������ʾ����ռ��  by lyy
#define Scope_Uart_Port			USART2   
#define Scope_Uart_TX_Port			GPIOA
#define Scope_Uart_TX_PinSource	GPIO_PinSource2
#define Scope_Uart_TX_Pin			((uint16_t)(1<<Scope_Uart_TX_PinSource))
#define Scope_Uart_TX_AF			GPIO_AF_USART2
#define Scope_Uart_RX_Port			GPIOA
#define Scope_Uart_RX_PinSource	GPIO_PinSource3
#define Scope_Uart_RX_Pin			((uint16_t)(1<<Scope_Uart_RX_PinSource))
#define Scope_Uart_RX_AF			GPIO_AF_USART2
#define Scope_Uart_IRQn			USART2_IRQn
#define Scope_Uart_IRQHandler		USART2_IRQHandler

//#define Serial_TX_DMA_Channel			USART2_TX_DMA_Channel
//#define Serial_TX_DMA_Stream			USART2_TX_DMA_Stream
//#define Serial_RX_DMA_Channel			USART2_RX_DMA_Channel
//#define Serial_RX_DMA_Stream			USART2_RX_DMA_Stream


//ͨ���ô���  ��RosͨѶ��
#define Ros_Uart_Port			USART3
#define Ros_Uart_TX_Port		GPIOB
#define Ros_Uart_TX_PinSource	GPIO_PinSource10
#define Ros_Uart_TX_Pin		((uint16_t)(1<<Ros_Uart_TX_PinSource))
#define Ros_Uart_TX_AF			GPIO_AF_USART3
#define Ros_Uart_RX_Port		GPIOB
#define Ros_Uart_RX_PinSource	GPIO_PinSource11
#define Ros_Uart_RX_Pin		((uint16_t)(1<<Ros_Uart_RX_PinSource))
#define Ros_Uart_RX_AF			GPIO_AF_USART3
#define Ros_Uart_IRQn			USART3_IRQn
#define Ros_Uart_IRQHandler	USART3_IRQHandler

#define Ros_TX_DMA_Channel		USART3_TX_DMA_Channel
#define Ros_TX_DMA_Stream			USART3_TX_DMA_Stream
#define Ros_RX_DMA_Channel		USART3_RX_DMA_Channel
#define Ros_RX_DMA_Stream			USART3_RX_DMA_Stream


////���ô���
//#define Reserve_Uart_Port			UART4
//#define Reserve_Uart_TX_Port		GPIOC
//#define Reserve_Uart_TX_PinSource	GPIO_PinSource10
//#define Reserve_Uart_TX_Pin			((uint16_t)(1<<Reserve_Uart_TX_PinSource))
//#define Reserve_Uart_TX_AF			GPIO_AF_UART4
//#define Reserve_Uart_RX_Port		GPIOC
//#define Reserve_Uart_RX_PinSource	GPIO_PinSource11
//#define Reserve_Uart_RX_Pin			((uint16_t)(1<<Reserve_Uart_RX_PinSource))
//#define Reserve_Uart_RX_AF			GPIO_AF_UART4
//#define Reserve_Uart_IRQn			UART4_IRQn
//#define Reserve_Uart_IRQHandler		UART4_IRQHandler

//#define Reserve_TX_DMA_Channel			UART4_TX_DMA_Channel
//#define Reserve_TX_DMA_Stream			UART4_RX_DMA_Stream
//#define Reserve_RX_DMA_Channel			UART4_TX_DMA_Channel
//#define Reserve_RX_DMA_Stream			UART4_RX_DMA_Stream

////PGVͨ���ô���
//#define PGV_Uart_Port				UART5
//#define PGV_Uart_TX_Port			GPIOC
//#define PGV_Uart_TX_PinSource		GPIO_PinSource12
//#define PGV_Uart_TX_Pin				((uint16_t)(1<<PGV_Uart_TX_PinSource))
//#define PGV_Uart_TX_AF				GPIO_AF_UART5
//#define PGV_Uart_RX_Port			GPIOD
//#define PGV_Uart_RX_PinSource		GPIO_PinSource2
//#define PGV_Uart_RX_Pin				((uint16_t)(1<<PGV_Uart_RX_PinSource))
//#define PGV_Uart_RX_AF				GPIO_AF_UART5
//#define PGV_Uart_DIR_Port			GPIOD
//#define PGV_Uart_DIR_Pin			GPIO_Pin_0
//#define PGV_Uart_IRQn				UART5_IRQn
//#define PGV_Uart_IRQHandler			UART5_IRQHandler

//#define PGV_TX_DMA_Channel				UART5_TX_DMA_Channel
//#define PGV_TX_DMA_Stream				UART5_TX_DMA_Stream
//#define PGV_RX_DMA_Channel				UART5_RX_DMA_Channel
//#define PGV_RX_DMA_Stream				UART5_RX_DMA_Stream





/* ����APB1��Ƶϵ����4��APB2��Ƶϵ����2�����Թ���APB1��APB2����Ķ�ʱ��Ƶ�ʷֱ�Ϊ���ǵ�������  
	  ����APB1����Ķ�ʱ����TIM2/3/4/5/6/7/12/13/14 ������APB2����Ķ�ʱ����TIM1/8/9/10/11��
		APB1Ƶ��Ϊ42MHz��APB2Ƶ��Ϊ84MHz������TIM2/3/4/5/6/7/12/13/14ʱ��Ƶ��Ϊ84MHz��TIM1/8/9/10/11ʱ��Ƶ��Ϊ168MHz */
		
//�����˵���ͱ�������Ӳ������  By lyy
#define LEFT_MOTOR_TIM				TIM9			//��ʱ��9
#define LEFT_MOTOR_TIM_CHANNEL		1				//TIM9_CH1
#define LEFT_MOTOR_SPEED_Port			GPIOE		
#define LEFT_MOTOR_SPEED_PinSource	GPIO_PinSource5
#define LEFT_MOTOR_SPEED_Pin			(1<<LEFT_MOTOR_SPEED_PinSource)	//TIM9_CH1 PWM
#define LEFT_MOTOR_SPEED_AF			GPIO_AF_TIM9	//ǰ�����ٶ�IO
#define LEFT_MOTOR_DIR1_Port			GPIOB		
#define LEFT_MOTOR_DIR1_Pin			GPIO_Pin_0		//ǰ���ַ���IO
#define LEFT_MOTOR_DIR2_Port			GPIOB		
#define LEFT_MOTOR_DIR2_Pin			GPIO_Pin_1		//ǰ���ַ���IO
//#define LEFT_MOTOR_BRAKE_Port			GPIOC
//#define LEFT_MOTOR_BRAKE_Pin			GPIO_Pin_5		//ǰ����ɲ��IO
//#define LEFT_MOTOR_STOP_Port			GPIOB
//#define LEFT_MOTOR_STOP_Pin			GPIO_Pin_0		//ǰ��������IO
#define LEFT_ENCODER_TIM				TIM4			//ǰ�������
#define LEFT_ENCODER_A_Port			GPIOD
#define LEFT_ENCODER_A_PinSource		GPIO_PinSource12
#define LEFT_ENCODER_A_Pin			(1<<LEFT_ENCODER_A_PinSource) //TIM4  CH1
#define LEFT_ENCODER_A_AF				GPIO_AF_TIM4	//ǰ�������A��
#define LEFT_ENCODER_B_Port			GPIOD
#define LEFT_ENCODER_B_PinSource		GPIO_PinSource13
#define LEFT_ENCODER_B_Pin			(1<<LEFT_ENCODER_B_PinSource)//TIM4 CH2
#define LEFT_ENCODER_B_AF				GPIO_AF_TIM4	//ǰ�������B��




#define RIGHT_MOTOR_TIM				TIM9			//��ʱ��9
#define RIGHT_MOTOR_TIM_CHANNEL		2				//TIM9_CH2
#define RIGHT_MOTOR_SPEED_Port		GPIOE		
#define RIGHT_MOTOR_SPEED_PinSource	GPIO_PinSource6
#define RIGHT_MOTOR_SPEED_Pin			(1<<RIGHT_MOTOR_SPEED_PinSource)//TIM9_CH2 PWM
#define RIGHT_MOTOR_SPEED_AF			GPIO_AF_TIM9	//ǰ�����ٶ�IO
#define RIGHT_MOTOR_DIR1_Port			GPIOB		
#define RIGHT_MOTOR_DIR1_Pin			GPIO_Pin_6		//ǰ���ַ���IO
#define RIGHT_MOTOR_DIR2_Port			GPIOB		
#define RIGHT_MOTOR_DIR2_Pin			GPIO_Pin_5		//ǰ���ַ���IO  PB7 -> PB5
//#define RIGHT_MOTOR_BRAKE_Port		GPIOB
//#define RIGHT_MOTOR_BRAKE_Pin			GPIO_Pin_2		//ǰ����ɲ��IO
//#define RIGHT_MOTOR_STOP_Port			GPIOE
//#define RIGHT_MOTOR_STOP_Pin			GPIO_Pin_7		//ǰ��������IO
#define RIGHT_ENCODER_TIM				TIM3			//ǰ�ұ�����
#define RIGHT_ENCODER_A_Port			GPIOA
#define RIGHT_ENCODER_A_PinSource		GPIO_PinSource6
#define RIGHT_ENCODER_A_Pin			(1<<RIGHT_ENCODER_A_PinSource)//TIM3 CH1
#define RIGHT_ENCODER_A_AF			GPIO_AF_TIM3	//ǰ�ұ�����A��
#define RIGHT_ENCODER_B_Port			GPIOA
#define RIGHT_ENCODER_B_PinSource		GPIO_PinSource7
#define RIGHT_ENCODER_B_Pin			(1<<RIGHT_ENCODER_B_PinSource)//TIM3 CH2
#define RIGHT_ENCODER_B_AF			GPIO_AF_TIM3	//ǰ�ұ�����B��


//���ں���ң�����Ķ�ʱ��
#define REMOTE_TIM				    TIM1			//��ʱ��1


//���ڼ�����������ٶ��õĶ�ʱ��
#define ENCODER_OMEGA_TIM				    TIM6			//��ʱ��6
#define ENCODER_OMEGA_IRQn          TIM6_DAC_IRQn
#define ENCODER_OMEGA_IRQHandler		TIM6_DAC_IRQHandler

//����agvλ�ú����ÿ����������ö�ʱ��
#define CAL_POS_TIM           TIM6
#define CAL_POS_IRQn          TIM6_DAC_IRQn
#define CAL_POS_IRQHandler		TIM6_DAC_IRQHandler


//PID�����ö�ʱ��
#define PID_TIM           TIM7
#define PID_IRQn          TIM7_IRQn
#define PID_IRQHandler		TIM7_IRQHandler


// �������ö�ʱ�� TIM8 CH3 PC8 CH4 PC9/TIM11 CH1 PF7/TIM12 CH2 PB15 
#define ULTRASONIC_TIM11					TIM11			//��ʱ��11
#define ULTRASONIC_TIM11_Port			GPIOF		
#define ULTRASONIC_TIM11_PinSource	GPIO_PinSource7
#define ULTRASONIC_TIM11_Pin			(1<<ULTRASONIC_TIM11_PinSource)	//TIM11_CH1 ���벶��
#define ULTRASONIC_TIM11_AF			GPIO_AF_TIM11	//PF7����Ϊ ��ʱ��11

#define ULTRASONIC_TIM12					TIM12			//��ʱ��12
#define ULTRASONIC_TIM12_Port			GPIOB		
#define ULTRASONIC_TIM12_PinSource	GPIO_PinSource15
#define ULTRASONIC_TIM12_Pin			(1<<ULTRASONIC_TIM12_PinSource)	//TIM12_CH2 ���벶��
#define ULTRASONIC_TIM12_AF			GPIO_AF_TIM12	//PB15����Ϊ ��ʱ��11

#define ULTRASONIC_TIM8					TIM8			//��ʱ��8
#define ULTRASONIC_TIM8_3_Port			GPIOC		
#define ULTRASONIC_TIM8_3_PinSource	GPIO_PinSource8
#define ULTRASONIC_TIM8_3_Pin			(1<<ULTRASONIC_TIM8_3_PinSource)	//TIM8_CH3 ���벶��
#define ULTRASONIC_TIM8_3_AF			GPIO_AF_TIM8	//PC8����Ϊ ��ʱ��8

#define ULTRASONIC_TIM8_4_Port			GPIOC		
#define ULTRASONIC_TIM8_4_PinSource	GPIO_PinSource9
#define ULTRASONIC_TIM8_4_Pin			(1<<ULTRASONIC_TIM8_4_PinSource)	//TIM8_CH4 ���벶��
#define ULTRASONIC_TIM8_4_AF			GPIO_AF_TIM8	//PC9����Ϊ ��ʱ��8

#define ULTRASONIC_TRIG_PORT  GPIOC
#define ULTRASONIC_TRIG_Pin1  GPIO_Pin_1
#define ULTRASONIC_TRIG_Pin2  GPIO_Pin_2 
#define ULTRASONIC_TRIG_Pin3  GPIO_Pin_3
#define ULTRASONIC_TRIG_Pin4  GPIO_Pin_4 


//����������IO
#define BEEP_GPIO_Port	GPIOF
#define BEEP_GPIO_Pin		GPIO_Pin_8

//����LED0ָʾ��
#define LED0_GPIO_Port	GPIOF
#define LED0_GPIO_Pin	  GPIO_Pin_9

//����LED1ָʾ��
#define LED1_GPIO_Port	GPIOF
#define LED1_GPIO_Pin	  GPIO_Pin_10


//����ң����IO  By lyy
#define Remote_GPIO_Port  GPIOA
#define Remote_GPIO_Pin   GPIO_Pin_8

//ADC��ѹ�ɼ�IO  By lyy
#define ADC_GPIO_Port     GPIOA
#define ADC_GPIO_Pin      GPIO_Pin_5

//OLED��ʾIO  By lyy
#define OLED_CS_PORT 			GPIOB
#define OLED_CS_Pin 			GPIO_Pin_7
#define OLED_SCLK_Port  	GPIOC
#define OLED_SCLK_Pin     GPIO_Pin_6
#define OLED_SDIN_Pin     GPIO_Pin_7 
#define OLED_RS_Port      GPIOD
#define OLED_RS_Pin       GPIO_Pin_6
#define OLED_RST_Port     GPIOG
#define OLED_RST_Pin      GPIO_Pin_15


// PB12 PB13 �������֡����� �̵���
#define RELAY_L_PORT			GPIOB
#define RELAY_L_Pin				GPIO_Pin_12
#define RELAY_R_PORT			GPIOB
#define RELAY_R_Pin				GPIO_Pin_13


////IIC������SDA��SCL GPIO
//#define IIC_SDA_GPIO_Port	GPIOC
//#define IIC_SDA_GPIO_Pin	GPIO_Pin_9
//#define IIC_SCL_GPIO_Port	GPIOC
//#define IIC_SCL_GPIO_Pin	GPIO_Pin_8

////���Ź�ι����IO
//#define WTD_GPIO_Port	GPIOC
//#define WTD_GPIO_Pin	GPIO_Pin_3

////����SPI1����
//#define SPI1_CLK_Port	GPIOB
//#define SPI1_CLK_Pin	GPIO_Pin_3
//#define SPI1_MISO_Port	GPIOB
//#define SPI1_MISO_Pin	GPIO_Pin_4
//#define SPI1_MOSI_Port	GPIOB
//#define SPI1_MOSI_Pin	GPIO_Pin_5

////����SPI_FlashоƬ
//#define SPI_Flash_Chip			W25Q16
//#define SPI_Flash_SPI			SPI1
//#define SPI_Flash_CS_GPIO_Port	GPIOD
//#define SPI_Flash_CS_GPIO_Pin	GPIO_Pin_7



//�����˱���������
#define ENCODER_FIX_WHEEL			false	//ָʾ�����������ӹ�����һ��
#define REDUCTION_RATIO				27		//���ٱ�
#define ENCODER_RESOLUTION_INIT		500	//�������ķֱ��ʣ�������
#define Enco_cout_inv  0.000037037f    // ���������תһȦ����Ӧ����������13500�����ٳ��� 2 ��Ƶ�ʣ��õ� 27000 �����壬�˴� 0.000037037 Ϊ 27000 ����
#define Enco_rad_per_pulse  0.00023271f // ÿ�������Ӧ���� ����rad


/* PWM���� */
/* PWM:x�ͽ��ٶ�:y֮�����Թ�ϵ y = 0.0031*x - 1.73 */
#define PWM_Base   600   // ���ǵ�������ѹ����1000��ʼ
#define PWM_Max    8000   // ����8400  ������ 600 ~ 8000 ֮�䣬�ͽ��ٶȱ������Թ�ϵ
#define Omega_Base 0.0f   // ����/��
#define Omega_Max  22.0f // ����/��
#define PWM_Omega_Ratio   (PWM_Max - PWM_Base )/(Omega_Max - Omega_Base)
#define Omega_PWM_Ratio  (Omega_Max - Omega_Base)/(PWM_Max - PWM_Base )


//���峵�����
#define DISTANCE_OF_WHEEL	     0.3f		//�������ֵľ��룬��λ����
#define INV_DISTANCE_OF_WHEEL	 3.33f		//�������־���ĵ�������λ����
#define WHEEL_RADIUS		       0.032f		//�������Ӱ뾶����λ����
#define AGV_Max_omega					 WHEEL_RADIUS*INV_DISTANCE_OF_WHEEL*(Omega_Max-Omega_Base)*_180_div_PI  // ��/��  134.319641
#define AGV_Min_omega					 -AGV_Max_omega                                             						// ��/��  -134.319641
#define AGV_Max_v							 WHEEL_RADIUS*Omega_Max																									// ��/��  0.704
#define AGV_Min_v							 -AGV_Max_v																								  						// ��/��  -0.704
#define AGV_Max_vacc          3.0f                                                                   // ��/(��*��)
#define AGV_Max_vacc_inv      (1 / AGV_Max_vacc)     
#define AGV_Max_wacc_rad      20.0f                                                                 // ����/(��*��)
#define AGV_Max_wacc          (AGV_Max_wacc_rad * _180_div_PI )                                       // ��/(��*��)
#define AGV_Max_wacc_inv      (1 / AGV_Max_wacc)




//��������ж����ȼ�
#define ENCODER_OMEGA_TIM_PreemptionPriority  0  //��ռ���ȼ�1
#define ENCODER_OMEGA_TIM_SubPriority         0  //�����ȼ�0
//#define PID_TIM_PreemptionPriority  					1  //��ռ���ȼ�1
//#define PID_TIM_SubPriority         					0  //�����ȼ�0
#define Remote_PreemptionPriority  						2  //��ռ���ȼ�2
#define Remote_SubPriority         						1  //�����ȼ�0
#define RemoteCC_PreemptionPriority  					2  //��ռ���ȼ�2
#define RemoteCC_SubPriority         					1  //�����ȼ�1
#define USART1_PreemptionPriority  						3  //��ռ���ȼ�3
#define USART1_SubPriority         						1  //�����ȼ�1
#define ROS_PreemptionPriority  							2  //��ռ���ȼ�1
#define ROS_SubPriority        		 						0  //�����ȼ�1
//#define SCOPE_PreemptionPriority  						3  //��ռ���ȼ�3
//#define SCOPE_SubPriority         						3  //�����ȼ�3
#define ULTRASONIC_TIM11_1_PreptPrio  				1  //��ռ���ȼ�3
#define ULTRASONIC_TIM11_1_SubPrio     				1  //�����ȼ�3
#define ULTRASONIC_TIM12_2_PreptPrio  				1  //��ռ���ȼ�3
#define ULTRASONIC_TIM12_2_SubPrio         		2  //�����ȼ�3
#define ULTRASONIC_TIM8_34_PreptPrio  				1  //��ռ���ȼ�3
#define ULTRASONIC_TIM8_34_SubPrio         		3  //�����ȼ�3



////������ת��
////��ǰѡ�õĵ���������޷�ʶ���100%ռ�ձȣ��ʽ�100%����Ϊ6000ת��50%����Ϊ3000ת
//#define MOTOR_MAX_ROTATIONL_VELOCITY_HARD 6000.0f	//Ӳ������ĵ�����ת��
//#define MOTOR_MIN_ROTATIONL_VELOCITY_HARD 0.0f		//Ӳ������ĵ�����ת��


////����1��DMA
//#define USART1_TX_DMA_Channel	DMA_Channel_4
//#define USART1_TX_DMA_Stream	DMA2_Stream7
//#define USART1_RX_DMA_Channel	DMA_Channel_4
//#define USART1_RX_DMA_Stream	DMA2_Stream2

////����2��DMA
//#define USART2_TX_DMA_Channel	DMA_Channel_4
//#define USART2_TX_DMA_Stream	DMA1_Stream6
//#define USART2_RX_DMA_Channel	DMA_Channel_4
//#define USART2_RX_DMA_Stream	DMA1_Stream5

//����3��DMA
#define USART3_TX_DMA_Channel	DMA_Channel_4
#define USART3_TX_DMA_Stream	DMA1_Stream3
#define USART3_RX_DMA_Channel	DMA_Channel_4
#define USART3_RX_DMA_Stream	DMA1_Stream1

////����4��DMA
//#define UART4_TX_DMA_Channel	DMA_Channel_4
//#define UART4_TX_DMA_Stream		DMA1_Stream4
//#define UART4_RX_DMA_Channel	DMA_Channel_4
//#define UART4_RX_DMA_Stream		DMA1_Stream2

////����5��DMA
//#define UART5_TX_DMA_Channel	DMA_Channel_4
//#define UART5_TX_DMA_Stream		DMA1_Stream7
//#define UART5_RX_DMA_Channel	DMA_Channel_4
//#define UART5_RX_DMA_Stream		DMA1_Stream0


//����6��DMA
#define USART6_TX_DMA_Channel	DMA_Channel_5  //By lyy
#define USART6_TX_DMA_Stream  DMA2_Stream7
#define USART6_RX_DMA_Channel	DMA_Channel_5
#define USART6_RX_DMA_Stream	DMA2_Stream1

    
#endif


