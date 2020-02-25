#ifndef __VERSION_1_0_H
#define __VERSION_1_0_H   

#define CONTROL_PERIOD_TIME  0.0008f  // 控制周期大概是 0.0008秒
#define MY_PI       3.14159f
#define MY_PI_INV   0.3183f
#define _180_div_PI 57.2957795f
#define PI_div_180  0.01745f


//陀螺仪用串口  By lyy
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


//USART2 PA2 PA3 被虚拟示波器占用  by lyy
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


//通信用串口  和Ros通讯用
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


////备用串口
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

////PGV通信用串口
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





/* 由于APB1分频系数是4，APB2分频系数是2，所以挂在APB1和APB2下面的定时器频率分别为它们的两倍！  
	  挂在APB1下面的定时器有TIM2/3/4/5/6/7/12/13/14 ，挂在APB2下面的定时器有TIM1/8/9/10/11，
		APB1频率为42MHz，APB2频率为84MHz，所以TIM2/3/4/5/6/7/12/13/14时钟频率为84MHz，TIM1/8/9/10/11时钟频率为168MHz */
		
//定义了电机和编码器的硬件参数  By lyy
#define LEFT_MOTOR_TIM				TIM9			//定时器9
#define LEFT_MOTOR_TIM_CHANNEL		1				//TIM9_CH1
#define LEFT_MOTOR_SPEED_Port			GPIOE		
#define LEFT_MOTOR_SPEED_PinSource	GPIO_PinSource5
#define LEFT_MOTOR_SPEED_Pin			(1<<LEFT_MOTOR_SPEED_PinSource)	//TIM9_CH1 PWM
#define LEFT_MOTOR_SPEED_AF			GPIO_AF_TIM9	//前左轮速度IO
#define LEFT_MOTOR_DIR1_Port			GPIOB		
#define LEFT_MOTOR_DIR1_Pin			GPIO_Pin_0		//前左轮方向IO
#define LEFT_MOTOR_DIR2_Port			GPIOB		
#define LEFT_MOTOR_DIR2_Pin			GPIO_Pin_1		//前左轮方向IO
//#define LEFT_MOTOR_BRAKE_Port			GPIOC
//#define LEFT_MOTOR_BRAKE_Pin			GPIO_Pin_5		//前左轮刹车IO
//#define LEFT_MOTOR_STOP_Port			GPIOB
//#define LEFT_MOTOR_STOP_Pin			GPIO_Pin_0		//前左轮启动IO
#define LEFT_ENCODER_TIM				TIM4			//前左编码器
#define LEFT_ENCODER_A_Port			GPIOD
#define LEFT_ENCODER_A_PinSource		GPIO_PinSource12
#define LEFT_ENCODER_A_Pin			(1<<LEFT_ENCODER_A_PinSource) //TIM4  CH1
#define LEFT_ENCODER_A_AF				GPIO_AF_TIM4	//前左编码器A相
#define LEFT_ENCODER_B_Port			GPIOD
#define LEFT_ENCODER_B_PinSource		GPIO_PinSource13
#define LEFT_ENCODER_B_Pin			(1<<LEFT_ENCODER_B_PinSource)//TIM4 CH2
#define LEFT_ENCODER_B_AF				GPIO_AF_TIM4	//前左编码器B相




#define RIGHT_MOTOR_TIM				TIM9			//定时器9
#define RIGHT_MOTOR_TIM_CHANNEL		2				//TIM9_CH2
#define RIGHT_MOTOR_SPEED_Port		GPIOE		
#define RIGHT_MOTOR_SPEED_PinSource	GPIO_PinSource6
#define RIGHT_MOTOR_SPEED_Pin			(1<<RIGHT_MOTOR_SPEED_PinSource)//TIM9_CH2 PWM
#define RIGHT_MOTOR_SPEED_AF			GPIO_AF_TIM9	//前右轮速度IO
#define RIGHT_MOTOR_DIR1_Port			GPIOB		
#define RIGHT_MOTOR_DIR1_Pin			GPIO_Pin_6		//前右轮方向IO
#define RIGHT_MOTOR_DIR2_Port			GPIOB		
#define RIGHT_MOTOR_DIR2_Pin			GPIO_Pin_5		//前右轮方向IO  PB7 -> PB5
//#define RIGHT_MOTOR_BRAKE_Port		GPIOB
//#define RIGHT_MOTOR_BRAKE_Pin			GPIO_Pin_2		//前右轮刹车IO
//#define RIGHT_MOTOR_STOP_Port			GPIOE
//#define RIGHT_MOTOR_STOP_Pin			GPIO_Pin_7		//前右轮启动IO
#define RIGHT_ENCODER_TIM				TIM3			//前右编码器
#define RIGHT_ENCODER_A_Port			GPIOA
#define RIGHT_ENCODER_A_PinSource		GPIO_PinSource6
#define RIGHT_ENCODER_A_Pin			(1<<RIGHT_ENCODER_A_PinSource)//TIM3 CH1
#define RIGHT_ENCODER_A_AF			GPIO_AF_TIM3	//前右编码器A相
#define RIGHT_ENCODER_B_Port			GPIOA
#define RIGHT_ENCODER_B_PinSource		GPIO_PinSource7
#define RIGHT_ENCODER_B_Pin			(1<<RIGHT_ENCODER_B_PinSource)//TIM3 CH2
#define RIGHT_ENCODER_B_AF			GPIO_AF_TIM3	//前右编码器B相


//用于红外遥控器的定时器
#define REMOTE_TIM				    TIM1			//定时器1


//用于计算编码器角速度用的定时器
#define ENCODER_OMEGA_TIM				    TIM6			//定时器6
#define ENCODER_OMEGA_IRQn          TIM6_DAC_IRQn
#define ENCODER_OMEGA_IRQHandler		TIM6_DAC_IRQHandler

//计算agv位置和设置卡尔曼噪音用定时器
#define CAL_POS_TIM           TIM6
#define CAL_POS_IRQn          TIM6_DAC_IRQn
#define CAL_POS_IRQHandler		TIM6_DAC_IRQHandler


//PID计算用定时器
#define PID_TIM           TIM7
#define PID_IRQn          TIM7_IRQn
#define PID_IRQHandler		TIM7_IRQHandler


// 超声波用定时器 TIM8 CH3 PC8 CH4 PC9/TIM11 CH1 PF7/TIM12 CH2 PB15 
#define ULTRASONIC_TIM11					TIM11			//定时器11
#define ULTRASONIC_TIM11_Port			GPIOF		
#define ULTRASONIC_TIM11_PinSource	GPIO_PinSource7
#define ULTRASONIC_TIM11_Pin			(1<<ULTRASONIC_TIM11_PinSource)	//TIM11_CH1 输入捕获
#define ULTRASONIC_TIM11_AF			GPIO_AF_TIM11	//PF7复用为 定时器11

#define ULTRASONIC_TIM12					TIM12			//定时器12
#define ULTRASONIC_TIM12_Port			GPIOB		
#define ULTRASONIC_TIM12_PinSource	GPIO_PinSource15
#define ULTRASONIC_TIM12_Pin			(1<<ULTRASONIC_TIM12_PinSource)	//TIM12_CH2 输入捕获
#define ULTRASONIC_TIM12_AF			GPIO_AF_TIM12	//PB15复用为 定时器11

#define ULTRASONIC_TIM8					TIM8			//定时器8
#define ULTRASONIC_TIM8_3_Port			GPIOC		
#define ULTRASONIC_TIM8_3_PinSource	GPIO_PinSource8
#define ULTRASONIC_TIM8_3_Pin			(1<<ULTRASONIC_TIM8_3_PinSource)	//TIM8_CH3 输入捕获
#define ULTRASONIC_TIM8_3_AF			GPIO_AF_TIM8	//PC8复用为 定时器8

#define ULTRASONIC_TIM8_4_Port			GPIOC		
#define ULTRASONIC_TIM8_4_PinSource	GPIO_PinSource9
#define ULTRASONIC_TIM8_4_Pin			(1<<ULTRASONIC_TIM8_4_PinSource)	//TIM8_CH4 输入捕获
#define ULTRASONIC_TIM8_4_AF			GPIO_AF_TIM8	//PC9复用为 定时器8

#define ULTRASONIC_TRIG_PORT  GPIOC
#define ULTRASONIC_TRIG_Pin1  GPIO_Pin_1
#define ULTRASONIC_TRIG_Pin2  GPIO_Pin_2 
#define ULTRASONIC_TRIG_Pin3  GPIO_Pin_3
#define ULTRASONIC_TRIG_Pin4  GPIO_Pin_4 


//蜂鸣器控制IO
#define BEEP_GPIO_Port	GPIOF
#define BEEP_GPIO_Pin		GPIO_Pin_8

//板载LED0指示灯
#define LED0_GPIO_Port	GPIOF
#define LED0_GPIO_Pin	  GPIO_Pin_9

//板载LED1指示灯
#define LED1_GPIO_Port	GPIOF
#define LED1_GPIO_Pin	  GPIO_Pin_10


//红外遥控器IO  By lyy
#define Remote_GPIO_Port  GPIOA
#define Remote_GPIO_Pin   GPIO_Pin_8

//ADC电压采集IO  By lyy
#define ADC_GPIO_Port     GPIOA
#define ADC_GPIO_Pin      GPIO_Pin_5

//OLED显示IO  By lyy
#define OLED_CS_PORT 			GPIOB
#define OLED_CS_Pin 			GPIO_Pin_7
#define OLED_SCLK_Port  	GPIOC
#define OLED_SCLK_Pin     GPIO_Pin_6
#define OLED_SDIN_Pin     GPIO_Pin_7 
#define OLED_RS_Port      GPIOD
#define OLED_RS_Pin       GPIO_Pin_6
#define OLED_RST_Port     GPIOG
#define OLED_RST_Pin      GPIO_Pin_15


// PB12 PB13 控制左轮、右轮 继电器
#define RELAY_L_PORT			GPIOB
#define RELAY_L_Pin				GPIO_Pin_12
#define RELAY_R_PORT			GPIOB
#define RELAY_R_Pin				GPIO_Pin_13


////IIC器件的SDA、SCL GPIO
//#define IIC_SDA_GPIO_Port	GPIOC
//#define IIC_SDA_GPIO_Pin	GPIO_Pin_9
//#define IIC_SCL_GPIO_Port	GPIOC
//#define IIC_SCL_GPIO_Pin	GPIO_Pin_8

////看门狗喂狗用IO
//#define WTD_GPIO_Port	GPIOC
//#define WTD_GPIO_Pin	GPIO_Pin_3

////定义SPI1引脚
//#define SPI1_CLK_Port	GPIOB
//#define SPI1_CLK_Pin	GPIO_Pin_3
//#define SPI1_MISO_Port	GPIOB
//#define SPI1_MISO_Pin	GPIO_Pin_4
//#define SPI1_MOSI_Port	GPIOB
//#define SPI1_MOSI_Pin	GPIO_Pin_5

////定义SPI_Flash芯片
//#define SPI_Flash_Chip			W25Q16
//#define SPI_Flash_SPI			SPI1
//#define SPI_Flash_CS_GPIO_Port	GPIOD
//#define SPI_Flash_CS_GPIO_Pin	GPIO_Pin_7



//定义了编码器参数
#define ENCODER_FIX_WHEEL			false	//指示编码器和轮子固连在一起
#define REDUCTION_RATIO				27		//减速比
#define ENCODER_RESOLUTION_INIT		500	//编码器的分辨率（线数）
#define Enco_cout_inv  0.000037037f    // 电机减速器转一圈，对应编码器脉冲13500个，再乘以 2 倍频率，得到 27000 个脉冲，此处 0.000037037 为 27000 倒数
#define Enco_rad_per_pulse  0.00023271f // 每个脉冲对应多少 弧度rad


/* PWM设置 */
/* PWM:x和角速度:y之间线性关系 y = 0.0031*x - 1.73 */
#define PWM_Base   600   // 考虑到死区电压，从1000开始
#define PWM_Max    8000   // 满幅8400  限制在 600 ~ 8000 之间，和角速度保持线性关系
#define Omega_Base 0.0f   // 弧度/秒
#define Omega_Max  22.0f // 弧度/秒
#define PWM_Omega_Ratio   (PWM_Max - PWM_Base )/(Omega_Max - Omega_Base)
#define Omega_PWM_Ratio  (Omega_Max - Omega_Base)/(PWM_Max - PWM_Base )


//定义车体参数
#define DISTANCE_OF_WHEEL	     0.3f		//左右两轮的距离，单位：米
#define INV_DISTANCE_OF_WHEEL	 3.33f		//左右两轮距离的倒数，单位：米
#define WHEEL_RADIUS		       0.032f		//定义轮子半径，单位：米
#define AGV_Max_omega					 WHEEL_RADIUS*INV_DISTANCE_OF_WHEEL*(Omega_Max-Omega_Base)*_180_div_PI  // 度/秒  134.319641
#define AGV_Min_omega					 -AGV_Max_omega                                             						// 度/秒  -134.319641
#define AGV_Max_v							 WHEEL_RADIUS*Omega_Max																									// 米/秒  0.704
#define AGV_Min_v							 -AGV_Max_v																								  						// 米/秒  -0.704
#define AGV_Max_vacc          3.0f                                                                   // 米/(秒*秒)
#define AGV_Max_vacc_inv      (1 / AGV_Max_vacc)     
#define AGV_Max_wacc_rad      20.0f                                                                 // 弧度/(秒*秒)
#define AGV_Max_wacc          (AGV_Max_wacc_rad * _180_div_PI )                                       // 度/(秒*秒)
#define AGV_Max_wacc_inv      (1 / AGV_Max_wacc)




//定义各个中断优先级
#define ENCODER_OMEGA_TIM_PreemptionPriority  0  //抢占优先级1
#define ENCODER_OMEGA_TIM_SubPriority         0  //子优先级0
//#define PID_TIM_PreemptionPriority  					1  //抢占优先级1
//#define PID_TIM_SubPriority         					0  //子优先级0
#define Remote_PreemptionPriority  						2  //抢占优先级2
#define Remote_SubPriority         						1  //子优先级0
#define RemoteCC_PreemptionPriority  					2  //抢占优先级2
#define RemoteCC_SubPriority         					1  //子优先级1
#define USART1_PreemptionPriority  						3  //抢占优先级3
#define USART1_SubPriority         						1  //子优先级1
#define ROS_PreemptionPriority  							2  //抢占优先级1
#define ROS_SubPriority        		 						0  //子优先级1
//#define SCOPE_PreemptionPriority  						3  //抢占优先级3
//#define SCOPE_SubPriority         						3  //子优先级3
#define ULTRASONIC_TIM11_1_PreptPrio  				1  //抢占优先级3
#define ULTRASONIC_TIM11_1_SubPrio     				1  //子优先级3
#define ULTRASONIC_TIM12_2_PreptPrio  				1  //抢占优先级3
#define ULTRASONIC_TIM12_2_SubPrio         		2  //子优先级3
#define ULTRASONIC_TIM8_34_PreptPrio  				1  //抢占优先级3
#define ULTRASONIC_TIM8_34_SubPrio         		3  //子优先级3



////定义电机转速
////当前选用的电机驱动器无法识别出100%占空比，故将100%定义为6000转，50%定义为3000转
//#define MOTOR_MAX_ROTATIONL_VELOCITY_HARD 6000.0f	//硬件定义的电机最高转速
//#define MOTOR_MIN_ROTATIONL_VELOCITY_HARD 0.0f		//硬件定义的电机最低转速


////串口1用DMA
//#define USART1_TX_DMA_Channel	DMA_Channel_4
//#define USART1_TX_DMA_Stream	DMA2_Stream7
//#define USART1_RX_DMA_Channel	DMA_Channel_4
//#define USART1_RX_DMA_Stream	DMA2_Stream2

////串口2用DMA
//#define USART2_TX_DMA_Channel	DMA_Channel_4
//#define USART2_TX_DMA_Stream	DMA1_Stream6
//#define USART2_RX_DMA_Channel	DMA_Channel_4
//#define USART2_RX_DMA_Stream	DMA1_Stream5

//串口3用DMA
#define USART3_TX_DMA_Channel	DMA_Channel_4
#define USART3_TX_DMA_Stream	DMA1_Stream3
#define USART3_RX_DMA_Channel	DMA_Channel_4
#define USART3_RX_DMA_Stream	DMA1_Stream1

////串口4用DMA
//#define UART4_TX_DMA_Channel	DMA_Channel_4
//#define UART4_TX_DMA_Stream		DMA1_Stream4
//#define UART4_RX_DMA_Channel	DMA_Channel_4
//#define UART4_RX_DMA_Stream		DMA1_Stream2

////串口5用DMA
//#define UART5_TX_DMA_Channel	DMA_Channel_4
//#define UART5_TX_DMA_Stream		DMA1_Stream7
//#define UART5_RX_DMA_Channel	DMA_Channel_4
//#define UART5_RX_DMA_Stream		DMA1_Stream0


//串口6用DMA
#define USART6_TX_DMA_Channel	DMA_Channel_5  //By lyy
#define USART6_TX_DMA_Stream  DMA2_Stream7
#define USART6_RX_DMA_Channel	DMA_Channel_5
#define USART6_RX_DMA_Stream	DMA2_Stream1

    
#endif


