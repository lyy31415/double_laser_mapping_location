 #include "DataScope_DP.h"

static unsigned char DataScope_OutPut_Buffer[42] = {0};	   //串口发送缓冲区



//函数说明：将单精度浮点数据转成4字节数据并存入指定地址 
//附加说明：用户无需直接操作此函数 
//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
//函数无返回 
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
 
 
//函数说明：将待发送通道的单精度浮点数据写入发送缓冲区
//Data：通道数据
//Channel：选择通道（1-10）
//函数无返回 
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
	if ( (Channel > 10) || (Channel == 0) ) return;  //通道个数大于10或等于0，直接跳出，不执行函数
  else
  {
     switch (Channel)
		{
      case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
      case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
		  case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
		  case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
		  case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
		  case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
		  case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
		  case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
		  case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
		  case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
		}
  }	 
}


//函数说明：生成 DataScopeV1.0 能正确识别的帧格式
//Channel_Number，需要发送的通道个数
//返回发送缓冲区数据个数
//返回0表示帧格式生成失败 
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //通道个数大于10或等于0，直接跳出，不执行函数
  else
  {	
	 DataScope_OutPut_Buffer[0] = '$';  //帧头
		
	 switch(Channel_Number)   
   { 
		 case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6;  
		 case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10;
		 case 3:   DataScope_OutPut_Buffer[13] = 13; return 14; 
		 case 4:   DataScope_OutPut_Buffer[17] = 17; return 18;
		 case 5:   DataScope_OutPut_Buffer[21] = 21; return 22;  
		 case 6:   DataScope_OutPut_Buffer[25] = 25; return 26;
		 case 7:   DataScope_OutPut_Buffer[29] = 29; return 30; 
		 case 8:   DataScope_OutPut_Buffer[33] = 33; return 34; 
		 case 9:   DataScope_OutPut_Buffer[37] = 37; return 38;
     case 10:  DataScope_OutPut_Buffer[41] = 41; return 42; 
   }	 
  }
	return 0;
}



//往上位机发送数据
void DataScope(pid_struct pid)
{   
		unsigned char Send_Count; //串口需要发送的数据个数
		unsigned char i;          //计数变量
	
		DataScope_Get_Channel_Data( pid.Ka, 1 );       //显示调整值
		DataScope_Get_Channel_Data( pid.Kp, 2 );       //显示比例系数
		DataScope_Get_Channel_Data( pid.Ki, 3 );       //显示积分系数
		DataScope_Get_Channel_Data( pid.Kd , 4 );      //显示微分系数
//	  DataScope_Get_Channel_Data(enc_omega[1], 5 );
//	  DataScope_Get_Channel_Data(enc_omega[0], 6 ); 
		DataScope_Get_Channel_Data(w_overall, 5 );// 当前角速度
	  DataScope_Get_Channel_Data(v_overall, 6 );// 当前线速度
//	  DataScope_Get_Channel_Data(encdr_data[1], 7 );  // 左电机角速度 rad/s
//		DataScope_Get_Channel_Data(encdr_data[0] , 8 );//  右电机角速度 rad/s
		DataScope_Get_Channel_Data(target_omega_after_acc , 7);// 加减速分解后的角速度
		DataScope_Get_Channel_Data(target_v_after_acc, 8);  // 加减速分解后的线速度
		DataScope_Get_Channel_Data(target_omega_before_acc , 9);// 加减速分解后的角速度
		DataScope_Get_Channel_Data(target_v_before_acc, 10);  // 加减速分解后的线速度
//	  DataScope_Get_Channel_Data(target_omega, 9 );// 上位机的目标 角速度
//		DataScope_Get_Channel_Data(target_v, 10 ); //上位机的目标 线速度
//		DataScope_Get_Channel_Data(Encoder_Left*0.01, 9 );// 上位机的目标 角速度
//		DataScope_Get_Channel_Data(Encoder_Right*0.01, 10 ); //上位机的目标 线速度
//	  DataScope_Get_Channel_Data(z_rate, 7 );  //IMU角速度
//	  DataScope_Get_Channel_Data(pos_b[0], 8 );// 根据IMU角速度计算出来的 agv 线速度
//	  DataScope_Get_Channel_Data(pos_b[1] , 9 );//编码器计算的agv 角速度
//		DataScope_Get_Channel_Data(pos_b[2], 10 ); //编码器计算的agv 线速度
		

		Send_Count = DataScope_Data_Generate(10);
		for( i = 0 ; i < Send_Count; i++) 
		{
		while((Scope_Uart_Port->SR&0X40)==0);  
		Scope_Uart_Port->DR = DataScope_OutPut_Buffer[i]; 
		}
}


void DataScope_USART2(void)
{
	//GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
	
	/////////////////////// 串口初始化 //////////////////////////////////
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使RCC_AHB1Periph_GPIOA能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	
	//串口6对应引脚复用映射
	GPIO_PinAFConfig(Scope_Uart_TX_Port,Scope_Uart_TX_PinSource,Scope_Uart_TX_AF); //GPIOA2复用为USART2   TX
	GPIO_PinAFConfig(Scope_Uart_RX_Port,Scope_Uart_RX_PinSource,Scope_Uart_RX_AF); //GPIOA3复用为USART2  RX
	
	//USART2端口配置
  GPIO_InitStructure.GPIO_Pin = Scope_Uart_TX_Pin; //GPIOA2  TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(Scope_Uart_TX_Port,&GPIO_InitStructure); //初始化PA2
	
	GPIO_InitStructure.GPIO_Pin = Scope_Uart_RX_Pin; //GPIOA3  RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //
	GPIO_Init(Scope_Uart_RX_Port,&GPIO_InitStructure); //初始化PA3

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = 256000;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(Scope_Uart_Port, &USART_InitStructure); //初始化串口2
	

//	//Usart2 NVIC 配置
//  NVIC_InitStructure.NVIC_IRQChannel = Scope_Uart_IRQn;//串口2中断通道
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SCOPE_PreemptionPriority;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SCOPE_SubPriority;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
//	
//	USART_ITConfig(Scope_Uart_Port, USART_IT_IDLE, ENABLE);//开启空闲中断，若总线空闲，则产生中断
	
	USART_Cmd(Scope_Uart_Port, ENABLE);  //使能串口2
}



