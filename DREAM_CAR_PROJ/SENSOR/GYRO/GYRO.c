#include "GYRO.h"

char TX_buf[MA10_TX+1];			//发送数据的缓冲区，若缓冲区满，则不会发送
uint8_t RX_buf[MA10_RX]; //接收数据的缓冲区
uint8_t data_Buf[MA10_RX]; //数据暂存，避免数据遭到破坏
float z_rate;		 //Z轴角速率(°/s)
//float z_heading;	 //Z轴方位角，-180°~+180°
//float forward_accel;	//前向加速度(mm/s2)

static char data_run[]=">@RUN1";
static char data_stop[]=">@RUN0";
data_trans hex2flot;//16进制转换成浮点数联合体变量






//开启一次DMA传输
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:数据传输量  
void GYRODMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
}


//清除串口空闲中断，先读入SR，再读入DR清除IDLE中断标志
void Clear_IDLE_Flag(USART_TypeDef* USARTx)
{	
	uint16_t temp1;
//	uint16_t temp = USARTx->SR;
//	temp = USARTx->DR;

	//清空 USART_IT_RXNE 中断标志，通过对 USART_DR 寄存器执行读入操作将RXNE 标志位清零
	temp1 = USARTx->DR;
}


//关闭DMA，设置DMA传输数据的数量，打开DMA通道，返回DMA剩余未传输数据的数量
uint32_t Set_Data_Num(DMA_Stream_TypeDef *DMA_Streamx, uint16_t number)
{
	/*
	* 在stm32f4的手册中，DMA-SR寄存器说明下面有一句话
	* 将 EN 位置“1”以启动新传输之前， DMA_LISR 或 DMA_HISR 寄存器中与数据流相对应的事件标志必须清零
	*
	* 在传输结束前禁止数据流（EN位清零），会产生传输完成事件
	*/

	uint8_t x = 0;		   //指示当前是哪个流，DMAy_Streamx
	uint32_t add_temp = 0; //保存需修改寄存器的地址
	uint32_t temp = 0;
	uint32_t remaining;

	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
	if (DMA_Streamx < DMA2_Stream0)
	{
		x = ((uint32_t)DMA_Streamx - (uint32_t)DMA1_Stream0) / 0x18; //获取当前流控制器
		add_temp = (uint32_t) & (DMA1->LIFCR) + (x & (0x04));		 //保存寄存器地址
	}
	else
	{
		x = ((uint32_t)DMA_Streamx - (uint32_t)DMA2_Stream0) / 0x18;
		add_temp = (uint32_t) & (DMA2->LIFCR) + (x & (0x04)); //保存寄存器地址
	}
	//temp = x & 0x02;
	//temp = temp<< 3;
	//temp = (0x3D << (6 * (x & 0x01)))<<((x&0x02)<<3);
	//*(uint32_t*)add_temp = temp;
	temp = (0x3D << (6 * (x & 0x01))) << ((x & 0x02) << 3);
	*(uint32_t *)add_temp = temp;//清除所有中断标志
	//*(uint32_t*)add_temp = (0x3D << (6 * (x & 0x01))) << ((x & 0x02) << 3);	

	remaining = DMA_Streamx->NDTR;
	DMA_Streamx->NDTR = number;
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
	return remaining;
}



void GYRODMA_Config(DMA_Stream_TypeDef *DMA_Stream_Tx, DMA_Stream_TypeDef *DMA_Stream_Rx)
{ 
	//GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
	
	/******************************* DMA初始化 **************************************/
	if( ((u32)DMA_Stream_Tx >(u32)DMA2) || ((u32)DMA_Stream_Rx >(u32)DMA2) )//初始化DMA时钟
	{
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
		
	}
	if(  ((u32)DMA_Stream_Tx <(u32)DMA2) || ((u32)DMA_Stream_Rx <(u32)DMA2) )
	{
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	}
  DMA_DeInit(DMA_Stream_Tx);
	DMA_DeInit(DMA_Stream_Rx);
	
	while (DMA_GetCmdStatus(DMA_Stream_Tx) != DISABLE){}//等待DMA可配置 
	while (DMA_GetCmdStatus(DMA_Stream_Rx) != DISABLE){}//等待DMA可配置 
	

		
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = Gyro_TX_DMA_Channel;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART6->DR);//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&TX_buf;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = (uint32_t)MA10_TX;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA_Stream_Tx, &DMA_InitStructure);//初始化DMA Stream
		
	GYRODMA_Enable(Gyro_TX_DMA_Stream,MA10_TX);


	//配置接收中断
	DMA_InitStructure.DMA_BufferSize = (uint32_t)MA10_RX;
	DMA_InitStructure.DMA_Channel = Gyro_RX_DMA_Channel;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //外设到内存
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&RX_buf;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA_Stream_Rx, &DMA_InitStructure);//初始化DMA Stream
	
	GYRODMA_Enable(Gyro_RX_DMA_Stream,MA10_RX);
	/******************************* DMA初始化 **************************************/
	
	
	/************************** 串口初始化 *************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //使能GPIOC时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟
	
	//串口6对应引脚复用映射
	GPIO_PinAFConfig(Gyro_Uart_TX_Port,Gyro_Uart_TX_PinSource,Gyro_Uart_TX_AF); //GPIOG14复用为USART6   TX
	GPIO_PinAFConfig(Gyro_Uart_RX_Port,Gyro_Uart_RX_PinSource,Gyro_Uart_RX_AF); //GPIOG9复用为USART6  RX
	
	//USART6端口配置
  GPIO_InitStructure.GPIO_Pin = Gyro_Uart_TX_Pin; //GPIOG14  TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(Gyro_Uart_TX_Port,&GPIO_InitStructure); //初始化PG14
	
	GPIO_InitStructure.GPIO_Pin = Gyro_Uart_RX_Pin; //GPIOG9  RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //
	GPIO_Init(Gyro_Uart_RX_Port,&GPIO_InitStructure); //初始化PG9

   //USART6 初始化设置
	USART_InitStructure.USART_BaudRate = 256000;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(Gyro_Uart_Port, &USART_InitStructure); //初始化串口6
	

	//Usart6 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = Gyro_Uart_IRQn;//串口6中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = Gyro_PreemptionPriority;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = Gyro_SubPriority;		//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	
	
	USART_ITConfig(Gyro_Uart_Port, USART_IT_RXNE, ENABLE);//开启接收非空中断，若串口收到数据，则产生中断
	USART_DMACmd(Gyro_Uart_Port, USART_DMAReq_Rx, ENABLE);//开启串口DMA接收
	USART_DMACmd(Gyro_Uart_Port, USART_DMAReq_Tx, ENABLE);//开启串口DMA发送
	USART_Cmd(Gyro_Uart_Port, ENABLE);  //使能串口6
/*********************************** 串口初始化 *****************************************/	
	

	//软件序列清空中断标志
	Clear_IDLE_Flag(USART6);	
} 
	

void Gyro_Uart_IRQHandler(void)                	//串口6中断服务程序
{	
	if (USART6->SR&USART_FLAG_RXNE)	//接收非空中断
	{
		//软件序列清空中断标志
		Clear_IDLE_Flag(USART6);
		Set_Data_Num(Gyro_RX_DMA_Stream,MA10_RX);
	}
}



/**  float HEX2FLOAT(uint8_t *src_8) 函数说明
*
*将JYX-MA10型号惯导返回数据转换成浮点数格式
*
*		IEEE标准中，PC存储浮点数的格式：
*		 ____________________________________________________________
*		|_S_|_____Exponent______|_________Mantissa___________________|
*		 1位         8位                      23位
*		符号位    阶码,即指数            尾数,即小数部分
*
*将16进制机器码转换成浮点数格式：
*		x=(-1)^S × (1.M) × 2^e
*		其中：e=E-127, x是浮点数, E=指数e的移码-1, 移码 是补码的符号位取反后的值
*/
float HEX2FLOAT(uint8_t *src_8)
{
	u8 i;
	float temp;	
	
	/** 把16进制的数据，通过联合体转换成浮点数 **/
	for(i=0; i<4; i++)
	{
		hex2flot.data_byte[3-i] = src_8[i];//把16进制数放入联合体，待转换成浮点数
	}
	
	temp = hex2flot.data_float;// 把转换成16进制的数据，放入待发送数组中
	
	return temp;
}



//解析MA10的数据
u8 Analyze_MA10Data(void)
{
	u16 length = MA10_RX;
//	u16 bias_length,bias_len_beg,bias_len_end;
	u16 i,j;
	u8 bc = 0;
	
	u8 cond1=0,cond2=0;//寻找帧头判断条件
	
	
	for (j = 0; j + 64 <= length; j++)//搜寻完整一帧数据
	{
		(data_Buf[j] != (uint8_t)0x55) ? (cond1=0) : (cond1=1);//寻找帧头判断条件1
		(data_Buf[j+1] != (uint8_t)0x41) ? (cond2=0) : (cond2=1);//寻找帧头判断条件2
		
		if ( !(cond1 && cond2) )
		{
			continue;//寻找帧头
		}
//		printf("找到帧头\r\n");
		

		for (i = 0; i < 64; i++)
		{
			bc += data_Buf[i + j];//求接收到的所有数据之和(区别于TL740D的排除帧头的求和方法)，用于校验
		}
		
		

		if (bc == data_Buf[j + 64])//数据校验
		{
//			if(data_Buf[j+1] == 0x41)//判断姿态角帧头2
//			{
//				z_heading = HEX2FLOAT((uint8_t *)(&data_Buf[j + 2]));//航向角YAW 单位：度
//			}
//			
//			if(data_Buf[j+15] == 0x42)//判断加速度帧头2
//			{
//				forward_accel = HEX2FLOAT((uint8_t *)(&data_Buf[j + 16]));//加速度 X 单位：g
//			}
			
			if(data_Buf[j+29] == 0x43)//判断陀螺仪帧头2
			{
				z_rate = HEX2FLOAT((uint8_t *)(&data_Buf[j + 38]));//陀螺仪z 单位：deg/s
			}
			return 1;
		}
	}
	return 0;
}


void external_imu_setup(void)
{
	u16 i;
	static u8 auto_i=0;
	
	// 开机后自动发送 3 次 IMU启动字符串，每次间隔 1 秒
	for(; auto_i < 3; auto_i++)
	{
		for(i=0;i<MA10_TX+1;i++)
		{
			TX_buf[i]=data_run[i];
		}
		
		Set_Data_Num(Gyro_TX_DMA_Stream,MA10_TX);
		
//		delay_ms(1000);
//		printf("进入自动启动IMU代码一次\r\n");
	}
	
	
	if(key_local==2) // 按下按键KEY1时，向IMU发送开始触发数据
	{	
		if( key_local==2 )
		{
			beep_on(1,100);
		}		
		
		printf("\r\nF407: 用户按键 KEY1 已按下!\r\n");
		printf("\r\n向MA10发送run指令\r\n");

		
//			while(USART_GetFlagStatus(USART6, USART_FLAG_TC)==RESET);//防止首次发生字符出现问题
		while((USART6->SR&0X40)==0);//Flag_Show=0  发送完成判断
//			USART_SendArray(USART6, data_run, 7);
//			
		for(i=0;i<MA10_TX+1;i++)
		{
			TX_buf[i]=data_run[i];
		}
		
		Set_Data_Num(Gyro_TX_DMA_Stream,MA10_TX);
	}
		
		
			
		
	if(key_local==3)  // 按下开发板按键KEY2，向MA10发送数据停止命令
	{
		beep_on(1,100);
		
		printf("\r\nF407: 用户按键 KEY2 已按下!\r\n");
		printf("\r\n向MA10发送stop指令\r\n");

		while((USART6->SR&0X40)==0);//Flag_Show=0  发送完成判断			
		
		for(i=0;i<MA10_TX+1;i++)
		{
			TX_buf[i]=data_stop[i];
		}	
		
		Set_Data_Num(Gyro_TX_DMA_Stream,MA10_TX);
		
		for(i=0;i<MA10_RX;i++)
		{
			RX_buf[i]=0;
		}	
	}
	
	memcpy(data_Buf, RX_buf, MA10_RX);
	
	if(Analyze_MA10Data() == 0)//解析MA10数据
	{
		printf("MA10数据校验不成功\r\n");
	}

//	printf("\r\n陀螺仪z  z_rate：       %f deg/s\r\n",z_rate);
}








