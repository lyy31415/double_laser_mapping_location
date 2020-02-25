#include "ros_interface.h"


float vw_from_ros[3];
static Mcu2PcFrame_Typedef mcu2pcFrame;
static pc2mcuFrame_Typedef pc2mcuFrame;


#define RXFRAMEBUFSIZE  (int)(PC2MCULEN+PC2MCULEN)
static uint8_t PC2MCUFrameBuf[RXFRAMEBUFSIZE];



/** 处理STM32 和 ROS之间的数据 **/
void handle_data(void)
{
	static u8 count = 0;
	if(++count > 20)
	{
		count = 0;
		recieve_from_ros();// 从ros接收数据
		send_to_ros();// 向ros发送数据	
	}
}



/**
		向ros发送位置、角度、速度、角速度等信息
*/
void send_to_ros(void)
{
//	uint16_t temp1;
	
	if(Ros_Uart_Port->SR&USART_FLAG_TC)// 数据发送完成后
	{
//		//清空 USART_IT_TC 标志，读取USART_SR寄存器，然后写入USART_DR寄存器，清除TC 标志位
//		temp1 = Ros_Uart_Port->SR;
//		Ros_Uart_Port->DR = (u8)temp1;
		Ros_Uart_Port->SR &= ~USART_FLAG_TC;// TC 位也可以通过向USART_SR该位写入‘0’来清零
		
//		printf("mcu2pcFrame: %d\r\n",sizeof(mcu2pcFrame));
		
		memset((uint8_t *)(&mcu2pcFrame),0,MCU2PCLEN);
		mcu2pcFrame.frameHead= PC2MCU_HEAD;

		mcu2pcFrame.isUltraStop = ultra_stop_status;
		mcu2pcFrame.isGyroDataOk = gyro_status;
		mcu2pcFrame.isMotorErrorL = motor_status_L;
		mcu2pcFrame.isMotorErrorR = motor_status_R;

		mcu2pcFrame.ultrasonic1_dist = ultrasonic_dist1;
		mcu2pcFrame.ultrasonic2_dist = ultrasonic_dist2;
		mcu2pcFrame.ultrasonic3_dist = ultrasonic_dist3;
		mcu2pcFrame.ultrasonic4_dist = ultrasonic_dist4;

		mcu2pcFrame.curRobotVel = v_overall;
		mcu2pcFrame.curRobotOmega = w_overall; 
		mcu2pcFrame.leftOmega = Enco_wL;
		mcu2pcFrame.rightOmega = Enco_wR;
		mcu2pcFrame.pos_x = pos_b[0]; 
		mcu2pcFrame.pos_y = pos_b[1];
		mcu2pcFrame.pos_theta = pos_b[2];

		mcu2pcFrame.batteryVoltage = adc_100 * 0.01f;
//		mcu2pcFrame.batteryVoltage = acc_v_count_target;
		mcu2pcFrame.temperature = robot_temperature * 0.01f;
//		mcu2pcFrame.temperature = acc_omega_count_target;
//		mcu2pcFrame.temperature = times_agv_pos_temp;

		mcu2pcFrame.checksum = checkSum((uint8_t *)(&mcu2pcFrame),MCU2PCLEN-2);
		mcu2pcFrame.frameEnd = PC2MCU_END;
		
		ROS_Set_Data_Num(Ros_TX_DMA_Stream,MCU2PCLEN);// 每次传输结束后，重新设置DMA发送数据流和宽度
	}
}



/**
		解析ros下发的线速度和角速度
*/
void recieve_from_ros(void)
{
	uint8_t csum=0, pRead=0;
	
	for(pRead = 0; pRead < RXFRAMEBUFSIZE-PC2MCULEN+1; pRead++)
	{
		
		if((PC2MCUFrameBuf[pRead] == PC2MCU_HEAD) && (PC2MCUFrameBuf[pRead+PC2MCULEN-1] == PC2MCU_END))
		{
			csum = checkSum((uint8_t *)(&PC2MCUFrameBuf[pRead]),PC2MCULEN-2);
//			printf("找到帧头和帧尾\r\n");
			if(PC2MCUFrameBuf[pRead+PC2MCULEN-2] == csum)
			{
				pc2mcuFrame.frameHead = PC2MCUFrameBuf[pRead+0];
				pc2mcuFrame.robot_vel_float_x = *((float*)(&PC2MCUFrameBuf[pRead+1]));
				pc2mcuFrame.robot_vel_float_y	= *((float*)(&PC2MCUFrameBuf[pRead+5]));
				pc2mcuFrame.robot_omega_float_z = *((float*)(&PC2MCUFrameBuf[pRead+9]));
				pc2mcuFrame.isPosClear = *((uint8_t*)(&PC2MCUFrameBuf[pRead+13])) & 0x1;
				
				pc2mcuFrame.checksum = PC2MCUFrameBuf[pRead+14];
				pc2mcuFrame.frameEnd = PC2MCUFrameBuf[pRead+15];
//				printf("帧校验成功 \r\n");
//				printf("pc2mcuFrame结构体长度: %d",sizeof(pc2mcuFrame));
				break;
			}
			else
			{
				memset((uint8_t *)(&pc2mcuFrame),0,sizeof(pc2mcuFrame));
//				printf("帧校验失败 \r\n");
			}	
		}
		else
		{
//			printf("未找到帧头和帧尾，继续向后查找\r\n");
		}
	}
	
	vw_from_ros[0] = pc2mcuFrame.robot_vel_float_x;
	vw_from_ros[1] = pc2mcuFrame.robot_vel_float_y;
	vw_from_ros[2] = pc2mcuFrame.robot_omega_float_z;
	pos_clear_flag = pc2mcuFrame.isPosClear;
//	printf("vel_x   = %f\r\n",vw_from_ros[0]);
//	printf("vel_y   = %f\r\n",vw_from_ros[1]);
//	printf("omega_z = %f\r\n",vw_from_ros[2]);
	
}



static uint8_t checkSum(uint8_t *dataBuf,uint32_t num)
{
	uint8_t sum=0, i=1;
	for(i=1;i<num;i++)
	{
		sum+=*(dataBuf+i);
	}
	return sum;
}





//开启一次DMA传输
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:数据传输量  
void ROS_DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
}


//清除串口空闲中断，先读入SR，再读入DR清除IDLE中断标志
//清除读取数据寄存器非空中断，通过对 USART_DR 寄存器执行读入操作将该位清零，RXNE 标志也可以通过向该位写入零来清零。
//void ROS_Clear_IDLE_Flag(USART_TypeDef* USARTx)
//{	
//	uint16_t temp;
//	temp = USARTx->SR;
//	temp = USARTx->DR;
//}


//关闭DMA，设置DMA传输数据的数量，打开DMA通道，返回DMA剩余未传输数据的数量
uint32_t ROS_Set_Data_Num(DMA_Stream_TypeDef *DMA_Streamx, uint16_t number)
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
		add_temp = (uint32_t) & (DMA1->LIFCR) + (x & (0x04));//保存DMA_Streamx流对应的寄存器(DMA1->LIFCR或DMA1->HIFCR)地址   x&(0x04)这一步是把DMA1_Stream4 以后的流对应到 DMA1->HIFCR寄存器上
	}
	else
	{
		x = ((uint32_t)DMA_Streamx - (uint32_t)DMA2_Stream0) / 0x18;
		add_temp = (uint32_t) & (DMA2->LIFCR) + (x & (0x04)); //保存DMA_Streamx流对应的寄存器(DMA2->LIFCR或(DMA2->HIFCR))地址  x&(0x04)这一步是把DMA2_Stream4 以后的流对应到 DMA2->HIFCR寄存器上
	}
	
	//temp = x & 0x02;
	//temp = temp<< 3;
	//temp = (0x3D << (6 * (x & 0x01)))<<((x&0x02)<<3);
	//*(uint32_t*)add_temp = temp;
	temp = (0x3D << (6 * (x & 0x01))) << ((x & 0x02) << 3);
	*(uint32_t *)add_temp = temp;//清除 DMA_Streamx 流对应 DMA->LIFCR 或 DMA->HIFCR中的 中断标志，向对应标志位写“1”清零 
	//*(uint32_t*)add_temp = (0x3D << (6 * (x & 0x01))) << ((x & 0x02) << 3);	

	remaining = DMA_Streamx->NDTR;
	DMA_Streamx->NDTR = number;
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
	return remaining;
}



void ROS_DMA_Config(DMA_Stream_TypeDef *DMA_Stream_Tx, DMA_Stream_TypeDef *DMA_Stream_Rx)
{ 
	//GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	uint16_t clear_temp;
	
	
	
	
	/********************************* DMA初始化 *********************************/
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
  DMA_InitStructure.DMA_Channel = Ros_TX_DMA_Channel;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&Ros_Uart_Port->DR);//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&mcu2pcFrame;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = (u32)MCU2PCLEN;//数据传输量 
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
		
	ROS_DMA_Enable(DMA_Stream_Tx,(u16)MCU2PCLEN);


	//配置接收中断
	DMA_InitStructure.DMA_BufferSize = (u32)RXFRAMEBUFSIZE;
	DMA_InitStructure.DMA_Channel = Ros_RX_DMA_Channel;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //外设到内存
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&PC2MCUFrameBuf;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA_Stream_Rx, &DMA_InitStructure);//初始化DMA Stream
	
	ROS_DMA_Enable(DMA_Stream_Rx,(u16)RXFRAMEBUFSIZE);
		/********************************* DMA初始化 *********************************/
	
	
	
	
	
	/******************************串口初始化***********************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
	
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(Ros_Uart_TX_Port,Ros_Uart_TX_PinSource,Ros_Uart_TX_AF); //GPIOB10复用为USART3   TX
	GPIO_PinAFConfig(Ros_Uart_RX_Port,Ros_Uart_RX_PinSource,Ros_Uart_RX_AF); //GPIOB11复用为USART3  RX
	
	//USART3端口配置
  GPIO_InitStructure.GPIO_Pin = Ros_Uart_TX_Pin; //GPIOB10  TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(Ros_Uart_TX_Port,&GPIO_InitStructure); //初始化GPIOB
	
	GPIO_InitStructure.GPIO_Pin = Ros_Uart_RX_Pin; //GPIOB11  RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //
	GPIO_Init(Ros_Uart_RX_Port,&GPIO_InitStructure); //初始化GPIOB

   //USART3 初始化设置
//	USART_InitStructure.USART_BaudRate = 460800;//波特率设置
	USART_InitStructure.USART_BaudRate = 230400;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(Ros_Uart_Port, &USART_InitStructure); //初始化串口3
	

	//Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = Ros_Uart_IRQn;//串口3中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ROS_PreemptionPriority;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = ROS_SubPriority;		//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化NVIC寄存器
	
		
	USART_ITConfig(Ros_Uart_Port, USART_IT_RXNE, ENABLE);//开启接收非空中断，若串口接收到数据，则产生中断
	USART_DMACmd(Ros_Uart_Port, USART_DMAReq_Rx, ENABLE);//开启串口DMA接收
	USART_DMACmd(Ros_Uart_Port, USART_DMAReq_Tx, ENABLE);//开启串口DMA发送
	USART_Cmd(Ros_Uart_Port, ENABLE);  //使能串口3
/******************************串口初始化***********************************/
	
	
	//清空 USART_IT_RXNE 中断标志，通过对 USART_DR 寄存器执行读入操作将RXNE 标志位清零
	clear_temp = Ros_Uart_Port->DR;
	
//	//清空 USART_IT_TC 中断标志，读取USART_SR寄存器，然后写入USART_DR寄存器，清除TC 标志位
//	clear_temp = Ros_Uart_Port->SR;
//	Ros_Uart_Port->DR = clear_temp;
} 





void Ros_Uart_IRQHandler(void)                	//串口3中断服务程序
{
	if (Ros_Uart_Port->SR&USART_FLAG_RXNE)	//接收非空中断
	{
		uint16_t temp1;
		
		
		//清空 USART_IT_RXNE 中断标志，通过对 USART_DR 寄存器执行读入操作将RXNE 标志位清零
		temp1 = Ros_Uart_Port->DR;
		
		ROS_Set_Data_Num(Ros_RX_DMA_Stream,RXFRAMEBUFSIZE);// 每次传输结束后，重新设置DMA接收数量流和宽度
	}
	
//	printf("进入ROS串口中断\r\n");
}
