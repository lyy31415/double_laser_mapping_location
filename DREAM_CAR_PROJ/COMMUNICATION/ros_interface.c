#include "ros_interface.h"


float vw_from_ros[3];
static Mcu2PcFrame_Typedef mcu2pcFrame;
static pc2mcuFrame_Typedef pc2mcuFrame;


#define RXFRAMEBUFSIZE  (int)(PC2MCULEN+PC2MCULEN)
static uint8_t PC2MCUFrameBuf[RXFRAMEBUFSIZE];



/** ����STM32 �� ROS֮������� **/
void handle_data(void)
{
	static u8 count = 0;
	if(++count > 20)
	{
		count = 0;
		recieve_from_ros();// ��ros��������
		send_to_ros();// ��ros��������	
	}
}



/**
		��ros����λ�á��Ƕȡ��ٶȡ����ٶȵ���Ϣ
*/
void send_to_ros(void)
{
//	uint16_t temp1;
	
	if(Ros_Uart_Port->SR&USART_FLAG_TC)// ���ݷ�����ɺ�
	{
//		//��� USART_IT_TC ��־����ȡUSART_SR�Ĵ�����Ȼ��д��USART_DR�Ĵ��������TC ��־λ
//		temp1 = Ros_Uart_Port->SR;
//		Ros_Uart_Port->DR = (u8)temp1;
		Ros_Uart_Port->SR &= ~USART_FLAG_TC;// TC λҲ����ͨ����USART_SR��λд�롮0��������
		
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
		
		ROS_Set_Data_Num(Ros_TX_DMA_Stream,MCU2PCLEN);// ÿ�δ����������������DMA�����������Ϳ��
	}
}



/**
		����ros�·������ٶȺͽ��ٶ�
*/
void recieve_from_ros(void)
{
	uint8_t csum=0, pRead=0;
	
	for(pRead = 0; pRead < RXFRAMEBUFSIZE-PC2MCULEN+1; pRead++)
	{
		
		if((PC2MCUFrameBuf[pRead] == PC2MCU_HEAD) && (PC2MCUFrameBuf[pRead+PC2MCULEN-1] == PC2MCU_END))
		{
			csum = checkSum((uint8_t *)(&PC2MCUFrameBuf[pRead]),PC2MCULEN-2);
//			printf("�ҵ�֡ͷ��֡β\r\n");
			if(PC2MCUFrameBuf[pRead+PC2MCULEN-2] == csum)
			{
				pc2mcuFrame.frameHead = PC2MCUFrameBuf[pRead+0];
				pc2mcuFrame.robot_vel_float_x = *((float*)(&PC2MCUFrameBuf[pRead+1]));
				pc2mcuFrame.robot_vel_float_y	= *((float*)(&PC2MCUFrameBuf[pRead+5]));
				pc2mcuFrame.robot_omega_float_z = *((float*)(&PC2MCUFrameBuf[pRead+9]));
				pc2mcuFrame.isPosClear = *((uint8_t*)(&PC2MCUFrameBuf[pRead+13])) & 0x1;
				
				pc2mcuFrame.checksum = PC2MCUFrameBuf[pRead+14];
				pc2mcuFrame.frameEnd = PC2MCUFrameBuf[pRead+15];
//				printf("֡У��ɹ� \r\n");
//				printf("pc2mcuFrame�ṹ�峤��: %d",sizeof(pc2mcuFrame));
				break;
			}
			else
			{
				memset((uint8_t *)(&pc2mcuFrame),0,sizeof(pc2mcuFrame));
//				printf("֡У��ʧ�� \r\n");
			}	
		}
		else
		{
//			printf("δ�ҵ�֡ͷ��֡β������������\r\n");
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





//����һ��DMA����
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:���ݴ�����  
void ROS_DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //�ر�DMA���� 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//ȷ��DMA���Ա�����  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //���ݴ�����  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA���� 
}


//������ڿ����жϣ��ȶ���SR���ٶ���DR���IDLE�жϱ�־
//�����ȡ���ݼĴ����ǿ��жϣ�ͨ���� USART_DR �Ĵ���ִ�ж����������λ���㣬RXNE ��־Ҳ����ͨ�����λд���������㡣
//void ROS_Clear_IDLE_Flag(USART_TypeDef* USARTx)
//{	
//	uint16_t temp;
//	temp = USARTx->SR;
//	temp = USARTx->DR;
//}


//�ر�DMA������DMA�������ݵ���������DMAͨ��������DMAʣ��δ�������ݵ�����
uint32_t ROS_Set_Data_Num(DMA_Stream_TypeDef *DMA_Streamx, uint16_t number)
{
	/*
	* ��stm32f4���ֲ��У�DMA-SR�Ĵ���˵��������һ�仰
	* �� EN λ�á�1���������´���֮ǰ�� DMA_LISR �� DMA_HISR �Ĵ����������������Ӧ���¼���־��������
	*
	* �ڴ������ǰ��ֹ��������ENλ���㣩���������������¼�
	*/

	uint8_t x = 0;		   //ָʾ��ǰ���ĸ�����DMAy_Streamx
	uint32_t add_temp = 0; //�������޸ļĴ����ĵ�ַ
	uint32_t temp = 0;
	uint32_t remaining;

	DMA_Cmd(DMA_Streamx, DISABLE);                      //�ر�DMA���� 
	if (DMA_Streamx < DMA2_Stream0)
	{
		x = ((uint32_t)DMA_Streamx - (uint32_t)DMA1_Stream0) / 0x18; //��ȡ��ǰ��������
		add_temp = (uint32_t) & (DMA1->LIFCR) + (x & (0x04));//����DMA_Streamx����Ӧ�ļĴ���(DMA1->LIFCR��DMA1->HIFCR)��ַ   x&(0x04)��һ���ǰ�DMA1_Stream4 �Ժ������Ӧ�� DMA1->HIFCR�Ĵ�����
	}
	else
	{
		x = ((uint32_t)DMA_Streamx - (uint32_t)DMA2_Stream0) / 0x18;
		add_temp = (uint32_t) & (DMA2->LIFCR) + (x & (0x04)); //����DMA_Streamx����Ӧ�ļĴ���(DMA2->LIFCR��(DMA2->HIFCR))��ַ  x&(0x04)��һ���ǰ�DMA2_Stream4 �Ժ������Ӧ�� DMA2->HIFCR�Ĵ�����
	}
	
	//temp = x & 0x02;
	//temp = temp<< 3;
	//temp = (0x3D << (6 * (x & 0x01)))<<((x&0x02)<<3);
	//*(uint32_t*)add_temp = temp;
	temp = (0x3D << (6 * (x & 0x01))) << ((x & 0x02) << 3);
	*(uint32_t *)add_temp = temp;//��� DMA_Streamx ����Ӧ DMA->LIFCR �� DMA->HIFCR�е� �жϱ�־�����Ӧ��־λд��1������ 
	//*(uint32_t*)add_temp = (0x3D << (6 * (x & 0x01))) << ((x & 0x02) << 3);	

	remaining = DMA_Streamx->NDTR;
	DMA_Streamx->NDTR = number;
	DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA���� 
	return remaining;
}



void ROS_DMA_Config(DMA_Stream_TypeDef *DMA_Stream_Tx, DMA_Stream_TypeDef *DMA_Stream_Rx)
{ 
	//GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	uint16_t clear_temp;
	
	
	
	
	/********************************* DMA��ʼ�� *********************************/
	if( ((u32)DMA_Stream_Tx >(u32)DMA2) || ((u32)DMA_Stream_Rx >(u32)DMA2) )//��ʼ��DMAʱ��
	{
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2ʱ��ʹ�� 
		
	}
	if(  ((u32)DMA_Stream_Tx <(u32)DMA2) || ((u32)DMA_Stream_Rx <(u32)DMA2) )
	{
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ�� 
	}
  DMA_DeInit(DMA_Stream_Tx);
	DMA_DeInit(DMA_Stream_Rx);
	
	while (DMA_GetCmdStatus(DMA_Stream_Tx) != DISABLE){}//�ȴ�DMA������ 
	while (DMA_GetCmdStatus(DMA_Stream_Rx) != DISABLE){}//�ȴ�DMA������ 
	

		
  /* ���� DMA Stream */
  DMA_InitStructure.DMA_Channel = Ros_TX_DMA_Channel;  //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&Ros_Uart_Port->DR);//DMA�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&mcu2pcFrame;//DMA �洢��0��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
  DMA_InitStructure.DMA_BufferSize = (u32)MCU2PCLEN;//���ݴ����� 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
  DMA_Init(DMA_Stream_Tx, &DMA_InitStructure);//��ʼ��DMA Stream
		
	ROS_DMA_Enable(DMA_Stream_Tx,(u16)MCU2PCLEN);


	//���ý����ж�
	DMA_InitStructure.DMA_BufferSize = (u32)RXFRAMEBUFSIZE;
	DMA_InitStructure.DMA_Channel = Ros_RX_DMA_Channel;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //���赽�ڴ�
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&PC2MCUFrameBuf;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA_Stream_Rx, &DMA_InitStructure);//��ʼ��DMA Stream
	
	ROS_DMA_Enable(DMA_Stream_Rx,(u16)RXFRAMEBUFSIZE);
		/********************************* DMA��ʼ�� *********************************/
	
	
	
	
	
	/******************************���ڳ�ʼ��***********************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
	
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(Ros_Uart_TX_Port,Ros_Uart_TX_PinSource,Ros_Uart_TX_AF); //GPIOB10����ΪUSART3   TX
	GPIO_PinAFConfig(Ros_Uart_RX_Port,Ros_Uart_RX_PinSource,Ros_Uart_RX_AF); //GPIOB11����ΪUSART3  RX
	
	//USART3�˿�����
  GPIO_InitStructure.GPIO_Pin = Ros_Uart_TX_Pin; //GPIOB10  TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(Ros_Uart_TX_Port,&GPIO_InitStructure); //��ʼ��GPIOB
	
	GPIO_InitStructure.GPIO_Pin = Ros_Uart_RX_Pin; //GPIOB11  RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //
	GPIO_Init(Ros_Uart_RX_Port,&GPIO_InitStructure); //��ʼ��GPIOB

   //USART3 ��ʼ������
//	USART_InitStructure.USART_BaudRate = 460800;//����������
	USART_InitStructure.USART_BaudRate = 230400;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(Ros_Uart_Port, &USART_InitStructure); //��ʼ������3
	

	//Usart3 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = Ros_Uart_IRQn;//����3�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ROS_PreemptionPriority;//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = ROS_SubPriority;		//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���
	
		
	USART_ITConfig(Ros_Uart_Port, USART_IT_RXNE, ENABLE);//�������շǿ��жϣ������ڽ��յ����ݣ�������ж�
	USART_DMACmd(Ros_Uart_Port, USART_DMAReq_Rx, ENABLE);//��������DMA����
	USART_DMACmd(Ros_Uart_Port, USART_DMAReq_Tx, ENABLE);//��������DMA����
	USART_Cmd(Ros_Uart_Port, ENABLE);  //ʹ�ܴ���3
/******************************���ڳ�ʼ��***********************************/
	
	
	//��� USART_IT_RXNE �жϱ�־��ͨ���� USART_DR �Ĵ���ִ�ж��������RXNE ��־λ����
	clear_temp = Ros_Uart_Port->DR;
	
//	//��� USART_IT_TC �жϱ�־����ȡUSART_SR�Ĵ�����Ȼ��д��USART_DR�Ĵ��������TC ��־λ
//	clear_temp = Ros_Uart_Port->SR;
//	Ros_Uart_Port->DR = clear_temp;
} 





void Ros_Uart_IRQHandler(void)                	//����3�жϷ������
{
	if (Ros_Uart_Port->SR&USART_FLAG_RXNE)	//���շǿ��ж�
	{
		uint16_t temp1;
		
		
		//��� USART_IT_RXNE �жϱ�־��ͨ���� USART_DR �Ĵ���ִ�ж��������RXNE ��־λ����
		temp1 = Ros_Uart_Port->DR;
		
		ROS_Set_Data_Num(Ros_RX_DMA_Stream,RXFRAMEBUFSIZE);// ÿ�δ����������������DMA�����������Ϳ��
	}
	
//	printf("����ROS�����ж�\r\n");
}
