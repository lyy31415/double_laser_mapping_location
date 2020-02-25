#include "GYRO.h"

char TX_buf[MA10_TX+1];			//�������ݵĻ��������������������򲻻ᷢ��
uint8_t RX_buf[MA10_RX]; //�������ݵĻ�����
uint8_t data_Buf[MA10_RX]; //�����ݴ棬���������⵽�ƻ�
float z_rate;		 //Z�������(��/s)
//float z_heading;	 //Z�᷽λ�ǣ�-180��~+180��
//float forward_accel;	//ǰ����ٶ�(mm/s2)

static char data_run[]=">@RUN1";
static char data_stop[]=">@RUN0";
data_trans hex2flot;//16����ת���ɸ��������������






//����һ��DMA����
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:���ݴ�����  
void GYRODMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //�ر�DMA���� 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//ȷ��DMA���Ա�����  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //���ݴ�����  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA���� 
}


//������ڿ����жϣ��ȶ���SR���ٶ���DR���IDLE�жϱ�־
void Clear_IDLE_Flag(USART_TypeDef* USARTx)
{	
	uint16_t temp1;
//	uint16_t temp = USARTx->SR;
//	temp = USARTx->DR;

	//��� USART_IT_RXNE �жϱ�־��ͨ���� USART_DR �Ĵ���ִ�ж��������RXNE ��־λ����
	temp1 = USARTx->DR;
}


//�ر�DMA������DMA�������ݵ���������DMAͨ��������DMAʣ��δ�������ݵ�����
uint32_t Set_Data_Num(DMA_Stream_TypeDef *DMA_Streamx, uint16_t number)
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
		add_temp = (uint32_t) & (DMA1->LIFCR) + (x & (0x04));		 //����Ĵ�����ַ
	}
	else
	{
		x = ((uint32_t)DMA_Streamx - (uint32_t)DMA2_Stream0) / 0x18;
		add_temp = (uint32_t) & (DMA2->LIFCR) + (x & (0x04)); //����Ĵ�����ַ
	}
	//temp = x & 0x02;
	//temp = temp<< 3;
	//temp = (0x3D << (6 * (x & 0x01)))<<((x&0x02)<<3);
	//*(uint32_t*)add_temp = temp;
	temp = (0x3D << (6 * (x & 0x01))) << ((x & 0x02) << 3);
	*(uint32_t *)add_temp = temp;//��������жϱ�־
	//*(uint32_t*)add_temp = (0x3D << (6 * (x & 0x01))) << ((x & 0x02) << 3);	

	remaining = DMA_Streamx->NDTR;
	DMA_Streamx->NDTR = number;
	DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA���� 
	return remaining;
}



void GYRODMA_Config(DMA_Stream_TypeDef *DMA_Stream_Tx, DMA_Stream_TypeDef *DMA_Stream_Rx)
{ 
	//GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
	
	/******************************* DMA��ʼ�� **************************************/
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
  DMA_InitStructure.DMA_Channel = Gyro_TX_DMA_Channel;  //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART6->DR);//DMA�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&TX_buf;//DMA �洢��0��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
  DMA_InitStructure.DMA_BufferSize = (uint32_t)MA10_TX;//���ݴ����� 
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
		
	GYRODMA_Enable(Gyro_TX_DMA_Stream,MA10_TX);


	//���ý����ж�
	DMA_InitStructure.DMA_BufferSize = (uint32_t)MA10_RX;
	DMA_InitStructure.DMA_Channel = Gyro_RX_DMA_Channel;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //���赽�ڴ�
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&RX_buf;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA_Stream_Rx, &DMA_InitStructure);//��ʼ��DMA Stream
	
	GYRODMA_Enable(Gyro_RX_DMA_Stream,MA10_RX);
	/******************************* DMA��ʼ�� **************************************/
	
	
	/************************** ���ڳ�ʼ�� *************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //ʹ��GPIOCʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//ʹ��USART6ʱ��
	
	//����6��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(Gyro_Uart_TX_Port,Gyro_Uart_TX_PinSource,Gyro_Uart_TX_AF); //GPIOG14����ΪUSART6   TX
	GPIO_PinAFConfig(Gyro_Uart_RX_Port,Gyro_Uart_RX_PinSource,Gyro_Uart_RX_AF); //GPIOG9����ΪUSART6  RX
	
	//USART6�˿�����
  GPIO_InitStructure.GPIO_Pin = Gyro_Uart_TX_Pin; //GPIOG14  TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(Gyro_Uart_TX_Port,&GPIO_InitStructure); //��ʼ��PG14
	
	GPIO_InitStructure.GPIO_Pin = Gyro_Uart_RX_Pin; //GPIOG9  RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //
	GPIO_Init(Gyro_Uart_RX_Port,&GPIO_InitStructure); //��ʼ��PG9

   //USART6 ��ʼ������
	USART_InitStructure.USART_BaudRate = 256000;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(Gyro_Uart_Port, &USART_InitStructure); //��ʼ������6
	

	//Usart6 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = Gyro_Uart_IRQn;//����6�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = Gyro_PreemptionPriority;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = Gyro_SubPriority;		//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	
	
	USART_ITConfig(Gyro_Uart_Port, USART_IT_RXNE, ENABLE);//�������շǿ��жϣ��������յ����ݣ�������ж�
	USART_DMACmd(Gyro_Uart_Port, USART_DMAReq_Rx, ENABLE);//��������DMA����
	USART_DMACmd(Gyro_Uart_Port, USART_DMAReq_Tx, ENABLE);//��������DMA����
	USART_Cmd(Gyro_Uart_Port, ENABLE);  //ʹ�ܴ���6
/*********************************** ���ڳ�ʼ�� *****************************************/	
	

	//�����������жϱ�־
	Clear_IDLE_Flag(USART6);	
} 
	

void Gyro_Uart_IRQHandler(void)                	//����6�жϷ������
{	
	if (USART6->SR&USART_FLAG_RXNE)	//���շǿ��ж�
	{
		//�����������жϱ�־
		Clear_IDLE_Flag(USART6);
		Set_Data_Num(Gyro_RX_DMA_Stream,MA10_RX);
	}
}



/**  float HEX2FLOAT(uint8_t *src_8) ����˵��
*
*��JYX-MA10�ͺŹߵ���������ת���ɸ�������ʽ
*
*		IEEE��׼�У�PC�洢�������ĸ�ʽ��
*		 ____________________________________________________________
*		|_S_|_____Exponent______|_________Mantissa___________________|
*		 1λ         8λ                      23λ
*		����λ    ����,��ָ��            β��,��С������
*
*��16���ƻ�����ת���ɸ�������ʽ��
*		x=(-1)^S �� (1.M) �� 2^e
*		���У�e=E-127, x�Ǹ�����, E=ָ��e������-1, ���� �ǲ���ķ���λȡ�����ֵ
*/
float HEX2FLOAT(uint8_t *src_8)
{
	u8 i;
	float temp;	
	
	/** ��16���Ƶ����ݣ�ͨ��������ת���ɸ����� **/
	for(i=0; i<4; i++)
	{
		hex2flot.data_byte[3-i] = src_8[i];//��16���������������壬��ת���ɸ�����
	}
	
	temp = hex2flot.data_float;// ��ת����16���Ƶ����ݣ����������������
	
	return temp;
}



//����MA10������
u8 Analyze_MA10Data(void)
{
	u16 length = MA10_RX;
//	u16 bias_length,bias_len_beg,bias_len_end;
	u16 i,j;
	u8 bc = 0;
	
	u8 cond1=0,cond2=0;//Ѱ��֡ͷ�ж�����
	
	
	for (j = 0; j + 64 <= length; j++)//��Ѱ����һ֡����
	{
		(data_Buf[j] != (uint8_t)0x55) ? (cond1=0) : (cond1=1);//Ѱ��֡ͷ�ж�����1
		(data_Buf[j+1] != (uint8_t)0x41) ? (cond2=0) : (cond2=1);//Ѱ��֡ͷ�ж�����2
		
		if ( !(cond1 && cond2) )
		{
			continue;//Ѱ��֡ͷ
		}
//		printf("�ҵ�֡ͷ\r\n");
		

		for (i = 0; i < 64; i++)
		{
			bc += data_Buf[i + j];//����յ�����������֮��(������TL740D���ų�֡ͷ����ͷ���)������У��
		}
		
		

		if (bc == data_Buf[j + 64])//����У��
		{
//			if(data_Buf[j+1] == 0x41)//�ж���̬��֡ͷ2
//			{
//				z_heading = HEX2FLOAT((uint8_t *)(&data_Buf[j + 2]));//�����YAW ��λ����
//			}
//			
//			if(data_Buf[j+15] == 0x42)//�жϼ��ٶ�֡ͷ2
//			{
//				forward_accel = HEX2FLOAT((uint8_t *)(&data_Buf[j + 16]));//���ٶ� X ��λ��g
//			}
			
			if(data_Buf[j+29] == 0x43)//�ж�������֡ͷ2
			{
				z_rate = HEX2FLOAT((uint8_t *)(&data_Buf[j + 38]));//������z ��λ��deg/s
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
	
	// �������Զ����� 3 �� IMU�����ַ�����ÿ�μ�� 1 ��
	for(; auto_i < 3; auto_i++)
	{
		for(i=0;i<MA10_TX+1;i++)
		{
			TX_buf[i]=data_run[i];
		}
		
		Set_Data_Num(Gyro_TX_DMA_Stream,MA10_TX);
		
//		delay_ms(1000);
//		printf("�����Զ�����IMU����һ��\r\n");
	}
	
	
	if(key_local==2) // ���°���KEY1ʱ����IMU���Ϳ�ʼ��������
	{	
		if( key_local==2 )
		{
			beep_on(1,100);
		}		
		
		printf("\r\nF407: �û����� KEY1 �Ѱ���!\r\n");
		printf("\r\n��MA10����runָ��\r\n");

		
//			while(USART_GetFlagStatus(USART6, USART_FLAG_TC)==RESET);//��ֹ�״η����ַ���������
		while((USART6->SR&0X40)==0);//Flag_Show=0  ��������ж�
//			USART_SendArray(USART6, data_run, 7);
//			
		for(i=0;i<MA10_TX+1;i++)
		{
			TX_buf[i]=data_run[i];
		}
		
		Set_Data_Num(Gyro_TX_DMA_Stream,MA10_TX);
	}
		
		
			
		
	if(key_local==3)  // ���¿����尴��KEY2����MA10��������ֹͣ����
	{
		beep_on(1,100);
		
		printf("\r\nF407: �û����� KEY2 �Ѱ���!\r\n");
		printf("\r\n��MA10����stopָ��\r\n");

		while((USART6->SR&0X40)==0);//Flag_Show=0  ��������ж�			
		
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
	
	if(Analyze_MA10Data() == 0)//����MA10����
	{
		printf("MA10����У�鲻�ɹ�\r\n");
	}

//	printf("\r\n������z  z_rate��       %f deg/s\r\n",z_rate);
}








