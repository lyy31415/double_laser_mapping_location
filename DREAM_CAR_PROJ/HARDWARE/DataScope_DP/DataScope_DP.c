 #include "DataScope_DP.h"

static unsigned char DataScope_OutPut_Buffer[42] = {0};	   //���ڷ��ͻ�����



//����˵�����������ȸ�������ת��4�ֽ����ݲ�����ָ����ַ 
//����˵�����û�����ֱ�Ӳ����˺��� 
//target:Ŀ�굥��������
//buf:��д������
//beg:ָ��������ڼ���Ԫ�ؿ�ʼд��
//�����޷��� 
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //�õ�float�ĵ�ַ
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
 
 
//����˵������������ͨ���ĵ����ȸ�������д�뷢�ͻ�����
//Data��ͨ������
//Channel��ѡ��ͨ����1-10��
//�����޷��� 
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
	if ( (Channel > 10) || (Channel == 0) ) return;  //ͨ����������10�����0��ֱ����������ִ�к���
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


//����˵�������� DataScopeV1.0 ����ȷʶ���֡��ʽ
//Channel_Number����Ҫ���͵�ͨ������
//���ط��ͻ��������ݸ���
//����0��ʾ֡��ʽ����ʧ�� 
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //ͨ����������10�����0��ֱ����������ִ�к���
  else
  {	
	 DataScope_OutPut_Buffer[0] = '$';  //֡ͷ
		
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



//����λ����������
void DataScope(pid_struct pid)
{   
		unsigned char Send_Count; //������Ҫ���͵����ݸ���
		unsigned char i;          //��������
	
		DataScope_Get_Channel_Data( pid.Ka, 1 );       //��ʾ����ֵ
		DataScope_Get_Channel_Data( pid.Kp, 2 );       //��ʾ����ϵ��
		DataScope_Get_Channel_Data( pid.Ki, 3 );       //��ʾ����ϵ��
		DataScope_Get_Channel_Data( pid.Kd , 4 );      //��ʾ΢��ϵ��
//	  DataScope_Get_Channel_Data(enc_omega[1], 5 );
//	  DataScope_Get_Channel_Data(enc_omega[0], 6 ); 
		DataScope_Get_Channel_Data(w_overall, 5 );// ��ǰ���ٶ�
	  DataScope_Get_Channel_Data(v_overall, 6 );// ��ǰ���ٶ�
//	  DataScope_Get_Channel_Data(encdr_data[1], 7 );  // �������ٶ� rad/s
//		DataScope_Get_Channel_Data(encdr_data[0] , 8 );//  �ҵ�����ٶ� rad/s
		DataScope_Get_Channel_Data(target_omega_after_acc , 7);// �Ӽ��ٷֽ��Ľ��ٶ�
		DataScope_Get_Channel_Data(target_v_after_acc, 8);  // �Ӽ��ٷֽ������ٶ�
		DataScope_Get_Channel_Data(target_omega_before_acc , 9);// �Ӽ��ٷֽ��Ľ��ٶ�
		DataScope_Get_Channel_Data(target_v_before_acc, 10);  // �Ӽ��ٷֽ������ٶ�
//	  DataScope_Get_Channel_Data(target_omega, 9 );// ��λ����Ŀ�� ���ٶ�
//		DataScope_Get_Channel_Data(target_v, 10 ); //��λ����Ŀ�� ���ٶ�
//		DataScope_Get_Channel_Data(Encoder_Left*0.01, 9 );// ��λ����Ŀ�� ���ٶ�
//		DataScope_Get_Channel_Data(Encoder_Right*0.01, 10 ); //��λ����Ŀ�� ���ٶ�
//	  DataScope_Get_Channel_Data(z_rate, 7 );  //IMU���ٶ�
//	  DataScope_Get_Channel_Data(pos_b[0], 8 );// ����IMU���ٶȼ�������� agv ���ٶ�
//	  DataScope_Get_Channel_Data(pos_b[1] , 9 );//�����������agv ���ٶ�
//		DataScope_Get_Channel_Data(pos_b[2], 10 ); //�����������agv ���ٶ�
		

		Send_Count = DataScope_Data_Generate(10);
		for( i = 0 ; i < Send_Count; i++) 
		{
		while((Scope_Uart_Port->SR&0X40)==0);  
		Scope_Uart_Port->DR = DataScope_OutPut_Buffer[i]; 
		}
}


void DataScope_USART2(void)
{
	//GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
	
	/////////////////////// ���ڳ�ʼ�� //////////////////////////////////
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹRCC_AHB1Periph_GPIOA��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
	
	//����6��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(Scope_Uart_TX_Port,Scope_Uart_TX_PinSource,Scope_Uart_TX_AF); //GPIOA2����ΪUSART2   TX
	GPIO_PinAFConfig(Scope_Uart_RX_Port,Scope_Uart_RX_PinSource,Scope_Uart_RX_AF); //GPIOA3����ΪUSART2  RX
	
	//USART2�˿�����
  GPIO_InitStructure.GPIO_Pin = Scope_Uart_TX_Pin; //GPIOA2  TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(Scope_Uart_TX_Port,&GPIO_InitStructure); //��ʼ��PA2
	
	GPIO_InitStructure.GPIO_Pin = Scope_Uart_RX_Pin; //GPIOA3  RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //
	GPIO_Init(Scope_Uart_RX_Port,&GPIO_InitStructure); //��ʼ��PA3

   //USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = 256000;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(Scope_Uart_Port, &USART_InitStructure); //��ʼ������2
	

//	//Usart2 NVIC ����
//  NVIC_InitStructure.NVIC_IRQChannel = Scope_Uart_IRQn;//����2�ж�ͨ��
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SCOPE_PreemptionPriority;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SCOPE_SubPriority;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//	
//	USART_ITConfig(Scope_Uart_Port, USART_IT_IDLE, ENABLE);//���������жϣ������߿��У�������ж�
	
	USART_Cmd(Scope_Uart_Port, ENABLE);  //ʹ�ܴ���2
}



