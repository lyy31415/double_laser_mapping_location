#include "beep.h" 



//��ʼ��PF8Ϊ�����		    
//BEEP IO��ʼ��
void BEEP_Init(void)
{   
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOFʱ��
  
  //��ʼ����������Ӧ����GPIOF8
  GPIO_InitStructure.GPIO_Pin = BEEP_GPIO_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
  GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStructure);//��ʼ��GPIO
	
  GPIO_ResetBits(BEEP_GPIO_Port,BEEP_GPIO_Pin);  //��������Ӧ����GPIOF8����
}


// ����������times�Σ�ÿ��nmsʱ��
void beep_on(u8 times, u8 nms)
{
	u8 i;
	
	for(i=0;i<times;i++)
	{
		BEEP=1;//����������
	  delay_ms(nms);
	  BEEP=0;//�������ر�
		delay_ms(nms);
	}
	
}






