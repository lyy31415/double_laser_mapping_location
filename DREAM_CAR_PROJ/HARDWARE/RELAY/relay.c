#include "relay.h"


void relay_gpio_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

  //GPIOF9,F10��ʼ������
  GPIO_InitStructure.GPIO_Pin = RELAY_L_Pin | RELAY_R_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(RELAY_L_PORT, &GPIO_InitStructure);//��ʼ��
	
//	GPIO_SetBits(RELAY_L_PORT,RELAY_L_Pin | RELAY_R_Pin);
	GPIO_ResetBits(RELAY_L_PORT,RELAY_L_Pin | RELAY_R_Pin);
}

