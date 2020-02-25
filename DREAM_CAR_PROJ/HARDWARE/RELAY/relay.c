#include "relay.h"


void relay_gpio_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = RELAY_L_Pin | RELAY_R_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(RELAY_L_PORT, &GPIO_InitStructure);//初始化
	
//	GPIO_SetBits(RELAY_L_PORT,RELAY_L_Pin | RELAY_R_Pin);
	GPIO_ResetBits(RELAY_L_PORT,RELAY_L_Pin | RELAY_R_Pin);
}

