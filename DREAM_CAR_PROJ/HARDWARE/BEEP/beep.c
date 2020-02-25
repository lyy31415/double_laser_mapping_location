#include "beep.h" 



//初始化PF8为输出口		    
//BEEP IO初始化
void BEEP_Init(void)
{   
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOF时钟
  
  //初始化蜂鸣器对应引脚GPIOF8
  GPIO_InitStructure.GPIO_Pin = BEEP_GPIO_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStructure);//初始化GPIO
	
  GPIO_ResetBits(BEEP_GPIO_Port,BEEP_GPIO_Pin);  //蜂鸣器对应引脚GPIOF8拉低
}


// 蜂鸣器响起times次，每次nms时间
void beep_on(u8 times, u8 nms)
{
	u8 i;
	
	for(i=0;i<times;i++)
	{
		BEEP=1;//蜂鸣器开启
	  delay_ms(nms);
	  BEEP=0;//蜂鸣器关闭
		delay_ms(nms);
	}
	
}






