#ifndef __BEEP_H
#define __BEEP_H	 
#include "sys.h" 
#include "delay.h"



//LED端口定义
#define BEEP PFout(8)	// 蜂鸣器控制IO 

void BEEP_Init(void);//初始化		 


// 蜂鸣器响起times次，每次nms时间
void beep_on(u8 times, u8 nms);
	
#endif

















