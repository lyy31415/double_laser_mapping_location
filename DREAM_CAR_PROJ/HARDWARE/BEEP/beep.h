#ifndef __BEEP_H
#define __BEEP_H	 
#include "sys.h" 
#include "delay.h"



//LED�˿ڶ���
#define BEEP PFout(8)	// ����������IO 

void BEEP_Init(void);//��ʼ��		 


// ����������times�Σ�ÿ��nmsʱ��
void beep_on(u8 times, u8 nms);
	
#endif

















