#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H

#include "sys.h"
#include "usart.h"
#include "GYRO.h"
#include "motor.h"

void Float2Byte(float *target,unsigned char *buf,unsigned char beg);
void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // дͨ�������� ������֡���ݻ�����
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // ����֡�������ɺ��� 
void DataScope(pid_struct pid);//����λ����������
void DataScope_USART2(void);
 
#endif 



