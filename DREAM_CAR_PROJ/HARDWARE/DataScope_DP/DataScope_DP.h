#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H

#include "sys.h"
#include "usart.h"
#include "GYRO.h"
#include "motor.h"

void Float2Byte(float *target,unsigned char *buf,unsigned char beg);
void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // 写通道数据至 待发送帧数据缓存区
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // 发送帧数据生成函数 
void DataScope(pid_struct pid);//往上位机发送数据
void DataScope_USART2(void);
 
#endif 



