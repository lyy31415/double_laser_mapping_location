#ifndef GYRO_H
#define GYRO_H


#include "sys.h"
#include "string.h"
#include "math.h"
#include "usart.h"
#include "delay.h"
#include "encoder.h"
#include "beep.h"

#define MA10_TX 6 
#define MA10_RX 140

////浮点与16进制的转换，可以直接读内存实现，通过联合体获取浮点与16进制
//typedef	union{float fv; uint8_t cv[4];}  float_union;

extern uint8_t RX_buf[MA10_RX];
extern char TX_buf[MA10_TX+1];
extern uint8_t data_Buf[MA10_RX];
extern float z_rate;		 //Z轴角速率(°/s)
extern float z_heading;	 //Z轴方位角，-180°~+180°
extern float forward_accel;	//前向加速度(mm/s2)



void GYRODMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void Clear_IDLE_Flag(USART_TypeDef* USARTx);
uint32_t Set_Data_Num(DMA_Stream_TypeDef *DMA_Streamx, uint16_t number);
void GYRODMA_Config(DMA_Stream_TypeDef *DMA_Stream_Tx, DMA_Stream_TypeDef *DMA_Stream_Rx);
float HEX2FLOAT(uint8_t *src_8);
u8 Analyze_MA10Data(void);
void external_imu_setup(void);



#endif


