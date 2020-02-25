#ifndef __LOCATION_H
#define __LOCATION_H

#include "sys.h"
#include "string.h"
#include "math.h"
#include "usart.h"
#include "encoder.h"
#include "arm_math.h"
#include "GYRO.h"
#include "beep.h"
#include "delay.h"


extern float mat_int_data[2];
extern float encdr_data[2];

extern arm_matrix_instance_f32 pos_encdr_mat;	//根据编码器数据计算的agv 线速度和角速度
extern arm_matrix_instance_f32 encdr_mat;//左右编码器数据矩阵
extern arm_matrix_instance_f32 encdr_trans_mat;//编码器数据 转换 成agv速度的矩阵

void matrix_Init(void);
float* pos_from_encoder(void);//从编码器数据计算AGV的角度和角速度


void cal_sensor_angle(void);
void cal_agv_pos(void);




#endif


