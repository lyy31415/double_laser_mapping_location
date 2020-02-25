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

extern arm_matrix_instance_f32 pos_encdr_mat;	//���ݱ��������ݼ����agv ���ٶȺͽ��ٶ�
extern arm_matrix_instance_f32 encdr_mat;//���ұ��������ݾ���
extern arm_matrix_instance_f32 encdr_trans_mat;//���������� ת�� ��agv�ٶȵľ���

void matrix_Init(void);
float* pos_from_encoder(void);//�ӱ��������ݼ���AGV�ĽǶȺͽ��ٶ�


void cal_sensor_angle(void);
void cal_agv_pos(void);




#endif


