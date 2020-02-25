#ifndef __KALMAN_H
#define __KALMAN_H

#include "sys.h"
#include "string.h"
#include "math.h"
#include "usart.h"
#include "ENCODER.h"
#include "arm_math.h"


////////////////////////////////////////////  �ԽǶȼ���Ŀ������˲���  ///////////////////////////////////////////////////

extern float noise_process_omega;	//���������ٶ�������ִ������
extern float noise_measurement_omega;	//�����ǽ��ٶ���������������

extern float A_data[4];
extern float B_data[2];

extern float process_noise_covariance_matrix_data[4];	//ִ������Э�������
extern float measurement_noise_covariance_matrix_data[4];	//��������Э�������
extern float gain_matrix_data[4];	//����ϵ������

extern arm_matrix_instance_f32 state_variable_matrix;	//�������˲����е�״̬��
extern arm_matrix_instance_f32 vcovariance_matrix;	//�������˲����е�Э����

extern arm_matrix_instance_f32 A_matrix;	//�������˲����е�A
extern arm_matrix_instance_f32 B_matrix;	//�������˲����е�B

extern arm_matrix_instance_f32 process_noise_covariance_matrix;	//ִ������Э����
extern arm_matrix_instance_f32 measurement_noise_covariance_matrix;	//��������Э����

extern arm_matrix_instance_f32 gain_matrix;	//�������˲����е�����K

extern float state_variable_data[2];	//״̬����������
extern float vcovariance_data[4];	//Э������������
extern float process_data;
extern float measurement_data[2];

extern arm_matrix_instance_f32 process_matrix;
extern arm_matrix_instance_f32 measurement_matrix;

extern float matrix_data_temp1[10];

void kalman_Init(void);
void kalman_Init_Data(void);
void kalman_Set_Noise(float noise);	//���ÿ������˲�����ִ����������������
	//���ݿ������Ͳ���������µ�״̬��

arm_matrix_instance_f32 Kalman_Filter(arm_matrix_instance_f32 input, arm_matrix_instance_f32 measurement); 


//������һʱ�̵�״̬���Ϳ�����Ԥ����һʱ�̵�״̬��
void Forecast_State_Variable_Process_Model( arm_matrix_instance_f32 input);
void Forecast_Covariance(void);	//Ԥ��Э�������
void Cal_Kalman_Gain(void);//���㿨�����������
void Update_State_Variable( arm_matrix_instance_f32 measurement );	//ʹ�ò���ֵ����״̬��
void Update_Covariance(void);	//����Э����



#endif
