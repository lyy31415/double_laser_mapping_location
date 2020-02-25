#ifndef __KALMAN_H
#define __KALMAN_H

#include "sys.h"
#include "string.h"
#include "math.h"
#include "usart.h"
#include "ENCODER.h"
#include "arm_math.h"


////////////////////////////////////////////  对角度计算的卡尔曼滤波器  ///////////////////////////////////////////////////

extern float noise_process_omega;	//编码器角速度噪声，执行噪声
extern float noise_measurement_omega;	//陀螺仪角速度噪声，测量噪声

extern float A_data[4];
extern float B_data[2];

extern float process_noise_covariance_matrix_data[4];	//执行噪声协方差矩阵
extern float measurement_noise_covariance_matrix_data[4];	//测量噪声协方差矩阵
extern float gain_matrix_data[4];	//增益系数矩阵

extern arm_matrix_instance_f32 state_variable_matrix;	//卡尔曼滤波器中的状态量
extern arm_matrix_instance_f32 vcovariance_matrix;	//卡尔曼滤波器中的协方差

extern arm_matrix_instance_f32 A_matrix;	//卡尔曼滤波器中的A
extern arm_matrix_instance_f32 B_matrix;	//卡尔曼滤波器中的B

extern arm_matrix_instance_f32 process_noise_covariance_matrix;	//执行噪声协方差
extern arm_matrix_instance_f32 measurement_noise_covariance_matrix;	//测量噪声协方差

extern arm_matrix_instance_f32 gain_matrix;	//卡尔曼滤波器中的增益K

extern float state_variable_data[2];	//状态量数据数组
extern float vcovariance_data[4];	//协方差数据数组
extern float process_data;
extern float measurement_data[2];

extern arm_matrix_instance_f32 process_matrix;
extern arm_matrix_instance_f32 measurement_matrix;

extern float matrix_data_temp1[10];

void kalman_Init(void);
void kalman_Init_Data(void);
void kalman_Set_Noise(float noise);	//设置卡尔曼滤波器的执行噪声、测量噪声
	//根据控制量和测量量输出新的状态量

arm_matrix_instance_f32 Kalman_Filter(arm_matrix_instance_f32 input, arm_matrix_instance_f32 measurement); 


//根据上一时刻的状态量和控制量预测这一时刻的状态量
void Forecast_State_Variable_Process_Model( arm_matrix_instance_f32 input);
void Forecast_Covariance(void);	//预测协方差矩阵
void Cal_Kalman_Gain(void);//计算卡尔曼增益矩阵
void Update_State_Variable( arm_matrix_instance_f32 measurement );	//使用测量值更新状态量
void Update_Covariance(void);	//更新协方差



#endif
