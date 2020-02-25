#include "location.h"
#include "tri_lookup.h"
#include "mpu6050.h"
#include "inv_mpu.h"


extern u8 timeout_triple;


static float *pos_encoder;

float times_enco_pos_temp;//由于采用一个定时器对三处进行分别计时，所以需要分三段分别计时，再根据需要进行时间相加
float times_sensor_angle_temp;
float times_agv_pos_temp;

float theta_overall;//kalman滤波计算出的每次角度增量之和，即累加角度，
float kalman_theta;//假设每个周期内角速度kalman_omga恒定，计算出来的角度增量
float kalman_omga;

float encdr_data[2];//编码器数据数组

float times_enco_pos;//每次进行编码器角速度计算的时间间隔   单位：秒
float times_sensor_angle;//每次进行kalman噪音计算的时间间隔   单位：秒
float times_agv_pos;//每次进行agv坐标计算的时间间隔   单位：秒

arm_matrix_instance_f32 pos_encdr_mat;
arm_matrix_instance_f32 encdr_mat;//左右编码器数据矩阵 encdr_mat[0]=right; encdr_mat[1]=left;
arm_matrix_instance_f32 encdr_trans_mat;//编码器数据 转换 成agv速度的矩阵
arm_matrix_instance_f32 encdr_trans_mat_for_inv;//为求逆设置的备份，因为Gauss-Jordan法求逆会把源数据变换成单位矩阵！！！

float w_init_data;
float w_overall;

float v_init_data;
float v_overall;
	
	
static float pos_a[3];// agv位于 a 点的位姿 pos_a[0]: x, pos_a[1]: y, pos_a[2]: theta
float pos_b[3];// agv位于 b 点的位姿


float mat_int_data[2];
static float mat_int_data_for_inv[4];

int32_t Encoder_Left,Encoder_Right;
float Enco_wL,Enco_wR;//左右编码器角速度 单位: 度/秒

short robot_temperature;
short robot_gx, robot_gy, robot_gz;
u8 gyro_status;
static float imu_present_w;
float v_min, omega_min;

u8 pos_clear_flag = 0;

//转换矩阵初始化数据
static float trans_data[4]={0.5f*WHEEL_RADIUS, 0.5f*WHEEL_RADIUS, 
													WHEEL_RADIUS*INV_DISTANCE_OF_WHEEL, -WHEEL_RADIUS*INV_DISTANCE_OF_WHEEL};//转换矩阵初始化数据


void matrix_Init(void)
{
	float *omega_tmp;
	
	arm_mat_init_f32(&pos_encdr_mat, 2, 1, mat_int_data);
	arm_mat_init_f32(&encdr_trans_mat_for_inv, 2, 2, mat_int_data_for_inv);
	arm_mat_init_f32(&encdr_trans_mat, 2, 2, trans_data);
	
	*encdr_trans_mat_for_inv.pData = *encdr_trans_mat.pData;//直接用元素赋值，防止通过指针赋值改变源数据
	*(encdr_trans_mat_for_inv.pData + 1) = *(encdr_trans_mat.pData + 1);
	*(encdr_trans_mat_for_inv.pData + 2) = *(encdr_trans_mat.pData + 2);
	*(encdr_trans_mat_for_inv.pData + 3) = *(encdr_trans_mat.pData + 3);
	
	encdr_data[0] = omegaR_min;
	encdr_data[1] = omegaL_min;
	
	arm_mat_init_f32(&encdr_mat, 2, 1, encdr_data);
	arm_mat_mult_f32(&encdr_trans_mat, &encdr_mat, &pos_encdr_mat);
  omega_tmp = pos_encdr_mat.pData;
	v_min = omega_tmp[0];
	omega_min = omega_tmp[1];
	
	printf("matrix_Init完成\r\n");
	printf("v_min=%f\r\n",v_min);
	printf("omega_min=%f\r\n",omega_min);
	printf("AGV_Max_omega=%f\r\n",AGV_Max_omega);
	printf("AGV_Min_omega=%f\r\n",AGV_Min_omega);
	printf("AGV_Max_v=%f\r\n",AGV_Max_v);
	printf("AGV_Min_v=%f\r\n",AGV_Min_v);
}


float* pos_from_encoder(void)
{
	float * pos;
	
	Encoder_Left=Read_Encoder(4);//定时器清零放入此函数中
	Encoder_Right=Read_Encoder(3);//定时器清零放入此函数中
	
	times_enco_pos=0.0001f * ( TIM_GetCounter(CAL_POS_TIM) + (u32)timeout_triple * 65536 );//计算运行时间 单位：秒
	TIM_SetCounter(CAL_POS_TIM,0);//重设TIM2定时器的计数器值
	timeout_triple=0;//定时器超时次数
	
	times_enco_pos_temp=times_sensor_angle + times_agv_pos + times_enco_pos;//计算从上次此处清零后开始，到本次清零时，所经历的三段时间之和
//	printf("times_enco_pos_temp = %f\r\n",times_enco_pos_temp);
	
	Enco_wL = Encoder_Left * Enco_rad_per_pulse / times_enco_pos_temp;                    // 得到编码器每秒转的度数 ，单位：弧度/秒
	Enco_wR = Encoder_Right * Enco_rad_per_pulse / times_enco_pos_temp;                   // 得到编码器每秒转的度数 ，单位：弧度/秒
	
	encdr_data[0] = Enco_wR; 
	encdr_data[1] = Enco_wL; 
	arm_mat_init_f32(&encdr_mat, 2, 1, encdr_data);
	
//	printf("实际右轮角速度encdr_data[0]:%f\r\n",encdr_data[0]);
//	printf("实际左轮角速度encdr_data[1]:%f\r\n",encdr_data[1]);
	
	arm_mat_mult_f32(&encdr_trans_mat, &encdr_mat, &pos_encdr_mat);
  pos = pos_encdr_mat.pData;
	
	return pos;
}


void cal_sensor_angle(void)
{
	times_sensor_angle=0.0001f * ( TIM_GetCounter(CAL_POS_TIM) + (u32)timeout_triple * 65536 );//计算运行时间 单位：秒
  TIM_SetCounter(CAL_POS_TIM,0);//重设TIM2定时器的计数器值
	timeout_triple=0;//定时器超时次数
	times_sensor_angle_temp=times_agv_pos + times_enco_pos + times_sensor_angle;//计算从上次此处清零后开始，到本次清零时，所经历的三段时间之和
		
	robot_temperature = MPU_Get_Temperature();
	gyro_status = MPU_Get_Gyroscope(&robot_gx, &robot_gy, &robot_gz);
	pos_encoder = pos_from_encoder();
	
	imu_present_w = robot_gz * 0.061f; // 单位：度/秒
	
	if(fabs(imu_present_w) < 1.0f)
	{
		imu_present_w = 0.0f;
	}
	
	v_init_data =  pos_encoder[0];// 根据左右编码器计算的 agv 线速度   米/秒
	w_init_data =  pos_encoder[1] * 57.295779f;// 根据左右编码器计算的 agv 角速度 单位：度/秒

//	printf("imu_present_w: %f °/s\r\n",imu_present_w);
}




void cal_agv_pos(void)
{
	u8 i;
	
	v_overall = v_init_data;//米/秒
	
	if(fabs(w_init_data) < 0.5*fabs(imu_present_w))
	{
		w_overall = imu_present_w;//单位：度/秒
	}
	else
	{
		w_overall = w_init_data;//单位：度/秒
	}
	
	times_agv_pos=0.0001f * ( TIM_GetCounter(CAL_POS_TIM) + (u32)timeout_triple * 65536 );//计算运行时间 单位：秒
	TIM_SetCounter(CAL_POS_TIM,0);//重设TIM2定时器的计数器值
	timeout_triple=0;//定时器超时次数
	times_agv_pos_temp=times_enco_pos + times_sensor_angle + times_agv_pos;//计算从上次此处清零后开始，到本次清零时，所经历的三段时间之和
	
//	printf("times_agv_pos_temp = %f\r\n",times_agv_pos_temp);
	theta_overall += times_agv_pos_temp * w_overall;
	
	pos_b[0] = pos_a[0] + v_overall * times_agv_pos_temp * Cos_Lookup(theta_overall);//计算 agv x轴坐标，单位：米
	pos_b[1] = pos_a[1] + v_overall * times_agv_pos_temp * Sin_Lookup(theta_overall);//计算 agv y轴坐标，单位：米
	pos_b[2] = theta_overall;//计算 agv 在世界坐标系中的角度  单位：度
	
	for(i=0;i<3;i++)
	{
		pos_a[i] = pos_b[i];//把计算出的新坐标存储，用于下一次估算
	}
	
	if(pos_clear_flag == 1)
	{
		pos_clear_flag = 0;
		
		pos_a[0] = 0.0f;
		pos_b[0] = 0.0f;
		pos_a[1] = 0.0f;
		pos_b[1] = 0.0f;
		pos_a[2] = 0.0f;
		pos_b[2] = 0.0f;
		theta_overall = 0.0f;
	}
}










