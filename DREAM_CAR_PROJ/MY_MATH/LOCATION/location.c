#include "location.h"
#include "tri_lookup.h"
#include "mpu6050.h"
#include "inv_mpu.h"


extern u8 timeout_triple;


static float *pos_encoder;

float times_enco_pos_temp;//���ڲ���һ����ʱ�����������зֱ��ʱ��������Ҫ�����ηֱ��ʱ���ٸ�����Ҫ����ʱ�����
float times_sensor_angle_temp;
float times_agv_pos_temp;

float theta_overall;//kalman�˲��������ÿ�νǶ�����֮�ͣ����ۼӽǶȣ�
float kalman_theta;//����ÿ�������ڽ��ٶ�kalman_omga�㶨����������ĽǶ�����
float kalman_omga;

float encdr_data[2];//��������������

float times_enco_pos;//ÿ�ν��б��������ٶȼ����ʱ����   ��λ����
float times_sensor_angle;//ÿ�ν���kalman���������ʱ����   ��λ����
float times_agv_pos;//ÿ�ν���agv��������ʱ����   ��λ����

arm_matrix_instance_f32 pos_encdr_mat;
arm_matrix_instance_f32 encdr_mat;//���ұ��������ݾ��� encdr_mat[0]=right; encdr_mat[1]=left;
arm_matrix_instance_f32 encdr_trans_mat;//���������� ת�� ��agv�ٶȵľ���
arm_matrix_instance_f32 encdr_trans_mat_for_inv;//Ϊ�������õı��ݣ���ΪGauss-Jordan��������Դ���ݱ任�ɵ�λ���󣡣���

float w_init_data;
float w_overall;

float v_init_data;
float v_overall;
	
	
static float pos_a[3];// agvλ�� a ���λ�� pos_a[0]: x, pos_a[1]: y, pos_a[2]: theta
float pos_b[3];// agvλ�� b ���λ��


float mat_int_data[2];
static float mat_int_data_for_inv[4];

int32_t Encoder_Left,Encoder_Right;
float Enco_wL,Enco_wR;//���ұ��������ٶ� ��λ: ��/��

short robot_temperature;
short robot_gx, robot_gy, robot_gz;
u8 gyro_status;
static float imu_present_w;
float v_min, omega_min;

u8 pos_clear_flag = 0;

//ת�������ʼ������
static float trans_data[4]={0.5f*WHEEL_RADIUS, 0.5f*WHEEL_RADIUS, 
													WHEEL_RADIUS*INV_DISTANCE_OF_WHEEL, -WHEEL_RADIUS*INV_DISTANCE_OF_WHEEL};//ת�������ʼ������


void matrix_Init(void)
{
	float *omega_tmp;
	
	arm_mat_init_f32(&pos_encdr_mat, 2, 1, mat_int_data);
	arm_mat_init_f32(&encdr_trans_mat_for_inv, 2, 2, mat_int_data_for_inv);
	arm_mat_init_f32(&encdr_trans_mat, 2, 2, trans_data);
	
	*encdr_trans_mat_for_inv.pData = *encdr_trans_mat.pData;//ֱ����Ԫ�ظ�ֵ����ֹͨ��ָ�븳ֵ�ı�Դ����
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
	
	printf("matrix_Init���\r\n");
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
	
	Encoder_Left=Read_Encoder(4);//��ʱ���������˺�����
	Encoder_Right=Read_Encoder(3);//��ʱ���������˺�����
	
	times_enco_pos=0.0001f * ( TIM_GetCounter(CAL_POS_TIM) + (u32)timeout_triple * 65536 );//��������ʱ�� ��λ����
	TIM_SetCounter(CAL_POS_TIM,0);//����TIM2��ʱ���ļ�����ֵ
	timeout_triple=0;//��ʱ����ʱ����
	
	times_enco_pos_temp=times_sensor_angle + times_agv_pos + times_enco_pos;//������ϴδ˴������ʼ������������ʱ��������������ʱ��֮��
//	printf("times_enco_pos_temp = %f\r\n",times_enco_pos_temp);
	
	Enco_wL = Encoder_Left * Enco_rad_per_pulse / times_enco_pos_temp;                    // �õ�������ÿ��ת�Ķ��� ����λ������/��
	Enco_wR = Encoder_Right * Enco_rad_per_pulse / times_enco_pos_temp;                   // �õ�������ÿ��ת�Ķ��� ����λ������/��
	
	encdr_data[0] = Enco_wR; 
	encdr_data[1] = Enco_wL; 
	arm_mat_init_f32(&encdr_mat, 2, 1, encdr_data);
	
//	printf("ʵ�����ֽ��ٶ�encdr_data[0]:%f\r\n",encdr_data[0]);
//	printf("ʵ�����ֽ��ٶ�encdr_data[1]:%f\r\n",encdr_data[1]);
	
	arm_mat_mult_f32(&encdr_trans_mat, &encdr_mat, &pos_encdr_mat);
  pos = pos_encdr_mat.pData;
	
	return pos;
}


void cal_sensor_angle(void)
{
	times_sensor_angle=0.0001f * ( TIM_GetCounter(CAL_POS_TIM) + (u32)timeout_triple * 65536 );//��������ʱ�� ��λ����
  TIM_SetCounter(CAL_POS_TIM,0);//����TIM2��ʱ���ļ�����ֵ
	timeout_triple=0;//��ʱ����ʱ����
	times_sensor_angle_temp=times_agv_pos + times_enco_pos + times_sensor_angle;//������ϴδ˴������ʼ������������ʱ��������������ʱ��֮��
		
	robot_temperature = MPU_Get_Temperature();
	gyro_status = MPU_Get_Gyroscope(&robot_gx, &robot_gy, &robot_gz);
	pos_encoder = pos_from_encoder();
	
	imu_present_w = robot_gz * 0.061f; // ��λ����/��
	
	if(fabs(imu_present_w) < 1.0f)
	{
		imu_present_w = 0.0f;
	}
	
	v_init_data =  pos_encoder[0];// �������ұ���������� agv ���ٶ�   ��/��
	w_init_data =  pos_encoder[1] * 57.295779f;// �������ұ���������� agv ���ٶ� ��λ����/��

//	printf("imu_present_w: %f ��/s\r\n",imu_present_w);
}




void cal_agv_pos(void)
{
	u8 i;
	
	v_overall = v_init_data;//��/��
	
	if(fabs(w_init_data) < 0.5*fabs(imu_present_w))
	{
		w_overall = imu_present_w;//��λ����/��
	}
	else
	{
		w_overall = w_init_data;//��λ����/��
	}
	
	times_agv_pos=0.0001f * ( TIM_GetCounter(CAL_POS_TIM) + (u32)timeout_triple * 65536 );//��������ʱ�� ��λ����
	TIM_SetCounter(CAL_POS_TIM,0);//����TIM2��ʱ���ļ�����ֵ
	timeout_triple=0;//��ʱ����ʱ����
	times_agv_pos_temp=times_enco_pos + times_sensor_angle + times_agv_pos;//������ϴδ˴������ʼ������������ʱ��������������ʱ��֮��
	
//	printf("times_agv_pos_temp = %f\r\n",times_agv_pos_temp);
	theta_overall += times_agv_pos_temp * w_overall;
	
	pos_b[0] = pos_a[0] + v_overall * times_agv_pos_temp * Cos_Lookup(theta_overall);//���� agv x�����꣬��λ����
	pos_b[1] = pos_a[1] + v_overall * times_agv_pos_temp * Sin_Lookup(theta_overall);//���� agv y�����꣬��λ����
	pos_b[2] = theta_overall;//���� agv ����������ϵ�еĽǶ�  ��λ����
	
	for(i=0;i<3;i++)
	{
		pos_a[i] = pos_b[i];//�Ѽ������������洢��������һ�ι���
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










