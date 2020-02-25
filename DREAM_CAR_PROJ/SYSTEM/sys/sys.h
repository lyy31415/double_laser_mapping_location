#ifndef __SYS_H
#define __SYS_H	 
#include "stm32f4xx.h" 
#include "arm_math.h"
#include "version_1.0.h"


//0,��֧��ucos
//1,֧��ucos
#define SYSTEM_SUPPORT_OS		0		//����ϵͳ�ļ����Ƿ�֧��UCOS
																	    
	 
//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).M4ͬM3����,ֻ�ǼĴ�����ַ����.
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //��� 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //����

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //��� 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //����

//����Ϊ��ຯ��
void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(u32 addr);	//���ö�ջ��ַ 





extern float times_enco_pos;//ÿ�ν��б��������ٶȼ����ʱ����   ��λ����
extern float times_sensor_angle;//ÿ�ν���kalman���������ʱ����   ��λ����
extern float times_agv_pos;//ÿ�ν���agv��������ʱ����   ��λ����
extern u16 adc_100;
extern int Moto1;
extern int Moto2;
extern int32_t Encoder_Left,Encoder_Right;                  //���ұ��������������
extern float Enco_wL,Enco_wR;                             //���ұ��������ٶ� ��λ: ��/��

extern float times_enco_pos_temp;//���ڲ���һ����ʱ�����������зֱ��ʱ��������Ҫ�����ηֱ��ʱ���ٸ�����Ҫ����ʱ�����
extern float times_sensor_angle_temp;
extern float times_agv_pos_temp;
extern float v;//�����ұ������������ agv �ٶ� ��λ����/��
extern arm_matrix_instance_f32 encdr_trans_mat;//���������� ת�� ��agv�ٶȵľ���
extern arm_matrix_instance_f32 encdr_trans_mat_for_inv;//���ݱ��ݣ���ֹ���޸�Ϊ��λ����
extern float kalman_theta;
extern float kalman_omga;

extern u8 key, key_local;//������ң���������ͱ��ذ���
//extern u8 timeout_left,timeout_right;

extern float w_init_data;

extern float w_overall;//PID���ƶ�Ӧ�ĵ�ǰֵ
extern float v_overall;//PID���ƶ�Ӧ�ĵ�ǰֵ
extern float theta_overall;//PID���ƶ�Ӧ�ĵ�ǰֵ

extern float v_init_data;
extern float imu_v;


extern float target_theta;//PID����Ŀ��ֵ
extern float target_v;//PID����Ŀ��ֵ
extern float target_omega;//PID����Ŀ��ֵ


extern float pos_b[3];// agvλ�� b ���λ��


typedef struct PID   //����һ��PID�ṹ��
{
	float Ka;
	float Kp;
	float Ki;
	float Kd;
	float last_Ka;
	float last_Kp;
	float last_Ki;
	float last_Kd;
	
	float time;
	float err;
	float last_err;
	float sum_err;
	float dev_err;
	
	float output;
	float old_target_val;
	float target_der;
	u8 sumerr_flag;
	s32 temp;
	float pid_i_backup;
	
	float ideal_wheel_L;
	float ideal_wheel_R;
}pid_struct;




typedef union DATA_TRANS
{
	u8 data_byte[4];
	float data_float;
}data_trans;


extern float vw_from_ros[3];
extern float agv_omega, agv_v, agv_theta;
extern u8 timeout_pid;

#define __PID_TIME__ 0 // PID�Ƿ�ʹ�ÿ������ڵ� �����ء�
#define omegaL_min 0.01f
#define omegaR_min 0.01f

extern u8 motor_pid_debug;// �Ƿ��ڵ��� ������ PID

extern float encdr_data[2];//��������������
extern float enc_omega[2];   
extern float ultrasonic_dist1, ultrasonic_dist2, ultrasonic_dist3, ultrasonic_dist4;//������������

extern u8 is_nan(float *num);
extern u8 trans_nan(float *num);

extern short robot_temperature;
extern short robot_gx, robot_gy, robot_gz;
extern u8 gyro_status;
extern u8 motor_status_L, motor_status_R;
extern u8 ultra_stop_status;
extern float target_v_before_acc, target_omega_before_acc;
extern float target_v_after_acc, target_omega_after_acc;
extern float v_min, omega_min;
extern u8 acc_complete_status;
extern u16 acc_v_count_target, acc_omega_count_target;
extern u8 pos_clear_flag;
#endif











