#ifndef __SYS_H
#define __SYS_H	 
#include "stm32f4xx.h" 
#include "arm_math.h"
#include "version_1.0.h"


//0,不支持ucos
//1,支持ucos
#define SYSTEM_SUPPORT_OS		0		//定义系统文件夹是否支持UCOS
																	    
	 
//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
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
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入

//以下为汇编函数
void WFI_SET(void);		//执行WFI指令
void INTX_DISABLE(void);//关闭所有中断
void INTX_ENABLE(void);	//开启所有中断
void MSR_MSP(u32 addr);	//设置堆栈地址 





extern float times_enco_pos;//每次进行编码器角速度计算的时间间隔   单位：秒
extern float times_sensor_angle;//每次进行kalman噪音计算的时间间隔   单位：秒
extern float times_agv_pos;//每次进行agv坐标计算的时间间隔   单位：秒
extern u16 adc_100;
extern int Moto1;
extern int Moto2;
extern int32_t Encoder_Left,Encoder_Right;                  //左右编码器的脉冲计数
extern float Enco_wL,Enco_wR;                             //左右编码器角速度 单位: 度/秒

extern float times_enco_pos_temp;//由于采用一个定时器对三处进行分别计时，所以需要分三段分别计时，再根据需要进行时间相加
extern float times_sensor_angle_temp;
extern float times_agv_pos_temp;
extern float v;//由左右编码器计算出的 agv 速度 单位：米/秒
extern arm_matrix_instance_f32 encdr_trans_mat;//编码器数据 转换 成agv速度的矩阵
extern arm_matrix_instance_f32 encdr_trans_mat_for_inv;//数据备份，防止被修改为单位矩阵
extern float kalman_theta;
extern float kalman_omga;

extern u8 key, key_local;//开发板遥控器按键和本地按键
//extern u8 timeout_left,timeout_right;

extern float w_init_data;

extern float w_overall;//PID控制对应的当前值
extern float v_overall;//PID控制对应的当前值
extern float theta_overall;//PID控制对应的当前值

extern float v_init_data;
extern float imu_v;


extern float target_theta;//PID控制目标值
extern float target_v;//PID控制目标值
extern float target_omega;//PID控制目标值


extern float pos_b[3];// agv位于 b 点的位姿


typedef struct PID   //定义一个PID结构体
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

#define __PID_TIME__ 0 // PID是否使用控制周期的 “开关”
#define omegaL_min 0.01f
#define omegaR_min 0.01f

extern u8 motor_pid_debug;// 是否在调试 左右轮 PID

extern float encdr_data[2];//编码器数据数组
extern float enc_omega[2];   
extern float ultrasonic_dist1, ultrasonic_dist2, ultrasonic_dist3, ultrasonic_dist4;//超声波测距距离

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











