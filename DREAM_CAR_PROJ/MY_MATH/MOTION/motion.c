#include "motion.h"

u8 motor_pid_debug = 0;// 是否在调试 左右轮 PID，若为 1 则是在调试 左右轮 PID

static s16 enc_omega_pwm[2];
static s16 encdr_data_pwm[2];
float target_theta;
float target_v;
float target_omega;
float target_v_before_acc, target_omega_before_acc;
float target_v_after_acc, target_omega_after_acc;
float enc_omega[2];   
float agv_omega, agv_v, agv_theta;

static arm_matrix_instance_f32 enc_inv_trans_matrix;	//编码器转换矩阵的逆
static float32_t enc_inv_trans_data[4];  //编码器转换逆矩阵数据

static float agv_move_data[2];	  //agv速度和角速度数据
static float enc_omega_data[2];	//左右编码器角速度数据

static arm_matrix_instance_f32 agv_move_matrix;	//agv速度和角速度向量
static arm_matrix_instance_f32 enc_omega_matrix;	//左右编码器角速度向量

//static pid_struct THETA_TO_VEHICLE = {0.4f, 4.5f, 0.5f, 0.6f};//定义结构体变量
static pid_struct VEHICLE_TO_LR_L  = {0.2f, 0.22f, 0.0f, 0.5f};
static pid_struct VEHICLE_TO_LR_R  = {0.2f, 0.22f, 0.0f, 0.5f};

static pid_struct LR_TO_PWM_L      = {0.1f, 0.2f, 0.0f, 0.0f};
static pid_struct LR_TO_PWM_R      = {0.1f, 0.2f, 0.0f, 0.0f};
	
static float vehiclelr[2];

u8 motor_status_L, motor_status_R;
u8 acc_complete_status = 1;

u16 acc_v_count_target = 0, acc_omega_count_target = 0;

void PID_Math_Init(void)
{
	arm_mat_init_f32(&enc_inv_trans_matrix, 2, 2, enc_inv_trans_data);//初始化
	arm_mat_inverse_f32(&encdr_trans_mat_for_inv, &enc_inv_trans_matrix);//求矩阵的逆
	arm_mat_init_f32(&enc_omega_matrix, 2, 1, enc_omega_data);//初始化, vehicle_to_LR()函数中使用
}

u8 is_nan(float *num)
{
	if(isnan(*num))
	{
		*num = 0.0f;
		
		return 1;
	}
	return 0;
}

u8 trans_nan(float *num)
{
	static int32_t num_tmp = 0;
	if(isnan(*num))
	{
		num_tmp = *num * 10000;
		*num = num_tmp * 0.0001f;
		
		is_nan(num);
		
		return 1;
	}
	return 0;
}



float positional_PID_Control(float target_val, float current_val, pid_struct* pid, float max_val)
{
	
//	ratio = s_acc(target_val, current_val);// S加减速表中对应的系数
	
	// 截断目标值，避免发生意外
	if(target_val > max_val)  target_val = max_val;
	if(target_val < -max_val) target_val = -max_val;
	
	
//	if(pid->last_Ka != pid->Ka || pid->last_Kp != pid->Kp ||pid->last_Ki != pid->Ki || pid->last_Kd != pid->Kd)
//	{
//		pid->sum_err = 0.0f;// pid 参数变化时，清空 积分项
//		pid->temp = 0;
//	}
	
	//PID计算
	pid->err = target_val - current_val;//误差
	trans_nan(&pid->err);
	
	pid->sum_err += pid->err;
//	pid->temp += (u32)(pid->err * 10000.0f);//由于浮点数相乘返回 NAN ，在此做个变换，截断小数位数
//	pid->sum_err = pid->temp * 0.0001f;
//	pid->target_der = target_val - pid->old_target_val;
	pid->dev_err = pid->err - pid->last_err;

	
/* 如果PID积分项  超出AGV速度和角速度极限值，则截断 */
	if(pid->sum_err > max_val)  pid->sum_err = max_val;
	if(pid->sum_err < -max_val)  pid->sum_err = -max_val;
	
	
//	/* 如果一开始PID积分项 累加和过大，则会发生“积分过冲”，要进行“积分分离”，待其较小时才让其发挥作用 */
//	if(pid->sum_err > 1.0f * max_val)
//	{
//		pid->sumerr_flag = 1;
//		pid->pid_i_backup = pid->Ki;
//		pid->Ki = 0.0f;
//	}
//	
//	if(pid->sumerr_flag == 1 && pid->sum_err <= 1.0f * max_val)
//	{
//		pid->sumerr_flag = 0;
//		pid->Ki = pid->pid_i_backup;
//	}
	
	
//		output = output + pid->Ka * ( pid->Kp * (err-last_err) + pid->Ki * err + pid->Kd * (err-2*last_err+last_last_err) );
//		output = output + Ka*(Kd*(err-last_err)+Kp*err+Ki*(err-2*last_err+last_last_err));
//		output = pid->Ka * (pid->Kp * err + err_sum - pid->Kd * target_der);
//		output = pid->Ka * (pid->Kp * pid->err + pid->Ki*pid->sum_err + pid->Kd * pid->dev_err);
		pid->output = pid->Ka * ( pid->Kp * ( pid->err + pid->Ki * pid->sum_err + pid->Kd * pid->dev_err ) );

		
		//迭代更新
		pid->last_err = pid->err;
//		pid->old_target_val = target_val;
		
		
		/* 如果PID输出超出AGV速度和角速度极限值，则截断 */
		if(pid->output > max_val)  pid->output = max_val;
		if(pid->output < -max_val)  pid->output = -max_val;
		
		
//		pid->last_Ka = pid->Ka;
//		pid->last_Kp = pid->Kp;
//		pid->last_Ki = pid->Ki;
//		pid->last_Kd = pid->Kd;
//		printf("pid->output=%f\r\n",pid->output);
		return pid->output;
}


/**  运动控制 **/
void MOTION_CONTROL(void)
{
	pid_control();// PID闭环控制
	pid_select();// 按下 "DELETEE" 按键，开始PID参数选择与调整
	ULTRA_OBSTAC_AVOID();// 超声波避障
}


/** 函数功能：

* S 曲线加减速
*/
float s_acc(float target, float current, u8 type)
{
	static u16 index_v = 0, index_w = 0, count_v = 0, count_w = 0;
	static float target_old_v = 0.0f, target_old_w = 0.0f, return_val = 0.0f;
	static float target_v = 0.0f, target_w = 0.0f;
	float min = 0.0f;
	static u8 v_status = 0, w_status = 0;
	
	// acc_v = v / t;   acc_omega = omega / t;
	if(type == 0)       
	{
		min = v_min;
		target_v = target;
		
		if(times_agv_pos_temp > 0.0f)
			acc_v_count_target = fabs(target_v-target_old_v) * AGV_Max_vacc_inv / times_agv_pos_temp;
		
		if(acc_v_count_target < 0.0f)
			acc_v_count_target = - acc_v_count_target;
	}
	else if(type == 1)
	{
		min = omega_min;
		target_w = target;
		
		if(times_agv_pos_temp > 0.0f)
			acc_omega_count_target = fabs(target_w-target_old_w) * AGV_Max_wacc_inv / times_agv_pos_temp;
		
		if(acc_omega_count_target < 0.0f)
			acc_omega_count_target = - acc_omega_count_target;
	}
	
	if(fabs(target_v) > fabs(AGV_Max_v))      {target_v = AGV_Max_v;}
	if(fabs(target_w) > fabs(AGV_Max_omega))  {target_w = AGV_Max_omega;}
	
	switch(type)
	{
		case 0:
			if(target_old_v != target_v)
			{
				if(fabs(target_v - target_old_v) > 0.09f)
				{
					if(v_status == 1)  index_v = 0;
					
					if(index_v == 120)
					{
						v_status = 1;
						target_old_v = target_v;
					}
					else
					{
						if(++count_v >= acc_v_count_target)
						{
							count_v = 0;
							index_v += 5;
							if(index_v > 120)  index_v %= 120;
							v_status = 0;
						}
					}			
				}
				else
				{
					target_old_v = target_v;// 目标线速度过小，不执行加减速过程
				}
			}
			
			return_val = (target_v-target_old_v)*get_data(index_v) + target_old_v;

			
//			printf("\r\ntarget-v :%f\r\n",target);
//			printf("v_status :%d\r\n",v_status);
//			printf("index_v: %d\r\n",index_v);
//			printf("count_v: %d\r\n",count_v);
			break;
			
		case 1:
			
			if(target_old_w != target_w)
			{
				if(fabs(target_w - target_old_w) > 20.0f)
				{
					if(w_status == 1)  index_w = 0;
					
					if(index_w == 120)
					{
						w_status = 1;
						target_old_w = target_w;
					}
					else
					{
						if(++count_w >= acc_omega_count_target)
						{
							count_w = 0;
							index_w += 5;
							if(index_w > 120)  index_w %= 120;
							w_status = 0;
						}
					}
				}
				else
				{
					target_old_w = target_w;// 目标角速度过小，不执行加减速过程
				}
			}
			
			return_val = (target_w-target_old_w)*get_data(index_w) + target_old_w;
			
//			printf("target-w :%f\r\n",target_w);
//			printf("w_status :%d\r\n",w_status);
//			printf("index_w: %d\r\n",index_w);
//			printf("count_w: %d\r\n",count_w);
			break;
			
		default:
			return_val = 0.0f;
		
			break;
	}
	return return_val;
}



/** void pid_control(void) 函数功能：
* 
* 角速度和线速度pid 闭环控制
*/
void pid_control(void)
{	
		trans_nan(&target_v);
		trans_nan(&target_omega);

		if(motor_pid_debug == 0)
		{
//			if(ultra_stop_status == 0 && acc_complete_status == 1)
			if(ultra_stop_status == 0)
			{
				if(fabs(target_omega) < fabs(vw_from_ros[2]))  target_omega_before_acc = vw_from_ros[2];//从ros获取的角速度  度/s
				else                                           target_omega_before_acc = target_omega;
				
				if(fabs(target_v) < fabs(vw_from_ros[0]))      target_v_before_acc = vw_from_ros[0];//从ros获取的x方向（前进方向）的线速度 m/s
				else                                           target_v_before_acc = target_v;				
			}

//			printf("target_v_before_acc：    %f  m/s\r\n",target_v_before_acc);
//			printf("target_omega：%f  °/s\r\n",target_omega);
//			printf("target_v_before_acc: %f\r\n",target_v_before_acc);
//			printf("target_omega_before_acc: %f\r\n",target_omega_before_acc);
			if(target_v_before_acc > AGV_Max_v)          {target_v_before_acc = AGV_Max_v;}
			else if (target_v_before_acc < AGV_Min_v)    {target_v_before_acc = AGV_Min_v;}
			
			if(target_omega_before_acc > AGV_Max_omega)       {target_omega_before_acc = AGV_Max_omega;}
			else if(target_omega_before_acc < AGV_Min_omega)  {target_omega_before_acc = AGV_Min_omega;}
			
	/******************************************vehicle pid control***********************************************/			

			// 遍历 S 曲线数据进行加减速计算
			target_v_after_acc = s_acc(target_v_before_acc, v_overall, 0);// 类型 0 代表速度 v
			target_omega_after_acc = s_acc(target_omega_before_acc, w_overall, 1);// 类型 1 代表加速度 omega

			if(target_v_after_acc > AGV_Max_v)          {target_v_after_acc = AGV_Max_v;}
			else if (target_v_after_acc < AGV_Min_v)    {target_v_after_acc = AGV_Min_v;}
			
			if(target_omega_after_acc > AGV_Max_omega)       {target_omega_after_acc = AGV_Max_omega;}
			else if(target_omega_after_acc < AGV_Min_omega)  {target_omega_after_acc = AGV_Min_omega;}
			
//			printf("target_v_after_acc: %f\r\n",target_v_after_acc);
//			printf("target_omega_after_acc: %f\r\n",target_omega_after_acc);
		

		
			vehicle_to_LR(target_v_after_acc,target_omega_after_acc);// 目标值分解成左右轮角速度
//			vehicle_to_LR(target_v_before_acc,target_omega_before_acc);// 目标值分解成左右轮角速度
			enc_omega[0] = vehiclelr[0]; enc_omega[1] = vehiclelr[1];
				
			trans_nan(&enc_omega[0]);
			trans_nan(&enc_omega[1]);	
		}			
	/******************************************vehicle pid control***********************************************/			

		
	/*******************************motor pid control************************************/
		if(fabs(enc_omega[0])>= omegaR_min || fabs(enc_omega[1])>= omegaL_min)
		{
			GPIO_SetBits(RELAY_L_PORT,RELAY_L_Pin | RELAY_R_Pin);
			
			enc_omega_pwm[0] =  omega_to_pwm(enc_omega[0],PWM_Base);
			encdr_data_pwm[0] = omega_to_pwm(encdr_data[0],PWM_Base);
			
			enc_omega_pwm[1] =  omega_to_pwm(enc_omega[1],PWM_Base);
			encdr_data_pwm[1] = omega_to_pwm(encdr_data[1],PWM_Base);
			
//			if(acc_complete_status == 0)
//			{
//				Moto2 = omega_to_pwm( enc_omega[0], PWM_Base );
//				Moto1 = omega_to_pwm( enc_omega[1], PWM_Base );	
//			}
//			if(acc_complete_status == 1)
//			{
				// 右电机角速度 转化为 PWM值，并进行PID控制：       右轮角速度目标值，当前角速度，pid控制参数，角速度极限值
				Moto2 += positional_PID_Control(enc_omega_pwm[0],encdr_data_pwm[0] , &LR_TO_PWM_R, PWM_Max);

				// 左电机角速度 转化为 PWM值，并进行PID控制：       左轮角速度目标值，当前角速度，pid控制参数，角速度极限值
				Moto1 += positional_PID_Control(enc_omega_pwm[1], encdr_data_pwm[1], &LR_TO_PWM_L, PWM_Max);
//			}
			
	//		printf("pwm_target 右= %d\r\n",enc_omega_pwm[0]);
	//		printf("pwm_current 右= %d\r\n",encdr_data_pwm[0]);		
			
	//		printf("pwm_target 左= %d\r\n",enc_omega_pwm[1]);
	//		printf("pwm_current 左= %d\r\n",encdr_data_pwm[1]);
						

			// 判断电机运行状态
			if(fabs(encdr_data[0]) < 0.05f*fabs(enc_omega[0])) motor_status_R = 1;
			else                                               motor_status_R = 0;
			
			if(fabs(encdr_data[1]) < 0.05f*fabs(enc_omega[1])) motor_status_L = 1;
			else                                               motor_status_L = 0;
		}
		else
		{
			// 当左右轮目标或实际角速度都小于 1.0rad/s 时，两个轮子都要停转
			vehiclelr[0] = 0.0f;
			vehiclelr[1] = 0.0f;
			enc_omega[0] = 0.0f;
			enc_omega[1] = 0.0f;
			enc_omega_pwm[0] = 0.0f;
			enc_omega_pwm[1] = 0.0f;
			encdr_data_pwm[0] = 0.0f;
			encdr_data_pwm[1] = 0.0f;
			Moto1 = 0;
			Moto2 = 0;

			if(fabs(encdr_data[0]) < omegaR_min && fabs(encdr_data[1]) < omegaL_min)
			{
				GPIO_ResetBits(RELAY_L_PORT,RELAY_L_Pin | RELAY_R_Pin);
			}
		}
		
//		printf("左Moto1= %d\r\n",Moto1);
//		printf("右Moto2= %d\r\n",Moto2);
		motor_setup();// 设置电机控制占空比
	/*******************************motor pid control************************************/
}


// 把小车 线速度 和 角速度 转换成左右轮速度
void vehicle_to_LR(float v, float omega)
{
	agv_move_data[1] = omega * PI_div_180;//PID返回值  单位：由 度/秒 转化成 弧度/秒
	agv_move_data[0] = v;    //PID返回值  单位：米/秒
	
//	printf("v == %f\r\n",v);
//	printf("omega == %f\r\n",omega);
	
	arm_mat_init_f32(&agv_move_matrix, 2, 1, agv_move_data);//初始化
	arm_mat_mult_f32(&enc_inv_trans_matrix, &agv_move_matrix, &enc_omega_matrix);//计算得到左右编码器的角速度 rad/s
	vehiclelr[0] = enc_omega_matrix.pData[0];// 把 arm_matrix_instance_f32对象中pData指针数据转化成数组数据 rad/s   右轮
	vehiclelr[1] = enc_omega_matrix.pData[1];// 把 arm_matrix_instance_f32对象中pData指针数据转化成数组数据 rad/s   左轮
//	printf("vehiclelr[0] = %f\r\n",vehiclelr[0]);
//	printf("vehiclelr[1] = %f\r\n",vehiclelr[1]);

}



/* 角速度 转化成 PWM占空比数值 */
int omega_to_pwm(float omega, s16 pwm_base)
{
	s16 pwm_temp;

	if( omega >= omegaR_min )  
	{
		pwm_temp = fabs(omega) * PWM_Omega_Ratio + pwm_base;
	}
	else if( omega <= -omegaR_min )
	{
		pwm_temp = - (fabs(omega) * PWM_Omega_Ratio + pwm_base);
	}
	else
	{
		pwm_temp = 0;
	}

//	printf("current pwm = %d\r\n",pwm_temp);
	return pwm_temp;
}



/*  void pid_select(void) 函数功能：

1. 按下 DELETEE 按键，开始切换 角速度PID/线速度PID/角度PID参数，从角速度PID参数开始
2. 按下"alien"按键，退出切换
3. 然后任意顺序按下 p i d 按键，进入 pid参数调试，按下"alien"按键，结束调试

*/
void pid_select(void)
{
	static u8 select_pid=0;//指示哪个pid参数正在被调试
	
	if((key==82) || (select_pid & 0x80))//红外遥控器DELETE按键按下，开始选择pid种类，“ALIEN”按键按下则退出
	{
		
  //                            ___ ___ ___ ___ ___ ___ ___ ___
	//pid_status各个位表示的含义 |___|___|___|___|___|___|___|___|
	//                        continue         R   L theta v VEHICLE_TO_LR_R
		
		static u8 i=0, j=0, rising_edge=1;
		select_pid |= (key==82)<<7;//把 continue 位置1
//		printf("select_pid:0x%x\r\n",select_pid);
		
		
		if((((select_pid) & 0xff) <= 0x9f) && (rising_edge==1) && (key==82))
		{
			select_pid |= (key==82)<<(i++);   //由于要在select_pid的前五位之间不断循环状态，当超过最大状态时，要从头再开始循环
			rising_edge = 0;//清零，按键不松开，就不会有下一个上升沿信号过来
			j=0;//把j清零，为下一次蜂鸣器响准备
		}
		if(((select_pid) & 0xff) > 0x9f)// 当超过最大状态时，要从头再开始循环
		{
			select_pid =0x81;//把select_pid置为初始状态
			i=0;//把i清零，从0个移位量开始移位
			j=0;//把j清零，为下一次蜂鸣器响准备
		}
		
		if(!(key==82) )
		{
			rising_edge =1;//当"DELETE"按键松开时，才能继续有下一个上升沿信号过来
		}
		
		
		//角速度PID参数选择
		if(select_pid==0x81)
		{
			if(j==0)
			{
				j=1;//保证蜂鸣器进来时响一回，其他时间不响
				beep_on(1,100);
			}
			
			pid_setup(&VEHICLE_TO_LR_R);//PID参数调试 
	    DataScope(VEHICLE_TO_LR_R);//示波器显示波形
			motor_pid_debug = 0;
//			printf("VEHICLE_TO_LR_R\r\n");
		}
		
		
		///线速度PID选择
		if(select_pid==0x83)
		{
			if(j==0)
			{
				j=1;//保证蜂鸣器进来时响一回，其他时间不响
				beep_on(2,100);
			}
			
			pid_setup(&VEHICLE_TO_LR_L);//PID参数调试 
	    DataScope(VEHICLE_TO_LR_L);//示波器显示波形
			motor_pid_debug = 0;
//			printf("VEHICLE_TO_LR_L\r\n");
		}
		
		
		///角度PID选择
		if(select_pid==0x87)
		{
			if(j==0)
			{
				j=1;//保证蜂鸣器进来时响一回，其他时间不响
				beep_on(3,100);
			}
			
//			pid_setup(&THETA_TO_VEHICLE);//PID参数调试 
//			DataScope(THETA_TO_VEHICLE);//示波器显示波形
			motor_pid_debug = 0;
			printf("THETA_TO_VEHICLE\r\n");
		}
		
		
		///左轮角速度PID选择
		if(select_pid==0x8f)
		{
			if(j==0)
			{
				j=1;//保证蜂鸣器进来时响一回，其他时间不响
				beep_on(4,100);
			}
			
			pid_setup(&LR_TO_PWM_L);//PID参数调试 
			DataScope(LR_TO_PWM_L);//示波器显示波形
			motor_pid_debug = 1;
			printf("LR_TO_PWM_L\r\n");
		}
		
		
		///右轮角速度PID选择
		if(select_pid==0x9f)
		{
			if(j==0)
			{
				j=1;//保证蜂鸣器进来时响一回，其他时间不响
				beep_on(5,100);
			}
			
			pid_setup(&LR_TO_PWM_R);//PID参数调试 
			DataScope(LR_TO_PWM_R);//示波器显示波形
			motor_pid_debug = 1;
			printf("LR_TO_PWM_R\r\n");
		}
		
		
		// "ALIEN"按键被按下，表示PID参数选择结束
		if( (select_pid & 0x80)&&(key==226) )
		{
			select_pid &=0x0;//所有位清零，结束PID参数选择
			beep_on(3,100);
			
			motor_pid_debug = 0;
		}
	}
}





/*  void pid_setup(struct PID* pid)函数功能：

1. 任意顺序按下 p i d 按键，进入 pid参数调试
2. 按下"alien"按键，结束调试
3. 进入pid调试后，按下 p 按键，表示调试p参数，按 vol-/vol+按键改变参数值
4. 按下 i 按键，表示调试 i 参数，按 vol-/vol+按键改变参数值
5. 按下 d 按键，表示调试 d 参数，按 vol-/vol+按键改变参数值
*/
void pid_setup(pid_struct* pid)
{
	//                            ___ ___ ___ ___ ___ ___ ___ ___
	//pid_status各个位表示的含义 |___|___|___|___|___|___|___|___|
	//                      continue start end off a   d   i   p

	 static u8 pid_status=0;
	
	//                (按键2/a)  红外按键 3/d             4/i             7/p  同时按下
	 pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
	 pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//同时按下3/d   4/i  7/p 触发 start状态
	
	// end位为1，start位也为1时，说明是再次调试pid，需要把end位置零
	if( (pid_status & 0x60)==0x60 )
	{
		pid_status &= 0xdf;// 把 pid_status的end位置零，开始新一轮pid调试
	}
	 
	// 分别判断pid_status的 start  continue end 位的状态，开始调整PID值
	 if( ( (pid_status & 0x40)==0x40 || (pid_status & 0x80)==0x80 ) && (pid_status & 0x20)==0 ) 
	 {
		 if( (pid_status & 0x10)==0 )
		 {
			 beep_on(2,100);//每次开始pid调试，蜂鸣器响2次，每次响100ms
			 pid_status &= 0xb8;//把刚刚进入时的p i d 三个位，以及start置零
		 }
		 
		 
		 pid_status |= 0x10;// 把pid_status第4位置1，保证beep只在刚进入pid设置时响一下
		 pid_status |= 0x80;// 把 pid_status的continue位打开，保证触发start后，可以持续调试pid
		 pid_status &= 0xdf;// 把 pid_status的end位置零，保证触发start后，可以持续调试pid
		 
		 
		 // 调整 p
		 if ((pid_status & 0x1) == 0x1)
		 {
				//                (按键2/a)  红外按键 3/d             4/i             7/p  同时按下
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//同时按下3/d   4/i  7/p 触发 start状态
			 
			 
			 //除当前调整值，其他 a d i p 位清零
			 if( (pid_status & 0x2) == 0x2 )
			 {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x2;//调整i
			 }
			 
			 if( (pid_status & 0x4) == 0x4 )
			 {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x4;//调整d
			 }
			 
			 if( (pid_status & 0x8) == 0x8 )
			 {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x8;//调整a
			 }
			 
			 if( (pid_status & 0x1) == 0x1 )
			 {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x1;//调整p
			 }

			 
			 //调整p
			if(key==144) //VOL+按键
			{
				beep_on(1,100);//蜂鸣器响1次，表示调试中
				pid->Kp+=0.05*RmtCnt;
			}
			
			if(key==224) //VOL-按键
			{
				beep_on(1,100);//蜂鸣器响1次，表示调试中
				pid->Kp-=0.05*RmtCnt;
			}
			
//			printf("调整p:\r\nKp=%f\r\n",Kp);
//			printf("pid_status-p:\r\n0x%x\r\n",pid_status);
		 }
		 
		 
		 
		 
		 // 调整 i
		 if ((pid_status & 0x2) == 0x2)
		 {
				//                (按键2/a)  红外按键 3/d             4/i             7/p  同时按下
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//同时按下3/d   4/i  7/p 触发 start状态

			 
				if( (pid_status & 0x4) == 0x4 )
				{
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x4;//调整d
				}

				if( (pid_status & 0x8) == 0x8 )
				{
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x8;//调整a
				}


				if( (pid_status & 0x1) == 0x1 )
				{
					pid_status &=0xf0;//把p i d a 位清零
					pid_status |=0x1;//调整p
				}

				if( (pid_status & 0x2) == 0x2 )
				{
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x2;//调整i
				}

			 
			 //调整i
			 if(key==144) //VOL+按键
			{
				beep_on(1,100);//蜂鸣器响1次，表示调试中
				pid->Ki+=0.05*RmtCnt;
			}
			
			if(key==224) //VOL-按键
			{
				beep_on(1,100);//蜂鸣器响1次，表示调试中
				pid->Ki-=0.05*RmtCnt;
			}
			
//			printf("调整i:\r\nKi=%f\r\n",Ki);
//			printf("pid_status-i:\r\n0x%x\r\n",pid_status);
		 }
		 
		 
		 
		 
		 // 调整 d
		 if ((pid_status & 0x4) == 0x4)
		 {
			 	//                (按键2/a)  红外按键 3/d             4/i             7/p  同时按下
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//同时按下3/d   4/i  7/p 触发 start状态
			 
			 
				if( (pid_status & 0x8) == 0x8 )
				{
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x8;//调整a
				}
				 
				if( (pid_status & 0x1) == 0x1 )
				{
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x1;//调整p
				}
				 
				 if( (pid_status & 0x2) == 0x2 )
			  {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x2;//调整i
			  }
			 
				 
				 if( (pid_status & 0x4) == 0x4 )
				 {
					 pid_status &=0xf0;//把p i d a 位清零
					 pid_status |=0x4;//调整d
				 }

			 
			 //调整d
			 if(key==144) //VOL+按键
			{
				beep_on(1,100);//蜂鸣器响1次，表示调试中
				pid->Kd+=0.05*RmtCnt;
			}
			
			if(key==224) //VOL-按键
			{
				beep_on(1,100);//蜂鸣器响1次，表示调试中
				pid->Kd-=0.05*RmtCnt;
			}
			
//			printf("调整d:\r\nKd=%f\r\n",Kd);
//			printf("pid_status-d:\r\n0x%x\r\n",pid_status);
		 }
		 
		 
		 
		 
		 // 调整 a
		 if ((pid_status & 0x8) == 0x8) 
		 {
				//                (按键2/a)  红外按键 3/d             4/i             7/p  同时按下
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//同时按下3/d   4/i  7/p 触发 start状态
			 
			 if( (pid_status & 0x1) == 0x1 )
			 {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x1;//调整p
			 }
				 
			 if( (pid_status & 0x2) == 0x2 )
			 {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x2;//调整i
			 }
			 
			 if( (pid_status & 0x4) == 0x4 )
			 {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x4;//调整d
			 }
			 
			 
			 if( (pid_status & 0x8) == 0x8 )
			 {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x8;//调整a
			 }
			 
				//调整a
				if(key==144) //VOL+按键
				{
					beep_on(1,100);//蜂鸣器响1次，表示调试中
					pid->Ka+=0.05*RmtCnt;
				}
				
				if(key==224) //VOL-按键
				{
					beep_on(1,100);//蜂鸣器响1次，表示调试中
					pid->Ka-=0.05*RmtCnt;
				}
			
//			printf("调整a:\r\nKa=%f\r\n",Ka);
//			printf("pid_status-a:\r\n0x%x\r\n",pid_status);
		 }
		 
		 
		 /////////////////////////////////保证p i d a 每个参数后面都有剩余参数可选////////////////////////////////////
		 
		 // 调整 p
		 if ((pid_status & 0x1) == 0x1)
		 {
				//                (按键2/a)  红外按键 3/d             4/i             7/p  同时按下
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//同时按下3/d   4/i  7/p 触发 start状态
			 
			 
			 //除当前调整值，其他 a d i p 位清零
			 if( (pid_status & 0x2) == 0x2 )
			 {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x2;//调整i
			 }
			 
			 if( (pid_status & 0x4) == 0x4 )
			 {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x4;//调整d
			 }
			 
			 if( (pid_status & 0x8) == 0x8 )
			 {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x8;//调整a
			 }
			 
			 if( (pid_status & 0x1) == 0x1 )
			 {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x1;//调整p
			 }

			 
			 //调整p
			if(key==144) //VOL+按键
			{
				beep_on(1,100);//蜂鸣器响1次，表示调试中
				pid->Kp+=0.05*RmtCnt;
			}
			
			if(key==224) //VOL-按键
			{
				beep_on(1,100);//蜂鸣器响1次，表示调试中
				pid->Kp-=0.05*RmtCnt;
			}
			
//			printf("调整p:\r\nKp=%f\r\n",Kp);
//			printf("pid_status-p:\r\n0x%x\r\n",pid_status);
		 }
		 
		 
		 
		 
		 // 调整 i
		 if ((pid_status & 0x2) == 0x2)
		 {
				//                (按键2/a)  红外按键 3/d             4/i             7/p  同时按下
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//同时按下3/d   4/i  7/p 触发 start状态

			 
				if( (pid_status & 0x4) == 0x4 )
				{
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x4;//调整d
				}

				if( (pid_status & 0x8) == 0x8 )
				{
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x8;//调整a
				}


				if( (pid_status & 0x1) == 0x1 )
				{
					pid_status &=0xf0;//把p i d a 位清零
					pid_status |=0x1;//调整p
				}

				if( (pid_status & 0x2) == 0x2 )
				{
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x2;//调整i
				}

			 
			 //调整i
			 if(key==144) //VOL+按键
			{
				beep_on(1,100);//蜂鸣器响1次，表示调试中
				pid->Ki+=0.05*RmtCnt;
			}
			
			if(key==224) //VOL-按键
			{
				beep_on(1,100);//蜂鸣器响1次，表示调试中
				pid->Ki-=0.05*RmtCnt;
			}
			
//			printf("调整i:\r\nKi=%f\r\n",Ki);
//			printf("pid_status-i:\r\n0x%x\r\n",pid_status);
		 }
		 
		 
		 
		 
		 // 调整 d
		 if ((pid_status & 0x4) == 0x4)
		 {
				//                (按键2/a)  红外按键 3/d             4/i             7/p  同时按下
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//同时按下3/d   4/i  7/p 触发 start状态
			 
			 
				if( (pid_status & 0x8) == 0x8 )
				{
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x8;//调整a
				}
				 
				if( (pid_status & 0x1) == 0x1 )
				{
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x1;//调整p
				}
				 
				 if( (pid_status & 0x2) == 0x2 )
			  {
				 pid_status &=0xf0;//把p i d a 位清零
				 pid_status |=0x2;//调整i
			  }
			 
				 
				 if( (pid_status & 0x4) == 0x4 )
				 {
					 pid_status &=0xf0;//把p i d a 位清零
					 pid_status |=0x4;//调整d
				 }

			 
			 //调整d
			 if(key==144) //VOL+按键
			{
				beep_on(1,100);//蜂鸣器响1次，表示调试中
				pid->Kd+=0.05*RmtCnt;
			}
			
			if(key==224) //VOL-按键
			{
				beep_on(1,100);//蜂鸣器响1次，表示调试中
				pid->Kd-=0.05*RmtCnt;
			}
			
//			printf("调整d:\r\nKd=%f\r\n",Kd);
//			printf("pid_status-d:\r\n0x%x\r\n",pid_status);
		 }
		 
		 /////////////////////////////////////////////////////////////////////////
		 
		 
		 
		 // 结束调整
		 if( ( (pid_status & 0x90) == 0x90 ) && (key==226) )// 在pid调试过程中，按下“alien”按键，结束调整
		 {
			 beep_on(3,100);//每次pid结束，蜂鸣器响3次
		   pid_status &= 0x0;// 把pid_status 所有位置0
			 pid_status |=0x20;//把end位置1，结束pid调节
		 }
	}
	 
//	printf("pid_status:\r\n0x%x\r\n",pid_status);
//	printf("key:\r\n%d\r\n",key);
	
}




#if __PID_TIME__

//基本定时器7 中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器7!
void TIM7_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///使能 TIM7 时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(PID_TIM,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(PID_TIM,TIM_IT_Update,ENABLE); //允许 定时器7 更新中断
	TIM_Cmd(PID_TIM,ENABLE); //使能 定时器7
	
	NVIC_InitStructure.NVIC_IRQChannel = PID_IRQn; //定时器7 中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PID_TIM_PreemptionPriority; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = PID_TIM_SubPriority; //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}



//定时器7中断服务函数
void PID_IRQHandler(void)
{
	if(TIM_GetITStatus(PID_TIM,TIM_IT_Update)==SET) //溢出中断
	{
			timeout_pid++;
	}
	TIM_ClearITPendingBit(PID_TIM,TIM_IT_Update);  //清除中断标志位
}
#endif


