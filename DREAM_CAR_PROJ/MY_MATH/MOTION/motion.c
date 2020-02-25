#include "motion.h"

u8 motor_pid_debug = 0;// �Ƿ��ڵ��� ������ PID����Ϊ 1 �����ڵ��� ������ PID

static s16 enc_omega_pwm[2];
static s16 encdr_data_pwm[2];
float target_theta;
float target_v;
float target_omega;
float target_v_before_acc, target_omega_before_acc;
float target_v_after_acc, target_omega_after_acc;
float enc_omega[2];   
float agv_omega, agv_v, agv_theta;

static arm_matrix_instance_f32 enc_inv_trans_matrix;	//������ת���������
static float32_t enc_inv_trans_data[4];  //������ת�����������

static float agv_move_data[2];	  //agv�ٶȺͽ��ٶ�����
static float enc_omega_data[2];	//���ұ��������ٶ�����

static arm_matrix_instance_f32 agv_move_matrix;	//agv�ٶȺͽ��ٶ�����
static arm_matrix_instance_f32 enc_omega_matrix;	//���ұ��������ٶ�����

//static pid_struct THETA_TO_VEHICLE = {0.4f, 4.5f, 0.5f, 0.6f};//����ṹ�����
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
	arm_mat_init_f32(&enc_inv_trans_matrix, 2, 2, enc_inv_trans_data);//��ʼ��
	arm_mat_inverse_f32(&encdr_trans_mat_for_inv, &enc_inv_trans_matrix);//��������
	arm_mat_init_f32(&enc_omega_matrix, 2, 1, enc_omega_data);//��ʼ��, vehicle_to_LR()������ʹ��
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
	
//	ratio = s_acc(target_val, current_val);// S�Ӽ��ٱ��ж�Ӧ��ϵ��
	
	// �ض�Ŀ��ֵ�����ⷢ������
	if(target_val > max_val)  target_val = max_val;
	if(target_val < -max_val) target_val = -max_val;
	
	
//	if(pid->last_Ka != pid->Ka || pid->last_Kp != pid->Kp ||pid->last_Ki != pid->Ki || pid->last_Kd != pid->Kd)
//	{
//		pid->sum_err = 0.0f;// pid �����仯ʱ����� ������
//		pid->temp = 0;
//	}
	
	//PID����
	pid->err = target_val - current_val;//���
	trans_nan(&pid->err);
	
	pid->sum_err += pid->err;
//	pid->temp += (u32)(pid->err * 10000.0f);//���ڸ�������˷��� NAN ���ڴ������任���ض�С��λ��
//	pid->sum_err = pid->temp * 0.0001f;
//	pid->target_der = target_val - pid->old_target_val;
	pid->dev_err = pid->err - pid->last_err;

	
/* ���PID������  ����AGV�ٶȺͽ��ٶȼ���ֵ����ض� */
	if(pid->sum_err > max_val)  pid->sum_err = max_val;
	if(pid->sum_err < -max_val)  pid->sum_err = -max_val;
	
	
//	/* ���һ��ʼPID������ �ۼӺ͹�����ᷢ�������ֹ��塱��Ҫ���С����ַ��롱�������Сʱ�����䷢������ */
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

		
		//��������
		pid->last_err = pid->err;
//		pid->old_target_val = target_val;
		
		
		/* ���PID�������AGV�ٶȺͽ��ٶȼ���ֵ����ض� */
		if(pid->output > max_val)  pid->output = max_val;
		if(pid->output < -max_val)  pid->output = -max_val;
		
		
//		pid->last_Ka = pid->Ka;
//		pid->last_Kp = pid->Kp;
//		pid->last_Ki = pid->Ki;
//		pid->last_Kd = pid->Kd;
//		printf("pid->output=%f\r\n",pid->output);
		return pid->output;
}


/**  �˶����� **/
void MOTION_CONTROL(void)
{
	pid_control();// PID�ջ�����
	pid_select();// ���� "DELETEE" ��������ʼPID����ѡ�������
	ULTRA_OBSTAC_AVOID();// ����������
}


/** �������ܣ�

* S ���߼Ӽ���
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
					target_old_v = target_v;// Ŀ�����ٶȹ�С����ִ�мӼ��ٹ���
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
					target_old_w = target_w;// Ŀ����ٶȹ�С����ִ�мӼ��ٹ���
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



/** void pid_control(void) �������ܣ�
* 
* ���ٶȺ����ٶ�pid �ջ�����
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
				if(fabs(target_omega) < fabs(vw_from_ros[2]))  target_omega_before_acc = vw_from_ros[2];//��ros��ȡ�Ľ��ٶ�  ��/s
				else                                           target_omega_before_acc = target_omega;
				
				if(fabs(target_v) < fabs(vw_from_ros[0]))      target_v_before_acc = vw_from_ros[0];//��ros��ȡ��x����ǰ�����򣩵����ٶ� m/s
				else                                           target_v_before_acc = target_v;				
			}

//			printf("target_v_before_acc��    %f  m/s\r\n",target_v_before_acc);
//			printf("target_omega��%f  ��/s\r\n",target_omega);
//			printf("target_v_before_acc: %f\r\n",target_v_before_acc);
//			printf("target_omega_before_acc: %f\r\n",target_omega_before_acc);
			if(target_v_before_acc > AGV_Max_v)          {target_v_before_acc = AGV_Max_v;}
			else if (target_v_before_acc < AGV_Min_v)    {target_v_before_acc = AGV_Min_v;}
			
			if(target_omega_before_acc > AGV_Max_omega)       {target_omega_before_acc = AGV_Max_omega;}
			else if(target_omega_before_acc < AGV_Min_omega)  {target_omega_before_acc = AGV_Min_omega;}
			
	/******************************************vehicle pid control***********************************************/			

			// ���� S �������ݽ��мӼ��ټ���
			target_v_after_acc = s_acc(target_v_before_acc, v_overall, 0);// ���� 0 �����ٶ� v
			target_omega_after_acc = s_acc(target_omega_before_acc, w_overall, 1);// ���� 1 ������ٶ� omega

			if(target_v_after_acc > AGV_Max_v)          {target_v_after_acc = AGV_Max_v;}
			else if (target_v_after_acc < AGV_Min_v)    {target_v_after_acc = AGV_Min_v;}
			
			if(target_omega_after_acc > AGV_Max_omega)       {target_omega_after_acc = AGV_Max_omega;}
			else if(target_omega_after_acc < AGV_Min_omega)  {target_omega_after_acc = AGV_Min_omega;}
			
//			printf("target_v_after_acc: %f\r\n",target_v_after_acc);
//			printf("target_omega_after_acc: %f\r\n",target_omega_after_acc);
		

		
			vehicle_to_LR(target_v_after_acc,target_omega_after_acc);// Ŀ��ֵ�ֽ�������ֽ��ٶ�
//			vehicle_to_LR(target_v_before_acc,target_omega_before_acc);// Ŀ��ֵ�ֽ�������ֽ��ٶ�
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
				// �ҵ�����ٶ� ת��Ϊ PWMֵ��������PID���ƣ�       ���ֽ��ٶ�Ŀ��ֵ����ǰ���ٶȣ�pid���Ʋ��������ٶȼ���ֵ
				Moto2 += positional_PID_Control(enc_omega_pwm[0],encdr_data_pwm[0] , &LR_TO_PWM_R, PWM_Max);

				// �������ٶ� ת��Ϊ PWMֵ��������PID���ƣ�       ���ֽ��ٶ�Ŀ��ֵ����ǰ���ٶȣ�pid���Ʋ��������ٶȼ���ֵ
				Moto1 += positional_PID_Control(enc_omega_pwm[1], encdr_data_pwm[1], &LR_TO_PWM_L, PWM_Max);
//			}
			
	//		printf("pwm_target ��= %d\r\n",enc_omega_pwm[0]);
	//		printf("pwm_current ��= %d\r\n",encdr_data_pwm[0]);		
			
	//		printf("pwm_target ��= %d\r\n",enc_omega_pwm[1]);
	//		printf("pwm_current ��= %d\r\n",encdr_data_pwm[1]);
						

			// �жϵ������״̬
			if(fabs(encdr_data[0]) < 0.05f*fabs(enc_omega[0])) motor_status_R = 1;
			else                                               motor_status_R = 0;
			
			if(fabs(encdr_data[1]) < 0.05f*fabs(enc_omega[1])) motor_status_L = 1;
			else                                               motor_status_L = 0;
		}
		else
		{
			// ��������Ŀ���ʵ�ʽ��ٶȶ�С�� 1.0rad/s ʱ���������Ӷ�Ҫͣת
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
		
//		printf("��Moto1= %d\r\n",Moto1);
//		printf("��Moto2= %d\r\n",Moto2);
		motor_setup();// ���õ������ռ�ձ�
	/*******************************motor pid control************************************/
}


// ��С�� ���ٶ� �� ���ٶ� ת�����������ٶ�
void vehicle_to_LR(float v, float omega)
{
	agv_move_data[1] = omega * PI_div_180;//PID����ֵ  ��λ���� ��/�� ת���� ����/��
	agv_move_data[0] = v;    //PID����ֵ  ��λ����/��
	
//	printf("v == %f\r\n",v);
//	printf("omega == %f\r\n",omega);
	
	arm_mat_init_f32(&agv_move_matrix, 2, 1, agv_move_data);//��ʼ��
	arm_mat_mult_f32(&enc_inv_trans_matrix, &agv_move_matrix, &enc_omega_matrix);//����õ����ұ������Ľ��ٶ� rad/s
	vehiclelr[0] = enc_omega_matrix.pData[0];// �� arm_matrix_instance_f32������pDataָ������ת������������ rad/s   ����
	vehiclelr[1] = enc_omega_matrix.pData[1];// �� arm_matrix_instance_f32������pDataָ������ת������������ rad/s   ����
//	printf("vehiclelr[0] = %f\r\n",vehiclelr[0]);
//	printf("vehiclelr[1] = %f\r\n",vehiclelr[1]);

}



/* ���ٶ� ת���� PWMռ�ձ���ֵ */
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



/*  void pid_select(void) �������ܣ�

1. ���� DELETEE ��������ʼ�л� ���ٶ�PID/���ٶ�PID/�Ƕ�PID�������ӽ��ٶ�PID������ʼ
2. ����"alien"�������˳��л�
3. Ȼ������˳���� p i d ���������� pid�������ԣ�����"alien"��������������

*/
void pid_select(void)
{
	static u8 select_pid=0;//ָʾ�ĸ�pid�������ڱ�����
	
	if((key==82) || (select_pid & 0x80))//����ң����DELETE�������£���ʼѡ��pid���࣬��ALIEN�������������˳�
	{
		
  //                            ___ ___ ___ ___ ___ ___ ___ ___
	//pid_status����λ��ʾ�ĺ��� |___|___|___|___|___|___|___|___|
	//                        continue         R   L theta v VEHICLE_TO_LR_R
		
		static u8 i=0, j=0, rising_edge=1;
		select_pid |= (key==82)<<7;//�� continue λ��1
//		printf("select_pid:0x%x\r\n",select_pid);
		
		
		if((((select_pid) & 0xff) <= 0x9f) && (rising_edge==1) && (key==82))
		{
			select_pid |= (key==82)<<(i++);   //����Ҫ��select_pid��ǰ��λ֮�䲻��ѭ��״̬�����������״̬ʱ��Ҫ��ͷ�ٿ�ʼѭ��
			rising_edge = 0;//���㣬�������ɿ����Ͳ�������һ���������źŹ���
			j=0;//��j���㣬Ϊ��һ�η�������׼��
		}
		if(((select_pid) & 0xff) > 0x9f)// ���������״̬ʱ��Ҫ��ͷ�ٿ�ʼѭ��
		{
			select_pid =0x81;//��select_pid��Ϊ��ʼ״̬
			i=0;//��i���㣬��0����λ����ʼ��λ
			j=0;//��j���㣬Ϊ��һ�η�������׼��
		}
		
		if(!(key==82) )
		{
			rising_edge =1;//��"DELETE"�����ɿ�ʱ�����ܼ�������һ���������źŹ���
		}
		
		
		//���ٶ�PID����ѡ��
		if(select_pid==0x81)
		{
			if(j==0)
			{
				j=1;//��֤����������ʱ��һ�أ�����ʱ�䲻��
				beep_on(1,100);
			}
			
			pid_setup(&VEHICLE_TO_LR_R);//PID�������� 
	    DataScope(VEHICLE_TO_LR_R);//ʾ������ʾ����
			motor_pid_debug = 0;
//			printf("VEHICLE_TO_LR_R\r\n");
		}
		
		
		///���ٶ�PIDѡ��
		if(select_pid==0x83)
		{
			if(j==0)
			{
				j=1;//��֤����������ʱ��һ�أ�����ʱ�䲻��
				beep_on(2,100);
			}
			
			pid_setup(&VEHICLE_TO_LR_L);//PID�������� 
	    DataScope(VEHICLE_TO_LR_L);//ʾ������ʾ����
			motor_pid_debug = 0;
//			printf("VEHICLE_TO_LR_L\r\n");
		}
		
		
		///�Ƕ�PIDѡ��
		if(select_pid==0x87)
		{
			if(j==0)
			{
				j=1;//��֤����������ʱ��һ�أ�����ʱ�䲻��
				beep_on(3,100);
			}
			
//			pid_setup(&THETA_TO_VEHICLE);//PID�������� 
//			DataScope(THETA_TO_VEHICLE);//ʾ������ʾ����
			motor_pid_debug = 0;
			printf("THETA_TO_VEHICLE\r\n");
		}
		
		
		///���ֽ��ٶ�PIDѡ��
		if(select_pid==0x8f)
		{
			if(j==0)
			{
				j=1;//��֤����������ʱ��һ�أ�����ʱ�䲻��
				beep_on(4,100);
			}
			
			pid_setup(&LR_TO_PWM_L);//PID�������� 
			DataScope(LR_TO_PWM_L);//ʾ������ʾ����
			motor_pid_debug = 1;
			printf("LR_TO_PWM_L\r\n");
		}
		
		
		///���ֽ��ٶ�PIDѡ��
		if(select_pid==0x9f)
		{
			if(j==0)
			{
				j=1;//��֤����������ʱ��һ�أ�����ʱ�䲻��
				beep_on(5,100);
			}
			
			pid_setup(&LR_TO_PWM_R);//PID�������� 
			DataScope(LR_TO_PWM_R);//ʾ������ʾ����
			motor_pid_debug = 1;
			printf("LR_TO_PWM_R\r\n");
		}
		
		
		// "ALIEN"���������£���ʾPID����ѡ�����
		if( (select_pid & 0x80)&&(key==226) )
		{
			select_pid &=0x0;//����λ���㣬����PID����ѡ��
			beep_on(3,100);
			
			motor_pid_debug = 0;
		}
	}
}





/*  void pid_setup(struct PID* pid)�������ܣ�

1. ����˳���� p i d ���������� pid��������
2. ����"alien"��������������
3. ����pid���Ժ󣬰��� p ��������ʾ����p�������� vol-/vol+�����ı����ֵ
4. ���� i ��������ʾ���� i �������� vol-/vol+�����ı����ֵ
5. ���� d ��������ʾ���� d �������� vol-/vol+�����ı����ֵ
*/
void pid_setup(pid_struct* pid)
{
	//                            ___ ___ ___ ___ ___ ___ ___ ___
	//pid_status����λ��ʾ�ĺ��� |___|___|___|___|___|___|___|___|
	//                      continue start end off a   d   i   p

	 static u8 pid_status=0;
	
	//                (����2/a)  ���ⰴ�� 3/d             4/i             7/p  ͬʱ����
	 pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
	 pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//ͬʱ����3/d   4/i  7/p ���� start״̬
	
	// endλΪ1��startλҲΪ1ʱ��˵�����ٴε���pid����Ҫ��endλ����
	if( (pid_status & 0x60)==0x60 )
	{
		pid_status &= 0xdf;// �� pid_status��endλ���㣬��ʼ��һ��pid����
	}
	 
	// �ֱ��ж�pid_status�� start  continue end λ��״̬����ʼ����PIDֵ
	 if( ( (pid_status & 0x40)==0x40 || (pid_status & 0x80)==0x80 ) && (pid_status & 0x20)==0 ) 
	 {
		 if( (pid_status & 0x10)==0 )
		 {
			 beep_on(2,100);//ÿ�ο�ʼpid���ԣ���������2�Σ�ÿ����100ms
			 pid_status &= 0xb8;//�Ѹոս���ʱ��p i d ����λ���Լ�start����
		 }
		 
		 
		 pid_status |= 0x10;// ��pid_status��4λ��1����֤beepֻ�ڸս���pid����ʱ��һ��
		 pid_status |= 0x80;// �� pid_status��continueλ�򿪣���֤����start�󣬿��Գ�������pid
		 pid_status &= 0xdf;// �� pid_status��endλ���㣬��֤����start�󣬿��Գ�������pid
		 
		 
		 // ���� p
		 if ((pid_status & 0x1) == 0x1)
		 {
				//                (����2/a)  ���ⰴ�� 3/d             4/i             7/p  ͬʱ����
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//ͬʱ����3/d   4/i  7/p ���� start״̬
			 
			 
			 //����ǰ����ֵ������ a d i p λ����
			 if( (pid_status & 0x2) == 0x2 )
			 {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x2;//����i
			 }
			 
			 if( (pid_status & 0x4) == 0x4 )
			 {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x4;//����d
			 }
			 
			 if( (pid_status & 0x8) == 0x8 )
			 {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x8;//����a
			 }
			 
			 if( (pid_status & 0x1) == 0x1 )
			 {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x1;//����p
			 }

			 
			 //����p
			if(key==144) //VOL+����
			{
				beep_on(1,100);//��������1�Σ���ʾ������
				pid->Kp+=0.05*RmtCnt;
			}
			
			if(key==224) //VOL-����
			{
				beep_on(1,100);//��������1�Σ���ʾ������
				pid->Kp-=0.05*RmtCnt;
			}
			
//			printf("����p:\r\nKp=%f\r\n",Kp);
//			printf("pid_status-p:\r\n0x%x\r\n",pid_status);
		 }
		 
		 
		 
		 
		 // ���� i
		 if ((pid_status & 0x2) == 0x2)
		 {
				//                (����2/a)  ���ⰴ�� 3/d             4/i             7/p  ͬʱ����
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//ͬʱ����3/d   4/i  7/p ���� start״̬

			 
				if( (pid_status & 0x4) == 0x4 )
				{
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x4;//����d
				}

				if( (pid_status & 0x8) == 0x8 )
				{
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x8;//����a
				}


				if( (pid_status & 0x1) == 0x1 )
				{
					pid_status &=0xf0;//��p i d a λ����
					pid_status |=0x1;//����p
				}

				if( (pid_status & 0x2) == 0x2 )
				{
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x2;//����i
				}

			 
			 //����i
			 if(key==144) //VOL+����
			{
				beep_on(1,100);//��������1�Σ���ʾ������
				pid->Ki+=0.05*RmtCnt;
			}
			
			if(key==224) //VOL-����
			{
				beep_on(1,100);//��������1�Σ���ʾ������
				pid->Ki-=0.05*RmtCnt;
			}
			
//			printf("����i:\r\nKi=%f\r\n",Ki);
//			printf("pid_status-i:\r\n0x%x\r\n",pid_status);
		 }
		 
		 
		 
		 
		 // ���� d
		 if ((pid_status & 0x4) == 0x4)
		 {
			 	//                (����2/a)  ���ⰴ�� 3/d             4/i             7/p  ͬʱ����
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//ͬʱ����3/d   4/i  7/p ���� start״̬
			 
			 
				if( (pid_status & 0x8) == 0x8 )
				{
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x8;//����a
				}
				 
				if( (pid_status & 0x1) == 0x1 )
				{
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x1;//����p
				}
				 
				 if( (pid_status & 0x2) == 0x2 )
			  {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x2;//����i
			  }
			 
				 
				 if( (pid_status & 0x4) == 0x4 )
				 {
					 pid_status &=0xf0;//��p i d a λ����
					 pid_status |=0x4;//����d
				 }

			 
			 //����d
			 if(key==144) //VOL+����
			{
				beep_on(1,100);//��������1�Σ���ʾ������
				pid->Kd+=0.05*RmtCnt;
			}
			
			if(key==224) //VOL-����
			{
				beep_on(1,100);//��������1�Σ���ʾ������
				pid->Kd-=0.05*RmtCnt;
			}
			
//			printf("����d:\r\nKd=%f\r\n",Kd);
//			printf("pid_status-d:\r\n0x%x\r\n",pid_status);
		 }
		 
		 
		 
		 
		 // ���� a
		 if ((pid_status & 0x8) == 0x8) 
		 {
				//                (����2/a)  ���ⰴ�� 3/d             4/i             7/p  ͬʱ����
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//ͬʱ����3/d   4/i  7/p ���� start״̬
			 
			 if( (pid_status & 0x1) == 0x1 )
			 {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x1;//����p
			 }
				 
			 if( (pid_status & 0x2) == 0x2 )
			 {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x2;//����i
			 }
			 
			 if( (pid_status & 0x4) == 0x4 )
			 {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x4;//����d
			 }
			 
			 
			 if( (pid_status & 0x8) == 0x8 )
			 {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x8;//����a
			 }
			 
				//����a
				if(key==144) //VOL+����
				{
					beep_on(1,100);//��������1�Σ���ʾ������
					pid->Ka+=0.05*RmtCnt;
				}
				
				if(key==224) //VOL-����
				{
					beep_on(1,100);//��������1�Σ���ʾ������
					pid->Ka-=0.05*RmtCnt;
				}
			
//			printf("����a:\r\nKa=%f\r\n",Ka);
//			printf("pid_status-a:\r\n0x%x\r\n",pid_status);
		 }
		 
		 
		 /////////////////////////////////��֤p i d a ÿ���������涼��ʣ�������ѡ////////////////////////////////////
		 
		 // ���� p
		 if ((pid_status & 0x1) == 0x1)
		 {
				//                (����2/a)  ���ⰴ�� 3/d             4/i             7/p  ͬʱ����
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//ͬʱ����3/d   4/i  7/p ���� start״̬
			 
			 
			 //����ǰ����ֵ������ a d i p λ����
			 if( (pid_status & 0x2) == 0x2 )
			 {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x2;//����i
			 }
			 
			 if( (pid_status & 0x4) == 0x4 )
			 {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x4;//����d
			 }
			 
			 if( (pid_status & 0x8) == 0x8 )
			 {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x8;//����a
			 }
			 
			 if( (pid_status & 0x1) == 0x1 )
			 {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x1;//����p
			 }

			 
			 //����p
			if(key==144) //VOL+����
			{
				beep_on(1,100);//��������1�Σ���ʾ������
				pid->Kp+=0.05*RmtCnt;
			}
			
			if(key==224) //VOL-����
			{
				beep_on(1,100);//��������1�Σ���ʾ������
				pid->Kp-=0.05*RmtCnt;
			}
			
//			printf("����p:\r\nKp=%f\r\n",Kp);
//			printf("pid_status-p:\r\n0x%x\r\n",pid_status);
		 }
		 
		 
		 
		 
		 // ���� i
		 if ((pid_status & 0x2) == 0x2)
		 {
				//                (����2/a)  ���ⰴ�� 3/d             4/i             7/p  ͬʱ����
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//ͬʱ����3/d   4/i  7/p ���� start״̬

			 
				if( (pid_status & 0x4) == 0x4 )
				{
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x4;//����d
				}

				if( (pid_status & 0x8) == 0x8 )
				{
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x8;//����a
				}


				if( (pid_status & 0x1) == 0x1 )
				{
					pid_status &=0xf0;//��p i d a λ����
					pid_status |=0x1;//����p
				}

				if( (pid_status & 0x2) == 0x2 )
				{
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x2;//����i
				}

			 
			 //����i
			 if(key==144) //VOL+����
			{
				beep_on(1,100);//��������1�Σ���ʾ������
				pid->Ki+=0.05*RmtCnt;
			}
			
			if(key==224) //VOL-����
			{
				beep_on(1,100);//��������1�Σ���ʾ������
				pid->Ki-=0.05*RmtCnt;
			}
			
//			printf("����i:\r\nKi=%f\r\n",Ki);
//			printf("pid_status-i:\r\n0x%x\r\n",pid_status);
		 }
		 
		 
		 
		 
		 // ���� d
		 if ((pid_status & 0x4) == 0x4)
		 {
				//                (����2/a)  ���ⰴ�� 3/d             4/i             7/p  ͬʱ����
				pid_status |= ( (key==152)<<3  | (key==176)<<2  | (key==48)<<1 | (key==16)<<0 );
				pid_status |= ( (pid_status & 0x7)==0x7 ) << 6;//ͬʱ����3/d   4/i  7/p ���� start״̬
			 
			 
				if( (pid_status & 0x8) == 0x8 )
				{
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x8;//����a
				}
				 
				if( (pid_status & 0x1) == 0x1 )
				{
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x1;//����p
				}
				 
				 if( (pid_status & 0x2) == 0x2 )
			  {
				 pid_status &=0xf0;//��p i d a λ����
				 pid_status |=0x2;//����i
			  }
			 
				 
				 if( (pid_status & 0x4) == 0x4 )
				 {
					 pid_status &=0xf0;//��p i d a λ����
					 pid_status |=0x4;//����d
				 }

			 
			 //����d
			 if(key==144) //VOL+����
			{
				beep_on(1,100);//��������1�Σ���ʾ������
				pid->Kd+=0.05*RmtCnt;
			}
			
			if(key==224) //VOL-����
			{
				beep_on(1,100);//��������1�Σ���ʾ������
				pid->Kd-=0.05*RmtCnt;
			}
			
//			printf("����d:\r\nKd=%f\r\n",Kd);
//			printf("pid_status-d:\r\n0x%x\r\n",pid_status);
		 }
		 
		 /////////////////////////////////////////////////////////////////////////
		 
		 
		 
		 // ��������
		 if( ( (pid_status & 0x90) == 0x90 ) && (key==226) )// ��pid���Թ����У����¡�alien����������������
		 {
			 beep_on(3,100);//ÿ��pid��������������3��
		   pid_status &= 0x0;// ��pid_status ����λ��0
			 pid_status |=0x20;//��endλ��1������pid����
		 }
	}
	 
//	printf("pid_status:\r\n0x%x\r\n",pid_status);
//	printf("key:\r\n%d\r\n",key);
	
}




#if __PID_TIME__

//������ʱ��7 �жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��7!
void TIM7_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///ʹ�� TIM7 ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(PID_TIM,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(PID_TIM,TIM_IT_Update,ENABLE); //���� ��ʱ��7 �����ж�
	TIM_Cmd(PID_TIM,ENABLE); //ʹ�� ��ʱ��7
	
	NVIC_InitStructure.NVIC_IRQChannel = PID_IRQn; //��ʱ��7 �ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PID_TIM_PreemptionPriority; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = PID_TIM_SubPriority; //�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}



//��ʱ��7�жϷ�����
void PID_IRQHandler(void)
{
	if(TIM_GetITStatus(PID_TIM,TIM_IT_Update)==SET) //����ж�
	{
			timeout_pid++;
	}
	TIM_ClearITPendingBit(PID_TIM,TIM_IT_Update);  //����жϱ�־λ
}
#endif


