#ifndef __MOTION_H
#define __MOTION_H

#include "sys.h"
#include "string.h"
#include "math.h"
#include "usart.h"
#include "arm_math.h"
#include "GYRO.h"
#include "remote.h"
#include "beep.h"
#include "DataScope_DP.h"
#include "motor.h"
#include "s_lookup.h"
#include "ultrasonic.h"

void PID_Math_Init(void);
float positional_PID_Control(float target_val, float current_val, pid_struct* pid, float max_val);
void MOTION_CONTROL(void);
void pid_select(void);
void pid_setup(pid_struct* pid);
void pid_control(void);
float s_acc(float target, float current, u8 type);
void TIM7_Init(u16 arr,u16 psc);
int omega_to_pwm(float omega, s16 pwm_base);
void vehicle_to_LR(float v, float omega);

#endif
