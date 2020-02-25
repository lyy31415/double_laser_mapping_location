#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
#include "beep.h"
#include "delay.h"
#include "usart.h"


#define PWMA   TIM9->CCR1  //PE5
#define AIN2   PBout(0)
#define AIN1   PBout(1)
#define BIN1   PBout(6)
#define BIN2   PBout(5)
#define PWMB   TIM9->CCR2  //PE6
void DreamCar_PWM_Init(u16 arr,u16 psc);
void DreamCar_Motor_Init(void);
void Set_Pwm(int moto1,int moto2);
void Xianfu_Pwm(void);
int myabs(int a);
void motor_setup(void);
#endif
