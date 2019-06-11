#ifndef __CANTOL_H__
#define __CANTOL_H__
#include "include.h"
void balance_control(void);
int balance(float Angle,float Gyro);
void motor_PID_control(void);
void car_speed_set(void);
void car_break_function(void);
void motor_PWN_updat(void);
void tingche(void);
void duoji_PID (void);      //¶æ»úPID
void Kalman_Filter(float Accel,float Gyro);
void Set_Pwm(int Balance_Pwm_L_d,int Balance_Pwm_R_d);
void Xianfu_Pwm(void);


#endif 