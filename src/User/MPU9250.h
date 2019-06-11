#ifndef MPU9250_H_
#define MPU9250_H_


extern void calibration(void);
extern void Angle_Updata(void);
extern void Init_9250(void);
float Get_Gro_Y(void);
float Get_Pitch(void);
#endif