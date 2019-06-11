#ifndef __INERTIAL_SENSOR_H
#define __INERTIAL_SENSOR_H
#include "include.h"
typedef struct
{
	float x;
	float y;
	float z;
}vector3f_t;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}vector3i_t;
  extern   vector3f_t _acc_vector;
  extern  vector3f_t _gyro_vector;
  extern  vector3f_t _mag_vector;    
  extern  vector3f_t new_gyro_vector;
  extern vector3f_t new_acc_vector;

typedef struct
{
	uint8_t (* init)(void);
	void (* calibration)(void);
	void (* update)(void);
	vector3f_t (* get_gyro)(void);
	vector3f_t (* get_acc)(void);
}ins_t;

extern ins_t ins;

//extern vector3f_t _gyro_vector;
//extern vector3f_t _acc_vector;
//extern vector3f_t _mag_vector;      //ȫ�ֱ�������Ŵ����ĵش�����


#endif //__INERTIAL_H

