#include "include.h"
#define G					  9.80665f		      // m/s^2	
#define RadtoDeg    57.324841f				//弧度到角度 (弧度 * 180/3.1415)
#define DegtoRad    0.0174533f				//角度到弧度 (角度 * 3.1415/180)
#define Acc_Gain  	0.0001221f				//加速度变成G (初始化加速度满量程-+4g LSBa = 2*4/65535.0)
#define Gyro_Gain 	0.0609756f				//角速度变成度 (初始化陀螺仪满量程+-2000 LSBg = 2*2000/65535.0)
#define Gyro_Gr	    0.0010641f			  //角速度变成弧度(3.1415/180 * LSBg)   
#define VAR         0.001f                  //方差

typedef struct
{
	float x;
	float y;
	float z;
}vector3f_t;

typedef struct 
{
	float q0;
	float q1;
	float q2;
	float q3;
}Q4_t;

int16_t ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z,MAG_X,MAG_Y,MAG_Z;

static Q4_t Q4 = {1, 0, 0, 0};  //四元数

const float ahrs_kp = 50.0f; //PI控制器，修正机体坐标系
const float ahrs_ki = 1.0f;

static vector3f_t integral;  //机体坐标误差积分
static vector3f_t ahrs_angle;  //机体坐标误差积分

float alpha=0.1;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}vector3i_t;

    vector3i_t _acc_vector_offset;   //加计零偏值
    vector3i_t _gyro_vector_offset;  //角速度计零偏值
    vector3i_t _mag_vector_offset;  //地磁计零偏值
    vector3i_t acc_vector;
    vector3i_t gyro_vector;
  
    vector3f_t new_gyro_vector;
    vector3f_t new_acc_vector;
    
/**************************************
5.28号加，为了传递回来下面数据
**************************************/   
float get_gro_y = 0;
float get_pitch = 0;


float apply(float sample, float  _output) 
{
    _output += (sample - _output) * alpha;
    return _output;
}

void AHRS_quat_update(vector3f_t gyro, vector3f_t acc, float interval)
{
	float q0 = Q4.q0;
	float q1 = Q4.q1;
	float q2 = Q4.q2;
	float q3 = Q4.q3;
/***********  模长  ************/	
	float norm = 1/sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
/***********  加计测出的机体坐标系   **********/
	float ax = acc.x * norm;
	float ay = acc.y * norm;
	float az = acc.z * norm;
/***********  四元数解算出的机体坐标系  ***************/
	float half_vx = q1*q3 - q0*q2;
	float half_vy = q2*q3 + q0*q1;
	float half_vz = -0.5f + q0*q0 + q3*q3;
/***********  叉积求加计机体坐标和上次四元数解算机体坐标差  ************/	
	float half_ex = ay*half_vz - az*half_vy;
	float half_ey = az*half_vx - ax*half_vz;
	float half_ez = ax*half_vy - ay*half_vx;
/***********  使用PI控制器 修正机体坐标 *************/	
	integral.x += half_ex * ahrs_ki * interval;
	integral.y += half_ey * ahrs_ki * interval;
	integral.z += half_ez * ahrs_ki * interval;
	
	float gx = (gyro.x + ahrs_kp * half_ex + integral.x) * 0.5f * interval;
	float gy = (gyro.y + ahrs_kp * half_ey + integral.y) * 0.5f * interval;
	float gz = (gyro.z + ahrs_kp * half_ez + integral.z) * 0.5f * interval;
	
/**********  更新四元数  ********/
	Q4.q0 += (-q1 * gx - q2 * gy - q3 * gz); 
	Q4.q1 += ( q0 * gx + q2 * gz - q3 * gy); 
	Q4.q2 += ( q0 * gy - q1 * gz + q3 * gx); 
	Q4.q3 += ( q0 * gz + q1 * gy - q2 * gx); 
  //单位化四元数 	
	norm = 1/sqrt(Q4.q0 * Q4.q0 + Q4.q1 * Q4.q1 + Q4.q2 * Q4.q2 + Q4.q3 * Q4.q3);
	
	Q4.q0 *= norm;
	Q4.q1 *= norm;
	Q4.q2 *= norm;
	Q4.q3 *= norm;
}


void AHRS_quat_to_angle(void)
{
	float conv_x = 2.0f * (Q4.q0 * Q4.q2 - Q4.q1 * Q4.q3);  
	float conv_y = 2.0f * (Q4.q0 * Q4.q1 + Q4.q2 * Q4.q3);
	float conv_z = Q4.q0 * Q4.q0 - Q4.q1 * Q4.q1 - Q4.q2 * Q4.q2 + Q4.q3 * Q4.q3;
/*******  姿态解算  ********/
	ahrs_angle.x = atan(conv_y * 1/sqrt(conv_x * conv_x + conv_z * conv_z)) * 57.2958f;
	ahrs_angle.y = asin(2 * (Q4.q0 * Q4.q2 - Q4.q3 * Q4.q1)) * 57.2958f;
	ahrs_angle.z = atan2(2 * (Q4.q0 * Q4.q3 + Q4.q1 * Q4.q2), 1 - 2 * (Q4.q2 * Q4.q2 + Q4.q3 * Q4.q3)) * 57.2958f; 
      
      if(acc_vector.z>= 0)
      {
        get_pitch = ahrs_angle.y;
      }
      else
      {
        get_pitch = 180-ahrs_angle.y;
      }
      ANO_DT_send_int16(acc_vector.z,Q4.q1*1000,Q4.q2*1000,Q4.q3*1000,get_pitch*10);
/*******  角度微调  ********/
//	ahrs_angle.x -= 
//	ahrs_angle.y -= 
//    static float offset = 0;   
//    offset -= 0.0005585 * 0.5f;  //补偿yaw 根据自己解算速度 自行补偿  1ms解算一次 补偿 0.0005585
//	ahrs_angle.z += offset;      //补偿yaw 根据自己解算速度 自行补偿
	
}

vector3f_t _gyro_vector;     //全局变量，存放处理后的角速度数据
vector3f_t _acc_vector;      //全局变量，存放处理后的加数据
vector3f_t _mag_vector;      //全局变量，存放处理后的地磁数据


    
    
    
void Init_9250(void)
{
  IIC_Init();                         //初始化I2C1 
  
  ///////FXAS21002//////////////////////////////////////////////////////////////////////////////////////////
  // write 0000 0000 = 0x00 to CTRL_REG1 to place FXOS21002 in Standby
  // [7]: ZR_cond=0
  // [6]: RST=0
  // [5]: ST=0 self test disabled
  // [4-2]: DR[2-0]=000 for 800Hz
  // [1-0]: Active=0, Ready=0 for Standby mode
  IIC_WriteByteToSlave( FXAS21002_ADDR, FXAS21002_CTRL_REG1, 0x00); //设置采样率800Hz
  // write 0000 0000 = 0x00 to CTRL_REG0 to configure range and filters
  // [7-6]: BW[1-0]=00, LPF disabled
  // [5]: SPIW=0 4 wire SPI (irrelevant)
  // [4-3]: SEL[1-0]=00 for 10Hz HPF at 200Hz ODR
  // [2]: HPF_EN=0 disable HPF
  // [1-0]: FS[1-0]=00 for 1600dps (TBD CHANGE TO 2000dps when final trimmed parts available)
  IIC_WriteByteToSlave(FXAS21002_ADDR, FXAS21002_CTRL_REG0, 0x00);   //陀螺仪传感器,±2000dps     
  delay_ms(100);  
  // write 0000 0001 = 0x01 to CTRL_REG1 to configure 800Hz ODR and enter Active mode
  // [7]: ZR_cond=0
  // [6]: RST=0
  // [5]: ST=0 self test disabled
  // [4-2]: DR[2-0]=000 for 800Hz ODR
  // [1-0]: Active=1, Ready=0 for Active mode
  IIC_WriteByteToSlave(FXAS21002_ADDR, FXAS21002_CTRL_REG1, 0x03);   //陀螺仪工作
  
  //////FXOS8700///////////////////////////////////////////////////////////////////////////////////////////
  delay_ms(100);    
  uint8_t val;
  IIC_ReadByteFromSlave(FXOS8700_ADDR, FXOS8700_CTRL_REG1, &val);  //读CTRL1寄存器
  IIC_WriteByteToSlave(FXOS8700_ADDR, FXOS8700_CTRL_REG1, val & (uint8_t)~ACTIVE_MASK);   //使8700处于待机模式
  IIC_WriteByteToSlave(FXOS8700_ADDR, F_SETUP_REG,F_MODE_DISABLED);    //关FIFO
  IIC_WriteByteToSlave(FXOS8700_ADDR, FXOS8700_M_CTRL_REG2, MOD_HIGH_RES);   //高分辨率模式
  IIC_WriteByteToSlave( FXOS8700_ADDR, M_CTRL_REG1, (M_RST_MASK | M_OSR_MASK | M_HMS_MASK));   //混合模式，加计和地磁计同时使用
  IIC_WriteByteToSlave(FXOS8700_ADDR, M_CTRL_REG2, M_HYB_AUTOINC_MASK); 
  IIC_WriteByteToSlave(FXOS8700_ADDR, XYZ_DATA_CFG_REG, FULL_SCALE_4G);       //加计 正负4g模式
  IIC_WriteByteToSlave(FXOS8700_ADDR, FXOS8700_CTRL_REG1, (HYB_DATA_RATE_200HZ | ACTIVE_MASK));       //设置数据输出频率 200hz 并且激活FX8700
  delay_ms(10);
}

void Update9AX(short *ax,short *ay,short *az,short *gx,short *gy,short *gz, short *mx, short *my, short *mz)
{      
  uint8_t acc_buf[6]; 
  uint8_t mag_buf[6]; 
  uint8_t gyr_buf[6];
  IIC_ReadMultByteFromSlave(FXOS8700_ADDR,0x01,6,acc_buf);
  IIC_ReadMultByteFromSlave(FXOS8700_ADDR,0x33,6,mag_buf); 
  IIC_ReadMultByteFromSlave(FXAS21002_ADDR,0x01,6,gyr_buf);
  
  
  *ax = ((int16_t)((uint16_t)acc_buf[0]<<8 | (uint16_t)acc_buf[1]));  //加速度计14位的，低两位影响不大，直接按16位的用
  *ay = ((int16_t)((uint16_t)acc_buf[2]<<8 | (uint16_t)acc_buf[3]));
  *az = ((int16_t)((uint16_t)acc_buf[4]<<8 | (uint16_t)acc_buf[5]));

  *mx = (int16_t)((uint16_t)mag_buf[0]<<8 | (uint16_t)mag_buf[1]);
  *my = (int16_t)((uint16_t)mag_buf[2]<<8 | (uint16_t)mag_buf[3]);
  *mz = (int16_t)((uint16_t)mag_buf[4]<<8 | (uint16_t)mag_buf[5]);
  
  *gx = (int16_t)((uint16_t)gyr_buf[0]<<8 | (uint16_t)gyr_buf[1]);
  *gy = (int16_t)((uint16_t)gyr_buf[2]<<8 | (uint16_t)gyr_buf[3]);
  *gz = (int16_t)((uint16_t)gyr_buf[4]<<8 | (uint16_t)gyr_buf[5]);
 
}

void calibration(void)
{
      uint16_t i = 0;
          vector3i_t _acc_vector;   //存放加计读取的原始数据
    vector3i_t _gyro_vector;  //存放角速度计读取的原始数据
    vector3f_t _gyro_sum;     //存放角速度计和
    vector3f_t _acc_sum;      //存放加计和
    vector3f_t _gyro_var;     //存放角速度计平方
    vector3f_t _acc_var;      //存放加计平方
      while(i++ < 500)
    {

        Update9AX(&ACC_X, &ACC_Y, &ACC_Z, &GYRO_X, &GYRO_Y, &GYRO_Z, &MAG_X, &MAG_Y, &MAG_Z);
 
        _acc_vector.x = ACC_X;
        _acc_vector.y = ACC_Y;
        _acc_vector.z = ACC_Z;
        _gyro_vector.x = -GYRO_X;
        _gyro_vector.y = -GYRO_Y;
        _gyro_vector.z = GYRO_Z;
        
        _gyro_sum.x += _gyro_vector.x/500.0f;
        _gyro_sum.y += _gyro_vector.y/500.0f;
        _gyro_sum.z += _gyro_vector.z/500.0f;
        
        _acc_sum.x  += _acc_vector.x/500.0f;
        _acc_sum.y  += _acc_vector.y/500.0f;
        _acc_sum.z  += (_acc_vector.z - 8192)/500.0f; 
        
        _gyro_var.x += _gyro_vector.x * _gyro_vector.x/500.0f ;
        _gyro_var.y += _gyro_vector.y * _gyro_vector.y/500.0f;
        _gyro_var.z += _gyro_vector.z * _gyro_vector.z/500.0f;
        
        _acc_var.x  += _acc_vector.x * _acc_vector.x/500.0f;
        _acc_var.y  += _acc_vector.y * _acc_vector.y/500.0f;
        _acc_var.z  += (_acc_vector.z - 8192 ) * (_acc_vector.z - 8192)/500.0f; 
        delay_ms(2);
//        if(i % 50 == 0)
//        {
//            LED_Color_Reverse(red);     //红灯闪烁
//        }
    }
    if(   1/sqrt(_gyro_var.x - _gyro_sum.x * _gyro_sum.x) > VAR 
       && 1/sqrt(_gyro_var.y - _gyro_sum.y * _gyro_sum.y) > VAR 
       && 1/sqrt(_acc_var.x - _acc_sum.x * _acc_sum.x) > VAR 
       && 1/sqrt(_acc_var.y - _acc_sum.y * _gyro_sum.y) > VAR )     //所有数据方差在一定值内，说明校准时，没有运动
    {
        _acc_vector_offset.x   = (int16_t)_acc_sum.x;          //保存静止时的零偏值
        _acc_vector_offset.y   = (int16_t)_acc_sum.y;
        _acc_vector_offset.z   = (int16_t)_acc_sum.z;
        _gyro_vector_offset.x  = (int16_t)_gyro_sum.x;
        _gyro_vector_offset.y  = (int16_t)_gyro_sum.y;
        _gyro_vector_offset.z  = (int16_t)_gyro_sum.z;
    }
}

void Angle_Updata(void)
{
    Update9AX(&ACC_X, &ACC_Y, &ACC_Z, &GYRO_X, &GYRO_Y, &GYRO_Z, &MAG_X, &MAG_Y, &MAG_Z);
  
  acc_vector.x=ACC_X;
  acc_vector.y=ACC_Y;
  acc_vector.z=ACC_Z;
  gyro_vector.x=-GYRO_X;
  gyro_vector.y=-GYRO_Y;
  gyro_vector.z=GYRO_Z;

  
  //去零偏
    acc_vector.x -= _acc_vector_offset.x;   //去零偏
    acc_vector.y -= _acc_vector_offset.y;   //去零偏
    acc_vector.z -= _acc_vector_offset.z;   //去零偏
    gyro_vector.x -= _gyro_vector_offset.x;   //去零偏
    gyro_vector.y -= _gyro_vector_offset.y;   //去零偏
    gyro_vector.z -= _gyro_vector_offset.z;   //去零偏  
  
  //AD
  
  new_acc_vector.x = (float)acc_vector.x * Acc_Gain * G;
  new_acc_vector.y = (float)acc_vector.y * Acc_Gain * G;
  new_acc_vector.z = (float)acc_vector.z * Acc_Gain * G;
      
  new_gyro_vector.x = (float) gyro_vector.x * Gyro_Gr;  
  new_gyro_vector.y = (float) gyro_vector.y * Gyro_Gr;
  new_gyro_vector.z = (float) gyro_vector.z * Gyro_Gr;
  
    _acc_vector.x = apply(new_acc_vector.x, _acc_vector.x);
    _acc_vector.y = apply(new_acc_vector.y, _acc_vector.y);
    _acc_vector.z = apply(new_acc_vector.z, _acc_vector.z);
     _gyro_vector.x = apply(new_gyro_vector.x, _gyro_vector.x);
    _gyro_vector.y = apply(new_gyro_vector.y, _gyro_vector.y);
    _gyro_vector.z = apply(new_gyro_vector.z, _gyro_vector.z);
  
/**************************************
5.28号加，为了传递回来下面数据
**************************************/    

//  _acc_vector.x = new_acc_vector.x;
//  _acc_vector.y = new_acc_vector.y;
//  _acc_vector.z = new_acc_vector.z;
//  _gyro_vector.x = new_gyro_vector.x;
//  _gyro_vector.y = new_gyro_vector.y;
//  _gyro_vector.z = new_gyro_vector.z;
    get_gro_y = _gyro_vector.y;  

  
  AHRS_quat_update(_gyro_vector,_acc_vector,0.02);
  AHRS_quat_to_angle(); 
//  ANO_DT_send_int16(_acc_vector.x, _gyro_vector.x, ahrs_angle.y, 0, 0);

//    char txt[16];  
//  sprintf((char*)txt,"AX=%06d\r",(int)(_acc_vector.x*10));
//  UART_Put_Str(UART_4,txt);
//  OLED_P6x8Str(0,2,txt);
////  sprintf((char*)txt,"GX=%06d\r",(int)(_gyro_vector.z* 57.2958f));
////  UART_Put_Str(UART_4,txt);
////  OLED_P6x8Str(0,3,txt);  
////  sprintf((char*)txt,"X=%06d\r",(int)(_gyro_vector.y* 57.2958f));
////  UART_Put_Str(UART_4,txt);
////  OLED_P6x8Str(0,4,txt); 
////  
////  
//    sprintf((char*)txt,"GX=%06d\r",(int)(_gyro_vector.x* 57.2958f));
//  UART_Put_Str(UART_4,txt);
//  OLED_P6x8Str(0,3,txt);
////  sprintf((char*)txt,"GX=%06d\r",(int)(_gyro_vector.z* 57.2958f));
////  UART_Put_Str(UART_4,txt);
////  OLED_P6x8Str(0,3,txt);  
////  sprintf((char*)txt,"X=%06d\r",(int)(_gyro_vector.y* 57.2958f));
////  UART_Put_Str(UART_4,txt);
////  OLED_P6x8Str(0,4,txt);
//  
//  
// //   sprintf((char*)txt,"AX=%06d\r",(int)(ahrs_angle.x));
////UART_Put_Str(UART_4,txt);
// // OLED_P6x8Str(0,2,txt);
// // sprintf((char*)txt,"GX=%06d\r",(int)(ahrs_angle.z));
////  UART_Put_Str(UART_4,txt);
// // OLED_P6x8Str(0,3,txt);  
//  sprintf((char*)txt,"X=%06d\r",(int)(ahrs_angle.y));
//  UART_Put_Str(UART_4,txt);
//  OLED_P6x8Str(0,4,txt);

}


float Get_Gro_Y(void)
{
    return get_gro_y;
}
float Get_Pitch(void)
{
    return get_pitch;
}