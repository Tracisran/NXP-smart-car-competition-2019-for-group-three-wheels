#include "Inertial_Sensor.h"
#include "status.h"
#include "AP_Math.h"
#include "include.h"
#define G					  9.80665f		      // m/s^2	
#define RadtoDeg    57.324841f				//弧度到角度 (弧度 * 180/3.1415)
#define DegtoRad    0.0174533f				//角度到弧度 (角度 * 3.1415/180)
#define Acc_Gain  	0.0001220f				//加速度变成G (初始化加速度满量程-+4g LSBa = 2*4/65535.0)
#define Gyro_Gain 	0.0609756f				//角速度变成度 (初始化陀螺仪满量程+-2000 LSBg = 2*2000/65535.0)
#define Gyro_Gr	    0.0010641f			  //角速度变成弧度(3.1415/180 * LSBg)   
#define VAR         0.001f                  //方差
#define jiaozhun    1
/*低通滤波*/
//float alpha1;
///**设置低通滤波的频率sample_freq  和  截止频率cutoff_freq*/
//void compute_alpha(float sample_freq, float cutoff_freq) 
//{
//    if (cutoff_freq <= 0.0f || sample_freq <= 0.0f) {
//        alpha1 = 1.0;
//    } else {
//        float dt = 1.0/sample_freq;
//        float rc = 1.0f/(M_2PI*cutoff_freq);
//        alpha1 = constrain_float(dt/(dt+rc), 0.0f, 1.0f);
//    }
//}
///*需要滤波的信号sample  上次的输出信号_output*/
//float apply(float sample, float  _output) 
//{
//    _output += (sample - _output) * alpha1;
//    return _output;
//}



/*惯性传感器初始化*/
uint8_t ins_init(void)
{
  //  compute_alpha(1000, 20);    //输入信号 1khz  截止频率 20hz
//    return MPU6050_Init();
    //Init_LQ_9AX();
  return 0;
}


/*惯性传感器校准*/
vector3i_t _acc_vector_offset;   //加计零偏值
vector3i_t _gyro_vector_offset;  //角速度计零偏值
vector3i_t _mag_vector_offset;  //地磁计零偏值
void ins_calibration(void)
{
    uint16_t i = 0;

    vector3i_t _acc_vector;   //存放加计读取的原始数据
    vector3i_t _gyro_vector;  //存放角速度计读取的原始数据
    //vector3i_t _mag_vector;
    vector3f_t _gyro_sum;     //存放角速度计和
    vector3f_t _acc_sum;      //存放加计和
    vector3f_t _gyro_var;     //存放角速度计平方
    vector3f_t _acc_var;      //存放加计平方
    while(i++ < 500)
    {
        int16_t ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z,MAG_X,MAG_Y,MAG_Z;
        Update9AX(&ACC_X, &ACC_Y, &ACC_Z, &GYRO_X, &GYRO_Y, &GYRO_Z, &MAG_X, &MAG_Y, &MAG_Z);         //存放读取传感器 数据
            //获取原始数据  
        _acc_vector.x = ACC_X;
        _acc_vector.y = ACC_Y;
        _acc_vector.z = ACC_Z;
        _gyro_vector.x = -GYRO_X;
        _gyro_vector.y = -GYRO_Y;
        _gyro_vector.z = GYRO_Z;
        _mag_vector.x = MAG_X;
        _mag_vector.y = MAG_Y;
        _mag_vector.z = MAG_Z;
        
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

    }
    if(   invSqrt(_gyro_var.x - _gyro_sum.x * _gyro_sum.x) > VAR 
       && invSqrt(_gyro_var.y - _gyro_sum.y * _gyro_sum.y) > VAR 
       && invSqrt(_acc_var.x - _acc_sum.x * _acc_sum.x) > VAR 
       && invSqrt(_acc_var.y - _acc_sum.y * _gyro_sum.y) > VAR )     //所有数据方差在一定值内，说明校准时，没有运动
    {
        _acc_vector_offset.x   = (int16_t)_acc_sum.x;          //保存静止时的零偏值
        _acc_vector_offset.y   = (int16_t)_acc_sum.y;
        _acc_vector_offset.z   = (int16_t)_acc_sum.z;
        _gyro_vector_offset.x  = (int16_t)_gyro_sum.x;
        _gyro_vector_offset.y  = (int16_t)_gyro_sum.y;
        _gyro_vector_offset.z  = (int16_t)_gyro_sum.z;
    }    
        _status.ins_calibration = 0;                    //校准标志位清零
    
}

    vector3f_t _acc_vector;
    vector3f_t _gyro_vector;
    vector3f_t _mag_vector;    
    vector3f_t new_gyro_vector;
    vector3f_t new_acc_vector;
//vector3f_t _gyro_vector;     //全局变量，存放处理后的角速度数据
//vector3f_t _acc_vector;      //全局变量，存放处理后的加数据
//vector3f_t _mag_vector;      //全局变量，存放处理后的地磁数据
char  txt[16]="X:";
void ins_update(void)
{

    int16_t ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z,MAG_X,MAG_Y,MAG_Z;

    
    if(_status.ins_calibration)  //如果需要校准
    {
        ins_calibration();      //校准
    }
    
  //  LPTMR_time_start_us(); 
    Update9AX(&ACC_X, &ACC_Y, &ACC_Z, &GYRO_X, &GYRO_Y, &GYRO_Z, &MAG_X, &MAG_Y, &MAG_Z);
    
        _acc_vector.x = ACC_X;
        _acc_vector.y = ACC_Y;
        _acc_vector.z = ACC_Z;
        _gyro_vector.x = -GYRO_X;
        _gyro_vector.y = -GYRO_Y;
        _gyro_vector.z = GYRO_Z;
        _mag_vector.x = MAG_X;
        _mag_vector.y = MAG_Y;
        _mag_vector.z = MAG_Z;
//    if(LPTMR_time_get_us())
//   {
//     LCD_P6x8Str(0,1,"111");
//   }
//    LPTMR_Pulse_Clean();
    
//    acc_vector.x = ACC_X;               
//    acc_vector.y = ACC_Y;
//    acc_vector.z = ACC_Z;
//    gyro_vector.x = -GYRO_X;
//    gyro_vector.y = -GYRO_Y;
//    gyro_vector.z = GYRO_Z;
///***5.18关闭，原来的更新程序**********/
///*
//    Update9AX();    //获取原始数据  
//    acc_vector.x = chuandi(1);               ////改动把原来的读取数据换成update9ax里面的数据
//    acc_vector.y = chuandi(2);
//    acc_vector.z = chuandi(3);
//    gyro_vector.x = -chuandi(4);
//    gyro_vector.y = -chuandi(5);
//    gyro_vector.z = chuandi(6);
//*/  
///***********上传陀螺仪数据************/    
//   
    _acc_vector.x -= _acc_vector_offset.x;   //去零偏
    _acc_vector.y -= _acc_vector_offset.y;   //去零偏
    _acc_vector.z -= _acc_vector_offset.z;   //去零偏
    _gyro_vector.x -= _gyro_vector_offset.x;   //去零偏
    _gyro_vector.y -= _gyro_vector_offset.y;   //去零偏
    _gyro_vector.z -= _gyro_vector_offset.z;   //去零偏
    
    //加速度AD值 转换成 米/平方秒 
//	new_acc_vector.x = (float)_acc_vector.x * Acc_Gain * G;
//	new_acc_vector.y = (float)_acc_vector.y * Acc_Gain * G;
//	new_acc_vector.z = (float)_acc_vector.z * Acc_Gain * G;
//    
//	//陀螺仪AD值 转换成 弧度/秒    
//	new_gyro_vector.x = (float) _gyro_vector.x * Gyro_Gr;  
//	new_gyro_vector.y = (float) _gyro_vector.y * Gyro_Gr;
//	new_gyro_vector.z = (float) _gyro_vector.z * Gyro_Gr;
//    
//    //低通滤波
////    _acc_vector.x = apply(new_acc_vector.x, _acc_vector.x);
////    _acc_vector.y = apply(new_acc_vector.y, _acc_vector.y);
////    _acc_vector.z = apply(new_acc_vector.z, _acc_vector.z);
////    _gyro_vector.x = apply(new_gyro_vector.x, _gyro_vector.x);
////    _gyro_vector.y = apply(new_gyro_vector.y, _gyro_vector.y);
////    _gyro_vector.z = apply(new_gyro_vector.z, _gyro_vector.z);
//
///**************5.18更改，都乘了10原本没乘***********/
//     _acc_vector.x = new_acc_vector.x*jiaozhun;
//     _acc_vector.y = new_acc_vector.y*jiaozhun;
//     _acc_vector.z = new_acc_vector.z*jiaozhun;
//     _gyro_vector.x = new_gyro_vector.x*jiaozhun;
//     _gyro_vector.y = new_gyro_vector.y*jiaozhun;
//     _gyro_vector.z = new_gyro_vector.z*jiaozhun;
//     //ANO_DT_send_int16(_gyro_vector.x,_gyro_vector.y,_gyro_vector.z,0,0);
////     if(LPTMR_time_get_us())
////   {
////     LCD_P6x8Str(0,1,"111");
////   }
////     
///***********OLED显示******************/
////   LCD_Print16(90,0,(int)_acc_vector.x);
////   LCD_Print16(90,1,(int)_acc_vector.y);
////   LCD_Print16(90,2,(int)_acc_vector.z);
////   LCD_Print16(90,3,ACC_X);
////   LCD_Print16(90,4,ACC_Y);
////   LCD_Print16(90,5,ACC_Z);
//
//    
////    sprintf((char*)txt,"%06d",ACC_X);
////    LCD_P6x8Str(0,1,(u8*)txt);
////    sprintf((char*)txt,"%06d",ACC_Y);
////    LCD_P6x8Str(45,1,(u8*)txt);
////    sprintf((char*)txt,"%06d",ACC_Z);
////    LCD_P6x8Str(90,1,(u8*)txt);  
////     
/////***************上传去偏差后的***************/
////    char rx_1[16];
////    sprintf(rx_1,"rx=%f     ",_acc_vector.x);
////    UART_Put_Str(UART_4,rx_1);
////    
////    sprintf(rx_1,"ry=%f     ",_acc_vector.y);
////    UART_Put_Str(UART_4,rx_1);
////    
////    sprintf(rx_1,"rz=%f     \r\n",_acc_vector.z);
////    UART_Put_Str(UART_4,rx_1);
//    
////
////    sprintf(rx_1,"rx=%f     ",_gyro_vector.x);
////    UART_Put_Str(UART_4,rx_1);
////    sprintf(rx_1,"rx=%f\r\n",_gyro_vector_offset.x);
////    UART_Put_Str(UART_4,rx_1);
////    
////    sprintf(rx_1,"rx=%f     ",_gyro_vector.y);
////    UART_Put_Str(UART_4,rx_1);
////    sprintf(rx_1,"rx=%f\r\n",_gyro_vector_offset.y);
////    UART_Put_Str(UART_4,rx_1);
////    
////    sprintf(rx_1,"rx=%f     ",_gyro_vector.z);
////    UART_Put_Str(UART_4,rx_1);      
////    sprintf(rx_1,"rx=%f\r\n",_gyro_vector_offset.z);
////    UART_Put_Str(UART_4,rx_1);


}

vector3f_t get_gyro()
{    
	return _gyro_vector;
}

vector3f_t get_acc()
{
	return _acc_vector;
}


ins_t ins =
{

	ins_init,
	ins_calibration,
	ins_update,
	get_gyro,
	get_acc,

};

