#include "AHRS.h"
#include "Inertial_Sensor.h"
#include "LQ_SYSTICK.h"
#include "AP_Math.h"
#include "math.h"
#include "include.h"
////////////////////////////////////////////////////////////////////////////////
// Orientation
////////////////////////////////////////////////////////////////////////////////
// Convienience accessors for commonly used trig functions. These values are generated
// by the DCM through a few simple equations. They are used throughout the code where cos and sin
// would normally be used.
// The cos values are defaulted to 1 to get a decent initial value for a level state
/*static Q4_t Q4 = {1, 0, 0, 0};  //��Ԫ��
const float ahrs_kp = 1.08f; //PI��������������������ϵ         һ��ʼconst float ahrs_kp = 1.08f;
const float ahrs_ki = 0.05f;   //////һ��ʼconst float ahrs_ki = 0.05f;
static vector3f_t integral;  //��������������
static vector3f_t ahrs_angle;  //��������������
*******************������֮ǰ����Ԫ�ظ�����Ҫ��**************/

/******************�Ҹĵ���̬������Ҫ�Ĳ���*************/
#define Kp 125.0f                        // ��������֧�������������ٶȼ�/��ǿ��

#define Ki 0.000f                // ��������֧���ʵ�������ƫ�����ν�

#define Kd    1.0f

#define halfT 0.00714f  



float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // ��Ԫ����Ԫ�أ�������Ʒ���
float exInt = 0, eyInt = 0, ezInt = 0;        // ��������С�������
float  exDif=0,eyDif=0,ezDif=0;

float Yaw,Pitch,Roll;  //ƫ���ǣ������ǣ�������

/****����  AHRS_quat_update
	*����  ������Ԫ��
	*����
	*����ֵ         ֮ǰ��5.20�ر�
	***/

//
//void AHRS_quat_update(vector3f_t gyro, vector3f_t acc, float interval)   /////intervval ���
//{
//	float q0 = Q4.q0;
//	float q1 = Q4.q1;
//	float q2 = Q4.q2;
//	float q3 = Q4.q3;
///***********  ģ��  ************/	
//	float norm = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
///***********  �ӼƲ���Ļ�������ϵ   **********/
//	float ax = acc.x * norm;
//	float ay = acc.y * norm;
//	float az = acc.z * norm;
///***********  ��Ԫ��������Ļ�������ϵ  ***************/
//	float half_vx = q1*q3 - q0*q2;
//	float half_vy = q2*q3 + q0*q1;
//	float half_vz = -0.5f + q0*q0 + q3*q3;
///***********  �����Ӽƻ���������ϴ���Ԫ��������������  ************/	
//	float half_ex = ay*half_vz - az*half_vy;
//	float half_ey = az*half_vx - ax*half_vz;
//	float half_ez = ax*half_vy - ay*half_vx;
///***********  ʹ��PI������ ������������ *************/	
//	integral.x += half_ex * ahrs_ki * interval;
//	integral.y += half_ey * ahrs_ki * interval;
//	integral.z += half_ez * ahrs_ki * interval;
//	
//	float gx = (gyro.x + ahrs_kp * half_ex + integral.x) * 0.5f * interval;
//	float gy = (gyro.y + ahrs_kp * half_ey + integral.y) * 0.5f * interval;
//	float gz = (gyro.z + ahrs_kp * half_ez + integral.z) * 0.5f * interval;
//	
///**********  ������Ԫ��  ********/
//	Q4.q0 += (-q1 * gx - q2 * gy - q3 * gz); 
//	Q4.q1 += ( q0 * gx + q2 * gz - q3 * gy); 
//	Q4.q2 += ( q0 * gy - q1 * gz + q3 * gx); 
//	Q4.q3 += ( q0 * gz + q1 * gy - q2 * gx); 
//  //��λ����Ԫ�� 	
//	norm = invSqrt(Q4.q0 * Q4.q0 + Q4.q1 * Q4.q1 + Q4.q2 * Q4.q2 + Q4.q3 * Q4.q3);
//	
//	Q4.q0 *= norm;
//	Q4.q1 *= norm;
//	Q4.q2 *= norm;
//	Q4.q3 *= norm;
//}
//


/****����  
	*����  ������̬��   5.20���ϳ���
	*����
	*����ֵ
	***/


void gai_AHRS_quat_to_angle(float gx, float gy, float gz, float ax, float ay, float az)
{
//��Ԫ����ŷ����

        float norm;
        float vx, vy, vz;
        float ex, ey, ez;  
        static float last_ex,last_ey,last_ez;
        
        // ����������
        norm = sqrt(ax*ax + ay*ay + az*az);   
        ax = ax / norm;                   //��λ��
        ay = ay / norm;
        az = az / norm;     
        // ���Ʒ��������
        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        // ���������ͷ��򴫸��������ο�����֮��Ľ���˻����ܺ�
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);
        // ������������������
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;
        
        exDif = ex-last_ex;
    eyDif = ey-last_ey;
    ezDif = ez-last_ez;
    
    last_ex = ex;
    last_ey = ey;
    last_ez = ez;
        // ������������ǲ���
        gx = gx + Kp*ex + exInt + Kd*exDif;
        gy = gy + Kp*ey + eyInt + Kd*eyDif;
        gz = gz + Kp*ez + ezInt + Kd*ezDif;
        // ������Ԫ���ʺ�������
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        // ��������Ԫ
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
        Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch ,ת��Ϊ����
        Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv      
       //Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;                //Yaw,ת��Ϊ����
      // ANO_DT_send_int16(Pitch,Roll,0,0,0);

}

float Pitch_Get(void)
{
  return Pitch;
}
/*u8 ti_me = 0;
u8 clik_number=0;*/
/*��̬����*/
void ahrs_update()
{
      /*  if(ti_me == 0)
        {
          LPTMR_time_start_us(); 
          ti_me = 1;
        }
	
        */
	ins.update();///һ�ָ��¾�������
	//delay_ms(5);                
	// if the update call took more than 0.2 seconds then discard it,
    // otherwise we may move too far. This happens when arming motors 
    // in ArduCopter
	
/*********5.18��  ����ΪʲôҪ��********
 *********5.19 Ҫ�ģ��ñ�ļ�������*****
**********5.20  �ģ���Ϊ����Ҫ��dt�ˣ���û��Ҫ������***/
         //float dt = 0;
	/*float dt = (LPTMR_time_get_us())/1000;
	if (dt > 40)                                              //��̬�����������6ms�����ڵ������޸�
	{
          clik_number++;
          if(clik_number>6)
          {
             ;
          }
          //return;
        }
        LPTMR_time_close();
        LPTMR_Pulse_Clean();
        LPTMR_time_start_us();             //��������
            
       
    if(dt>100)
    {
      return ;
    }
 */
   // quat update
    //AHRS_quat_update(_gyro_vector, _acc_vector, dt);
   
    
    // quat to angle
    //gai_AHRS_quat_to_angle(_gyro_vector.x, _gyro_vector.y, _gyro_vector.z, _acc_vector.x, _acc_vector.y, _acc_vector.z);
    

}

AHRS_t AHRS =
{
	ahrs_update,

};
void Test_ahrs_init(void)
{
  ins.init();          //��ʼ��������
}
void Test_ahrs(void)    //������̬����   APM����ֲ�����ģ����ܳ�һ�㲻��Ҫ����ȫ������̬
{
    //uint16_t count = 0;
    //uint32_t current_time;   
    //uint64_t last_time;
    
    

    
    
        //current_time = systime.get_time_us() - last_time;   //��ǰʱ���ȥ��һ�����ڵĿ�ʼʱ��
         //delay_ms(5);
         AHRS.update();
         
  //          if(count++ % 200 == 0)                        ///5.17�Źرղ�֪��Ϊʲô�ۼӵ�200
  //          printf("X  %5.2f   Y  %5.2f   Z   %5.2f  \n",ahrs_angle.x, ahrs_angle.y, ahrs_angle.z);
            
 /***************���������5ms��ʱ����*******************************/           
/*if(current_time > 5000)  // һ������5ms
        {
           // last_time = systime.get_time_us();     //��ס��ʼʱ��
            AHRS.update();
            if(count++ % 200 == 0)
            printf("X  %5.2f   Y  %5.2f   Z   %5.2f  \n",ahrs_angle.x, ahrs_angle.y, ahrs_angle.z);
        }
       */
    
}

