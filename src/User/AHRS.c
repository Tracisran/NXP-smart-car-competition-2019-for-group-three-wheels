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
/*static Q4_t Q4 = {1, 0, 0, 0};  //四元数
const float ahrs_kp = 1.08f; //PI控制器，修正机体坐标系         一开始const float ahrs_kp = 1.08f;
const float ahrs_ki = 0.05f;   //////一开始const float ahrs_ki = 0.05f;
static vector3f_t integral;  //机体坐标误差积分
static vector3f_t ahrs_angle;  //机体坐标误差积分
*******************上面是之前的四元素更新需要的**************/

/******************我改的姿态更新需要的参数*************/
#define Kp 125.0f                        // 比例增益支配率收敛到加速度计/磁强计

#define Ki 0.000f                // 积分增益支配率的陀螺仪偏见的衔接

#define Kd    1.0f

#define halfT 0.00714f  



float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 四元数的元素，代表估计方向
float exInt = 0, eyInt = 0, ezInt = 0;        // 按比例缩小积分误差
float  exDif=0,eyDif=0,ezDif=0;

float Yaw,Pitch,Roll;  //偏航角，俯仰角，翻滚角

/****函数  AHRS_quat_update
	*作用  更新四元数
	*参数
	*返回值         之前的5.20关闭
	***/

//
//void AHRS_quat_update(vector3f_t gyro, vector3f_t acc, float interval)   /////intervval 间距
//{
//	float q0 = Q4.q0;
//	float q1 = Q4.q1;
//	float q2 = Q4.q2;
//	float q3 = Q4.q3;
///***********  模长  ************/	
//	float norm = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
///***********  加计测出的机体坐标系   **********/
//	float ax = acc.x * norm;
//	float ay = acc.y * norm;
//	float az = acc.z * norm;
///***********  四元数解算出的机体坐标系  ***************/
//	float half_vx = q1*q3 - q0*q2;
//	float half_vy = q2*q3 + q0*q1;
//	float half_vz = -0.5f + q0*q0 + q3*q3;
///***********  叉积求加计机体坐标和上次四元数解算机体坐标差  ************/	
//	float half_ex = ay*half_vz - az*half_vy;
//	float half_ey = az*half_vx - ax*half_vz;
//	float half_ez = ax*half_vy - ay*half_vx;
///***********  使用PI控制器 修正机体坐标 *************/	
//	integral.x += half_ex * ahrs_ki * interval;
//	integral.y += half_ey * ahrs_ki * interval;
//	integral.z += half_ez * ahrs_ki * interval;
//	
//	float gx = (gyro.x + ahrs_kp * half_ex + integral.x) * 0.5f * interval;
//	float gy = (gyro.y + ahrs_kp * half_ey + integral.y) * 0.5f * interval;
//	float gz = (gyro.z + ahrs_kp * half_ez + integral.z) * 0.5f * interval;
//	
///**********  更新四元数  ********/
//	Q4.q0 += (-q1 * gx - q2 * gy - q3 * gz); 
//	Q4.q1 += ( q0 * gx + q2 * gz - q3 * gy); 
//	Q4.q2 += ( q0 * gy - q1 * gz + q3 * gx); 
//	Q4.q3 += ( q0 * gz + q1 * gy - q2 * gx); 
//  //单位化四元数 	
//	norm = invSqrt(Q4.q0 * Q4.q0 + Q4.q1 * Q4.q1 + Q4.q2 * Q4.q2 + Q4.q3 * Q4.q3);
//	
//	Q4.q0 *= norm;
//	Q4.q1 *= norm;
//	Q4.q2 *= norm;
//	Q4.q3 *= norm;
//}
//


/****函数  
	*作用  更新姿态角   5.20网上抄的
	*参数
	*返回值
	***/


void gai_AHRS_quat_to_angle(float gx, float gy, float gz, float ax, float ay, float az)
{
//四元素求欧拉角

        float norm;
        float vx, vy, vz;
        float ex, ey, ez;  
        static float last_ex,last_ey,last_ez;
        
        // 测量正常化
        norm = sqrt(ax*ax + ay*ay + az*az);   
        ax = ax / norm;                   //单位化
        ay = ay / norm;
        az = az / norm;     
        // 估计方向的重力
        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        // 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);
        // 积分误差比例积分增益
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;
        
        exDif = ex-last_ex;
    eyDif = ey-last_ey;
    ezDif = ez-last_ez;
    
    last_ex = ex;
    last_ey = ey;
    last_ez = ez;
        // 调整后的陀螺仪测量
        gx = gx + Kp*ex + exInt + Kd*exDif;
        gy = gy + Kp*ey + eyInt + Kd*eyDif;
        gz = gz + Kp*ez + ezInt + Kd*ezDif;
        // 整合四元数率和正常化
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        // 正常化四元
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
        Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch ,转换为度数
        Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv      
       //Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;                //Yaw,转换为度数
      // ANO_DT_send_int16(Pitch,Roll,0,0,0);

}

float Pitch_Get(void)
{
  return Pitch;
}
/*u8 ti_me = 0;
u8 clik_number=0;*/
/*姿态更新*/
void ahrs_update()
{
      /*  if(ti_me == 0)
        {
          LPTMR_time_start_us(); 
          ti_me = 1;
        }
	
        */
	ins.update();///一轮更新九轴数据
	//delay_ms(5);                
	// if the update call took more than 0.2 seconds then discard it,
    // otherwise we may move too far. This happens when arming motors 
    // in ArduCopter
	
/*********5.18改  不懂为什么要加********
 *********5.19 要改，用别的计数来改*****
**********5.20  改，因为不需要用dt了，就没必要加上了***/
         //float dt = 0;
	/*float dt = (LPTMR_time_get_us())/1000;
	if (dt > 40)                                              //姿态解算周期最大6ms，大于的自行修改
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
        LPTMR_time_start_us();             //开启计数
            
       
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
  ins.init();          //初始化传感器
}
void Test_ahrs(void)    //测试姿态解算   APM上移植过来的，智能车一般不需要解算全部的姿态
{
    //uint16_t count = 0;
    //uint32_t current_time;   
    //uint64_t last_time;
    
    

    
    
        //current_time = systime.get_time_us() - last_time;   //当前时间减去上一个周期的开始时间
         //delay_ms(5);
         AHRS.update();
         
  //          if(count++ % 200 == 0)                        ///5.17号关闭不知道为什么累加到200
  //          printf("X  %5.2f   Y  %5.2f   Z   %5.2f  \n",ahrs_angle.x, ahrs_angle.y, ahrs_angle.z);
            
 /***************下面代码用5ms延时代替*******************************/           
/*if(current_time > 5000)  // 一个周期5ms
        {
           // last_time = systime.get_time_us();     //记住开始时间
            AHRS.update();
            if(count++ % 200 == 0)
            printf("X  %5.2f   Y  %5.2f   Z   %5.2f  \n",ahrs_angle.x, ahrs_angle.y, ahrs_angle.z);
        }
       */
    
}

