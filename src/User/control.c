#include "control.h"
#include "include.h"
/**********************宏定义**********************/

#define IMAGEH  120//行 HEIGHT 待采集摄像头图像高度行数
#define IMAGEW  188 //列 WIDTH  待采集摄像头图像宽度列数

#define LCDH    60  //OLED显示的行数
#define LCDW    94  //OLED显示的列数

#define DIANJI_PWM_max  3000    // 电机PWM的最大值
#define DIANJI_PWM_min  -3000    // 电机PWM的最小值



#define ZHONGZHI 3
/**********************全局变量定义**********************/

float Angle_Balance,Gyro_Turn,Acceleration_Z,Gyro_Balance;
float angle, angle_dot; 	
float Q_angle=0.001;// 1y3ì??éùμ?D-·?2?
float Q_gyro=0.003;//0.003 1y3ì??éùμ?D-·?2? 1y3ì??éùμ?D-·?2??aò???ò?DDá?áD???ó
float R_angle=0.5;// 2aá???éùμ?D-·?2? ?è2aá???2?
float dt=0.005;//                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
int Balance_Pwm,Balance_Pwm_L,Balance_Pwm_R,Balance_Pwm_L1,Balance_Pwm_L2,Balance_Pwm_R1,Balance_Pwm_R2;

int ceshidianji1=0;
int ceshidianji2=0;
int ceshidianji3=0;
int ceshihuan=0;
int ceshizhongxian=0;

int count=0;
int time;
int bnamaxian_time=0;
uint32 sum=0;
int num1=0;
int num3=0;
int x1=20;
int x2=40;
int y1=20;
int y2=40;
int y3=1;
int yuzhi_sign=0;


//各种标志
int bmx_stop=1;
int chusaidao_stop=1;
int dajingfa=1;

//图像参数

u16 Pixle1[LCDH][LCDW];              //二值化后用于OLED显示的数据
u8 Image_Data[IMAGEH][IMAGEW];      //图像原始数据存放
int W=94;                         //图像宽度
int H=60;                         //图像高度
uint8 image_new_bin[60][94];      //二值化图像数组
uint8 image_new_gray_bin[60][94];      //二值化图像数组
uint8 num;
uint8 x;
uint8 y;


//大津法参数
uint8 yuzhi[20][94];
uint8 THRESHOLD=55;
uint8 THRESHOLD_old=55;

//阈值可调参数
int yuzhi_max=70;
int yuzhi_min=40;


//中线参数
int slope_array[60]={40};       //中线偏差
int mid[60];                    //每一行的中线坐标数组
int right[60];                  //右边界
int left[60];                   //左边界
int lost_right[60];             //左丢线标志数组 1为丢线0为不丢
int lost_left[60];              //右丢线标志数组 1为丢线0为不丢
int youxiaohang[60];            //有效行标志数组 1为有效0为无效
int right_diu_budiu[3]={-1,-1,-1};
int left_diu_budiu[3]={-1,-1,-1};
int right_budiu_diu[3]={-1,-1,-1};
int left_budiu_diu[3]={-1,-1,-1};
float wild_real[60];            //实际赛道宽度

int mid_line_begin=0;           //最远的一行有效行
int begin=47;                   //第一次找左右边线的起点

int right_error_sun=0;          //右边线的误差和
uint8 R_num=0;                  //有效右边线的数量
int right_error_aver=0;          //有效右边线的误差的平均值

int left_error_sun=0;          //左边线的误差和
uint8 L_num=0;                  //有效左边线的数量
int left_error_aver=0;          //有效左边线的误差的平均值

//二乘法参数
float b=0;                    //二乘法斜率
float k=0;                    //二乘法常数
int y_co;                     //计算的的y坐标



//环标志位
uint8 ring_R_sign=0;            //右环标志位 1环外 2环外 3开始进环 4环内 5环内准备出环补线 6环外 0不在环中
uint8 ring_L_sign=0;            //左环标志位 1环外 2环外 3开始进环 4环内 5环内准备出环补线 6环外 0不在环中
uint8 chuhuan_sign=0;           //出环补线的标志位

int car_journey_PL_ring;

int ring_3_sign=0;

//环可调参数
int pianyi_R=8;
int pianyi_L=2;

int chuhuan_dajiao_R=4980;
int chuhuan_dajiao_L=3430;

float jinghuan_buchang_R=1.175;
float jinghuan_buchang_L=0;


//编码器
int    qd_result_L;                                  //编码器计数（向前正，向后负）左轮
int    qd_result_R;                                  //编码器计数（向前正，向后负）右轮
double  car_journey_PL=0;                          //小车行过的路程(脉冲个数)
int journey_1m=3310;
int begin_time=0;
int run_time=15*100;                                //2s
int end_time=0;
double speed_all;
int straight_away=0;


//舵机PID定义
float PID_P=900;               //学长600-900
float P_max=500;
float P_min=250;
float PID_D=650;               //学长9000-12000           速度大D小
float steer_pid_D_max=500;
float steer_pid_D_min=250;
float steer_error_temp;        //用于速度控制D

float P_max_ring=1100;
float P_min_ring=400;
float steer_pid_D_max_ring=700;
float steer_pid_D_min_ring=300;



float P_max_nor=700;
float P_min_nor=300;
float steer_pid_D_max_nor=540;
float steer_pid_D_min_nor=400;

//PWM限幅         PWM矫正
int steer_pwm_mid=12100;     //一号车
int steer_pwm_max=14850;
int steer_pwm_min=10050;

int yxh=22;                    //误差计算行
int yxh_nor=22;
int yxh_ring=30;

float temp_wucha_sum;          //误差和
float temp_error=0;           //中线误差归一化后的值
float temp_last_error=0;      //上一次的误差
float temp_error_error=0;     //误差的误差
float duty;                    //舵机PWM占空比

//电机PID控制，速度定义
float motor_pid_P=85;
float motor_pid_I_max=5;
float motor_pid_I_min=3.5;
float motor_pid_I=6.5;
float motor_pid_D=20;

float rel_speed=0;                                //实时速度
float rel_speed_L=0;                               //左轮实时速度
float rel_speed_R=0;                              //右轮实时速度
float set_speed=0;                                //设定速度
float speed_error=0;                              //速度误差
float speed_sprint=90;                             //起跑冲刺速度
float speed_ramp=66;                               //坡道定速
float speed_shizi=66;                              //十字道定速
float speed_up=10;                                 //长直道加速
int speed_ring_R=76;                                //环道速度
int speed_ring_L=76;

float speed_min=76;                                //设定的最小速度
float speed_max=116;                              //设定的最大速度
      //马达占空比
float motor_pwm_out=0;                            //占空比（输出）
float motor_pwm_out1=10;                           //占空比1线
float motor_pwm_out2=0;                           //占空比2线
float motor_pwm_max;                             //最大占空比限定
float motor_pwm_min;                             //最大占空比限定

float set_go_long=0;                              //起跑全速冲刺的距离设定

int stop_sign=1;                                //停车标志
int stop_num=0;

int break_flag=0;
int break_set=10;
int break_pwm_max=3000;
int break_pwm_min=1000;





float wild[60]={27.0,28.0,29.3,31.0,32.0,33.0,34.0,35.0,36.0,37.0,
                38.0,39.0,40.0,41.0,42.0,44.0,45.0,46.0,47.0,48.0,
                50.0,51.0,52.0,53.0,53.5,54.0,55.0,56.5,58.0,59.0,
                60.0,61.0,62.0,63.0,64.0,65.0,66.0,67.0,68.0,69.0,
                70.0,70.3,70.5,71.0,71.5,72.0,73.0,74.0,75.0,75.5,
                76.0,76.5,77.0,77.5,78.0,79.0,80.0,80.5,81.0,81.5};
/********************************************舵机PID控制函数********************************************/
void duoji_PID (void)      
{
  float left_error;                                //左边线与画面中心的误差
  float right_error;                               //右边线与画面中心的误差
  temp_wucha_sum=0;

  /*****动态有效行******/
  yxh_nor=(int)(-0.2*rel_speed+42);            //动态的有效行，速度与快有效行越远  36.3


  /****PID参数注入*******/
  if(ring_R_sign>1||ring_L_sign>1)                 //环PID的参数单独注入
  {
    P_max=P_max_ring;
    P_min=P_min_ring;
    steer_pid_D_max=steer_pid_D_max_ring;
    steer_pid_D_min=steer_pid_D_min_ring;
    yxh=yxh_ring;
  }
  else                                             //一般PID参数的注入
  {
    P_max=P_max_nor;
    P_min=P_min_nor;
    steer_pid_D_max=steer_pid_D_max_nor;
    steer_pid_D_min=steer_pid_D_min_nor;
    yxh=yxh_nor;
  }


  /*********误差计算*********/
  left_error=47-left[yxh];
  right_error=right[yxh]-47;

  temp_last_error = temp_error;                                                       //记录上次误差
  temp_error = ((right_error-left_error)/(left_error+right_error));                   //差除和

  if(ring_R_sign==1)                                                   //单边巡线（左）  8
    temp_error = (wild[yxh]/2+pianyi_R-left_error)/(wild[yxh]/2+pianyi_R+left_error);
  if(ring_L_sign==1)                                                   //单边巡线（右）
    temp_error = ((right_error-wild[yxh]/2-pianyi_L)/(wild[yxh]/2+pianyi_L+right_error));
  if(ring_R_sign==6)                                                   //单边巡线（左）
    temp_error = (wild[yxh]/2-left_error)/(wild[yxh]/2+left_error);
  if(ring_L_sign==6)                                                   //单边巡线（右）
    temp_error = ((right_error-wild[yxh]/2)/(wild[yxh]/2+right_error));

  temp_error_error=temp_error-temp_last_error;                                          //误差的误差

//  if(temp_error>1)                              //误差限幅  后来发现误差大于1是正常的
//     temp_error=1;
//  if(temp_error<-1)
//     temp_error=-1;


  /*-------P动态-----------*/
  PID_P=(P_max-P_min)*fabs(temp_error)+P_min;

  /*-------D动态-----------*/
    //  if(fabs(speed_max-speed_min)<0.5)steer_pid.D=steer_pid.D_max;                                                                          //匀速D取最大
    //  else steer_pid.D=steer_pid.D_max-((rel_speed-speed_min)/(speed_max-speed_min)*(steer_pid.D_max-steer_pid.D_min));                      //动态D

  steer_error_temp=(rel_speed-speed_min)/(speed_max-speed_min);   //速度偏差计算

  if(steer_error_temp<0)steer_error_temp=0;       //限幅
  if(steer_error_temp>1)steer_error_temp=1;

  PID_D=steer_pid_D_max-(steer_error_temp*(steer_pid_D_max-steer_pid_D_min));     //D关联速度                                              //更新动态D算法,和速度关联

  if(PID_D>steer_pid_D_max) PID_D=steer_pid_D_max;    //限幅值                                                                        //D极限限定
  if(PID_D<steer_pid_D_min) PID_D=steer_pid_D_min;                                                                            //D极限限定


/*-----------------------*/
  duty =steer_pwm_mid+(temp_error)*PID_P+PID_D*temp_error_error*10;            //PID控制，占空比计算

/*********出环动态打脚**************/
  if(ring_R_sign==5)                                            //出右环补线，路程决定打脚
  {
    if((lost_left[20])==1)                                      //左丢线出环
     if(chuhuan_sign==0)
     {
       chuhuan_sign=1;
       time=count;
     }
    if(chuhuan_sign)
      duty=chuhuan_dajiao_R-0.03*(car_journey_PL-car_journey_PL_ring);                //4950              //小4785  4686
    if(duty>4730)                                              //限幅度 待验证
      duty=4730;
  }


  if(ring_L_sign==5)                                           //出左环补线，路程决定打脚
  {
    if((lost_right[20])==1)                                     //右丢线，出环
     if(chuhuan_sign==0)
     {
       chuhuan_sign=1;
       time=count;
     }
    if(chuhuan_sign)
      duty=chuhuan_dajiao_L+0.02*(car_journey_PL-car_journey_PL_ring);
    if(duty<3500)                                             //限幅 待验证
      duty=3500;
  }
}
///**************PWM限幅***************/
//
//  if (duty>steer_pwm_max)  duty = steer_pwm_max;  //上限   右转                      舵机占空比限幅
//  if (duty<steer_pwm_min)  duty = steer_pwm_min;  //下限
//  PLL_Init(PLL200);         //初始化PLL为200M，总线为40MHZ
//  FTM_PWM_Duty(FTM3,FTM_CH6,(int)duty);//占空比设置           //输出舵机的占空比
//
//}


/********************************************小车速度控制********************************************/
//
////增量式PID算法(控制电机)
//void motor_PID_control(void)
//{
//    float pwm_temp=0.0;           //pwm计算值
//
//    static float speed_last_error=0;    //上次偏差
//    static float speed_pre_error=0;     //再上一次偏差
//
//    float P_out=0;                  //
//    float I_out=0;                  //
//    float D_out=0;                  //
//
//    if(stop_sign==1)
//    {
//      set_speed=0;
//    }
//
//    speed_error=(set_speed)-rel_speed;      //速度误差
///*---------------------------------*/
////      motor_pid_I=(motor_pid_I_max-motor_pid_I_min)*(fabs(speed_error)/40)+motor_pid_I_min;  //动态I  fabs为求绝对值
////      if(motor_pid_I>motor_pid_I_max)motor_pid_I=motor_pid_I_max;                            //44
////      if(motor_pid_I<motor_pid_I_min)motor_pid_I=motor_pid_I_min;                            //49
//
//    P_out=motor_pid_P*(speed_error-speed_last_error);
//    I_out=motor_pid_I*(speed_error);
//    D_out=motor_pid_D*(speed_error+speed_pre_error-2*speed_last_error);
//
//
//    pwm_temp=P_out+D_out+I_out;
//    motor_pwm_out= motor_pwm_out+pwm_temp;
//      
//    speed_pre_error  = speed_last_error;
//    speed_last_error = speed_error;
//
//
/////*---------------------------------*/
/////*--------停车要刹车---------------*/
//
/////*---------------------------------*/
/////*---------------------------------*/
////
////    //if((rel_speed<2)&&(motor_pwm_out>7000)&&(time_start==1)&&(car_journey>1.0)&&(motor_stop==0))motor_pwm_out=0,motor_stop=1,fmq_count=800;            //防止编码器脱出
////    if(((set_speed>500)||(set_speed<-10))&&(start_flag==1))motor_pwm_out=0,motor_stop=1,fmq_count=800;           //防止超速（存储芯片抽风）
/////*---------------------------------*/
//    if(motor_pwm_out<0)
//      motor_pwm_out=0;             //占空比限定
//    if(motor_pwm_out>DIANJI_PWM_max)
//      motor_pwm_out=DIANJI_PWM_max;             //占空比限定
//
////   ftm_pwm_init(ftm1,ftm_ch1,10000,motor_pwm_out);
//    ceshidianji2++;
//
//}


//刹车算法 函数

void car_break_function(void)
{
    if((stop_sign==0)&&(car_journey_PL>set_go_long))
    {
      //if((((fabs(steer_pwm_mid-steer_pid.pidout))/((steer_pwm_max-steer_pwm_min)/2))>0.2)||(shizi_flag!=0)||(ramp_flag!=0))           //舵机打过一定角度
      {
        if(((rel_speed-set_speed)>break_set))                                          //大于刹车门限，并且实际速度大于需要刹车速度
        {
          //if(fabs(steer_error)>0.2)                                                                     //差值一定大
          {
            break_flag=1;
          }
        }
      }
    }

    if(stop_sign==1)
    {
      motor_pwm_out=-2000;
      if((rel_speed-set_speed)<(5))                                                                     //至少刹车到设定速度或以下
      {
        break_flag=0;
        motor_pwm_out=0;                                                                                //清刹车标记调试电机输出给0，以免从负的加需要一定时间
      }
    }



    if(break_flag!=0)
    {
      motor_pwm_out=((((rel_speed-set_speed)/30)*(break_pwm_max-break_pwm_min))+break_pwm_min);         //动态刹车

      if(motor_pwm_out<break_pwm_min)
        motor_pwm_out=break_pwm_min;
      if(motor_pwm_out>break_pwm_max)
        motor_pwm_out=break_pwm_max;
      
      motor_pwm_out=-motor_pwm_out;                                                                     //别忘了刹车是负的

      if((rel_speed-set_speed)<(8))                                                                     //至少刹车到设定速度或以下
      {
        break_flag=0;
        motor_pwm_out=0;                                                                                //清刹车标记调试电机输出给0，以免从负的加需要一定时间
      }

    }
}


//设定小车固定速度

void car_speed_set(void)
{

        if(fabs(speed_max-speed_min)<1)
          set_speed=speed_min;                     //匀速设定
        else
        {
          if(duty>steer_pwm_mid)
          {
            set_speed=speed_max-(pow(((duty-steer_pwm_mid)/(steer_pwm_max-steer_pwm_mid)),2)*(speed_max-speed_min));

          }
          else if(duty<steer_pwm_mid)
          {
            set_speed=speed_max-(pow(((steer_pwm_mid-duty)/(steer_pwm_mid-steer_pwm_min)),2)*(speed_max-speed_min));
          }
          else 
            set_speed=speed_max;
        }
        //if((fabs(steer_pid.pidout-steer_pwm_mid)/((steer_pwm_max-steer_pwm_min)/2))<0.05)set_speed=speed_max;
/*-------------------------------------------------*/
/*-------速度限定----------------------------------*/
        if(set_speed<speed_min)
          set_speed=speed_min;                       //最小速度限定
        if(set_speed>speed_max)
          set_speed=speed_max;                       //最大速度限定
///*-------------------------------------------------*/
///*-------直道判断（加速）--------------------------*/
     if(temp_error<0.1)
     {
       ;
     }
     else
      straight_away=(int)car_journey_PL;

     if((car_journey_PL-straight_away)>=1500)
       set_speed=set_speed+speed_up;      //走过0.3米的直道，算是直道
///*-------------------------------------------------*/
///*---十字道定速 相当于提前减速 比最高速度小一点----*/
//      if(shizi_flag!=0)set_speed=set_speed-speed_shizi;
///*-------------------------------------------------*/
///*-------坡道处理（减速到坡道道设定值）--------*/
//      if(ramp_flag!=0)set_speed=speed_ramp;
///*-------------------------------------------------*/
///*-------直角道判断（减速到直角道设定值）----------*/
 //       if(vertical_flag!=0)set_speed=speed_vertical;                          //直角降速再降速

/***********************环**************************/

      if(ring_R_sign==2||ring_R_sign==3||ring_R_sign==4||ring_R_sign==5||ring_R_sign==6)
        {
        set_speed=speed_ring_R;
        }


       if(ring_L_sign==2||ring_L_sign==3||ring_L_sign==4||ring_L_sign==5||ring_L_sign==6)
        {
        set_speed=speed_ring_L;
        }

}


//电机PWM更新传递

void motor_PWN_updat(void)
{
// /* ceshidianji1++;
//    if(motor_pwm_out<DIANJI_PWM_min)motor_pwm_out=DIANJI_PWM_min;              //占空比限定 -3000
//    if(motor_pwm_out>DIANJI_PWM_max)motor_pwm_out=DIANJI_PWM_max;     */        //占空比限定 3000
///*---------------------------------*/
//    //确定电机的转动方向
// /*   if(motor_pwm_out<0)
//    {
//      motor_pwm_out1=0;
//      motor_pwm_out2=-motor_pwm_out;
//    }
//    else
//    {
//      motor_pwm_out1=motor_pwm_out;
//      motor_pwm_out2=0;
//    }*/
///*---------------------------------*/
    //电机PWM赋值

      FTM_PWM_Duty(FTM0,FTM_CH0,Balance_Pwm_R1);//控制电机转动，CH1不为零，CH2为零，正转  CH1右
      FTM_PWM_Duty(FTM0,FTM_CH1,Balance_Pwm_R2);//控制电机转动，CH2不为零，CH1为零，反转
      FTM_PWM_Duty(FTM0,FTM_CH2,Balance_Pwm_L1);//duty=1000
      FTM_PWM_Duty(FTM0,FTM_CH3,Balance_Pwm_L2);//   

      

}
//停车判断

void tingche()
{


      /*停车*/                             //待验证
  if(chusaidao_stop)
  {
      if((image_new_bin[55][10]+image_new_bin[55][20]+image_new_bin[55][30]+image_new_bin[55][40]+image_new_bin[55][50]+
        image_new_bin[40][10]+image_new_bin[40][20]+image_new_bin[40][30]+image_new_bin[40][40]+image_new_bin[40][50])==0)
    {
        stop_num++;         //图像近处全黑停车
        if(stop_num>10)
        {
          stop_sign=1;
          stop_num=0;
        }
    }
    else
        stop_num=0;
  }

       /*斑马线*/
    if(bmx_stop)
    {
        if(count>begin_time+500)
     {
           int num=0;
             for(int j=15;j<80;j++)
                if(image_new_bin[50][j]==0xFF&& image_new_bin[50][j+1]==0x00)
                  num++;
             if(num>4)
           {
        stop_sign=1;
       }
     }
    }

}



/**************************************************************************
卡尔曼绿博
**************************************************************************/
void Kalman_Filter(float Accel,float Gyro)		
{
	angle+=(Gyro - Q_bias) * dt; //?è?é1à??
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-?è?é1à???ó2?D-·?2?μ??￠·?

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-?è?é1à???ó2?D-·?2??￠·?μ??y·?
	PP[0][1] += Pdot[1] * dt;   // =?è?é1à???ó2?D-·?2?
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-?è?é1à??
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //oó?é1à???ó2?D-·?2?
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //oó?é1à??
	Q_bias	+= K_1 * Angle_err;	 //oó?é1à??
	angle_dot   = Gyro - Q_bias;	 //ê?3??μ(oó?é1à??)μ??￠·?=???ù?è
        
        Angle_Balance=angle;                                   //?üD???oa????
	Gyro_Turn=GYRO_Z;                                      //?üD?×a?ò???ù?è
	Acceleration_Z=ACC_Z; 
}

/**************************************************************************
确定赋值给PWM的值
**************************************************************************/
void Set_Pwm(int Balance_Pwm_L_d,int Balance_Pwm_R_d)
{
    	if(Balance_Pwm_L<0)	Balance_Pwm_L1=Balance_Pwm_L_d,	        Balance_Pwm_L2=0;
	else 	                Balance_Pwm_L1=0,			Balance_Pwm_L2=Balance_Pwm_L_d;
	if(Balance_Pwm_R<0)	Balance_Pwm_R1=Balance_Pwm_R_d,		Balance_Pwm_R2=0;
	else                    Balance_Pwm_R1=0,		        Balance_Pwm_R2=Balance_Pwm_R_d;

}

/**************************************************************************
限幅
**************************************************************************/
void Xianfu_Pwm(void)
{	
	int Amplitude=1000;    //===PWM?ú·ùê?7200 ?T???ú6900
        if(Balance_Pwm_L>Amplitude) Balance_Pwm_L=Amplitude;	
	if(Balance_Pwm_R>Amplitude) Balance_Pwm_R=Amplitude;	
	if(Balance_Pwm_R<-Amplitude) Balance_Pwm_R=-Amplitude;
        if(Balance_Pwm_L<-Amplitude) Balance_Pwm_L=-Amplitude;
}


/**************************************************************************
总控制			 
**************************************************************************/
void balance_control(void) 
{    
              //qd_result_L =FTM_AB_Get(FTM1);     //////读编码器
             // qd_result_R =-FTM_AB_Get(FTM2); 
  	  	Update9AX();          //////读取姿态   须改
                
                Gyro_Balance=-GYRO_Y;

                Kalman_Filter(ACC_Y,-GYRO_Y);
		
 		Balance_Pwm = balance(Angle_Balance,Gyro_Balance);                  /////平衡PID控制   须改
//		  Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);                /////速度PID控制   须改
//                Turn_Pwm=turn(Encoder_Left,Encoder_Right,Gyro_Turn);                /////位置PID控制   须改
 		 // Moto1=Balance_Pwm-Velocity_Pwm+Turn_Pwm;                          
 	  	//Moto2=Balance_Pwm-Velocity_Pwm-Turn_Pwm;                            /////计算左右pwm  须改
   		Balance_Pwm_L = Balance_Pwm_R =  Balance_Pwm;                                                
		Xianfu_Pwm();
                Set_Pwm(Balance_Pwm_L,Balance_Pwm_R);
// 		motor_PWN_updat();                                               /////设定pwm值    须改  
} 

/**************************************************************************
直立PD
角度、角速度
**************************************************************************/
int balance(float Angle,float Gyro)
{  
         float Bias,kp=-300,kd=1;
	 int balance_1;
	 Bias=Angle-ZHONGZHI;       /////计算
	 balance_1=(int)(kp*Bias+Gyro*kd);   
	 return balance_1;
}

/**************************************************************************
速度PI
左右编码器
**************************************************************************/
/*int velocity(int qd_result_L,int qd_result_R)
{  
     static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Encoder_Integral,Target_Velocity;
	  float kp=80,ki=0.4;
	
		Encoder_Least =(qd_result_L+qd_result_R)-0;                 
		Encoder *= 0.8;		                                           
		Encoder += Encoder_Least*0.2;	                                   
		Encoder_Integral +=Encoder;                                       
		Encoder_Integral=Encoder_Integral-Movement;                       
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             
		if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              	
		Velocity=Encoder*kp+Encoder_Integral*ki;                          
	////	if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   Encoder_Integral=0;      关闭电机后删除积分
	  return Velocity;
}
*/
/**************************************************************************
转向控制  修改转向速度，修改Turn_Amplitude
左右编码器   z陀螺仪
**************************************************************************/
/*int turn(int encoder_left,int encoder_right,float gyro)///////转向控制
{
	 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude=88/Flag_sudu,Kp=42,Kd=0;     
	  //============遥控左右旋转=======================//
  	if(1==Flag_Left||1==Flag_Right)                      //?aò?2?·??÷òaê??ù?YDy×a?°μ??ù?èμ÷???ù?èμ??eê??ù?è￡????óD?3μμ?êêó|D?
		{
			if(++Turn_Count==1)
			Encoder_temp=myabs(encoder_left+encoder_right);
			Turn_Convert=50/Encoder_temp;
			if(Turn_Convert<0.6)Turn_Convert=0.6;
			if(Turn_Convert>3)Turn_Convert=3;
		}	
	  else
		{
			Turn_Convert=0.9;
			Turn_Count=0;
			Encoder_temp=0;
		}			
		if(1==Flag_Left)	           Turn_Target-=Turn_Convert;
		else if(1==Flag_Right)	     Turn_Target+=Turn_Convert; 
		else Turn_Target=0;
	
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===×a?ò?ù?è?T·ù
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5;        
		else Kd=0;   //×a?òμ?ê±oòè???íó?Yò?μ??à?y óDμ??￡oyPIDμ?????
  	//=============转向PD======================//
		Turn=-Turn_Target*Kp -gyro*Kd;                 //结合z轴陀螺仪进行PD控制
	  return Turn;
}
*/

/**************************************************************************
oˉêy1|?ü￡oòì3￡1?±?μ??ú
è??ú2?êy￡o????oíμ??1
·μ??  ?μ￡o1￡oòì3￡  0￡o?y3￡
**************************************************************************/
/*u8 Turn_Off(float angle, int voltage)
{
	    u8 temp;
			if(angle<-40||angle>40||1==Flag_Stop||voltage<1110)//μ?3?μ??1μíóú11.1V1?±?μ??ú
			{	                                                 //===????′óóú40?è1?±?μ??ú
				temp=1;                                            //===Flag_Stop??11?±?μ??ú
				AIN1=0;                                            
				AIN2=0;
				BIN1=0;
				BIN2=0;
			}
			else
				temp=0;
      return temp;			
}
*/	
/**************************************************************************
oˉêy1|?ü￡o??è????è èy????·¨?-1y?ò??μ?μ÷D￡￡???·?3￡àí?? 
è??ú2?êy￡o??è????èμ???·¨ 1￡oDMP  2￡o?¨???ü 3￡o?￥21??2¨
·μ??  ?μ￡o?T
**************************************************************************/
/*void Get_Angle(u8 way)
{ 
	    float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_Z;
	   	Temperature=Read_Temperature();      //===?áè?MPU6050?ú?????è′??D?÷êy?Y￡??ü??±íê??÷°????è?￡
	    if(way==1)                           //===DMPμ??áè??úêy?Y2é?ˉ?D??ìáD?μ?ê±oò￡?????×??-ê±Dòòa?ó
			{	
					Read_DMP();                      //===?áè??ó?ù?è?￠???ù?è?￠????
					Angle_Balance=Pitch;             //===?üD???oa????
					Gyro_Balance=gyro[1];            //===?üD???oa???ù?è
					Gyro_Turn=gyro[2];               //===?üD?×a?ò???ù?è
				  Acceleration_Z=accel[2];         //===?üD?Z?á?ó?ù?è??
			}			
      else
      {
			Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //?áè?Y?áíó?Yò?
			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //?áè?Z?áíó?Yò?
		  Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //?áè?X?á?ó?ù?è??
	  	Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //?áè?Z?á?ó?ù?è??
		  if(Gyro_Y>32768)  Gyro_Y-=65536;                       //êy?YààDí×a??  ò2?éí¨1yshort????ààDí×a??
			if(Gyro_Z>32768)  Gyro_Z-=65536;                       //êy?YààDí×a??
	  	if(Accel_X>32768) Accel_X-=65536;                      //êy?YààDí×a??
		  if(Accel_Z>32768) Accel_Z-=65536;                      //êy?YààDí×a??
			Gyro_Balance=-Gyro_Y;                                  //?üD???oa???ù?è
	   	Accel_Y=atan2(Accel_X,Accel_Z)*180/PI;                 //????????	
		  Gyro_Y=Gyro_Y/16.4;                                    //íó?Yò?á?3ì×a??	
      if(Way_Angle==2)		  	Kalman_Filter(Accel_Y,-Gyro_Y);//?¨???ü??2¨	
			else if(Way_Angle==3)   Yijielvbo(Accel_Y,-Gyro_Y);    //?￥21??2¨
	    Angle_Balance=angle;                                   //?üD???oa????
			Gyro_Turn=Gyro_Z;                                      //?üD?×a?ò???ù?è
			Acceleration_Z=Accel_Z;                                //===?üD?Z?á?ó?ù?è??	
		}
}*/
/**************************************************************************
oˉêy1|?ü￡o?????μoˉêy
è??ú2?êy￡oint
·μ??  ?μ￡ounsigned int
**************************************************************************/
/*int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}*/