#include "camaro.h"
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
/********************************************图像采集及二值化********************************************/
void tuxiang_caiji()
{
    //int i,j;
    
    if(V034_Field_Over_Flag)      //图像压缩两行抽一行，存在bug 6.12       20ms/7ms
    {
      V034_Field_Over_Flag=0; 
      for(num=0; num<60; num++)
      {
        for(int a=0;a<94;a++)
        {
          y=num*2;
          x=a*2;
          image_new_gray_bin[num][a]=Image_Data[y][x];
        }
      }
      Image_binaryzation();
      yuzhi_sign=1;
    
    

}
}
/********************************************二值化********************************************/
void Image_binaryzation(void)                       //待优化阈值限幅6.12
{
 uint8 i,j;
 sum=0;
 num1=0;
// #if 1
//  for(j=0;j<94;j++)                                    //抽样
//  {
//    yuzhi[0][j] = image_new_bin[0][j];
//    yuzhi[1][j] = image_new_bin[2][j];
//    yuzhi[2][j] = image_new_bin[5][j];
//    yuzhi[3][j] = image_new_bin[8][j];
//    yuzhi[4][j] = image_new_bin[11][j];
//    yuzhi[5][j] = image_new_bin[14][j];
//    yuzhi[6][j] = image_new_bin[17][j];
//    yuzhi[7][j] = image_new_bin[20][j];
//    yuzhi[8][j] = image_new_bin[23][j];
//    yuzhi[9][j] = image_new_bin[26][j];
//    yuzhi[10][j] = image_new_bin[29][j];
//    yuzhi[11][j] = image_new_bin[32][j];
//    yuzhi[12][j] = image_new_bin[35][j];
//    yuzhi[13][j] = image_new_bin[38][j];
//    yuzhi[14][j] = image_new_bin[41][j];
//    yuzhi[15][j] = image_new_bin[44][j];
//    yuzhi[16][j] = image_new_bin[47][j];
//    yuzhi[17][j] = image_new_bin[50][j];
//    yuzhi[18][j] = image_new_bin[53][j];
//    yuzhi[19][j] = image_new_bin[56][j];
//  }

//if(count%40==0)

//if(fabs(THRESHOLD_old-THRESHOLD)>15)
//  THRESHOLD=THRESHOLD_old;
// if(temp>99 && temp<151)
// THRESHOLD = temp;


  if(THRESHOLD>yuzhi_max)
    THRESHOLD=yuzhi_max;
  if(THRESHOLD<yuzhi_min)
    THRESHOLD=yuzhi_min;
// #endif


#if 1
  //THRESHOLD=48;                                                  //调试用
//  uint8 *p_Image;
//  uint8 *q_Image;
//  q_Image=&image_new_bin[0][0];
//
//  for(p_Image=&image_new_bin[0][0];p_Image<=&image_new_bin[H-1][W-1];p_Image++)  //将采回的数据缓存给p_Image数组
//  {
//    if((*p_Image<THRESHOLD)&&(*(p_Image+1)<THRESHOLD)||(*p_Image<THRESHOLD)&&(*(p_Image-1)<THRESHOLD))   //此处进行了一次滤波
//      *q_Image++ =0;
//    else
//      *q_Image++ =255; //若采回的数据与后一个点或者上一个点均小于所设定的阈值，则认定该点为黑点否则为白点
//  }


  for(int i=0;i<60;i++)
    for(int j=0;j<94;j++)
    {
     if((image_new_gray_bin[i][j]<THRESHOLD&&image_new_gray_bin[i][j+1]<THRESHOLD)||(image_new_gray_bin[i][j]<THRESHOLD&&image_new_gray_bin[i][j-1]<THRESHOLD))
       image_new_bin[i][j]=0;
     else
       image_new_bin[i][j]=255;

    }
#endif

  /*******去噪点*******/

  for(i=1;i<H;i++)
  	{
     image_new_bin[i][0] = 0;
	 image_new_bin[i][W-1] = 0;
      }

  for(i=H-1;i>=1;i--)  //从下往上从左往右扫描
     {
       for(j=1;j< W;j++)
           {
             if((image_new_bin[i][j-1]==0)&&(image_new_bin[i][j+1]==0))
                 {
                   image_new_bin[i][j]=0;    //若上一个扫描点和下一个扫描点都为黑色，那这个点就是黑点
                 }
             else if ((image_new_bin[i][j-1]==255)&&(image_new_bin[i][j+1]==255))
                {
                   image_new_bin[i][j]=255;   //若上一个扫描点和下一个扫描点都为白色，那这个点就是白点
                }
           }
     }


}

/********************************************图像LCD显示********************************************/
void  LCD_tuxiang(void)
{ 	 
  u8 i = 0, j = 0,temp=0;
  
  //发送帧头标志
  for(i=8;i<56;i+=8)//6*8=48行 
  {
    LCD_Set_Pos(18,i/8+1);//起始位置
    for(j=0;j<LCDW;j++)  //列数
    { 
      temp=0;
      if(image_new_bin[0+i][j]) temp|=1;
      if(image_new_bin[1+i][j]) temp|=2;
      if(image_new_bin[2+i][j]) temp|=4;
      if(image_new_bin[3+i][j]) temp|=8;
      if(image_new_bin[4+i][j]) temp|=0x10;
      if(image_new_bin[5+i][j]) temp|=0x20;
      if(image_new_bin[6+i][j]) temp|=0x40;
      if(image_new_bin[7+i][j]) temp|=0x80;
      LCD_WrDat(temp); 	  	  	  	  
    }
  }  
}


/********************************************找中线函数********************************************/
void      mid_line( )
{
   uint16 i,j,k1,l1,l2,l3,l4=0;             //l1右边线不丢到丢的个数,l2右边线丢到不丢的个数,l1左边线不丢到丢的个数,l1右边线丢到不丢的个数
   
   if(begin>64)
     begin=64;
   if(begin<30)
     begin=30;
   mid[56]=begin;
   
   left_error_sun=0;                       //左误差和清零
   L_num=0;                                //左计数清零
   left_error_aver=0;                      //左误差平均值和清零
   right_error_sun=0;                      //右误差和清零
   R_num=0;                                //右计数清零
   right_error_aver=0;                     //右误差平均值和清零
   
   
   for(i=55 ;i>0 ;i--)
   {
       k1= mid[i+1];
       if (image_new_bin[i][k1]==0)
       {
         for(j=i;j>5;j--)
           youxiaohang[j]=0;                          //无效行
         break;
       }
       else
       {
         mid_line_begin=i;
         youxiaohang[i]=1;
       }                          //有效行
       
       
       for(j=mid[i+1];j>1;j--)  // 从右边向左边搜索，寻找白跳变黑的点
       {
           if(image_new_bin[i][j+1]==0xFF&& image_new_bin[i][j]==0xFF&& image_new_bin[i][j-1]==0x00 &&image_new_bin[i][j-2]==0x00)   //白白黑黑
           {
             left[i] = j;       //记录边界坐标
             lost_left[i]=0;
             L_num++;
             left_error_sun=abs(left[i+1]-left[i])+left_error_sun; //前期使用，后期没有使用
             break;             //找到就退出循环
           }
           if(j==2)             // 未找到左边黑点
           {
             left[i] = 2;       //左边界取最小
             lost_left[i]=1;    //该行丢线标志置一
             image_new_bin[i][2]=0;  //在图像上将右边界描黑
             break;
           }
       }
       
       
       for(j=mid[i+1];j<W-1;j++)          // 从左边向右边搜索，寻找白跳变黑的点
       {
           if( image_new_bin[i][j]==0xFF&& image_new_bin[i][j+1]== 0x00)
           {
             right[i] = j;
             lost_right[i]=0;
             L_num++;
             right_error_sun=(abs(right[i+1]-right[i])+right_error_sun);
             break;
           }
           if(j>=W-4)     //未找到右边黑点
           {
             lost_right[i]=1;
             image_new_bin[i][91]=0;
             right[i] = W-1;
             break;
           }
       }
       
       
       if(i<54&&l1<3) //寻找从不丢线到丢线的坐标Y方向（右边线）
         {
             if(lost_right[i]==1&&lost_right[i+1]==0)
             {
              right_budiu_diu[l1]=i;
               l1++;
             }
         }

       if(i<54&&l2<3)                                      //寻找从丢线到不丢线的坐标Y方向（右边线）
         if(lost_right[i]<lost_right[i+1])
         {
           right_diu_budiu[l2]=i;
           l2++;
         }

       if(i<54&&l3<3)                                      //寻找从不丢线到丢线的坐标Y方向（左边线）
         if(lost_left[i]>lost_left[i+1])
         {
           left_budiu_diu[l3]=i;
           l1++;
         }

       if(i<54&&l4<3)                                      //寻找从丢线到不丢线的坐标Y方向（左边线）
         if(lost_left[i]<lost_left[i+1])
         {
           left_diu_budiu[l4]=i;
           l1++;
         }


       mid[i]=(left[i]+right[i])/2;                       //左加右除以二计算赛道中点坐标
       if((lost_right[i]==1)&&(lost_left[i]==1))          //左右丢线去中间
          mid[i]=47;
       wild_real[i]=right[i]-left[i];                     //实际赛道的宽度
       image_new_bin[i][mid[i]]=0;                        //描黑中线

   }
   
   
   
   left_error_aver=left_error_sun*1000/L_num;           //左边线误差的平均值计算
   right_error_aver=right_error_sun*1000/R_num;         //右边线误差平均值计算
   begin=mid[54];                                       //跟随式算法，下一次扫描的起点为这次的中点
}


/********************************************最小二乘法********************************************/
//图像数组传出数据的坐标，行坐标作为x，列坐标作为函数值，进行拟合，求出k和b，k和b为全局变量

void Slope_Calculate(uint8 begin,uint8 end,int *p)    //最小二乘法拟合斜率         //验证可行 5.27
{
  b=0;
  k=0;
  float xsum=0,ysum=0,xysum=0,x2sum=0;                   //定义几个浮点型变量

   uint8 i=0;                                            //定义循环变量
   float result=0;
   static float resultlast;
   p=p+begin;                                            //定义数组起点
   for(i=begin;i<end;i++)                                 //求和函数
   {
	   xsum+=i;                               //对X求和，假设是行数
	   ysum+=*p;                              //对Y（*p）求和，假设是坐标
	   xysum+=i*(*p);                         //对XY求和
	   x2sum+=i*i;                            //对X*X求和
	   p=p+1;                                //P指向下一个Y的地址
   }
  if((end-begin)*x2sum-xsum*xsum) //判断除数是否为零
  {
    result=((end-begin)*xysum-xsum*ysum)/((end-begin)*x2sum-xsum*xsum); // 计算 k=D(XY)/D(X)
    resultlast=result;
  }
  else
  {
   result=resultlast;
  }
  b=ysum/(end-begin)-result*xsum/(end-begin);
  k=result;

}

/********************************************补线函数********************************************/
void buxian(void)
{
  num3=0;

  for(int i=55;i>5;i--)              //统计左右都丢线行数
  {
    if(lost_right[i]&&lost_left[i])
    {
      num3++;
    }
  }

  if(num3>5&&ring_R_sign<2&&ring_L_sign<2)    //全丢行大于5，并不在环内
  {
    if(lost_left[50]&&lost_left[40]&&lost_left[30])
    {
        for(int i=50;i>5;i--)                //近端            //右斜十字补线
      {
         right[i]=59;
         image_new_bin[i][right[i]]=0;
      }
    }


    if(lost_right[50]&&lost_right[40]&&lost_right[30])
    {
      for(int i=50;i>5;i--)                //近端            //右斜十字补线
      {
         left[i]=35;
         image_new_bin[i][left[i]]=0;
      }
    }

  }

//  int l;                                                                          十字补线6.5 不是很好用
//  if(((lost_right[34]&&lost_left[34])||(lost_right[44]&&lost_left[44]))==1)
//    {
//      Slope_Calculate(7,12,left);
//      gpio_init(E6,GPO,0);
//      for(int i=30;i<45;i++)
//      {
//        y_co=i*k+b;
//        if(y_co<0)
//           y_co=0;
//        image_new_bin[i][y_co]=40;
//        left[i]=y_co;
//      }
//      Slope_Calculate(7,12,right);
//      for(int i=30;i<45;i++)
//      {
//        y_co=i*k+b;
//        if(y_co>94)
//           y_co=94;
//        image_new_bin[i][y_co]=40;
//        right[i]=y_co;
//        mid[i]=(left[i]+right[i])/2;
//        l=mid[i];
//        image_new_bin[i][l]= 0;
//      }
//    }
//  if(lost_right[44]&&lost_left[44]==1)
//    {
//      Slope_Calculate(7,12,left);
//      gpio_init(E6,GPO,0);
//      for(int i=30;i<45;i++)
//      {
//        y_co=i*k+b;
//        image_new_bin[i][y_co]=40;
//        left[i]=y_co;
//      }
//      Slope_Calculate(7,12,right);
//      for(int i=30;i<45;i++)
//      {
//        y_co=i*k+b;
//        image_new_bin[i][y_co]=40;
//        right[i]=y_co;
//        mid[i]=(left[i]+right[i])/2;
//        l=mid[i];
//        image_new_bin[i][l]= 0;
//      }
//    }

   //右环


  if(ring_R_sign==3)                                         //进右环补线
  {
    int i2=0;
    for(i2=0;i2<30;i2++)
    {
      if(lost_right[i2]==1)
        break;
    }

    if(left[i2-4]>85)
      ring_3_sign=1;
    k=(right[i2-4])/(i2-4.0-60.0);
    b=-k*60.0;
    for(int i=20;i<45;i++)
      {
        y_co=(int)((k*i+b*jinghuan_buchang_R));
        if(y_co<0)
           y_co=0;
        image_new_bin[i][y_co]=40;
        left[i]=y_co;
        mid[i]=(left[i]+right[i])/2;
      }
  }

//   if(ring_R_sign==5)                                        //出右环补线
//  {
//  if((lost_left[20])==1)
//     if(chuhuan_sign==0)
//     {
//       chuhuan_sign=1;
//       time=count;
//     }
//  if(chuhuan_sign)
//    duty=4400+0.162*(car_journey_PL-car_journey_PL_ring);
//
//  }
//         Slope_Calculate(40,45,left);
/*左环*/

  if(ring_L_sign==3)                                         //进左环补线
  {
    int i2=0;
    for(i2=0;i2<30;i2++)
    {
      if(lost_left[i2]==1)
        break;
    }
    if(left[i2-4]<10)
      ring_3_sign=1;
    k=(left[i2-4]-94)/(i2-4.0-60);
    b=94-k*60.0;
    for(int i=20;i<45;i++)
      {
        y_co=(int)((k*i+b)-jinghuan_buchang_L);
        if(y_co<0)
           y_co=0;
        image_new_bin[i][y_co]=40;
        right[i]=y_co;
        mid[i]=(left[i]+right[i])/2;
      }
  }


}

/********************************************环识别函数********************************************/
void ring(void)       //勉强可用
{
  if(chuhuan_sign==1)    //出环蜂鸣器叫
    GPIO_Ctrl (PTB,23,0);
  else
    GPIO_Ctrl (PTB,23,1);
  
  /*******右环*********/
  if(ring_R_sign==6)        //彻底出环清所有标志位，右单边巡线
  {
      if((count==time+50))
      {
        ring_R_sign=0;
        chuhuan_sign=0;
        ring_3_sign=0;
        GPIO_Ctrl (PTE,4,1);
        GPIO_Ctrl (PTE,6,1);
        GPIO_Ctrl (PTB,23,1);
      }
  }

  if(ring_R_sign==5)    //环内，寻找出环特征
  {
      if(count==time+30)
      {
        ring_R_sign=6;
        time=count;

      }
  }
  if(ring_R_sign==4)        //过度用
  {
    if(count==time+80)
    {
      ring_R_sign=5;
    }
  }
  if(ring_R_sign==3)      //进环
  {
    if(count==time+25)  //大环25
      {
        ring_R_sign=4;
        time=count;
        car_journey_PL_ring=(int)car_journey_PL;  //记录进环时的路程
      }
  }

  if(ring_R_sign==2)                         //200毫秒清除标志位
    if(count>time+50)
      ring_R_sign=0;

  if(ring_R_sign==2)                         //进环时机
  {
    if((right[14]-right[11])>13)             //拐角
    {
      Slope_Calculate(8,20,left);
      if(k<-0.2&&k>-1)
        {
          //stop_sign=1;
          ring_R_sign=3;
          GPIO_Ctrl (PTE,6,0);
          time=count;
        }
    }
  }

  if(ring_R_sign==1)                  //精准判断
    for(int i=17;i<40;i=i+2)                                                                                       //前方环判断(右环)
    {
      if((right[i-10]>right[i])&&(right[i]<right[i+10])
       &&!lost_left[10]&&!lost_left[20]&&!lost_left[30]&&!lost_left[40]&&!lost_left[50]
       &&!lost_right[i-12]&&!lost_right[i]&&!lost_right[i+12]&&!lost_right[i-8]&&!lost_right[i+8])
      {
        Slope_Calculate(8,20,left);
        if(k<-0.2&&k>-1)
        {
          GPIO_Ctrl (PTE,4,0);
          GPIO_Ctrl (PTB,23,0);
          //stop_sign=1;
          ring_R_sign=2;
          break;
        }
      }
    }

  if(ring_R_sign==1)               //预判断
    if(count==time+10)
      ring_R_sign=0;
 if((ring_L_sign==0)&&(ring_R_sign==0)||ring_R_sign==1)
   if(!lost_left[10]&&!lost_left[20]&&!lost_left[30]&&!lost_left[40]&&!lost_left[50]
     &&((lost_right[20]&&lost_right[22]&&lost_right[24]&&lost_right[26])
       ||(lost_right[40]&&lost_right[42]&&lost_right[44]&&lost_right[46])))

   {
     Slope_Calculate(8,20,left);
     if(k<-0.2&&k>-1)
     {
     ring_R_sign=1;
     time=count;
     }
   }


/*左环*/
  if(ring_L_sign==6)                                                                           //单边巡线0.5s或者左边丢线
  {
      if((count>time+50)||(lost_right[20]))
      {
        ring_L_sign=0;       //环标志清零
        chuhuan_sign=0;      //出环标志清零
        ring_3_sign=0;       //补线异常标志清零
        GPIO_Ctrl (PTB,23,1);

      }
  }
  if(ring_L_sign==5)                                                                             //5号等待出环补线
  {
      if(count==time+30)
      {
        ring_L_sign=6;
        chuhuan_sign=0;
        time=count;
        GPIO_Ctrl (PTE,4,1);
        GPIO_Ctrl (PTE,6,1);
      }
  }

 if(ring_L_sign==4)
 {
   if(count==time+80)
   {
     ring_L_sign=5;
   }
 }


 if(ring_L_sign==3)      //进环前
  {
    if((count==time+20))     //不丢线                                             //等待0.3s或者补线异常
      {
        ring_L_sign=4;
        time=count;
        car_journey_PL_ring=(int)car_journey_PL;
      }
  }

 if(ring_L_sign==2)    //进环
    if(count>time+50)
    {
      ring_L_sign=0;
      GPIO_Ctrl (PTE,4,1);
      GPIO_Ctrl (PTB,23,1);
    }

 if(ring_L_sign==2)    //进环
  {
    if((left[11]-left[14])>13)                           // 进环时机控制
    {
      Slope_Calculate(8,20,right);
      if(k>0.2&&k<1)
        {
          //stop_sign=1;
          ring_L_sign=3;
        GPIO_Ctrl (PTE,6,0);
          time=count;
        }
    }
  }

  if(ring_L_sign==1)
    for(int i=17;i<45;i=i+2)                 //多寻找几次左边的特征
    {
    if((left[i-10]<left[i])&&(left[i]>left[i+10])                                                      //中间大，两边小
       &&!lost_right[10]&&!lost_right[20]&&!lost_right[30]&&!lost_right[40]
       &&!lost_left[i-12]&&!lost_left[i]&&!lost_left[i+12]&&!lost_left[i-8]&&!lost_left[i+8])
      {
        Slope_Calculate(8,20,right);
        if(k>0.2&&k<1)                                               //在车辆偏差时仍可以检测到环
        {
         GPIO_Ctrl (PTE,4,0);
        GPIO_Ctrl (PTB,23,0);
          //stop_sign=1;
          ring_L_sign=2;
          break;
        }
      }
    }

 if(ring_L_sign==1)
    if(count==time+10)
      ring_L_sign=0;
 if((ring_L_sign==0)&&(ring_R_sign==0)||ring_L_sign==1)
   if(!lost_right[10]&&!lost_right[20]&&!lost_right[30]&&!lost_right[40]
     &&((lost_left[20]&&lost_left[22]&&lost_left[24]&&lost_left[26])
       ||(lost_left[40]&&lost_left[42]&&lost_left[44]&&lost_left[46])))
   {
        Slope_Calculate(8,20,right);
        if(k>0.2&&k<1)
        {
          ring_L_sign=1;
          time=count;
        }
   }

}

/********************************************OLED显示********************************************/
//显示基本图像
//void OLED_xianshi_yuanshi()
//{
//  LCD_PrintU16(94,0,(int)set_speed);                            //OLED显示设定速度
//  LCD_PrintU16(94,1,(int)(temp_error*1000));                   //OLED显示舵机打角占空比
//  //LCD_PrintU16(94,2,yuzhi_max);                                 //OLED显示方向误差
//  //LCD_PrintU16(94,3,yuzhi_min);
// // LCD_PrintU16(94,4,THRESHOLD);                                 //OLED显示阈值
//  LCD_PrintU16(94,5,end_time-begin_time);                        //OLED显示时间
//  LCD_PrintU16(94,6,(int)speed_all);                               //OLED显示速度
//  LCD_PrintU16(94,7,(int)(car_journey_PL/331));                    //OLED显示路程
//  dis_bmp(60,94,&image_new_bin[0][0],125);                           //OLED显示图像
// // Draw_Road();
// 
//
//}
//
////PID数值显示
//void OLED_PID_shuzhi()
//{
//    OLED_P6x8Str(6,0,"      P_max:");
//    OLED_P6x8Str(6,1,"      P_min:");
//    OLED_P6x8Str(6,2,"      PID_P:");
//    OLED_P6x8Str(6,3,"      PID_D:");
//    OLED_P6x8Str(6,4,"  pid_D_max:");
//    OLED_P6x8Str(6,5,"  pid_D_min:");
//    OLED_P6x8Str(6,6,"   run_time:");
//    OLED_P6x8Str(6,7,"        yxh:");
//
//    LCD_PrintU16(78,0,(int)P_max_nor);             //OLED显示舵机P_max
//    LCD_PrintU16(78,1,(int)P_min_nor);             //OLED显示舵机P_min
//    LCD_PrintU16(78,2,(int)PID_P);                 //OLED显示舵机PID_P
//    LCD_PrintU16(78,3,(int)PID_D);                 //OLED显示舵机PID_D
//    LCD_PrintU16(78,4,(int)steer_pid_D_max_nor);   //OLED显示电机pid_D_max
//    LCD_PrintU16(78,5,(int)steer_pid_D_min_nor);   //OLED显示电机pid_D_min
//    LCD_PrintU16(78,6,run_time);              //OLED显示行驶时间
//    LCD_PrintU16(78,7,yxh_nor);               //OLED显示有效行
//
//
//}
//
////
//void OLED1()
//{
//    OLED_P6x8Str(6,0,"  speed_max:");
//    OLED_P6x8Str(6,1,"  speed_min:");
//    OLED_P6x8Str(6,2,"speed_ring_R:");
//    OLED_P6x8Str(6,3,"speed_ring_L:");
//    OLED_P6x8Str(6,4,"   speed_up:");
//    OLED_P6x8Str(6,5,"   bmx_stop:");
//    OLED_P6x8Str(6,6,"  chusaidao:");
//    OLED_P6x8Str(6,7,"   dajingfa:");
//
//    LCD_PrintU16(78,0,(int)speed_max);              //OLED显示speed_max
//    LCD_PrintU16(78,1,(int)speed_min);              //OLED显示speed_min
//    LCD_PrintU16(78,2,speed_ring_R);              //OLED显示速度
//    LCD_PrintU16(78,3,speed_ring_L);            //OLED显示中线误差*1000
//    LCD_PrintU16(78,4,(int)speed_up);               //OLED显示舵机占空比
//    LCD_PrintU16(78,5,bmx_stop);                //OLED显示右环标志位
//    LCD_PrintU16(78,6,chusaidao_stop);              //OLED显示左边线误差
//   // LCD_PrintU16(78,7,dajingfa);                //OLED显示有效行终点
//}
//
////
//void OLED2()
//{
//    OLED_P6x8Str(6,0,"   pianyi_R:");
//    OLED_P6x8Str(6,1,"   pianyi_L:");
//    OLED_P6x8Str(6,2,"   dajiao_R:");
//    OLED_P6x8Str(6,3,"   dajiao_L:");
//    OLED_P6x8Str(6,4,"  buchang_R:");
//    OLED_P6x8Str(6,5,"  buchang_L:");
////    OLED_P6x8Str(6,6,"        yxh:");
////    OLED_P6x8Str(6,7,"        yxh:");
//
//    LCD_PrintU16(78,0,pianyi_R);             //OLED显示舵机P_max
//    LCD_PrintU16(78,1,pianyi_L);             //OLED显示舵机P_min
//    LCD_PrintU16(78,2,chuhuan_dajiao_R);                 //OLED显示舵机PID_P
//    LCD_PrintU16(78,3,chuhuan_dajiao_L);                 //OLED显示舵机PID_D
//    LCD_PrintU16(78,4,jinghuan_buchang_R*1000);   //OLED显示电机pid_D_max
//    LCD_PrintU16(78,5,jinghuan_buchang_L*1000);   //OLED显示电机pid_D_min
////    LCD_PrintU16(78,6,bmx_stop);              //OLED显示行驶时间
////    LCD_PrintU16(78,7,chusaidao_stop);               //OLED显示有效行
//}


/********************************************编码器数据采集和处理********************************************/


//编码器采集递推滤波
float  result_meanL_FIFO[1][11];                  //递推滤波 L
float  result_meanR_FIFO[1][11];                  //递推滤波 R 
float  result_meanLR_FIFO[1][11];                  //递推滤波 LR
float  result_meanL_newValues(float dat1)
{
	uint8 i ;
	float sum=0;
	uint8 i_val=5;   																		//?ù?μ??2¨′?êy
	for(i=1;i<i_val;i++)															//FIFO 2ù×÷
	{
          result_meanL_FIFO[0][i-1]=result_meanL_FIFO[0][i];
	}
	result_meanL_FIFO[0][i_val-1]=dat1;
	sum=0;
	//for(i=0;i<i_val;i++)															//?óμ±?°êy×éμ?o?￡??ùè????ù?μ
	{
          sum=result_meanL_FIFO[0][0]*0.05+result_meanL_FIFO[0][1]*0.1+result_meanL_FIFO[0][2]*0.15+result_meanL_FIFO[0][3]*0.3+result_meanL_FIFO[0][4]*0.4;
	}
	result_meanL_FIFO[0][10]=sum;
        
        return result_meanL_FIFO[0][10]; 

}

float  result_meanR_newValues(float dat1)
{
	uint8 i ;
	float sum=0;
	uint8 i_val=5;   																		//?ù?μ??2¨′?êy
	for(i=1;i<i_val;i++)															//FIFO 2ù×÷
	{
          result_meanR_FIFO[0][i-1]=result_meanR_FIFO[0][i];
	}
	result_meanR_FIFO[0][i_val-1]=dat1;
	sum=0;
	//for(i=0;i<i_val;i++)															//?óμ±?°êy×éμ?o?￡??ùè????ù?μ
	{
          sum=result_meanR_FIFO[0][0]*0.05+result_meanR_FIFO[0][1]*0.1+result_meanR_FIFO[0][2]*0.15+result_meanR_FIFO[0][3]*0.3+result_meanR_FIFO[0][4]*0.4;
	}
	result_meanR_FIFO[0][10]=sum;
        
        return result_meanR_FIFO[0][10]; 

}

float  result_meanLR_newValues(float dat1)
{
	uint8 i ;
	float sum=0;
	uint8 i_val=5;   																		//?ù?μ??2¨′?êy
	for(i=1;i<i_val;i++)															//FIFO 2ù×÷
	{
          result_meanLR_FIFO[0][i-1]=result_meanLR_FIFO[0][i];
	}
	result_meanLR_FIFO[0][i_val-1]=dat1;
	sum=0;
	//for(i=0;i<i_val;i++)															//?óμ±?°êy×éμ?o?￡??ùè????ù?μ
	{
          sum=result_meanLR_FIFO[0][0]*0.05+result_meanLR_FIFO[0][1]*0.1+result_meanLR_FIFO[0][2]*0.15+result_meanLR_FIFO[0][3]*0.3+result_meanLR_FIFO[0][4]*0.4;
	}
	result_meanLR_FIFO[0][10]=sum;
        
        return result_meanLR_FIFO[0][10]; 
}

//速度、路程计算
void car_speed_dis(void)
{
  car_journey_PL+=rel_speed;
//  car_journey=car_journey_PL/journey_1m;
//  if(car_time>0.1)car_speed_m_s=car_journey/car_time;     //注意防止分母不为零
}


//采集编码器，记路程
void qd_speed_deal(void)
{

    qd_result_L =FTM_AB_Get(FTM1);
    qd_result_R =-FTM_AB_Get(FTM2);
    rel_speed=(qd_result_L+ qd_result_R)/2.0;
 
      
   result_meanL_newValues((float)qd_result_L);                   //递推滤波
    qd_result_L =(int)result_meanL_FIFO[0][10];                  //递推滤波
    result_meanR_newValues((float)qd_result_R);                   //递推滤波
    qd_result_R =(int)result_meanR_FIFO[0][10];                  //递推滤波
    result_meanLR_newValues((float)rel_speed);                   //递推滤波
    rel_speed=result_meanLR_FIFO[0][10];                  //递推滤波
    car_speed_dis();               //计里程

//#endif
//    if((motor_stop==0))car_speed_dis();                 //计里程
}