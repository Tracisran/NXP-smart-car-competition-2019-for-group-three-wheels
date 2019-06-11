#include "camaro.h"
#include "include.h"
/**********************�궨��**********************/

#define IMAGEH  120//�� HEIGHT ���ɼ�����ͷͼ��߶�����
#define IMAGEW  188 //�� WIDTH  ���ɼ�����ͷͼ��������

#define LCDH    60  //OLED��ʾ������
#define LCDW    94  //OLED��ʾ������

#define DIANJI_PWM_max  3000    // ���PWM�����ֵ
#define DIANJI_PWM_min  -3000    // ���PWM����Сֵ



#define ZHONGZHI 3
/**********************ȫ�ֱ�������**********************/

float Angle_Balance,Gyro_Turn,Acceleration_Z,Gyro_Balance;
float angle, angle_dot; 	
float Q_angle=0.001;// 1y3��??������?D-��?2?
float Q_gyro=0.003;//0.003 1y3��??������?D-��?2? 1y3��??������?D-��?2??a��???��?DD��?��D???��
float R_angle=0.5;// 2a��???������?D-��?2? ?��2a��???2?
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


//���ֱ�־
int bmx_stop=1;
int chusaidao_stop=1;
int dajingfa=1;

//ͼ�����

u16 Pixle1[LCDH][LCDW];              //��ֵ��������OLED��ʾ������
u8 Image_Data[IMAGEH][IMAGEW];      //ͼ��ԭʼ���ݴ��
int W=94;                         //ͼ����
int H=60;                         //ͼ��߶�
uint8 image_new_bin[60][94];      //��ֵ��ͼ������
uint8 image_new_gray_bin[60][94];      //��ֵ��ͼ������
uint8 num;
uint8 x;
uint8 y;


//��򷨲���
uint8 yuzhi[20][94];
uint8 THRESHOLD=55;
uint8 THRESHOLD_old=55;

//��ֵ�ɵ�����
int yuzhi_max=70;
int yuzhi_min=40;


//���߲���
int slope_array[60]={40};       //����ƫ��
int mid[60];                    //ÿһ�е�������������
int right[60];                  //�ұ߽�
int left[60];                   //��߽�
int lost_right[60];             //���߱�־���� 1Ϊ����0Ϊ����
int lost_left[60];              //�Ҷ��߱�־���� 1Ϊ����0Ϊ����
int youxiaohang[60];            //��Ч�б�־���� 1Ϊ��Ч0Ϊ��Ч
int right_diu_budiu[3]={-1,-1,-1};
int left_diu_budiu[3]={-1,-1,-1};
int right_budiu_diu[3]={-1,-1,-1};
int left_budiu_diu[3]={-1,-1,-1};
float wild_real[60];            //ʵ���������

int mid_line_begin=0;           //��Զ��һ����Ч��
int begin=47;                   //��һ�������ұ��ߵ����

int right_error_sun=0;          //�ұ��ߵ�����
uint8 R_num=0;                  //��Ч�ұ��ߵ�����
int right_error_aver=0;          //��Ч�ұ��ߵ�����ƽ��ֵ

int left_error_sun=0;          //����ߵ�����
uint8 L_num=0;                  //��Ч����ߵ�����
int left_error_aver=0;          //��Ч����ߵ�����ƽ��ֵ

//���˷�����
float b=0;                    //���˷�б��
float k=0;                    //���˷�����
int y_co;                     //����ĵ�y����



//����־λ
uint8 ring_R_sign=0;            //�һ���־λ 1���� 2���� 3��ʼ���� 4���� 5����׼���������� 6���� 0���ڻ���
uint8 ring_L_sign=0;            //�󻷱�־λ 1���� 2���� 3��ʼ���� 4���� 5����׼���������� 6���� 0���ڻ���
uint8 chuhuan_sign=0;           //�������ߵı�־λ

int car_journey_PL_ring;

int ring_3_sign=0;

//���ɵ�����
int pianyi_R=8;
int pianyi_L=2;

int chuhuan_dajiao_R=4980;
int chuhuan_dajiao_L=3430;

float jinghuan_buchang_R=1.175;
float jinghuan_buchang_L=0;


//������
int    qd_result_L;                                  //��������������ǰ������󸺣�����
int    qd_result_R;                                  //��������������ǰ������󸺣�����
double  car_journey_PL=0;                          //С���й���·��(�������)
int journey_1m=3310;
int begin_time=0;
int run_time=15*100;                                //2s
int end_time=0;
double speed_all;
int straight_away=0;


//���PID����
float PID_P=900;               //ѧ��600-900
float P_max=500;
float P_min=250;
float PID_D=650;               //ѧ��9000-12000           �ٶȴ�DС
float steer_pid_D_max=500;
float steer_pid_D_min=250;
float steer_error_temp;        //�����ٶȿ���D

float P_max_ring=1100;
float P_min_ring=400;
float steer_pid_D_max_ring=700;
float steer_pid_D_min_ring=300;



float P_max_nor=700;
float P_min_nor=300;
float steer_pid_D_max_nor=540;
float steer_pid_D_min_nor=400;

//PWM�޷�         PWM����
int steer_pwm_mid=12100;     //һ�ų�
int steer_pwm_max=14850;
int steer_pwm_min=10050;

int yxh=22;                    //��������
int yxh_nor=22;
int yxh_ring=30;

float temp_wucha_sum;          //����
float temp_error=0;           //��������һ�����ֵ
float temp_last_error=0;      //��һ�ε����
float temp_error_error=0;     //�������
float duty;                    //���PWMռ�ձ�

//���PID���ƣ��ٶȶ���
float motor_pid_P=85;
float motor_pid_I_max=5;
float motor_pid_I_min=3.5;
float motor_pid_I=6.5;
float motor_pid_D=20;

float rel_speed=0;                                //ʵʱ�ٶ�
float rel_speed_L=0;                               //����ʵʱ�ٶ�
float rel_speed_R=0;                              //����ʵʱ�ٶ�
float set_speed=0;                                //�趨�ٶ�
float speed_error=0;                              //�ٶ����
float speed_sprint=90;                             //���ܳ���ٶ�
float speed_ramp=66;                               //�µ�����
float speed_shizi=66;                              //ʮ�ֵ�����
float speed_up=10;                                 //��ֱ������
int speed_ring_R=76;                                //�����ٶ�
int speed_ring_L=76;

float speed_min=76;                                //�趨����С�ٶ�
float speed_max=116;                              //�趨������ٶ�
      //���ռ�ձ�
float motor_pwm_out=0;                            //ռ�ձȣ������
float motor_pwm_out1=10;                           //ռ�ձ�1��
float motor_pwm_out2=0;                           //ռ�ձ�2��
float motor_pwm_max;                             //���ռ�ձ��޶�
float motor_pwm_min;                             //���ռ�ձ��޶�

float set_go_long=0;                              //����ȫ�ٳ�̵ľ����趨

int stop_sign=1;                                //ͣ����־
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
/********************************************ͼ��ɼ�����ֵ��********************************************/
void tuxiang_caiji()
{
    //int i,j;
    
    if(V034_Field_Over_Flag)      //ͼ��ѹ�����г�һ�У�����bug 6.12       20ms/7ms
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
/********************************************��ֵ��********************************************/
void Image_binaryzation(void)                       //���Ż���ֵ�޷�6.12
{
 uint8 i,j;
 sum=0;
 num1=0;
// #if 1
//  for(j=0;j<94;j++)                                    //����
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
  //THRESHOLD=48;                                                  //������
//  uint8 *p_Image;
//  uint8 *q_Image;
//  q_Image=&image_new_bin[0][0];
//
//  for(p_Image=&image_new_bin[0][0];p_Image<=&image_new_bin[H-1][W-1];p_Image++)  //���ɻص����ݻ����p_Image����
//  {
//    if((*p_Image<THRESHOLD)&&(*(p_Image+1)<THRESHOLD)||(*p_Image<THRESHOLD)&&(*(p_Image-1)<THRESHOLD))   //�˴�������һ���˲�
//      *q_Image++ =0;
//    else
//      *q_Image++ =255; //���ɻص��������һ���������һ�����С�����趨����ֵ�����϶��õ�Ϊ�ڵ����Ϊ�׵�
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

  /*******ȥ���*******/

  for(i=1;i<H;i++)
  	{
     image_new_bin[i][0] = 0;
	 image_new_bin[i][W-1] = 0;
      }

  for(i=H-1;i>=1;i--)  //�������ϴ�������ɨ��
     {
       for(j=1;j< W;j++)
           {
             if((image_new_bin[i][j-1]==0)&&(image_new_bin[i][j+1]==0))
                 {
                   image_new_bin[i][j]=0;    //����һ��ɨ������һ��ɨ��㶼Ϊ��ɫ�����������Ǻڵ�
                 }
             else if ((image_new_bin[i][j-1]==255)&&(image_new_bin[i][j+1]==255))
                {
                   image_new_bin[i][j]=255;   //����һ��ɨ������һ��ɨ��㶼Ϊ��ɫ�����������ǰ׵�
                }
           }
     }


}

/********************************************ͼ��LCD��ʾ********************************************/
void  LCD_tuxiang(void)
{ 	 
  u8 i = 0, j = 0,temp=0;
  
  //����֡ͷ��־
  for(i=8;i<56;i+=8)//6*8=48�� 
  {
    LCD_Set_Pos(18,i/8+1);//��ʼλ��
    for(j=0;j<LCDW;j++)  //����
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


/********************************************�����ߺ���********************************************/
void      mid_line( )
{
   uint16 i,j,k1,l1,l2,l3,l4=0;             //l1�ұ��߲��������ĸ���,l2�ұ��߶��������ĸ���,l1����߲��������ĸ���,l1�ұ��߶��������ĸ���
   
   if(begin>64)
     begin=64;
   if(begin<30)
     begin=30;
   mid[56]=begin;
   
   left_error_sun=0;                       //����������
   L_num=0;                                //���������
   left_error_aver=0;                      //�����ƽ��ֵ������
   right_error_sun=0;                      //����������
   R_num=0;                                //�Ҽ�������
   right_error_aver=0;                     //�����ƽ��ֵ������
   
   
   for(i=55 ;i>0 ;i--)
   {
       k1= mid[i+1];
       if (image_new_bin[i][k1]==0)
       {
         for(j=i;j>5;j--)
           youxiaohang[j]=0;                          //��Ч��
         break;
       }
       else
       {
         mid_line_begin=i;
         youxiaohang[i]=1;
       }                          //��Ч��
       
       
       for(j=mid[i+1];j>1;j--)  // ���ұ������������Ѱ�Ұ�����ڵĵ�
       {
           if(image_new_bin[i][j+1]==0xFF&& image_new_bin[i][j]==0xFF&& image_new_bin[i][j-1]==0x00 &&image_new_bin[i][j-2]==0x00)   //�װ׺ں�
           {
             left[i] = j;       //��¼�߽�����
             lost_left[i]=0;
             L_num++;
             left_error_sun=abs(left[i+1]-left[i])+left_error_sun; //ǰ��ʹ�ã�����û��ʹ��
             break;             //�ҵ����˳�ѭ��
           }
           if(j==2)             // δ�ҵ���ߺڵ�
           {
             left[i] = 2;       //��߽�ȡ��С
             lost_left[i]=1;    //���ж��߱�־��һ
             image_new_bin[i][2]=0;  //��ͼ���Ͻ��ұ߽����
             break;
           }
       }
       
       
       for(j=mid[i+1];j<W-1;j++)          // ��������ұ�������Ѱ�Ұ�����ڵĵ�
       {
           if( image_new_bin[i][j]==0xFF&& image_new_bin[i][j+1]== 0x00)
           {
             right[i] = j;
             lost_right[i]=0;
             L_num++;
             right_error_sun=(abs(right[i+1]-right[i])+right_error_sun);
             break;
           }
           if(j>=W-4)     //δ�ҵ��ұߺڵ�
           {
             lost_right[i]=1;
             image_new_bin[i][91]=0;
             right[i] = W-1;
             break;
           }
       }
       
       
       if(i<54&&l1<3) //Ѱ�ҴӲ����ߵ����ߵ�����Y�����ұ��ߣ�
         {
             if(lost_right[i]==1&&lost_right[i+1]==0)
             {
              right_budiu_diu[l1]=i;
               l1++;
             }
         }

       if(i<54&&l2<3)                                      //Ѱ�ҴӶ��ߵ������ߵ�����Y�����ұ��ߣ�
         if(lost_right[i]<lost_right[i+1])
         {
           right_diu_budiu[l2]=i;
           l2++;
         }

       if(i<54&&l3<3)                                      //Ѱ�ҴӲ����ߵ����ߵ�����Y��������ߣ�
         if(lost_left[i]>lost_left[i+1])
         {
           left_budiu_diu[l3]=i;
           l1++;
         }

       if(i<54&&l4<3)                                      //Ѱ�ҴӶ��ߵ������ߵ�����Y��������ߣ�
         if(lost_left[i]<lost_left[i+1])
         {
           left_diu_budiu[l4]=i;
           l1++;
         }


       mid[i]=(left[i]+right[i])/2;                       //����ҳ��Զ����������е�����
       if((lost_right[i]==1)&&(lost_left[i]==1))          //���Ҷ���ȥ�м�
          mid[i]=47;
       wild_real[i]=right[i]-left[i];                     //ʵ�������Ŀ��
       image_new_bin[i][mid[i]]=0;                        //�������

   }
   
   
   
   left_error_aver=left_error_sun*1000/L_num;           //���������ƽ��ֵ����
   right_error_aver=right_error_sun*1000/R_num;         //�ұ������ƽ��ֵ����
   begin=mid[54];                                       //����ʽ�㷨����һ��ɨ������Ϊ��ε��е�
}


/********************************************��С���˷�********************************************/
//ͼ�����鴫�����ݵ����꣬��������Ϊx����������Ϊ����ֵ��������ϣ����k��b��k��bΪȫ�ֱ���

void Slope_Calculate(uint8 begin,uint8 end,int *p)    //��С���˷����б��         //��֤���� 5.27
{
  b=0;
  k=0;
  float xsum=0,ysum=0,xysum=0,x2sum=0;                   //���弸�������ͱ���

   uint8 i=0;                                            //����ѭ������
   float result=0;
   static float resultlast;
   p=p+begin;                                            //�����������
   for(i=begin;i<end;i++)                                 //��ͺ���
   {
	   xsum+=i;                               //��X��ͣ�����������
	   ysum+=*p;                              //��Y��*p����ͣ�����������
	   xysum+=i*(*p);                         //��XY���
	   x2sum+=i*i;                            //��X*X���
	   p=p+1;                                //Pָ����һ��Y�ĵ�ַ
   }
  if((end-begin)*x2sum-xsum*xsum) //�жϳ����Ƿ�Ϊ��
  {
    result=((end-begin)*xysum-xsum*ysum)/((end-begin)*x2sum-xsum*xsum); // ���� k=D(XY)/D(X)
    resultlast=result;
  }
  else
  {
   result=resultlast;
  }
  b=ysum/(end-begin)-result*xsum/(end-begin);
  k=result;

}

/********************************************���ߺ���********************************************/
void buxian(void)
{
  num3=0;

  for(int i=55;i>5;i--)              //ͳ�����Ҷ���������
  {
    if(lost_right[i]&&lost_left[i])
    {
      num3++;
    }
  }

  if(num3>5&&ring_R_sign<2&&ring_L_sign<2)    //ȫ���д���5�������ڻ���
  {
    if(lost_left[50]&&lost_left[40]&&lost_left[30])
    {
        for(int i=50;i>5;i--)                //����            //��бʮ�ֲ���
      {
         right[i]=59;
         image_new_bin[i][right[i]]=0;
      }
    }


    if(lost_right[50]&&lost_right[40]&&lost_right[30])
    {
      for(int i=50;i>5;i--)                //����            //��бʮ�ֲ���
      {
         left[i]=35;
         image_new_bin[i][left[i]]=0;
      }
    }

  }

//  int l;                                                                          ʮ�ֲ���6.5 ���Ǻܺ���
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

   //�һ�


  if(ring_R_sign==3)                                         //���һ�����
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

//   if(ring_R_sign==5)                                        //���һ�����
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
/*��*/

  if(ring_L_sign==3)                                         //���󻷲���
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

/********************************************��ʶ����********************************************/
void ring(void)       //��ǿ����
{
  if(chuhuan_sign==1)    //������������
    GPIO_Ctrl (PTB,23,0);
  else
    GPIO_Ctrl (PTB,23,1);
  
  /*******�һ�*********/
  if(ring_R_sign==6)        //���׳��������б�־λ���ҵ���Ѳ��
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

  if(ring_R_sign==5)    //���ڣ�Ѱ�ҳ�������
  {
      if(count==time+30)
      {
        ring_R_sign=6;
        time=count;

      }
  }
  if(ring_R_sign==4)        //������
  {
    if(count==time+80)
    {
      ring_R_sign=5;
    }
  }
  if(ring_R_sign==3)      //����
  {
    if(count==time+25)  //��25
      {
        ring_R_sign=4;
        time=count;
        car_journey_PL_ring=(int)car_journey_PL;  //��¼����ʱ��·��
      }
  }

  if(ring_R_sign==2)                         //200���������־λ
    if(count>time+50)
      ring_R_sign=0;

  if(ring_R_sign==2)                         //����ʱ��
  {
    if((right[14]-right[11])>13)             //�ս�
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

  if(ring_R_sign==1)                  //��׼�ж�
    for(int i=17;i<40;i=i+2)                                                                                       //ǰ�����ж�(�һ�)
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

  if(ring_R_sign==1)               //Ԥ�ж�
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


/*��*/
  if(ring_L_sign==6)                                                                           //����Ѳ��0.5s������߶���
  {
      if((count>time+50)||(lost_right[20]))
      {
        ring_L_sign=0;       //����־����
        chuhuan_sign=0;      //������־����
        ring_3_sign=0;       //�����쳣��־����
        GPIO_Ctrl (PTB,23,1);

      }
  }
  if(ring_L_sign==5)                                                                             //5�ŵȴ���������
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


 if(ring_L_sign==3)      //����ǰ
  {
    if((count==time+20))     //������                                             //�ȴ�0.3s���߲����쳣
      {
        ring_L_sign=4;
        time=count;
        car_journey_PL_ring=(int)car_journey_PL;
      }
  }

 if(ring_L_sign==2)    //����
    if(count>time+50)
    {
      ring_L_sign=0;
      GPIO_Ctrl (PTE,4,1);
      GPIO_Ctrl (PTB,23,1);
    }

 if(ring_L_sign==2)    //����
  {
    if((left[11]-left[14])>13)                           // ����ʱ������
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
    for(int i=17;i<45;i=i+2)                 //��Ѱ�Ҽ�����ߵ�����
    {
    if((left[i-10]<left[i])&&(left[i]>left[i+10])                                                      //�м������С
       &&!lost_right[10]&&!lost_right[20]&&!lost_right[30]&&!lost_right[40]
       &&!lost_left[i-12]&&!lost_left[i]&&!lost_left[i+12]&&!lost_left[i-8]&&!lost_left[i+8])
      {
        Slope_Calculate(8,20,right);
        if(k>0.2&&k<1)                                               //�ڳ���ƫ��ʱ�Կ��Լ�⵽��
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

/********************************************OLED��ʾ********************************************/
//��ʾ����ͼ��
//void OLED_xianshi_yuanshi()
//{
//  LCD_PrintU16(94,0,(int)set_speed);                            //OLED��ʾ�趨�ٶ�
//  LCD_PrintU16(94,1,(int)(temp_error*1000));                   //OLED��ʾ������ռ�ձ�
//  //LCD_PrintU16(94,2,yuzhi_max);                                 //OLED��ʾ�������
//  //LCD_PrintU16(94,3,yuzhi_min);
// // LCD_PrintU16(94,4,THRESHOLD);                                 //OLED��ʾ��ֵ
//  LCD_PrintU16(94,5,end_time-begin_time);                        //OLED��ʾʱ��
//  LCD_PrintU16(94,6,(int)speed_all);                               //OLED��ʾ�ٶ�
//  LCD_PrintU16(94,7,(int)(car_journey_PL/331));                    //OLED��ʾ·��
//  dis_bmp(60,94,&image_new_bin[0][0],125);                           //OLED��ʾͼ��
// // Draw_Road();
// 
//
//}
//
////PID��ֵ��ʾ
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
//    LCD_PrintU16(78,0,(int)P_max_nor);             //OLED��ʾ���P_max
//    LCD_PrintU16(78,1,(int)P_min_nor);             //OLED��ʾ���P_min
//    LCD_PrintU16(78,2,(int)PID_P);                 //OLED��ʾ���PID_P
//    LCD_PrintU16(78,3,(int)PID_D);                 //OLED��ʾ���PID_D
//    LCD_PrintU16(78,4,(int)steer_pid_D_max_nor);   //OLED��ʾ���pid_D_max
//    LCD_PrintU16(78,5,(int)steer_pid_D_min_nor);   //OLED��ʾ���pid_D_min
//    LCD_PrintU16(78,6,run_time);              //OLED��ʾ��ʻʱ��
//    LCD_PrintU16(78,7,yxh_nor);               //OLED��ʾ��Ч��
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
//    LCD_PrintU16(78,0,(int)speed_max);              //OLED��ʾspeed_max
//    LCD_PrintU16(78,1,(int)speed_min);              //OLED��ʾspeed_min
//    LCD_PrintU16(78,2,speed_ring_R);              //OLED��ʾ�ٶ�
//    LCD_PrintU16(78,3,speed_ring_L);            //OLED��ʾ�������*1000
//    LCD_PrintU16(78,4,(int)speed_up);               //OLED��ʾ���ռ�ձ�
//    LCD_PrintU16(78,5,bmx_stop);                //OLED��ʾ�һ���־λ
//    LCD_PrintU16(78,6,chusaidao_stop);              //OLED��ʾ��������
//   // LCD_PrintU16(78,7,dajingfa);                //OLED��ʾ��Ч���յ�
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
//    LCD_PrintU16(78,0,pianyi_R);             //OLED��ʾ���P_max
//    LCD_PrintU16(78,1,pianyi_L);             //OLED��ʾ���P_min
//    LCD_PrintU16(78,2,chuhuan_dajiao_R);                 //OLED��ʾ���PID_P
//    LCD_PrintU16(78,3,chuhuan_dajiao_L);                 //OLED��ʾ���PID_D
//    LCD_PrintU16(78,4,jinghuan_buchang_R*1000);   //OLED��ʾ���pid_D_max
//    LCD_PrintU16(78,5,jinghuan_buchang_L*1000);   //OLED��ʾ���pid_D_min
////    LCD_PrintU16(78,6,bmx_stop);              //OLED��ʾ��ʻʱ��
////    LCD_PrintU16(78,7,chusaidao_stop);               //OLED��ʾ��Ч��
//}


/********************************************���������ݲɼ��ʹ���********************************************/


//�������ɼ������˲�
float  result_meanL_FIFO[1][11];                  //�����˲� L
float  result_meanR_FIFO[1][11];                  //�����˲� R 
float  result_meanLR_FIFO[1][11];                  //�����˲� LR
float  result_meanL_newValues(float dat1)
{
	uint8 i ;
	float sum=0;
	uint8 i_val=5;   																		//?��?��??2����?��y
	for(i=1;i<i_val;i++)															//FIFO 2������
	{
          result_meanL_FIFO[0][i-1]=result_meanL_FIFO[0][i];
	}
	result_meanL_FIFO[0][i_val-1]=dat1;
	sum=0;
	//for(i=0;i<i_val;i++)															//?���̡�?�㨺y������?o?��??����????��?��
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
	uint8 i_val=5;   																		//?��?��??2����?��y
	for(i=1;i<i_val;i++)															//FIFO 2������
	{
          result_meanR_FIFO[0][i-1]=result_meanR_FIFO[0][i];
	}
	result_meanR_FIFO[0][i_val-1]=dat1;
	sum=0;
	//for(i=0;i<i_val;i++)															//?���̡�?�㨺y������?o?��??����????��?��
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
	uint8 i_val=5;   																		//?��?��??2����?��y
	for(i=1;i<i_val;i++)															//FIFO 2������
	{
          result_meanLR_FIFO[0][i-1]=result_meanLR_FIFO[0][i];
	}
	result_meanLR_FIFO[0][i_val-1]=dat1;
	sum=0;
	//for(i=0;i<i_val;i++)															//?���̡�?�㨺y������?o?��??����????��?��
	{
          sum=result_meanLR_FIFO[0][0]*0.05+result_meanLR_FIFO[0][1]*0.1+result_meanLR_FIFO[0][2]*0.15+result_meanLR_FIFO[0][3]*0.3+result_meanLR_FIFO[0][4]*0.4;
	}
	result_meanLR_FIFO[0][10]=sum;
        
        return result_meanLR_FIFO[0][10]; 
}

//�ٶȡ�·�̼���
void car_speed_dis(void)
{
  car_journey_PL+=rel_speed;
//  car_journey=car_journey_PL/journey_1m;
//  if(car_time>0.1)car_speed_m_s=car_journey/car_time;     //ע���ֹ��ĸ��Ϊ��
}


//�ɼ�����������·��
void qd_speed_deal(void)
{

    qd_result_L =FTM_AB_Get(FTM1);
    qd_result_R =-FTM_AB_Get(FTM2);
    rel_speed=(qd_result_L+ qd_result_R)/2.0;
 
      
   result_meanL_newValues((float)qd_result_L);                   //�����˲�
    qd_result_L =(int)result_meanL_FIFO[0][10];                  //�����˲�
    result_meanR_newValues((float)qd_result_R);                   //�����˲�
    qd_result_R =(int)result_meanR_FIFO[0][10];                  //�����˲�
    result_meanLR_newValues((float)rel_speed);                   //�����˲�
    rel_speed=result_meanLR_FIFO[0][10];                  //�����˲�
    car_speed_dis();               //�����

//#endif
//    if((motor_stop==0))car_speed_dis();                 //�����
}