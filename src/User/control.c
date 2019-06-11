#include "control.h"
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
/********************************************���PID���ƺ���********************************************/
void duoji_PID (void)      
{
  float left_error;                                //������뻭�����ĵ����
  float right_error;                               //�ұ����뻭�����ĵ����
  temp_wucha_sum=0;

  /*****��̬��Ч��******/
  yxh_nor=(int)(-0.2*rel_speed+42);            //��̬����Ч�У��ٶ������Ч��ԽԶ  36.3


  /****PID����ע��*******/
  if(ring_R_sign>1||ring_L_sign>1)                 //��PID�Ĳ�������ע��
  {
    P_max=P_max_ring;
    P_min=P_min_ring;
    steer_pid_D_max=steer_pid_D_max_ring;
    steer_pid_D_min=steer_pid_D_min_ring;
    yxh=yxh_ring;
  }
  else                                             //һ��PID������ע��
  {
    P_max=P_max_nor;
    P_min=P_min_nor;
    steer_pid_D_max=steer_pid_D_max_nor;
    steer_pid_D_min=steer_pid_D_min_nor;
    yxh=yxh_nor;
  }


  /*********������*********/
  left_error=47-left[yxh];
  right_error=right[yxh]-47;

  temp_last_error = temp_error;                                                       //��¼�ϴ����
  temp_error = ((right_error-left_error)/(left_error+right_error));                   //�����

  if(ring_R_sign==1)                                                   //����Ѳ�ߣ���  8
    temp_error = (wild[yxh]/2+pianyi_R-left_error)/(wild[yxh]/2+pianyi_R+left_error);
  if(ring_L_sign==1)                                                   //����Ѳ�ߣ��ң�
    temp_error = ((right_error-wild[yxh]/2-pianyi_L)/(wild[yxh]/2+pianyi_L+right_error));
  if(ring_R_sign==6)                                                   //����Ѳ�ߣ���
    temp_error = (wild[yxh]/2-left_error)/(wild[yxh]/2+left_error);
  if(ring_L_sign==6)                                                   //����Ѳ�ߣ��ң�
    temp_error = ((right_error-wild[yxh]/2)/(wild[yxh]/2+right_error));

  temp_error_error=temp_error-temp_last_error;                                          //�������

//  if(temp_error>1)                              //����޷�  ��������������1��������
//     temp_error=1;
//  if(temp_error<-1)
//     temp_error=-1;


  /*-------P��̬-----------*/
  PID_P=(P_max-P_min)*fabs(temp_error)+P_min;

  /*-------D��̬-----------*/
    //  if(fabs(speed_max-speed_min)<0.5)steer_pid.D=steer_pid.D_max;                                                                          //����Dȡ���
    //  else steer_pid.D=steer_pid.D_max-((rel_speed-speed_min)/(speed_max-speed_min)*(steer_pid.D_max-steer_pid.D_min));                      //��̬D

  steer_error_temp=(rel_speed-speed_min)/(speed_max-speed_min);   //�ٶ�ƫ�����

  if(steer_error_temp<0)steer_error_temp=0;       //�޷�
  if(steer_error_temp>1)steer_error_temp=1;

  PID_D=steer_pid_D_max-(steer_error_temp*(steer_pid_D_max-steer_pid_D_min));     //D�����ٶ�                                              //���¶�̬D�㷨,���ٶȹ���

  if(PID_D>steer_pid_D_max) PID_D=steer_pid_D_max;    //�޷�ֵ                                                                        //D�����޶�
  if(PID_D<steer_pid_D_min) PID_D=steer_pid_D_min;                                                                            //D�����޶�


/*-----------------------*/
  duty =steer_pwm_mid+(temp_error)*PID_P+PID_D*temp_error_error*10;            //PID���ƣ�ռ�ձȼ���

/*********������̬���**************/
  if(ring_R_sign==5)                                            //���һ����ߣ�·�̾������
  {
    if((lost_left[20])==1)                                      //���߳���
     if(chuhuan_sign==0)
     {
       chuhuan_sign=1;
       time=count;
     }
    if(chuhuan_sign)
      duty=chuhuan_dajiao_R-0.03*(car_journey_PL-car_journey_PL_ring);                //4950              //С4785  4686
    if(duty>4730)                                              //�޷��� ����֤
      duty=4730;
  }


  if(ring_L_sign==5)                                           //���󻷲��ߣ�·�̾������
  {
    if((lost_right[20])==1)                                     //�Ҷ��ߣ�����
     if(chuhuan_sign==0)
     {
       chuhuan_sign=1;
       time=count;
     }
    if(chuhuan_sign)
      duty=chuhuan_dajiao_L+0.02*(car_journey_PL-car_journey_PL_ring);
    if(duty<3500)                                             //�޷� ����֤
      duty=3500;
  }
}
///**************PWM�޷�***************/
//
//  if (duty>steer_pwm_max)  duty = steer_pwm_max;  //����   ��ת                      ���ռ�ձ��޷�
//  if (duty<steer_pwm_min)  duty = steer_pwm_min;  //����
//  PLL_Init(PLL200);         //��ʼ��PLLΪ200M������Ϊ40MHZ
//  FTM_PWM_Duty(FTM3,FTM_CH6,(int)duty);//ռ�ձ�����           //��������ռ�ձ�
//
//}


/********************************************С���ٶȿ���********************************************/
//
////����ʽPID�㷨(���Ƶ��)
//void motor_PID_control(void)
//{
//    float pwm_temp=0.0;           //pwm����ֵ
//
//    static float speed_last_error=0;    //�ϴ�ƫ��
//    static float speed_pre_error=0;     //����һ��ƫ��
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
//    speed_error=(set_speed)-rel_speed;      //�ٶ����
///*---------------------------------*/
////      motor_pid_I=(motor_pid_I_max-motor_pid_I_min)*(fabs(speed_error)/40)+motor_pid_I_min;  //��̬I  fabsΪ�����ֵ
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
/////*--------ͣ��Ҫɲ��---------------*/
//
/////*---------------------------------*/
/////*---------------------------------*/
////
////    //if((rel_speed<2)&&(motor_pwm_out>7000)&&(time_start==1)&&(car_journey>1.0)&&(motor_stop==0))motor_pwm_out=0,motor_stop=1,fmq_count=800;            //��ֹ�������ѳ�
////    if(((set_speed>500)||(set_speed<-10))&&(start_flag==1))motor_pwm_out=0,motor_stop=1,fmq_count=800;           //��ֹ���٣��洢оƬ��磩
/////*---------------------------------*/
//    if(motor_pwm_out<0)
//      motor_pwm_out=0;             //ռ�ձ��޶�
//    if(motor_pwm_out>DIANJI_PWM_max)
//      motor_pwm_out=DIANJI_PWM_max;             //ռ�ձ��޶�
//
////   ftm_pwm_init(ftm1,ftm_ch1,10000,motor_pwm_out);
//    ceshidianji2++;
//
//}


//ɲ���㷨 ����

void car_break_function(void)
{
    if((stop_sign==0)&&(car_journey_PL>set_go_long))
    {
      //if((((fabs(steer_pwm_mid-steer_pid.pidout))/((steer_pwm_max-steer_pwm_min)/2))>0.2)||(shizi_flag!=0)||(ramp_flag!=0))           //������һ���Ƕ�
      {
        if(((rel_speed-set_speed)>break_set))                                          //����ɲ�����ޣ�����ʵ���ٶȴ�����Ҫɲ���ٶ�
        {
          //if(fabs(steer_error)>0.2)                                                                     //��ֵһ����
          {
            break_flag=1;
          }
        }
      }
    }

    if(stop_sign==1)
    {
      motor_pwm_out=-2000;
      if((rel_speed-set_speed)<(5))                                                                     //����ɲ�����趨�ٶȻ�����
      {
        break_flag=0;
        motor_pwm_out=0;                                                                                //��ɲ����ǵ��Ե�������0������Ӹ��ļ���Ҫһ��ʱ��
      }
    }



    if(break_flag!=0)
    {
      motor_pwm_out=((((rel_speed-set_speed)/30)*(break_pwm_max-break_pwm_min))+break_pwm_min);         //��̬ɲ��

      if(motor_pwm_out<break_pwm_min)
        motor_pwm_out=break_pwm_min;
      if(motor_pwm_out>break_pwm_max)
        motor_pwm_out=break_pwm_max;
      
      motor_pwm_out=-motor_pwm_out;                                                                     //������ɲ���Ǹ���

      if((rel_speed-set_speed)<(8))                                                                     //����ɲ�����趨�ٶȻ�����
      {
        break_flag=0;
        motor_pwm_out=0;                                                                                //��ɲ����ǵ��Ե�������0������Ӹ��ļ���Ҫһ��ʱ��
      }

    }
}


//�趨С���̶��ٶ�

void car_speed_set(void)
{

        if(fabs(speed_max-speed_min)<1)
          set_speed=speed_min;                     //�����趨
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
/*-------�ٶ��޶�----------------------------------*/
        if(set_speed<speed_min)
          set_speed=speed_min;                       //��С�ٶ��޶�
        if(set_speed>speed_max)
          set_speed=speed_max;                       //����ٶ��޶�
///*-------------------------------------------------*/
///*-------ֱ���жϣ����٣�--------------------------*/
     if(temp_error<0.1)
     {
       ;
     }
     else
      straight_away=(int)car_journey_PL;

     if((car_journey_PL-straight_away)>=1500)
       set_speed=set_speed+speed_up;      //�߹�0.3�׵�ֱ��������ֱ��
///*-------------------------------------------------*/
///*---ʮ�ֵ����� �൱����ǰ���� ������ٶ�Сһ��----*/
//      if(shizi_flag!=0)set_speed=set_speed-speed_shizi;
///*-------------------------------------------------*/
///*-------�µ��������ٵ��µ����趨ֵ��--------*/
//      if(ramp_flag!=0)set_speed=speed_ramp;
///*-------------------------------------------------*/
///*-------ֱ�ǵ��жϣ����ٵ�ֱ�ǵ��趨ֵ��----------*/
 //       if(vertical_flag!=0)set_speed=speed_vertical;                          //ֱ�ǽ����ٽ���

/***********************��**************************/

      if(ring_R_sign==2||ring_R_sign==3||ring_R_sign==4||ring_R_sign==5||ring_R_sign==6)
        {
        set_speed=speed_ring_R;
        }


       if(ring_L_sign==2||ring_L_sign==3||ring_L_sign==4||ring_L_sign==5||ring_L_sign==6)
        {
        set_speed=speed_ring_L;
        }

}


//���PWM���´���

void motor_PWN_updat(void)
{
// /* ceshidianji1++;
//    if(motor_pwm_out<DIANJI_PWM_min)motor_pwm_out=DIANJI_PWM_min;              //ռ�ձ��޶� -3000
//    if(motor_pwm_out>DIANJI_PWM_max)motor_pwm_out=DIANJI_PWM_max;     */        //ռ�ձ��޶� 3000
///*---------------------------------*/
//    //ȷ�������ת������
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
    //���PWM��ֵ

      FTM_PWM_Duty(FTM0,FTM_CH0,Balance_Pwm_R1);//���Ƶ��ת����CH1��Ϊ�㣬CH2Ϊ�㣬��ת  CH1��
      FTM_PWM_Duty(FTM0,FTM_CH1,Balance_Pwm_R2);//���Ƶ��ת����CH2��Ϊ�㣬CH1Ϊ�㣬��ת
      FTM_PWM_Duty(FTM0,FTM_CH2,Balance_Pwm_L1);//duty=1000
      FTM_PWM_Duty(FTM0,FTM_CH3,Balance_Pwm_L2);//   

      

}
//ͣ���ж�

void tingche()
{


      /*ͣ��*/                             //����֤
  if(chusaidao_stop)
  {
      if((image_new_bin[55][10]+image_new_bin[55][20]+image_new_bin[55][30]+image_new_bin[55][40]+image_new_bin[55][50]+
        image_new_bin[40][10]+image_new_bin[40][20]+image_new_bin[40][30]+image_new_bin[40][40]+image_new_bin[40][50])==0)
    {
        stop_num++;         //ͼ�����ȫ��ͣ��
        if(stop_num>10)
        {
          stop_sign=1;
          stop_num=0;
        }
    }
    else
        stop_num=0;
  }

       /*������*/
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
�������̲�
**************************************************************************/
void Kalman_Filter(float Accel,float Gyro)		
{
	angle+=(Gyro - Q_bias) * dt; //?��?��1��??
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-?��?��1��???��2?D-��?2?��??�顤?

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-?��?��1��???��2?D-��?2??�顤?��??y��?
	PP[0][1] += Pdot[1] * dt;   // =?��?��1��???��2?D-��?2?
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-?��?��1��??
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //o��?��1��???��2?D-��?2?
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //o��?��1��??
	Q_bias	+= K_1 * Angle_err;	 //o��?��1��??
	angle_dot   = Gyro - Q_bias;	 //��?3??��(o��?��1��??)��??�顤?=???��?��
        
        Angle_Balance=angle;                                   //?��D???oa????
	Gyro_Turn=GYRO_Z;                                      //?��D?��a?��???��?��
	Acceleration_Z=ACC_Z; 
}

/**************************************************************************
ȷ����ֵ��PWM��ֵ
**************************************************************************/
void Set_Pwm(int Balance_Pwm_L_d,int Balance_Pwm_R_d)
{
    	if(Balance_Pwm_L<0)	Balance_Pwm_L1=Balance_Pwm_L_d,	        Balance_Pwm_L2=0;
	else 	                Balance_Pwm_L1=0,			Balance_Pwm_L2=Balance_Pwm_L_d;
	if(Balance_Pwm_R<0)	Balance_Pwm_R1=Balance_Pwm_R_d,		Balance_Pwm_R2=0;
	else                    Balance_Pwm_R1=0,		        Balance_Pwm_R2=Balance_Pwm_R_d;

}

/**************************************************************************
�޷�
**************************************************************************/
void Xianfu_Pwm(void)
{	
	int Amplitude=1000;    //===PWM?��������?7200 ?T???��6900
        if(Balance_Pwm_L>Amplitude) Balance_Pwm_L=Amplitude;	
	if(Balance_Pwm_R>Amplitude) Balance_Pwm_R=Amplitude;	
	if(Balance_Pwm_R<-Amplitude) Balance_Pwm_R=-Amplitude;
        if(Balance_Pwm_L<-Amplitude) Balance_Pwm_L=-Amplitude;
}


/**************************************************************************
�ܿ���			 
**************************************************************************/
void balance_control(void) 
{    
              //qd_result_L =FTM_AB_Get(FTM1);     //////��������
             // qd_result_R =-FTM_AB_Get(FTM2); 
  	  	Update9AX();          //////��ȡ��̬   ���
                
                Gyro_Balance=-GYRO_Y;

                Kalman_Filter(ACC_Y,-GYRO_Y);
		
 		Balance_Pwm = balance(Angle_Balance,Gyro_Balance);                  /////ƽ��PID����   ���
//		  Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);                /////�ٶ�PID����   ���
//                Turn_Pwm=turn(Encoder_Left,Encoder_Right,Gyro_Turn);                /////λ��PID����   ���
 		 // Moto1=Balance_Pwm-Velocity_Pwm+Turn_Pwm;                          
 	  	//Moto2=Balance_Pwm-Velocity_Pwm-Turn_Pwm;                            /////��������pwm  ���
   		Balance_Pwm_L = Balance_Pwm_R =  Balance_Pwm;                                                
		Xianfu_Pwm();
                Set_Pwm(Balance_Pwm_L,Balance_Pwm_R);
// 		motor_PWN_updat();                                               /////�趨pwmֵ    ���  
} 

/**************************************************************************
ֱ��PD
�Ƕȡ����ٶ�
**************************************************************************/
int balance(float Angle,float Gyro)
{  
         float Bias,kp=-300,kd=1;
	 int balance_1;
	 Bias=Angle-ZHONGZHI;       /////����
	 balance_1=(int)(kp*Bias+Gyro*kd);   
	 return balance_1;
}

/**************************************************************************
�ٶ�PI
���ұ�����
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
	////	if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   Encoder_Integral=0;      �رյ����ɾ������
	  return Velocity;
}
*/
/**************************************************************************
ת�����  �޸�ת���ٶȣ��޸�Turn_Amplitude
���ұ�����   z������
**************************************************************************/
/*int turn(int encoder_left,int encoder_right,float gyro)///////ת�����
{
	 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude=88/Flag_sudu,Kp=42,Kd=0;     
	  //============ң��������ת=======================//
  	if(1==Flag_Left||1==Flag_Right)                      //?a��?2?��??�¨�a��??��?YDy��a?���??��?���̡�???��?����??e��??��?����????��D?3�̦�?������|D?
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
	
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===��a?��?��?��?T����
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5;        
		else Kd=0;   //��a?����?����o����???����?Y��?��??��?y ��D��??��oyPID��?????
  	//=============ת��PD======================//
		Turn=-Turn_Target*Kp -gyro*Kd;                 //���z�������ǽ���PD����
	  return Turn;
}
*/

/**************************************************************************
o����y1|?����o����3��1?��?��??��
��??��2?��y��o????o����??1
����??  ?�̡�o1��o����3��  0��o?y3��
**************************************************************************/
/*u8 Turn_Off(float angle, int voltage)
{
	    u8 temp;
			if(angle<-40||angle>40||1==Flag_Stop||voltage<1110)//��?3?��??1�̨�����11.1V1?��?��??��
			{	                                                 //===????�䨮����40?��1?��?��??��
				temp=1;                                            //===Flag_Stop??11?��?��??��
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
o����y1|?����o??��????�� ��y????����?-1y?��??��?�̡�D���???��?3�ꨤ��?? 
��??��2?��y��o??��????����???���� 1��oDMP  2��o?��???�� 3��o?��21??2��
����??  ?�̡�o?T
**************************************************************************/
/*void Get_Angle(u8 way)
{ 
	    float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_Z;
	   	Temperature=Read_Temperature();      //===?����?MPU6050?��?????����??D?�¨�y?Y��??��??������??�¡�????��?��
	    if(way==1)                           //===DMP��??����??����y?Y2��?��?D??����D?��?����o����?????��??-����D����a?��
			{	
					Read_DMP();                      //===?����??��?��?��?��???��?��?��????
					Angle_Balance=Pitch;             //===?��D???oa????
					Gyro_Balance=gyro[1];            //===?��D???oa???��?��
					Gyro_Turn=gyro[2];               //===?��D?��a?��???��?��
				  Acceleration_Z=accel[2];         //===?��D?Z?��?��?��?��??
			}			
      else
      {
			Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //?����?Y?������?Y��?
			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //?����?Z?������?Y��?
		  Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //?����?X?��?��?��?��??
	  	Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //?����?Z?��?��?��?��??
		  if(Gyro_Y>32768)  Gyro_Y-=65536;                       //��y?Y����D����a??  ��2?������1yshort????����D����a??
			if(Gyro_Z>32768)  Gyro_Z-=65536;                       //��y?Y����D����a??
	  	if(Accel_X>32768) Accel_X-=65536;                      //��y?Y����D����a??
		  if(Accel_Z>32768) Accel_Z-=65536;                      //��y?Y����D����a??
			Gyro_Balance=-Gyro_Y;                                  //?��D???oa???��?��
	   	Accel_Y=atan2(Accel_X,Accel_Z)*180/PI;                 //????????	
		  Gyro_Y=Gyro_Y/16.4;                                    //����?Y��?��?3����a??	
      if(Way_Angle==2)		  	Kalman_Filter(Accel_Y,-Gyro_Y);//?��???��??2��	
			else if(Way_Angle==3)   Yijielvbo(Accel_Y,-Gyro_Y);    //?��21??2��
	    Angle_Balance=angle;                                   //?��D???oa????
			Gyro_Turn=Gyro_Z;                                      //?��D?��a?��???��?��
			Acceleration_Z=Accel_Z;                                //===?��D?Z?��?��?��?��??	
		}
}*/
/**************************************************************************
o����y1|?����o?????��o����y
��??��2?��y��oint
����??  ?�̡�ounsigned int
**************************************************************************/
/*int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}*/