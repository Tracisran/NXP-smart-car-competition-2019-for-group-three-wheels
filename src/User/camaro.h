#ifndef __CAMR_H__
#define __CAMR_H__
#include "include.h"

void tuxiang_caiji();
void mid_line();            //������
void quzhaodian(void);      //ȥ���

void ring(void);            //���ж�
void buxian(void);          //��������
void Slope_Calculate(uint8 begin,uint8 end,int *p);    //��С���˷�
int otsu (unsigned char *image, int rows, int cols, int x0, int y0, int dx, int dy, int vvv);   //���
void Image_binaryzation(void);   //�����ֵ��
void  result_mean_newValues(float dat1);           //�������˲�
void qd_speed_deal(void);                          //��������ȡ
void car_speed_dis(void);                          //�ٶ�·�̼���
void  LCD_tuxiang(void);

#endif 
