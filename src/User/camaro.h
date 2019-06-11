#ifndef __CAMR_H__
#define __CAMR_H__
#include "include.h"

void tuxiang_caiji();
void mid_line();            //算中线
void quzhaodian(void);      //去噪点

void ring(void);            //环判断
void buxian(void);          //补线声明
void Slope_Calculate(uint8 begin,uint8 end,int *p);    //最小二乘法
int otsu (unsigned char *image, int rows, int cols, int x0, int y0, int dx, int dy, int vvv);   //大津发
void Image_binaryzation(void);   //软件二值化
void  result_mean_newValues(float dat1);           //编码器滤波
void qd_speed_deal(void);                          //编码器读取
void car_speed_dis(void);                          //速度路程计算
void  LCD_tuxiang(void);

#endif 
