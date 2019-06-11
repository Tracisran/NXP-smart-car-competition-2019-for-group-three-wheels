#ifndef INCLUDE_H_
#define INCLUDE_H_

/*
 * 包含Cortex-M内核的通用头文件
 */
    #include    <stdio.h>                       //printf
    #include    <string.h>                      //memcpy
    #include    <stdlib.h>                      //malloc

    #include "common.h"
    #include "MK66F18.h"   //寄存器映像头文件
    #include "arm_math.h "
    #include "Systick.h"
    #include "GPIO.h"
    #include "PLL.h"
    #include "FTM.h"
    #include "LED.h"
    #include "KEY.h"
    #include "LQ12864.h"
    #include "UART.h"
    #include "ADC.h"
    #include "PLL.h"
    #include "vectors.h"
    #include "PIT.h"
    #include "I2C.h"
    #include "Serial_oscilloscope.h"
    #include "DMA.h"
    #include "Lptmr.h"
    #include "RTC.h"
    #include "LQ9AX.h"
    #include "OV7620.h"
    #include "GPIO_Cfg.h"
    #include "u_iic.h"
    #include "LQMPU6050.h"
    #include "LQMT9V034.h"

/***********自加头文件************/
//    #include "camaro.h"
//    #include "control.h"
    #include "MPU9250.h"
    #include "ANO_DT.h"




#endif