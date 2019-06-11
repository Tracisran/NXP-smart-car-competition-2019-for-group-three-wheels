#include "include.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
//IIC1   SCLk    J11
//IIC1   SDA     K11



/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【作  者】LQ-005
【功能说明】获取九轴原始数据
【软件版本】V1.0
【最后更新】2019年03月13日 
【函数名】
【返回值】无 
【参数值】无 
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Sensor_ReadData(int16_t DAT[9])
{
    fxos_data_t fxos_data;
    if (FXOS_ReadSensorData(&g_fxosHandle, &fxos_data) != kStatus_Success)             //读取原始数据
    {
        printf("Failed to read acceleration data!\r\n");
    }

    DAT[0] = (int16_t)(((uint16_t)fxos_data.accelXMSB << 8) | fxos_data.accelXLSB);     //加速度计14位的，低两位影响不大，直接按16位的用
    DAT[1] = (int16_t)(((uint16_t)fxos_data.accelYMSB << 8) | fxos_data.accelYLSB);
    DAT[2] = (int16_t)(((uint16_t)fxos_data.accelZMSB << 8) | fxos_data.accelZLSB);
    DAT[3] = (int16_t)(((uint16_t)fxos_data.magXMSB << 8) | fxos_data.magXLSB);         //地磁计16位的
    DAT[4] = (int16_t)(((uint16_t)fxos_data.magYMSB << 8) | fxos_data.magYLSB);
    DAT[5] = (int16_t)(((uint16_t)fxos_data.magZMSB << 8) | fxos_data.magZLSB);

    
    fxos2100_data_t  fxos2100_data;
    if (FXOS2100_ReadSensorData(&g_fxosHandle, &fxos2100_data) != kStatus_Success)      //读取原始数据
    {
        printf("Failed to read acceleration data!\r\n");
    }
    DAT[6] = (int16_t)(((uint16_t)fxos2100_data.gyroXMSB << 8) | fxos2100_data.gyroXLSB);//角速度计 16位的
    DAT[7] = (int16_t)(((uint16_t)fxos2100_data.gyroYMSB << 8) | fxos2100_data.gyroYLSB);
    DAT[8] = (int16_t)(((uint16_t)fxos2100_data.gyroZMSB << 8) | fxos2100_data.gyroZLSB);
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【作  者】LQ-005
【功能说明】初始化 九轴模块
【软件版本】V1.0
【最后更新】2019年03月19日 
【函数名】
【返回值】无
【参数值】无 
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_init9AX(void)
{
    uint8_t regResult = 0;
    LPI2C_Init(LPI2C1, 400000);                     //支持400K I2C

    g_fxosHandle.xfer.slaveAddress = 0x1e;          //8700地址
    if (FXOS_ReadReg(&g_fxosHandle, WHO_AM_I_REG, &regResult, 1) == kStatus_Success) 
    {
        if(regResult == kFXOS_WHO_AM_I_Device_ID)   //读取WHO_AM_I 寄存器，如果结果正确，则证明I2C地址正确
        {
             printf("\r\n FX8700 is OK!");
        }
    }
    else                                            //没有找到FX8700
    {
            printf("\r\n FX8700 is Fail!");

    }

    if (FXOS_Init(&g_fxosHandle) != kStatus_Success) //FX8700 初始化
    {
        return;
    } 
    
    g_fxosHandle.xfer.slaveAddress = 0x20;           //2100地址
    if (FXOS_ReadReg(&g_fxosHandle, FXAS21002_WHO_AM_I, &regResult, 1) == kStatus_Success) 
    {
        if(regResult == FXAS21002_WHO_AM_I_VALUE)    //读取WHO_AM_I 寄存器，如果结果正确，则证明I2C地址正确
        {
             printf("\r\n FX2100 is OK!");
        }
    }
    else                                             //没有找到FX2100
    {
            printf("\r\n FX2100 is Fail!");

    }

    if (Init2100(&g_fxosHandle) != kStatus_Success)   //FX2100 初始化
    {
        return;
    }
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【作  者】LQ-005
【功能说明】九轴模块测试例程
【软件版本】V1.0
【最后更新】2019年03月19日 
【函数名】
【返回值】无
【参数值】无 
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifdef TFT1_8
void Test_9AX(void)
{
    int16_t u16data[9];          //存放读取传感器 数据
    TFTSPI_Init(1);              //TFT1.8初始化  
    TFTSPI_CLS(u16BLUE);         //清屏
    LQ_init9AX();                //九轴初始化
    TFTSPI_P8X16Str(2,0,"LQ 9AX Test",u16RED,u16BLUE);
    printf("\r\nLQ 9AX Test");
    char txt[16];
    for (;;)
    {       
      printf("\r\n\r\n ");

      Sensor_ReadData(u16data); // Read sensor data  

      sprintf(txt,"AX:  %5d ",(int16_t)u16data[0]); 
      TFTSPI_P8X8Str(0,2,(uint8_t*)txt,u16RED,u16BLUE);
      sprintf(txt,"AY:  %5d ",(int16_t)u16data[1]); 
      TFTSPI_P8X8Str(0,3,(uint8_t*)txt,u16RED,u16BLUE); 
      sprintf(txt,"AZ:  %5d ",(int16_t)u16data[2]); 
      TFTSPI_P8X8Str(0,4,(uint8_t*)txt,u16RED,u16BLUE);
      sprintf(txt,"MX:  %5d ",(int16_t)u16data[3]);  
      TFTSPI_P8X8Str(0,5,(uint8_t*)txt,u16RED,u16BLUE);
      sprintf(txt,"MY:  %5d ",(int16_t)u16data[4]); 
      TFTSPI_P8X8Str(0,6,(uint8_t*)txt,u16RED,u16BLUE);
      sprintf(txt,"MZ:  %5d ",(int16_t)u16data[5]);
      TFTSPI_P8X8Str(0,7,(uint8_t*)txt,u16RED,u16BLUE);
      sprintf(txt,"GX:  %5d ",(int16_t)u16data[6]);  
      TFTSPI_P8X8Str(0,8,(uint8_t*)txt,u16RED,u16BLUE);
      sprintf(txt,"GY:  %5d ",(int16_t)u16data[7]);
      TFTSPI_P8X8Str(0,9,(uint8_t*)txt,u16RED,u16BLUE); 
      sprintf(txt,"GZ:  %5d ",(int16_t)u16data[8]); 
      TFTSPI_P8X8Str(0,10,(uint8_t*)txt,u16RED,u16BLUE);
      printf("\r\nAX: %d  ",(int16_t)u16data[0]); 
      printf("\r\nAY: %d  ",(int16_t)u16data[1]);
      printf("\r\nAZ: %d  ",(int16_t)u16data[2]); 
      printf("\r\nMX: %d  ",(int16_t)u16data[3]);
      printf("\r\nMY: %d  ",(int16_t)u16data[4]); 
      printf("\r\nMZ: %d  ",(int16_t)u16data[5]);
      printf("\r\nGX: %d  ",(int16_t)u16data[6]);
      printf("\r\nGY: %d  ",(int16_t)u16data[7]); 
      printf("\r\nGZ: %d  ",(int16_t)u16data[8]);
      delayms(100);//延时100毫秒   
    } /* End infinite loops */
}
#endif
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【作  者】LQ-005
【功能说明】九轴模块测试例程
【软件版本】V1.0
【最后更新】2019年03月19日 
【函数名】
【返回值】无
【参数值】无 
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifdef OLED
void Test_9AX(void)
{
    int16_t u16data[9];          //存放读取传感器 数据
    LCD_Init();                  //LCD初始化
    LCD_CLS();                   //LCD清屏
    LCD_P8x16Str(15,0,"LQ 9AX Test"); 
    LQ_init9AX();                //九轴初始化
    
    char txt[16];
    for (;;)
    {       
      printf("\r\n\r\n ");

      Sensor_ReadData(u16data); // Read sensor data  

      sprintf(txt,"AX:%5d ",(int16_t)u16data[0]); 
      LCD_P6x8Str(0,2,txt);
      sprintf(txt,"AY:%5d ",(int16_t)u16data[1]); 
      LCD_P6x8Str(0,3,txt); 
      sprintf(txt,"AZ:%5d ",(int16_t)u16data[2]); 
      LCD_P6x8Str(0,4,txt);
      sprintf(txt,"MX:%5d ",(int16_t)u16data[3]);  
      LCD_P6x8Str(0,5,txt);
      sprintf(txt,"MY:%5d ",(int16_t)u16data[4]); 
      LCD_P6x8Str(0,6,txt);
      sprintf(txt,"MZ:%5d ",(int16_t)u16data[5]);
      LCD_P6x8Str(0,7,txt);
      sprintf(txt,"GX:%5d ",(int16_t)u16data[6]);  
      LCD_P6x8Str(60,5,txt);
      sprintf(txt,"GY:%5d ",(int16_t)u16data[7]);
      LCD_P6x8Str(60,6,txt); 
      sprintf(txt,"GZ:%5d ",(int16_t)u16data[8]); 
      LCD_P6x8Str(60,7,txt);
      printf("\r\nAX: %d  ",(int16_t)u16data[0]); 
      printf("\r\nAY: %d  ",(int16_t)u16data[1]);
      printf("\r\nAZ: %d  ",(int16_t)u16data[2]); 
      printf("\r\nMX: %d  ",(int16_t)u16data[3]);
      printf("\r\nMY: %d  ",(int16_t)u16data[4]); 
      printf("\r\nMZ: %d  ",(int16_t)u16data[5]);
      printf("\r\nGX: %d  ",(int16_t)u16data[6]);
      printf("\r\nGY: %d  ",(int16_t)u16data[7]); 
      printf("\r\nGZ: %d  ",(int16_t)u16data[8]);
      delayms(100);//延时100毫秒   
    } /* End infinite loops */
}
#endif
