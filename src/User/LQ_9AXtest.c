#include "include.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
//IIC1   SCLk    J11
//IIC1   SDA     K11



/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����  �ߡ�LQ-005
������˵������ȡ����ԭʼ����
������汾��V1.0
�������¡�2019��03��13�� 
����������
������ֵ���� 
������ֵ���� 
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Sensor_ReadData(int16_t DAT[9])
{
    fxos_data_t fxos_data;
    if (FXOS_ReadSensorData(&g_fxosHandle, &fxos_data) != kStatus_Success)             //��ȡԭʼ����
    {
        printf("Failed to read acceleration data!\r\n");
    }

    DAT[0] = (int16_t)(((uint16_t)fxos_data.accelXMSB << 8) | fxos_data.accelXLSB);     //���ٶȼ�14λ�ģ�����λӰ�첻��ֱ�Ӱ�16λ����
    DAT[1] = (int16_t)(((uint16_t)fxos_data.accelYMSB << 8) | fxos_data.accelYLSB);
    DAT[2] = (int16_t)(((uint16_t)fxos_data.accelZMSB << 8) | fxos_data.accelZLSB);
    DAT[3] = (int16_t)(((uint16_t)fxos_data.magXMSB << 8) | fxos_data.magXLSB);         //�شż�16λ��
    DAT[4] = (int16_t)(((uint16_t)fxos_data.magYMSB << 8) | fxos_data.magYLSB);
    DAT[5] = (int16_t)(((uint16_t)fxos_data.magZMSB << 8) | fxos_data.magZLSB);

    
    fxos2100_data_t  fxos2100_data;
    if (FXOS2100_ReadSensorData(&g_fxosHandle, &fxos2100_data) != kStatus_Success)      //��ȡԭʼ����
    {
        printf("Failed to read acceleration data!\r\n");
    }
    DAT[6] = (int16_t)(((uint16_t)fxos2100_data.gyroXMSB << 8) | fxos2100_data.gyroXLSB);//���ٶȼ� 16λ��
    DAT[7] = (int16_t)(((uint16_t)fxos2100_data.gyroYMSB << 8) | fxos2100_data.gyroYLSB);
    DAT[8] = (int16_t)(((uint16_t)fxos2100_data.gyroZMSB << 8) | fxos2100_data.gyroZLSB);
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����  �ߡ�LQ-005
������˵������ʼ�� ����ģ��
������汾��V1.0
�������¡�2019��03��19�� 
����������
������ֵ����
������ֵ���� 
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_init9AX(void)
{
    uint8_t regResult = 0;
    LPI2C_Init(LPI2C1, 400000);                     //֧��400K I2C

    g_fxosHandle.xfer.slaveAddress = 0x1e;          //8700��ַ
    if (FXOS_ReadReg(&g_fxosHandle, WHO_AM_I_REG, &regResult, 1) == kStatus_Success) 
    {
        if(regResult == kFXOS_WHO_AM_I_Device_ID)   //��ȡWHO_AM_I �Ĵ�������������ȷ����֤��I2C��ַ��ȷ
        {
             printf("\r\n FX8700 is OK!");
        }
    }
    else                                            //û���ҵ�FX8700
    {
            printf("\r\n FX8700 is Fail!");

    }

    if (FXOS_Init(&g_fxosHandle) != kStatus_Success) //FX8700 ��ʼ��
    {
        return;
    } 
    
    g_fxosHandle.xfer.slaveAddress = 0x20;           //2100��ַ
    if (FXOS_ReadReg(&g_fxosHandle, FXAS21002_WHO_AM_I, &regResult, 1) == kStatus_Success) 
    {
        if(regResult == FXAS21002_WHO_AM_I_VALUE)    //��ȡWHO_AM_I �Ĵ�������������ȷ����֤��I2C��ַ��ȷ
        {
             printf("\r\n FX2100 is OK!");
        }
    }
    else                                             //û���ҵ�FX2100
    {
            printf("\r\n FX2100 is Fail!");

    }

    if (Init2100(&g_fxosHandle) != kStatus_Success)   //FX2100 ��ʼ��
    {
        return;
    }
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����  �ߡ�LQ-005
������˵��������ģ���������
������汾��V1.0
�������¡�2019��03��19�� 
����������
������ֵ����
������ֵ���� 
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifdef TFT1_8
void Test_9AX(void)
{
    int16_t u16data[9];          //��Ŷ�ȡ������ ����
    TFTSPI_Init(1);              //TFT1.8��ʼ��  
    TFTSPI_CLS(u16BLUE);         //����
    LQ_init9AX();                //�����ʼ��
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
      delayms(100);//��ʱ100����   
    } /* End infinite loops */
}
#endif
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����  �ߡ�LQ-005
������˵��������ģ���������
������汾��V1.0
�������¡�2019��03��19�� 
����������
������ֵ����
������ֵ���� 
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifdef OLED
void Test_9AX(void)
{
    int16_t u16data[9];          //��Ŷ�ȡ������ ����
    LCD_Init();                  //LCD��ʼ��
    LCD_CLS();                   //LCD����
    LCD_P8x16Str(15,0,"LQ 9AX Test"); 
    LQ_init9AX();                //�����ʼ��
    
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
      delayms(100);//��ʱ100����   
    } /* End infinite loops */
}
#endif
