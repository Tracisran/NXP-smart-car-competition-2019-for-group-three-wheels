/*******************************************************************************
【平    台】龙邱KV58F24智能车VD母板
【编    写】CHIUSIR
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2017年12月15日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】IAR7.80.4及以上版本
【Target  】MKV58F1M0VLQ24
【Crystal 】 50.000Mhz
【busclock】137.500MHz
【pllclock】275.000MHz

本程序尚在调试中，用户可以在次基础上修改--2018/11/20
如果你能调试并共享给大家，相信赠人玫瑰手留余香，大家都会感谢你的！！！

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"
//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
uint8_t MPU6050_Init(void)
{
    uint8_t res;
    IIC_Init();     ////MPU6050 支持400K I2C
    res=MPU_Read_Byte(MPU6050_ADDR,WHO_AM_I);           //读取MPU6050的ID
    /*if(res!=MPU6050_ID) //器件ID正确
    {
        printf("ID=%#X\r\n",res);
        printf("MPU6050 is fail!\n");
    }
    else  printf("MPU6050 is OK!\n");*/

    res = 0;
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU6050
    time_delay_ms(100);  //延时100ms
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X00);//唤醒MPU6050
    res += MPU_Set_Gyro_Fsr(3);					        	//陀螺仪传感器,±2000dps   
    res += MPU_Set_Accel_Fsr(1);					       	 	//加速度传感器,±4g
    res += MPU_Set_Rate(1000);						       	 	//设置采样率1000Hz
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,0x02);      //设置数字低通滤波器   98hz
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
//    MPU_Write_Byte(MPU6050_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO
//    MPU_Write_Byte(MPU6050_ADDR,MPU_INTBP_CFG_REG,0X80);//INT引脚低电平有效
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
    
   /* if(res == 0)  //上面寄存器都写入成功
    {
        printf("MPU set is OK!\n");
    }
    else return 1;*/
    
    return 0;
}



//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU6050_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU6050_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}

//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器  
}

//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU6050_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU6050_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return (short)temp*100;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res; 
	res=MPU_Read_Len(MPU6050_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;
}

//得到加计值、温度值、角速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Raw_data(short *ax,short *ay,short *az,short *gx,short *gy,short *gz)
{
    uint8_t buf[14],res;  
	res=MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
        *gx=((uint16_t)buf[8]<<8)|buf[9];  
		*gy=((uint16_t)buf[10]<<8)|buf[11];  
		*gz=((uint16_t)buf[12]<<8)|buf[13];
	} 	
    return res;
}


//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    return IIC_WriteMultByteToSlave(addr, reg, len, buf);
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    return IIC_ReadMultByteFromSlave(addr, reg, len, buf);     
}


//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t value)
{
    return IIC_WriteByteToSlave(addr, reg, value);
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t value;
    IIC_ReadByteFromSlave(addr, reg, &value);
    return value;
}


extern unsigned short send_data[8];
void Test_MPU6050(void)
{
  int16_t aacx,aacy,aacz;	//加速度传感器原始数据
  int16_t gyrox,gyroy,gyroz;	//陀螺仪原始数据
 // int16_t temp;                          //温度 
  //uint16_t a[8];
  uint8_t  txt[30]="X:";
  MPU6050_Init();                          //初始化MPU6050
  LCD_Init();
  while(1)
  {
//    temp=MPU_Get_Temperature();	//得到温度值
    MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);	//得到加速度传感器数据
    sprintf((char*)txt,"ax:%06d",aacx);
    LCD_P6x8Str(0,0,(u8*)txt);
    sprintf((char*)txt,"ay:%06d",aacy);
    LCD_P6x8Str(0,1,(u8*)txt);
    sprintf((char*)txt,"az:%06d",aacz);
    LCD_P6x8Str(0,2,(u8*)txt);
    sprintf((char*)txt,"gx:%06d",gyrox);
    LCD_P6x8Str(0,3,(u8*)txt);
    sprintf((char*)txt,"gy:%06d",gyroy);
    LCD_P6x8Str(0,4,(u8*)txt);
    sprintf((char*)txt,"gz:%06d",gyroz);
    LCD_P6x8Str(0,5,(u8*)txt);                    //OLED刷新数据
                                                  //串口助手显示
    
/*    send_data[0]=aacx;
    send_data[1]=aacy;
    send_data[2]=aacz;
    send_data[3]=gyrox;
    send_data[4]=gyroy;
    send_data[5]=gyroz;  
    Data_Send(UART_4,send_data); */  //虚拟示波器显示（匿名上位机）
    time_delay_ms(100);
  }
}

