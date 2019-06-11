/******************** LQ_K60_函数库 v1.0 ********************
 * 文件名           ：UART.H
 * 功能             ：设置UART工作模式
 * 备注             ：官方例程上修改
 * 日期             ：2015-10-16
 * 实验平台         ：龙丘 k60DN512Z核心板 
 * 作者             ：龙丘技术 006
 * 淘宝店           ：https://longqiu.taobao.com
 * 龙丘智能车讨论群 ：202949437
*************************************************************/

#ifndef __LQ9AX_H__
#define __LQ9AX_H__
// FXOS8700 registers and constants
#define FXOS8700_I2C_ADDR		0x1E  //0X1E  0X1F
#define FXOS8700_OUT_X_MSB       	0x01
#define FXOS8700_WHO_AM_I      		0x0D
#define FXOS8700_XYZ_DATA_CFG       	0x0E
#define FXOS8700_CTRL_REG1        	0x2A
#define FXOS8700_CTRL_REG2        	0x2B
#define FXOS8700_M_CTRL_REG1         	0x5B
#define FXOS8700_M_CTRL_REG2        	0x5C
#define FXOS8700_WHO_AM_I_VALUE     	0xC7

// FXAS21002 registers and constants
#define FXAS21002_I2C_ADDR		0x20  //0X20  0X21
#define FXAS21002_X_MSB                 0X01
#define FXAS21002_X_LSB                 0X02
#define FXAS21002_Y_MSB                 0X03
#define FXAS21002_Y_LSB                 0X04
#define FXAS21002_Z_MSB                 0X05
#define FXAS21002_Z_LSB                 0X06
#define FXAS21002_DATA_REG            	0x01 
#define FXAS21002_WHO_AM_I        	0x0C 
#define FXAS21002_CTRL_REG0           	0x0D
#define FXAS21002_CTRL_REG1           	0x13
#define FXAS21002_WHO_AM_I_VALUE	0xD4

#define OUT_X_MSB_REG         0x01
#define OUT_X_LSB_REG         0x02
#define OUT_Y_MSB_REG         0x03
#define OUT_Y_LSB_REG         0x04
#define OUT_Z_MSB_REG         0x05
#define OUT_Z_LSB_REG         0x06
typedef union
{
  u16 MYWORD;
  struct
  {
    u8 BYTEL;//低8位
    u8 BYTEH;//高8位，跟CODEWARRIOR大端模式不同
  } MYBYTE;
}LQ9AXt;

/*********************** UART功能函数 **************************/
//初始化
extern void Init_LQ_9AX(void);
void IIC_Read(unsigned char slave,unsigned char * p, u16 * q);
void Init2100();
void Init8700();
/********************************************************************/

#endif 
