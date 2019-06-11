/******************** LQ_K60_函数库 v1.0 ********************
 * 文件名           ：LQ9AX.c
 * 功能             ：设置IIC口工作模式
 * 备注             ：官方例程上修改
 * 日期             ：2016-01-23
 * 实验平台         ：龙丘 k66F18核心板VG2
 * 开发环境         ：IAR 7.3及以上
 * 作者             ：CHIUSIR
 * 淘宝店           ：https://longqiu.taobao.com
 * 龙丘智能车讨论群 ：202949437
*************************************************************/

#include "include.h"
void delay__ms(u8 ms)
{
  u16 i;
  while(ms--)
  {
    for(i=0;i<52000;i++);
  }
}

void Init_LQ_9AX(void)
{
    I2C_Init(I2C_1);                       //初始化I2C1 
    
///////FXAS21002//////////////////////////////////////////////////////////////////////////////////////////
    	// write 0000 0000 = 0x00 to CTRL_REG1 to place FXOS21002 in Standby
	// [7]: ZR_cond=0
	// [6]: RST=0
	// [5]: ST=0 self test disabled
	// [4-2]: DR[2-0]=000 for 800Hz
	// [1-0]: Active=0, Ready=0 for Standby mode
    I2C_WriteAddr(I2C_1, FXAS21002_I2C_ADDR, FXAS21002_CTRL_REG1, 0x00); 
        // write 0000 0000 = 0x00 to CTRL_REG0 to configure range and filters
	// [7-6]: BW[1-0]=00, LPF disabled
	// [5]: SPIW=0 4 wire SPI (irrelevant)
	// [4-3]: SEL[1-0]=00 for 10Hz HPF at 200Hz ODR
	// [2]: HPF_EN=0 disable HPF
	// [1-0]: FS[1-0]=00 for 1600dps (TBD CHANGE TO 2000dps when final trimmed parts available)
    I2C_WriteAddr(I2C_1, FXAS21002_I2C_ADDR, FXAS21002_CTRL_REG0, 0x00);     
    delay__ms(100);  
    	// write 0000 0001 = 0x01 to CTRL_REG1 to configure 800Hz ODR and enter Active mode
	// [7]: ZR_cond=0
	// [6]: RST=0
	// [5]: ST=0 self test disabled
	// [4-2]: DR[2-0]=000 for 800Hz ODR
	// [1-0]: Active=1, Ready=0 for Active mode
    I2C_WriteAddr(I2C_1, FXAS21002_I2C_ADDR, FXAS21002_CTRL_REG1, 0x03);
   
//////FXOS8700///////////////////////////////////////////////////////////////////////////////////////////
    delay__ms(100); 
	// write 0000 0000 = 0x00 to CTRL_REG1 to place FXOS8700 into standby
	// [7-1] = 0000 000
	// [0]: active=0    
    I2C_WriteAddr(I2C_1, FXOS8700_I2C_ADDR, FXOS8700_CTRL_REG1, 0x00);
    delay__ms(10);
    	// write 0001 1111 = 0x1F to M_CTRL_REG1
	// [7]: m_acal=0: auto calibration disabled
	// [6]: m_rst=0: one-shot magnetic reset disabled
	// [5]: m_ost=0: one-shot magnetic measurement disabled
	// [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
	// [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active
    I2C_WriteAddr(I2C_1, FXOS8700_I2C_ADDR, FXOS8700_M_CTRL_REG1, 0x1F);
    delay__ms(10);
    	// write 0010 0000 = 0x20 to magnetometer control register 2
	// [7]: reserved
	// [6]: reserved
	// [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the accelerometer registers
	// [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
	// [3]: m_maxmin_dis_ths=0
	// [2]: m_maxmin_rst=0
	// [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
    I2C_WriteAddr(I2C_1, FXOS8700_I2C_ADDR, FXOS8700_M_CTRL_REG2, 0x20); 
    delay__ms(10);
    // [1-0]: fs=00 for 2g mode: 2048 counts / g = 8192 counts / g after 2 bit left shift
    I2C_WriteAddr(I2C_1, FXOS8700_I2C_ADDR, FXOS8700_XYZ_DATA_CFG, 0x00); 
    delay__ms(10);
    	// write 0000 0010 = 0x02 to CTRL_REG2 to set MODS bits
	// [7]: st=0: self test disabled
	// [6]: rst=0: reset disabled
	// [5]: unused
	// [4-3]: smods=00
	// [2]: slpe=0: auto sleep disabled
	// [1-0]: mods=10 for high resolution (maximum over sampling)
    I2C_WriteAddr(I2C_1, FXOS8700_I2C_ADDR, FXOS8700_CTRL_REG2, 0x02); 
    delay__ms(10);
    	// write 0000 0101 = 0x05 to accelerometer control register 1
	// [7-6]: aslp_rate=00
	// [5-3]: dr=000=0 for 800Hz data rate (when in hybrid mode)
	// [2]: lnoise=1 for low noise mode (since we're in 2g mode)
	// [1]: f_read=0 for normal 16 bit reads
	// [0]: active=1 to take the part out of standby and enable sampling
   
    I2C_WriteAddr(I2C_1, FXOS8700_I2C_ADDR, FXOS8700_CTRL_REG1, 0x05); 
    delay__ms(10); 
}
void IIC_Read(unsigned char slave,unsigned char * p, u16 * q)
{
  int Zero=0x0000;

  p[0]  =    I2C_ReadAddr(I2C_0,slave,OUT_X_MSB_REG);
  Pause();
  p[1]  =    I2C_ReadAddr(I2C_0,slave,OUT_X_LSB_REG);
  Pause();
  p[2]  =    I2C_ReadAddr(I2C_0,slave,OUT_Y_MSB_REG);
  Pause();
  p[3]  =    I2C_ReadAddr(I2C_0,slave,OUT_Y_LSB_REG);
  Pause();
  p[4]  =    I2C_ReadAddr(I2C_0,slave,OUT_Z_MSB_REG);
  Pause();
  p[5]  =    I2C_ReadAddr(I2C_0,slave,OUT_Z_LSB_REG);
  Pause();


  q[0] = (((Zero | p[0])<<8)|p[1]);
  q[1] = (((Zero | p[2])<<8)|p[3]);
  q[2] = (((Zero | p[4])<<8)|p[5]); 
}

    void Init2100()
{
        I2C_WriteAddr(I2C_0,FXAS21002_I2C_ADDR,FXAS21002_CTRL_REG0,0x02);
        delay__ms(100); 
        I2C_WriteAddr(I2C_0,FXAS21002_I2C_ADDR,FXAS21002_CTRL_REG1,0x02);

       
}
    void Init8700()
{

     //  I2C_WriteAddr(I2C1,SlaveAddress8700,0x0f,0x33);

     //   Pause();

        I2C_WriteAddr(I2C_0,FXOS8700_I2C_ADDR,FXOS8700_CTRL_REG1,0x05);

        delay__ms(100); 
}