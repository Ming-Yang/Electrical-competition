#include "mpu6050.h"


volatile s16 MPU6050_ACC_X=0;
volatile s16 MPU6050_ACC_Y=0;
volatile s16 MPU6050_ACC_Z=0;
volatile s16 MPU6050_TEMP=0;
volatile s16 MPU6050_GYR_X=0;
volatile s16 MPU6050_GYR_Y=0;
volatile s16 MPU6050_GYR_Z=0;

static void MPU6050_WriteReg(u8 RegisterAddress, u8 Data);
static u8 MPU6050_ReadReg(u8 RegisterAddress);

/*
* 延时函数
*/
static void MPU6050_Delay()
{
  for(int n=1;n<500;n++);
}

u8 MPU6050_Init(void)
{
  u8 SELF_ID = 0;

  SELF_ID = MPU6050_ReadReg(MPU6050_WHO_AM_I);
  
  MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);
  MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
  MPU6050_WriteReg(MPU6050_CONFIG, 0x04);//低通滤波器带宽5Hz
  MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x08);//不自检   ±500 °/s
  MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x08);//不自检  ±4g
  
  return SELF_ID;
}

/*
*   MPU6050_WriteReg
*   该函数用于配置MPU6050的寄存器
*
*   参数：
*   RegisterAddress 
*    |__ MPU6050寄存器地址
*   Data
*    |__ 具体写入的字节型数据 
*/
void MPU6050_WriteReg(u8 RegisterAddress, u8 Data)
{
  I2C_Write(MPU6050_ADDR,RegisterAddress,&Data,1);
}

/*
*   MPU6050_WriteReg
*   该函数用于读取MPU6050的数据
*
*   参数：
*     RegisterAddress 
*        |__ MPU6050寄存器地址
*   返回值
*      加速传感器的测量值
*/
u8 MPU6050_ReadReg(u8 RegisterAddress)
{
  u8 result;
  I2C_Read(MPU6050_ADDR,RegisterAddress,&result,1);
  MPU6050_Delay();
  return result;
}


void MPU6050_GetData(MPU6050_DATA_STRUCT* MPU6050_DATA)
{
  MPU6050_DATA->acc_x=((s16)MPU6050_ReadReg(MPU6050_ACCEL_XOUT)<<8)
    |(s16)MPU6050_ReadReg(MPU6050_ACCEL_XOUT+1);
  MPU6050_DATA->acc_y=((s16)MPU6050_ReadReg(MPU6050_ACCEL_YOUT)<<8)
    |(s16)MPU6050_ReadReg(MPU6050_ACCEL_YOUT+1);
  MPU6050_DATA->acc_z=((s16)MPU6050_ReadReg(MPU6050_ACCEL_ZOUT)<<8)
    |(s16)MPU6050_ReadReg(MPU6050_ACCEL_ZOUT+1);
  MPU6050_DATA->temp=((s16)MPU6050_ReadReg(MPU6050_TEMP_OUT)<<8)
    |(s16)MPU6050_ReadReg(MPU6050_TEMP_OUT+1);
  MPU6050_DATA->gyr_x=((s16)MPU6050_ReadReg(MPU6050_GYRO_XOUT)<<8)
    |(s16)MPU6050_ReadReg(MPU6050_GYRO_XOUT+1);
  MPU6050_DATA->gyr_y=((s16)MPU6050_ReadReg(MPU6050_GYRO_YOUT)<<8)
    |(s16)MPU6050_ReadReg(MPU6050_GYRO_YOUT+1);
  MPU6050_DATA->gyr_z=((s16)MPU6050_ReadReg(MPU6050_GYRO_ZOUT)<<8)
    |(s16)MPU6050_ReadReg(MPU6050_GYRO_ZOUT+1);
}