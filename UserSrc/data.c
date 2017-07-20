#include "data.h"
#include "OLEDUI.h"
#include "SDFatfs.h"
#include "mpu6050_process.h"

DATA_IN_STRUCT indata;
MPU6050_PHYSICAL_STRUCT mpu6050_filted;
MPU6050_PHYSICAL_STRUCT mpu6050_offset;
MPU6050_EULER_STRUCT eulerRad;
MPU6050_EULER_STRUCT euler;

void DataInput()
{
  
  //read raw
  MPU6050_GetData(&indata.mpu6050);
}

void DataProcess()
{
  //×ËÌ¬ÈÚºÏ
  MPU6050_Process(&indata.mpu6050, &mpu6050_filted, &mpu6050_offset, &eulerRad, &euler);
   
}

void DataOutput()
{
  
  

}

void DataSave()
{
  SendOscilloscope();
  
  DataWriteFatfs();
  
}