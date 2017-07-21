#ifndef _DATA_H
#define _DATA_H
#include "userinc.h"

typedef struct 
{
  MPU6050_DATA_STRUCT mpu6050;
}DATA_IN_STRUCT;

typedef struct 
{
  uint32_t channel1;
  uint32_t channel2;
  uint32_t channel3;
  uint32_t channel4;
}CHANNEL_STRUCT;

typedef struct 
{
  CHANNEL_STRUCT tim2;
  CHANNEL_STRUCT tim3;
  
}DATA_OUT_STRUCT;


extern DATA_IN_STRUCT indata;
extern MPU6050_EULER_STRUCT euler;
extern DATA_OUT_STRUCT outdata;
void DataInput();
void DataSave();
void DataOutput();
void DataNoPut();
void DataProcess();



#endif

