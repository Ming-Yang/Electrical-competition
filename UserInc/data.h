#ifndef _DATA_H
#define _DATA_H
#include "userinc.h"

typedef struct 
{
  MPU6050_DATA_STRUCT mpu6050;
}DATA_IN_STRUCT;

extern DATA_IN_STRUCT indata;
extern MPU6050_EULER_STRUCT euler;
void DataInput();
void DataSave();
void DataOutput();
void DataProcess();



#endif

