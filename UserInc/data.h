#ifndef _DATA_H
#define _DATA_H
#include "userinc.h"
#include "mpu6050_process.h"
#include "PIDController.h"

typedef struct 
{
  int32_t raw;
  int32_t acc_roll;
  uint16_t lines;
  float ang_v;
  float line_v;
}DECODER_STRUCT;

typedef struct 
{
  MPU6050_DATA_STRUCT mpu6050;
  DECODER_STRUCT decoder1;
  DECODER_STRUCT decoder2;
  int16_t adc10;
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
  MPU6050_EULER_STRUCT euler;
  MPU6050_EULER_STRUCT gy25_euler;
  MPU6050_EULER_STRUCT gy25_euler_last;
  float speed;
  int pwm;
}DATA_OUT_STRUCT;


extern DATA_IN_STRUCT indata;
extern DATA_OUT_STRUCT outdata;
extern MPU6050_PHYSICAL_STRUCT mpu6050_offset;

void DataInput();
void DataSave();
void DataOutput();
void DataNoPut();
void DataProcess();


#endif

