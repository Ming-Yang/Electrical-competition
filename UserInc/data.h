#ifndef _DATA_H
#define _DATA_H
#include "userinc.h"
#include "mpu6050_process.h"
#include "PIDController.h"

typedef struct
{
  float x;
  float y;
}POINT_STRUCT;

//typedef struct 
//{
//  int32_t raw;
//  int32_t acc_roll;
//  uint16_t lines;
//  float ang_v;
//  float line_v;
//}DECODER_STRUCT;

typedef struct 
{
//  MPU6050_DATA_STRUCT mpu6050;
//  MPU6050_EULER_STRUCT gy25_euler;
//  MPU6050_EULER_STRUCT gy25_euler_last;
//  MPU6050_EULER_STRUCT global_euler;
//  MPU6050_EULER_STRUCT global_euler_last;
//  DECODER_STRUCT decoder1;
//  DECODER_STRUCT decoder2;
  POINT_STRUCT ball_position;
  POINT_STRUCT last_position;
  float speed;
}DATA_IN_STRUCT;

typedef struct 
{
  uint32_t channel1;
  uint32_t channel2;
  uint32_t channel3;
  uint32_t channel4;
}CHANNEL_STRUCT;

typedef struct{
  uint32_t frequency;
  uint8_t direction;
  uint8_t pre_direction;
  float lenth_mm;
  float set_lenth_mm;
  float diff_lenth_mm;
  int32_t speed;
  uint32_t angle;
  uint32_t enable;
}STEP_MOTER_STRUCT;

typedef struct 
{
//  CHANNEL_STRUCT tim2;
//  CHANNEL_STRUCT tim3;
//  MPU6050_EULER_STRUCT euler;
//  float speed;
  STEP_MOTER_STRUCT step_motor1;
  STEP_MOTER_STRUCT step_motor2;
}DATA_OUT_STRUCT;

typedef struct
{
  POINT_STRUCT center;
  POINT_STRUCT corner[4];
  POINT_STRUCT dot[10];//0,1~9
  POINT_STRUCT sub_dot[5];
  POINT_STRUCT transfer;
}AXIS_STRUCT;

extern DATA_IN_STRUCT indata;
extern DATA_OUT_STRUCT outdata;
extern MPU6050_PHYSICAL_STRUCT mpu6050_offset;
extern AXIS_STRUCT camera_axis;
extern AXIS_STRUCT board_axis;

void DataInput();
void DataSave();
void DataOutput();
void DataNoPut();
void DataProcess();


#endif

