#ifndef _MPU6050_PROCESS_H_
#define _MPU6050_PROCESS_H_
#include "mpu6050.h"

#define CONSTANTS_ONE_G		9.80665f		/* m/s^2		*/
#define M_PI_F 3.1415926

typedef struct
{
  float acc_x;
  float acc_y;
  float acc_z;
  float gyr_x;
  float gyr_y;
  float gyr_z;
  float temp;
}MPU6050_PHYSICAL_STRUCT;

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
}MPU6050_QUATERNION_STRUCT;

typedef struct mpu6050_euler_struct
{
  float roll;
  float pitch;
  float yaw;
}MPU6050_EULER_STRUCT;

extern MPU6050_PHYSICAL_STRUCT mpu6050_offset;

extern void MPU6050_Process(MPU6050_DATA_STRUCT* MPU6050_DATA,
                     MPU6050_PHYSICAL_STRUCT* MPU6050_FILTED,
                     MPU6050_PHYSICAL_STRUCT* MPU6050_OFFSET,
                     MPU6050_EULER_STRUCT* MPU6050_EULER_Rad,
                     MPU6050_EULER_STRUCT* MPU6050_EULER);

extern void InitOffset6050(MPU6050_DATA_STRUCT* MPU6050_DATA,
                    MPU6050_PHYSICAL_STRUCT* MPU6050_OFFSET
                    );
  

#endif /* _MPU6050_PROCESS_H_ */