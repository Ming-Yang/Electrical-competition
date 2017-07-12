#include "data.h"
#include "OLEDUI.h"
#include "SDFatfs.h"

DATA_IN_STRUCT indata;

void DataInput()
{
  
  
  
  MPU6050_GetData(&indata.mpu6050);
}

void DataProcess()
{
  
  
 
}

void DataOutput()
{
  
  

}

void DataSave()
{
  SendOscilloscope();
  
  DataWriteFatfs();
  
}