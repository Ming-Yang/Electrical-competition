#include "data.h"
#include "OLEDUI.h"
#include "SDFatfs.h"
#include "init.h"

DATA_IN_STRUCT indata;
DATA_OUT_STRUCT outdata;

void DataInput()
{
  
  
  
  MPU6050_GetData(&indata.mpu6050);
}

void DataProcess()
{
  
  
 outdata.tim2.channel2 = T/10%100;
}

void DataOutput()
{
  PWMStart();
  InputDecoder();
}

void DataNoPut()
{
  PWMStop();
}

void DataSave()
{
  if(!sys.osc_suspend)
    SendOscilloscope();
  
  if(sys.sd_write)
    DataWriteFatfs();

}