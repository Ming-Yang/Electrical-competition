#include "data.h"
#include "OLEDUI.h"
#include "SDFatfs.h"
#include "init.h"

DATA_IN_STRUCT indata;
DATA_OUT_STRUCT outdata;

//upto 1000lines£¬10ms,30000rmp
#define DECODER_LINES
#define DECODER_COUNT 10000
#define CheckData(data_in,data_th)  ((data_in) < (data_th/2.0f))?\
                                      (data_in):((data_in)-(data_th))
                                        
void DataInput()
{
  indata.decoder1.raw = CheckData(TIM4->CNT,DECODER_COUNT);
  TIM4->CNT = 0;
  indata.decoder2.raw = CheckData(TIM8->CNT,DECODER_COUNT);
  TIM8->CNT = 0;
  
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
//  if(!sys.osc_suspend)
//    SendOscilloscope();
  
  if(sys.sd_write)
    DataWriteFatfs();

}
