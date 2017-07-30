#include "data.h"
#include "OLEDUI.h"
#include "SDFatfs.h"
#include "usmart.h"
#include "mpu6050.h"
#include "interrupt.h"
#include "mpu6050_process.h"
#include "init.h"
#include "adc.h"

DATA_IN_STRUCT indata;
MPU6050_PHYSICAL_STRUCT mpu6050_filted;
MPU6050_PHYSICAL_STRUCT mpu6050_offset={0};
MPU6050_EULER_STRUCT eulerRad;

DATA_IN_STRUCT indata;
DATA_OUT_STRUCT outdata;

//upto 1000lines£¬10ms,30000rmp
#define DECODER_LINES
#define DECODER_COUNT 10000
#define MAX_PWM 10000
#define DECODER_RAW_GRY 27.27272727273f
#define CheckData(data_in,data_th)  ((data_in) < (data_th/2.0f))?\
(data_in):((data_in)-(data_th))
#define OCS_PIN PEin(12) 

void DataInput()
{
  indata.decoder1.raw = CheckData(TIM4->CNT,DECODER_COUNT);
  indata.decoder1.ang_v = indata.decoder1.raw * DECODER_RAW_GRY;
  TIM4->CNT = 0;
  indata.decoder2.raw = CheckData(TIM8->CNT,DECODER_COUNT);
  indata.decoder2.ang_v = indata.decoder2.raw * DECODER_RAW_GRY;
  TIM8->CNT = 0;
  MPU6050_GetData(&indata.mpu6050);
  
  if (HAL_ADC_PollForConversion(&hadc1, 0) == HAL_OK)
  {
    indata.adc10 = HAL_ADC_GetValue(&hadc1);
  }
}

void DataProcess()
{
  //×ËÌ¬ÈÚºÏ
  MPU6050_Process(&indata.mpu6050, &mpu6050_filted, &mpu6050_offset, &eulerRad, &outdata.euler);
  
  if(sys.status == READY)
  {
    outdata.tim2.channel1 = 0 ;
    outdata.tim2.channel2 = 0 ;
    outdata.tim2.channel3 = 0 ;
    outdata.tim2.channel4 = 0 ;
    outdata.tim3.channel1 = 0 ;
    outdata.tim3.channel2 = 0 ;
    outdata.tim3.channel3 = 0 ;
    outdata.tim3.channel4 = 0 ;
  }
  else
  {
    
    outdata.tim2.channel1 = MAX_PWM;
    outdata.tim2.channel2 = MAX_PWM/2;
    outdata.tim2.channel3 = MAX_PWM/3;
    outdata.tim2.channel4 = MAX_PWM/4;
    outdata.tim3.channel1 = MAX_PWM/5;
    outdata.tim3.channel2 = MAX_PWM/6;
    outdata.tim3.channel3 = MAX_PWM/7;
    outdata.tim3.channel4 = MAX_PWM/8;
  }
}

void DataOutput()
{
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void DataNoPut()
{
  PWMStop();
}

void DataSave()
{
  if(!(sys.osc_suspend || OCS_PIN))
    SendOscilloscope();
  
  if(sys.sd_write)
    DataWriteFatfs();
}
