#include "data.h"
#include "OLEDUI.h"
#include "SDFatfs.h"
#include "usmart.h"
#include "mpu6050.h"
#include "interrupt.h"
#include "mpu6050_process.h"
#include "PIDBasic.h"
#include "init.h"
#include "adc.h"

DATA_IN_STRUCT indata;
MPU6050_PHYSICAL_STRUCT mpu6050_filted;
MPU6050_PHYSICAL_STRUCT mpu6050_offset={0};
MPU6050_EULER_STRUCT eulerRad;

DATA_IN_STRUCT indata;
DATA_OUT_STRUCT outdata;

//upto 1000lines，10ms,30000rmp
#define DECODER_LINES
#define DECODER_COUNT 10000
#define CheckData(data_in,data_th)  ((data_in) < (data_th/2.0f))?\
(data_in):((data_in)-(data_th))
#define Limit(x,b,a) (x)>(a)?(a):((x)<(b)?(b):(x))
#define OCS_PIN PEin(12) 

void DataInput()
{
  indata.decoder1.raw = CheckData(TIM4->CNT,DECODER_COUNT);
  TIM4->CNT = 0;
  indata.decoder2.raw = CheckData(TIM8->CNT,DECODER_COUNT);
  TIM8->CNT = 0;
  MPU6050_GetData(&indata.mpu6050);
  
  if (HAL_ADC_PollForConversion(&hadc1, 0) == HAL_OK)
  {
    indata.adc10 = HAL_ADC_GetValue(&hadc1);
  }
}

  int pwmccc, dpwm_dt;  
  int d2pwm_dt2;

void DataProcess()
{
  //姿态融合
  MPU6050_Process(&indata.mpu6050, &mpu6050_filted, &mpu6050_offset, &eulerRad, &outdata.euler);

  
  if(sys.status == READY)
  {
    outdata.tim2.channel1 = 0;
    outdata.tim2.channel2 = 0;
    outdata.tim2.channel3 = 0;
    outdata.tim2.channel4 = 0;
    outdata.tim3.channel1 = 0;
    outdata.tim3.channel2 = 0;
    outdata.tim3.channel3 = 0;
    outdata.tim3.channel4 = 0;
  }
  else
  {
      //PID控制
  dpwm_dt = BasicPIDCalc(2047, indata.adc10);
//  dpwm_dt += d2pwm_dt2;
  pwmccc += dpwm_dt;
  pwmccc = Limit(pwmccc,0,8399);
  printf("%d,",dpwm_dt);
  printf("%d,",pwmccc); 
  printf("\r\n");
    //indata.adc10;
//    outdata.tim2.channel1 = 8000;
//    outdata.tim2.channel2 = 7000;
  //setpara.test  pwmccc/100
    outdata.tim2.channel3 = setpara.test;
    outdata.tim2.channel4 = pwmccc;
//    outdata.tim3.channel2 = 3000;
//    outdata.tim3.channel3 = 2000;
//    outdata.tim3.channel4 = 1000;
  }
}


    /****PIDtest****/  
#include "PIDBasic.h"
  PIDC kpid_parameter;

    /****\PIDtest****/  
void DataOutput()
{  
      /****PIDtest****/    
  //初始化PID控制参数
  kpid_parameter.Kp1000 = setpara.pid_para.PID_Kp;
  kpid_parameter.Ki1000 = setpara.pid_para.PID_Ki;
  kpid_parameter.Kd1000 = setpara.pid_para.PID_Kd;
  KPIDInit(&kpid_parameter);
    /********/
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
