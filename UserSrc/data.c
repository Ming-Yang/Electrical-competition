#include "data.h"
#include "OLEDUI.h"
#include "SDFatfs.h"
#include "usmart.h"
#include "interrupt.h"
#include "PIDController.h"
#include "init.h"
#include "adc.h"
#include "PIDController.h"

DATA_IN_STRUCT indata;
MPU6050_PHYSICAL_STRUCT mpu6050_filted;
MPU6050_PHYSICAL_STRUCT mpu6050_offset={0};
MPU6050_EULER_STRUCT eulerRad;

DATA_IN_STRUCT indata;
DATA_OUT_STRUCT outdata;
PID euler2speed;
PID speed2pwm;

//upto 1000lines，10ms,30000rmp
#define DECODER_LINES   256
#define DECODER_COUNT 10000
#define MAX_PWM 10000
#define DECODER1_RAW_GRY 27.27272727273f
#define DECODER2_RAW_GRY 0.3515625f
#define CheckData(data_in,data_th)  ((data_in) < (data_th/2.0f))?\
                                     (data_in):((data_in)-(data_th))
#define Limit(x,b,a) (x)>(a)?(a):((x)<(b)?(b):(x))
#define OCS_PIN PEin(12) 

void GetGY_25(MPU6050_EULER_STRUCT*);

void DataInput()
{
  //获得两个编码器参数
  indata.decoder1.raw = CheckData(TIM4->CNT,DECODER_COUNT);
  indata.decoder2.raw = CheckData(TIM8->CNT,DECODER_COUNT);
  indata.decoder1.acc_roll += indata.decoder1.raw;
  indata.decoder2.acc_roll += indata.decoder2.raw;
  
  indata.decoder1.ang_v = indata.decoder1.raw * DECODER1_RAW_GRY;
  indata.decoder2.ang_v = indata.decoder2.raw * DECODER2_RAW_GRY;
  
  TIM4->CNT = 0;
  
  outdata.gy25_euler_last = outdata.gy25_euler;
  GetGY_25(&outdata.gy25_euler);
  
//  if (HAL_ADC_PollForConversion(&hadc1, 0) == HAL_OK)
//  {
//    indata.adc10 = HAL_ADC_GetValue(&hadc1);
//  }
}

void DataProcess()
{  
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
    outdata.speed = 0;
    outdata.pwm = 0;
  }
  else
  { 
    if(fabs(fmod(indata.decoder2.ang_v,360.0)) < 70.0 )
    {
      euler2speed.set_point = setpara.test*0.1f;
      euler2speed.current_point = fmod(indata.decoder2.ang_v,360.0);
      IncPIDCalc(&euler2speed);
      outdata.speed = -euler2speed.sum_con;
    }
    else 
      outdata.speed = 0;
    //防止过转
    if(abs(indata.decoder1.acc_roll) < DECODER_LINES*12)
    {
      speed2pwm.set_point = outdata.speed;
      speed2pwm.current_point = indata.decoder1.ang_v;
      IncPIDCalc(&speed2pwm);
      outdata.pwm = (int)speed2pwm.sum_con;
    }
    else
      outdata.pwm = 0;
    if (outdata.pwm > 0)
    {
      outdata.tim2.channel1 = outdata.pwm;
      outdata.tim2.channel2 = 0;
    }
    else
    {
      outdata.tim2.channel1 = 0;
      outdata.tim2.channel2 = -outdata.pwm;
    }
  }

}

void DataOutput()
{  
  //开启pwm中断
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  //开启按键中断
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

void GetGY_25(MPU6050_EULER_STRUCT *gy25)
{
    int a;
    a= 0x51a5;
    HAL_UART_Transmit(&huart1,(uint8_t*)&(a),2,0xFFFF);
    HAL_UART_Receive(&huart1, (uint8_t*)uart1_rx_buff, 8, 0x01);
    
    gy25->pitch = ((int16_t)( (uart1_rx_buff[3] << 8) | uart1_rx_buff[4] )) / 100.0f; 
    gy25->roll = ((int16_t)( (uart1_rx_buff[5] << 8) | uart1_rx_buff[6] )) / 100.0f; 
    gy25->yaw = ((int16_t)( (uart1_rx_buff[1] << 8) | uart1_rx_buff[2] )) / 100.0f; 
}