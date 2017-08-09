#include "data.h"
#include "OLEDUI.h"
#include "SDFatfs.h"
#include "usmart.h"
#include "interrupt.h"
#include "PIDController.h"
#include "init.h"
#include "adc.h"
#include "PIDController.h"

//upto 1000lines，10ms,30000rmp
//#define DECODER_LINES   256
//#define DECODER_COUNT 10000
//#define MAX_PWM 1000
//#define DECODER1_RAW_GRY 27.27272727273f
//#define DECODER2_RAW_GRY 0.3515625f
//#define CheckData(data_in,data_th)  ((data_in) < (data_th/2.0f))?\
//                                    (data_in):((data_in)-(data_th))
#define OCS_PIN PEin(12) 
#define SD_PIN (!PEin(11))                                  
            
#define STEP_DIV        8
#define MM_1204 4
#define MM_PER_PULSE    0.005*MM_1204/STEP_DIV
#define MIN_STEP_FREQUENCY      800
#define MAX_STEP_FREQUENCY      8000  

DATA_IN_STRUCT indata;
DATA_OUT_STRUCT outdata;

//static void GetGY_25(MPU6050_EULER_STRUCT*);

void DataInput()
{
  //获得两个编码器参数
//  indata.decoder1.raw = CheckData(TIM4->CNT,DECODER_COUNT);
//  indata.decoder2.raw = CheckData(TIM8->CNT,DECODER_COUNT);
//  indata.decoder1.acc_roll += indata.decoder1.raw;
//  indata.decoder2.acc_roll += indata.decoder2.raw;
//  
//  indata.decoder1.ang_v = indata.decoder1.raw * DECODER1_RAW_GRY;
//  indata.decoder2.ang_v = indata.decoder2.raw * DECODER2_RAW_GRY;
//  
//  TIM4->CNT = 0;

  //获得摄像头数据
  HAL_UART_Receive_IT(&huart1, (uint8_t*)uart1_rx_buff, 4);
}

void DataProcess()
{  
  if(sys.status == BLOCKED)
  {
    outdata.step_motor1.speed = 0;
    outdata.step_motor2.speed = 0;
  }
  
  else if(sys.status == RUNNING)
  { 
    switch(setpara.task_num)
    {
    case 1:
      outdata.step_motor1.speed = setpara.test_1;
      outdata.step_motor2.speed = setpara.test_2;
      
      if(outdata.step_motor1.speed > 0)
        outdata.step_motor1.direction = 0;
      else
        outdata.step_motor1.direction = 1;
      
      if(outdata.step_motor2.speed > 0)
        outdata.step_motor2.direction = 0;
      else
        outdata.step_motor2.direction = 1;
      
      outdata.step_motor1.frequency = abs(outdata.step_motor1.speed);
      outdata.step_motor2.frequency = abs(outdata.step_motor2.speed);
      break;
    case 2:
      outdata.step_motor1.set_lenth_mm = setpara.test_1/10.0;
      outdata.step_motor2.set_lenth_mm = setpara.test_2/10.0;
      
      if(outdata.step_motor1.direction == 1)
        outdata.step_motor1.lenth_mm += (outdata.step_motor1.frequency*T_PERIOD_MS*MM_PER_PULSE/1000);
      else
        outdata.step_motor1.lenth_mm -= (outdata.step_motor1.frequency*T_PERIOD_MS*MM_PER_PULSE/1000);
      if(outdata.step_motor2.direction == 1)
        outdata.step_motor2.lenth_mm += (outdata.step_motor2.frequency*T_PERIOD_MS*MM_PER_PULSE/1000);
      else
        outdata.step_motor2.lenth_mm -= (outdata.step_motor2.frequency*T_PERIOD_MS*MM_PER_PULSE/1000);
      
      if(outdata.step_motor1.set_lenth_mm - outdata.step_motor1.lenth_mm > 0.1)
      {
        outdata.step_motor1.direction = 1;
        switch(outdata.step_motor1.frequency)
        {
        case 0:outdata.step_motor1.frequency = MIN_STEP_FREQUENCY;break;
        case MIN_STEP_FREQUENCY: outdata.step_motor1.frequency = MAX_STEP_FREQUENCY/5;break;
        case MAX_STEP_FREQUENCY/5: outdata.step_motor1.frequency = MAX_STEP_FREQUENCY/4;break;
        case MAX_STEP_FREQUENCY/4: outdata.step_motor1.frequency = MAX_STEP_FREQUENCY/2;break;
        case MAX_STEP_FREQUENCY/2: outdata.step_motor1.frequency = MAX_STEP_FREQUENCY;break;
        default: break;
        }
      }
      else if(outdata.step_motor1.set_lenth_mm - outdata.step_motor1.lenth_mm < -0.1)
      {
        outdata.step_motor1.direction = 0;
        switch(outdata.step_motor1.frequency)
        {
        case 0:outdata.step_motor1.frequency = MIN_STEP_FREQUENCY;break;
        case MIN_STEP_FREQUENCY: outdata.step_motor1.frequency = MAX_STEP_FREQUENCY/5;break;
        case MAX_STEP_FREQUENCY/5: outdata.step_motor1.frequency = MAX_STEP_FREQUENCY/4;break;
        case MAX_STEP_FREQUENCY/4: outdata.step_motor1.frequency = MAX_STEP_FREQUENCY/2;break;
        case MAX_STEP_FREQUENCY/2: outdata.step_motor1.frequency = MAX_STEP_FREQUENCY;break;
        default: break;
        }
      }
      else
      {
        outdata.step_motor1.frequency = 0;
      }

      break;
      
    default:
      
      break;
    }
  }

}

void DataOutput()
{    
  if(sys.status == RUNNING)
  {
    PAout(2) = outdata.step_motor1.enable = 1;
    PBout(0) = outdata.step_motor1.enable = 1;
  }
  else
  {
    PAout(2) = outdata.step_motor1.enable = 0;
    PBout(0) = outdata.step_motor1.enable = 0;
  }
  
  PAout(1) = outdata.step_motor1.direction;
  PAout(7) = outdata.step_motor2.direction;
  
  //开启pwm中断
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void DataNoPut()
{
  PWMStop();
}

void DataSave()
{
  if(!(sys.osc_suspend || OCS_PIN))
    SendOscilloscope();
  
#if SD_ENABLE
  if(sys.sd_write && SD_PIN)
    DataWriteFatfs();
#endif
}
//
//void GetGY_25(MPU6050_EULER_STRUCT *gy25)
//{
//  int a;
//  a= 0x51a5;//通信规则
//  HAL_UART_Transmit(&huart1,(uint8_t*)&(a),2,0xFFFF);
//  HAL_UART_Receive(&huart1, (uint8_t*)uart1_rx_buff, 8, 0x01);
//  
//  gy25->pitch = ((int16_t)( (uart1_rx_buff[3] << 8) | uart1_rx_buff[4] )) / 100.0f; 
//  gy25->roll = ((int16_t)( (uart1_rx_buff[5] << 8) | uart1_rx_buff[6] )) / 100.0f; 
//  gy25->yaw = ((int16_t)( (uart1_rx_buff[1] << 8) | uart1_rx_buff[2] )) / 100.0f; 
//}

