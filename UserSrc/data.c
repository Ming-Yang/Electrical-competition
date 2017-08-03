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
#define DECODER_LINES   256
#define DECODER_COUNT 10000
#define MAX_PWM 1000
#define DECODER1_RAW_GRY 27.27272727273f
#define DECODER2_RAW_GRY 0.3515625f
#define OMEGA   4.342447f
#define CheckData(data_in,data_th)  ((data_in) < (data_th/2.0f))?\
                                    (data_in):((data_in)-(data_th))
#define OCS_PIN PEin(12) 
                                      
DATA_IN_STRUCT indata;
DATA_OUT_STRUCT outdata;

PID axis_x;
PID axis_y;

PID axis_x_error;
PID axis_y_error;

static void GetGY_25(MPU6050_EULER_STRUCT*);

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
  
  indata.gy25_euler_last = indata.gy25_euler;
  GetGY_25(&indata.gy25_euler);
  
  //  if (HAL_ADC_PollForConversion(&hadc1, 0) == HAL_OK)
  //  {
  //    indata.adc10 = HAL_ADC_GetValue(&hadc1);
  //  }
}

void DataProcess()
{  
  if(sys.status == RUNNING)
  { 
    switch(setpara.task_num)
    {
    case 1:
      break;
    case 2:
      break;
      
    default:
      
      setpara.y_error_pid = setpara.x_error_pid;
      setpara.y_pid = setpara.x_pid;
      
      setpara.x_pid.bound = setpara.y_pid.bound = MAX_PWM;
      setpara.x_pid.death = setpara.y_pid.death = 0;
      setpara.x_error_pid.bound = setpara.y_pid.bound = MAX_PWM;
      setpara.x_error_pid.death = setpara.y_error_pid.death = 0;
      
      GetPIDPara(&axis_x,&setpara.x_pid);
      axis_x.set_point = setpara.test_x*sin(OMEGA*0.001*sys.T_RUN)/10;
      axis_x.current_point = indata.gy25_euler.pitch;
      IncPIDCalc(&axis_x);
//      outdata.pwm_x = (int)axis_x.sum_con;
      
      GetPIDPara(&axis_y,&setpara.y_pid);
      axis_y.set_point = setpara.test_y/10.0;
      axis_y.current_point = - indata.gy25_euler.roll;
      IncPIDCalc(&axis_y);
//      outdata.pwm_y = (int)axis_y.sum_con;
      
      GetPIDPara(&axis_x_error,&setpara.x_error_pid);
      axis_x_error.set_point = setpara.acc_x/10.0;
      axis_x_error.current_point = indata.gy25_euler.pitch - indata.gy25_euler_last.pitch;
      IncPIDCalc(&axis_x_error);
//      outdata.pwm_x = (int)axis_x_error.sum_con;
      
      GetPIDPara(&axis_y_error,&setpara.y_error_pid);
      axis_y_error.set_point = setpara.acc_y/10.0;
      axis_y_error.current_point = indata.gy25_euler_last.roll - indata.gy25_euler.roll;
      IncPIDCalc(&axis_y_error);
//      outdata.pwm_y = (int)axis_y_error.sum_con;
      
      outdata.pwm_x = (int)(axis_x.sum_con + axis_x_error.sum_con);
      outdata.pwm_y = (int)(axis_y.sum_con + axis_y_error.sum_con);
      
      break;
    }
  }
  else if(sys.status == BLOCKED)
  {
    outdata.pwm_x = outdata.pwm_y = 0;
  }
}

void DataOutput()
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
    outdata.pwm_x = 0;
    outdata.pwm_y = 0;
  }
  else
  { 
    if(outdata.pwm_x > 0)
    {
      outdata.tim2.channel1 = outdata.pwm_x ;
      outdata.tim2.channel2 = 0 ;
      outdata.tim2.channel3 = outdata.pwm_x ;
      outdata.tim2.channel4 = 0 ;
    }
    else
    {
      outdata.tim2.channel1 = 0 ;
      outdata.tim2.channel2 = -outdata.pwm_x ;
      outdata.tim2.channel3 = 0 ;
      outdata.tim2.channel4 = -outdata.pwm_x ;
    }    
    if(outdata.pwm_y < 0)
    {
      outdata.tim3.channel1 = -outdata.pwm_y ;
      outdata.tim3.channel2 = 0 ;
      outdata.tim3.channel3 = -outdata.pwm_y ;
      outdata.tim3.channel4 = 0 ;
    }            
    else        
    {            
      outdata.tim3.channel1 = 0 ;
      outdata.tim3.channel2 = outdata.pwm_y ;
      outdata.tim3.channel3 = 0 ;
      outdata.tim3.channel4 = outdata.pwm_y ;
    }
  }
  
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