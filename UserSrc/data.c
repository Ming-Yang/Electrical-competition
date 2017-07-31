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
int32_t roll_count=0;
int32_t roll_acc[10] = {0};
uint8_t roll_cnt;

//upto 1000lines£¬10ms,30000rmp
#define DECODER_LINES
#define DECODER_COUNT 10000
#define MAX_PWM 10000
#define DECODER_RAW_GRY 27.27272727273f
#define CheckData(data_in,data_th)  ((data_in) < (data_th/2.0f))?\
                                     (data_in):((data_in)-(data_th))
#define OCS_PIN PEin(12) 

void GetGY_25(MPU6050_EULER_STRUCT*);
static int MotorPID(uint8_t LR,float input_speed,float set_speed);


void DataInput()
{
  indata.decoder1.raw = CheckData(TIM4->CNT,DECODER_COUNT);
  indata.decoder2.raw = CheckData(TIM8->CNT,DECODER_COUNT);
  
  indata.decoder1.ang_v = indata.decoder1.raw * DECODER_RAW_GRY;
  indata.decoder2.ang_v = indata.decoder2.raw * DECODER_RAW_GRY;
  
  TIM4->CNT = 0;
  TIM8->CNT = 0;
  MPU6050_GetData(&indata.mpu6050);
  GetGY_25(&outdata.gy25_euler);
  
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
    outdata.speed = 0;
    outdata.pwm = 0;
  }
  else
  {
    if(fabs(outdata.gy25_euler.roll) < 45.0 )
      outdata.speed = MotorPID(1,outdata.gy25_euler.roll, setpara.test/10.0);
    else 
      outdata.speed = 0;
    //·ÀÖ¹¹ý×ª
    if(abs(roll_count) < 330*2)
      outdata.pwm = MotorPID(0,indata.decoder1.ang_v, outdata.speed);
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


int MotorPID(uint8_t LR,float input_speed,float set_speed)
{
  static float powerout_l,error_l,last_error_l,last_last_error_l;
  static float powerout_r,error_r,last_error_r,last_last_error_r;
  
  if(LR == 0)
  {
    error_l = set_speed - input_speed;
    float d_error_l = error_l-last_error_l;
    float dd_error_l = -2*last_error_l+error_l+last_last_error_l;
    powerout_l += setpara.speed_pid.kp * d_error_l/10 + setpara.speed_pid.ki * error_l/100 + setpara.speed_pid.kd * dd_error_l/100;
    last_last_error_l = last_error_l;
    last_error_l = error_l;
    
    //    if(powerout_l>_MAXPWM||error_l>setpara.SpeedBANGBANG)
    //      powerout_l=MAX_PWM;
    //    else if(powerout_l<-_MINPWM||error_l<-setpara.SpeedBANGBANG)
    //      powerout_l=-MAX_PWM;
    return (int)powerout_l;
  }
  
  else
  {
    error_r = set_speed - input_speed;
    float d_error_r = error_r-last_error_r;
    float dd_error_r = -2*last_error_r+error_r+last_last_error_r;
    powerout_r += setpara.angle_pid.kp * d_error_r/10 + setpara.angle_pid.ki * error_r/100 + setpara.angle_pid.kd * dd_error_r/100;
    last_last_error_r = last_error_r;
    last_error_r = error_r;
    
    //    if(powerout_r>_MAXPWM||error_r>setpara.SpeedBANGBANG)
    //      powerout_r=MAX_PWM;
    //    else if(powerout_r<-_MINPWM||error_r<-setpara.SpeedBANGBANG)
    //      powerout_r=-MAX_PWM;
    return (int)powerout_r;
  }
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