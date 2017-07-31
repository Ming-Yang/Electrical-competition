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
int32_t roll_count=0;
int32_t roll_acc[10] = {0};
uint8_t roll_cnt;

//upto 1000lines，10ms,30000rmp
#define DECODER_LINES
#define DECODER_COUNT 10000
#define MAX_PWM 10000
#define DECODER_RAW_GRY 27.27272727273f
#define CheckData(data_in,data_th)  ((data_in) < (data_th/2.0f))?\
                                     (data_in):((data_in)-(data_th))
#define Limit(x,b,a) (x)>(a)?(a):((x)<(b)?(b):(x))
#define OCS_PIN PEin(12) 

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
  
  if (HAL_ADC_PollForConversion(&hadc1, 0) == HAL_OK)
  {
    indata.adc10 = HAL_ADC_GetValue(&hadc1);
  }
}
    /****PIDtest****/  
#include "PIDBasic.h"
  extern PID pid_test;
  int pwm_con, dpwm_dt;  
  int last_dpwm_dt, last_pwm_con;
  int d2pwm_dt2;
    /****\PIDtest****/  

void DataProcess()
{
  //姿态融合
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
//    if(fabs(outdata.gy25_euler.roll) < 45.0 )
//      outdata.speed = MotorPID(1,outdata.gy25_euler.roll, setpara.test/10.0);
//    else 
//      outdata.speed = 0;
//    //防止过转
//    if(abs(roll_count) < 330*2)
      outdata.pwm = MotorPID(0,indata.decoder1.ang_v*100, outdata.speed);
//    else
//      outdata.pwm = 0;
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
  
        //PID控制
    pid_test.set_point = 2047;
    pid_test.current_point = indata.adc10;
    IncPIDCalc(&pid_test);
    //indata.adc10;
//    outdata.tim2.channel1 = 8000;
//    outdata.tim2.channel2 = 7000;
  //setpara.test  pwmccc/100
    outdata.tim2.channel3 = setpara.test;
    outdata.tim2.channel4 = Limit(pid_test.sum_con,0,8399);
//    outdata.tim3.channel2 = 3000;
//    outdata.tim3.channel3 = 2000;
//    outdata.tim3.channel4 = 1000;
}


void DataOutput()
{  
      /****PIDtest****/    
  //初始化PID控制参数
  pid_test.proportion   = setpara.pid_para.PID_Kp;
  pid_test.integral     = setpara.pid_para.PID_Ki;
  pid_test.differential = setpara.pid_para.PID_Kd;
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
    powerout_r = setpara.angle_pid.kp * d_error_r/10 + setpara.angle_pid.ki * error_r/100 + setpara.angle_pid.kd * dd_error_r/100;
    last_last_error_r = last_error_r;
    last_error_r = error_r;
    
    //    if(powerout_r>_MAXPWM||error_r>setpara.SpeedBANGBANG)
    //      powerout_r=MAX_PWM;
    //    else if(powerout_r<-_MINPWM||error_r<-setpara.SpeedBANGBANG)
    //      powerout_r=-MAX_PWM;
    return (int)powerout_r;
  }
}