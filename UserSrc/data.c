#include "data.h"
#include "OLEDUI.h"
#include "SDFatfs.h"
#include "usmart.h"
#include "interrupt.h"
#include "PIDController.h"
#include "init.h"
#include "adc.h"

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
                                      
#define ERROR_YAW 2.204615824775566f
#define GY25INIT_TIME_MS 0 //如果不是0，则为校准GY25的时间(ms)
#define STICK_LENGTH 0.52
#define AXIS_HIGH 0.64
                                      
//#define PHI_MAX 0.4383
#define PHI_MAX 10*DEG_TO_RAD
#define ROU_MAX 23.6702
                                      
DATA_IN_STRUCT indata;
DATA_OUT_STRUCT outdata;

PID axis_x;
PID axis_y;

PID axis_x_error;
PID axis_y_error;

PID axis_x_err_err;
PID axis_y_err_err;

PID axis_x_energy;
PID axis_y_energy;

ROT_MATRIX err_rot_matrix;
ROT_MATRIX mea_rot_matrix;
POSITION pendulum_pos;

static void AngleCalibrate(MPU6050_EULER_STRUCT * e, POSITION * p);
static void InitOffsetGy25();

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
    
//  static uint32_t t_last = T;
//  while(T - t_last <= GY25INIT_TIME_MS)
//    InitOffsetGy25(t_last);
    
  InitOffsetGy25();
  AngleCalibrate(&indata.gy25_euler, &pendulum_pos);
  indata.global_euler.roll =  100*STICK_LENGTH*pendulum_pos.x;
  indata.global_euler.pitch = 100*STICK_LENGTH*pendulum_pos.y;
  indata.global_euler.yaw =   100*STICK_LENGTH*pendulum_pos.z;
  
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
      //复制参数
      setpara.y_error_pid = setpara.x_error_pid;      
      setpara.x_error_pid.bound = setpara.y_pid.bound = MAX_PWM;
      setpara.x_error_pid.death = setpara.y_error_pid.death = 0;
      
      //正交分解到x方向
      GetPIDPara(&axis_x_err_err,&setpara.x_pid);
      //PHI_MAX是使摆幅足以超过50cm的摆动的最大角度
      axis_x_err_err.set_point = PHI_MAX*sin(OMEGA*0.001*sys.T_RUN)*RAD_TO_DEG;
      axis_x_err_err.current_point = atan(pendulum_pos.x/pendulum_pos.z)*RAD_TO_DEG;
      IncPIDCalc(&axis_x_err_err);
      outdata.pwm_x = (int)axis_x_err_err.sum_con;
      
      //正交分解到y方向
      GetPIDPara(&axis_y_err_err,&setpara.y_pid);
      axis_y_err_err.set_point = 0;
      axis_y_err_err.current_point = atan(pendulum_pos.y/pendulum_pos.z)*RAD_TO_DEG;
      IncPIDCalc(&axis_y_err_err);
      outdata.pwm_y = (int)axis_y_err_err.sum_con;
      break;
    case 2:
      setpara.y_error_pid = setpara.x_error_pid;
      setpara.y_pid = setpara.x_pid;
      
      setpara.x_pid.bound = setpara.y_pid.bound = MAX_PWM;
      setpara.x_pid.death = setpara.y_pid.death = 0;
      setpara.x_error_pid.bound = setpara.y_pid.bound = MAX_PWM;
      setpara.x_error_pid.death = setpara.y_error_pid.death = 0;
      
      GetPIDPara(&axis_x_err_err,&setpara.x_pid);
      axis_x_err_err.set_point = atan(setpara.test_x/100/AXIS_HIGH)*sin(OMEGA*0.001*sys.T_RUN);
      axis_x_err_err.current_point = atan(-pendulum_pos.x/pendulum_pos.z);
      IncPIDCalc(&axis_x_err_err);
      outdata.pwm_x = (int)axis_x_err_err.sum_con;
      
      GetPIDPara(&axis_y_err_err,&setpara.y_pid);
      axis_y_err_err.set_point = 0;
      axis_y_err_err.current_point = atan(-pendulum_pos.y/pendulum_pos.z);
      IncPIDCalc(&axis_y_err_err);
      outdata.pwm_y = (int)axis_y_err_err.sum_con;
      break;
    case 3:
      setpara.y_error_pid = setpara.x_error_pid;
      setpara.y_pid = setpara.x_pid;
      
      setpara.x_pid.bound = setpara.y_pid.bound = MAX_PWM;
      setpara.x_pid.death = setpara.y_pid.death = 0;
      setpara.x_error_pid.bound = setpara.y_pid.bound = MAX_PWM;
      setpara.x_error_pid.death = setpara.y_error_pid.death = 0;
      
      GetPIDPara(&axis_x_err_err,&setpara.x_pid);
      axis_x_err_err.set_point = ROU_MAX*cos(setpara.theta*DEG_TO_RAD)*sin(OMEGA*0.001*sys.T_RUN);
      axis_x_err_err.current_point = atan(pendulum_pos.x/pendulum_pos.z);
      IncPIDCalc(&axis_x_err_err);
      outdata.pwm_x = (int)axis_x_err_err.sum_con;
      
      GetPIDPara(&axis_y_err_err,&setpara.y_pid);
      axis_y_err_err.set_point = ROU_MAX*sin(setpara.theta*DEG_TO_RAD)*sin(OMEGA*0.001*sys.T_RUN);;
      axis_y_err_err.current_point = atan(pendulum_pos.y/pendulum_pos.z);
      IncPIDCalc(&axis_y_err_err);
      outdata.pwm_y = (int)axis_y_err_err.sum_con;
      break;
    case 4:
      setpara.y_error_pid = setpara.x_error_pid;
      setpara.y_pid = setpara.x_pid;
      
      setpara.x_pid.bound = setpara.y_pid.bound = MAX_PWM;
      setpara.x_pid.death = setpara.y_pid.death = 0;
      setpara.x_error_pid.bound = setpara.y_pid.bound = MAX_PWM;
      setpara.x_error_pid.death = setpara.y_error_pid.death = 0;
      
      GetPIDPara(&axis_x,&setpara.x_pid);
      axis_x.set_point = setpara.test_x * sin(OMEGA*0.001*sys.T_RUN)/10;
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
    case 5:
      setpara.y_error_pid = setpara.x_error_pid;
      setpara.y_pid = setpara.x_pid;
      
      setpara.x_pid.bound = setpara.y_pid.bound = MAX_PWM;
      setpara.x_pid.death = setpara.y_pid.death = 0;
      setpara.x_error_pid.bound = setpara.y_pid.bound = MAX_PWM;
      setpara.x_error_pid.death = setpara.y_error_pid.death = 0;
      
      GetPIDPara(&axis_x_err_err,&setpara.x_pid);
      axis_x_err_err.set_point = atan(setpara.test_x/100/AXIS_HIGH)*sin(OMEGA*0.001*sys.T_RUN);
      axis_x_err_err.current_point = atan(-pendulum_pos.x/pendulum_pos.z);
      IncPIDCalc(&axis_x_err_err);
      outdata.pwm_x = (int)axis_x_err_err.sum_con;
      
      GetPIDPara(&axis_y_err_err,&setpara.y_pid);
      axis_y_err_err.set_point = atan(setpara.test_x/100/AXIS_HIGH)*cos(OMEGA*0.001*sys.T_RUN);;
      axis_y_err_err.current_point = atan(-pendulum_pos.y/pendulum_pos.z);
      IncPIDCalc(&axis_y_err_err);
      outdata.pwm_y = (int)axis_y_err_err.sum_con;
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
    case 10://      setpara.y_err_err_pid = setpara.x_err_err_pid;
      //通过位置误差控制摆的PWM，最终失败了，因为摆不知道自己的速度，只要位置不到就会猛加速，最终很难控制到同相位振动。
      setpara.x_err_err_pid.bound = setpara.y_err_err_pid.bound = MAX_PWM;
      setpara.x_err_err_pid.death = setpara.y_err_err_pid.death = 0;
      
      setpara.y_err_err_pid = setpara.x_err_err_pid;
                                                 
      //正交分解到x方向
      GetPIDPara(&axis_x_err_err,&setpara.x_err_err_pid);
      //PHI_MAX是使摆幅足以超过50cm的摆动的最大角度
      axis_x_err_err.set_point = setpara.test_x*sin(OMEGA*0.001*sys.T_RUN)*RAD_TO_DEG/100;
      axis_x_err_err.current_point = atan(-pendulum_pos.x/pendulum_pos.z)*RAD_TO_DEG;
      IncPIDCalc(&axis_x_err_err);
      outdata.pwm_x = (int)axis_x_err_err.sum_con;
      
      //正交分解到y方向
      GetPIDPara(&axis_y_err_err,&setpara.y_err_err_pid);
      axis_y_err_err.set_point = 0;
      axis_y_err_err.current_point = atan(-pendulum_pos.y/pendulum_pos.z)*RAD_TO_DEG;
      IncPIDCalc(&axis_y_err_err);
      outdata.pwm_y = (int)axis_y_err_err.sum_con;
      break;
      break;
    default:
//      setpara.y_err_err_pid = setpara.x_err_err_pid;
            //复制参数
      setpara.x_energy_pid.bound = setpara.y_energy_pid.bound = MAX_PWM;
      setpara.x_energy_pid.death = setpara.y_energy_pid.death = 0;      
      setpara.y_energy_pid = setpara.x_energy_pid;                             
      GetPIDPara(&axis_x_energy,&setpara.x_energy_pid);
      GetPIDPara(&axis_y_energy,&setpara.y_energy_pid);
                                                   
      //正交分解到x方向
      setpara.last_real.pitch = setpara.real.pitch; 
      setpara.real.pitch = acos((pendulum_pos.x)/sqrt(pendulum_pos.x*pendulum_pos.x+pendulum_pos.z*pendulum_pos.z));
      float dpitch = (setpara.real.pitch - setpara.last_real.pitch)*DEG_TO_RAD*STICK_LENGTH/0.01;
      axis_x_energy.set_point = 100000*CONSTANTS_ONE_G*setpara.test_x/100;//setpara.test_x代表摆的高度，单位cm
      setpara.Ek = 100000*0.5*dpitch*dpitch;
      setpara.Ep = 100000*CONSTANTS_ONE_G*STICK_LENGTH*(1+pendulum_pos.z)*(pendulum_pos.x*pendulum_pos.x)/(pendulum_pos.x*pendulum_pos.x+pendulum_pos.y*pendulum_pos.y);
      axis_x_energy.current_point = setpara.Ek + setpara.Ep;
      IncPIDCalc(&axis_x_energy);
      if (dpitch > 0)
          outdata.pwm_x =-(int)axis_x_energy.sum_con;
      else
          outdata.pwm_x = (int)axis_x_energy.sum_con;
        
      
      //正交分解到y方向
      setpara.last_real.roll = setpara.real.roll; 
      setpara.real.roll = acos((pendulum_pos.y)/sqrt(pendulum_pos.y*pendulum_pos.y+pendulum_pos.z*pendulum_pos.z));
      float droll = (setpara.real.roll - setpara.last_real.roll)*DEG_TO_RAD*STICK_LENGTH/0.01;
      axis_y_energy.set_point = 0;//目标能量为0
      axis_y_energy.current_point = 100000*(0.5*droll*droll+CONSTANTS_ONE_G*STICK_LENGTH*(1+pendulum_pos.z)*
             (pendulum_pos.y*pendulum_pos.y)/(pendulum_pos.x*pendulum_pos.x+pendulum_pos.y*pendulum_pos.y));
      IncPIDCalc(&axis_y_energy);
      if (droll > 0)
          outdata.pwm_y =-(int)axis_y_energy.sum_con;
      else
          outdata.pwm_y = (int)axis_y_energy.sum_con;
       
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



/*******Global通过初始化的误差角和实际测量的欧拉角算坐标******/

void RotReverseInit(ROT_MATRIX* r, float a, float b, float c)
{
  a *= DEG_TO_RAD;
  b *= DEG_TO_RAD*0.8;
  c *= DEG_TO_RAD*1.005;
  r->r[0] = cos(c)*cos(a) - sin(c)*sin(a)*sin(b);
  r->r[1] = sin(a)*cos(b);
  r->r[2] = sin(c)*cos(a) + cos(c)*sin(a)*sin(b);
  r->r[3] =-sin(a)*cos(c) - sin(c)*cos(a)*sin(b);
  r->r[4] = cos(a)*cos(b);
  r->r[5] =-sin(c)*sin(a) + cos(c)*cos(a)*sin(b);
  r->r[6] =-sin(c)*cos(b);
  r->r[7] =-sin(b);
  r->r[8] = cos(c)*cos(b);  
  
}

void InitOffsetGy25()
{  
  static MPU6050_EULER_STRUCT gy25_offset;
  static int counts;
  if (GY25INIT_TIME_MS == 0)
  {
    RotReverseInit(&err_rot_matrix, ERROR_YAW,
                 1.51571765E-1,
                 -2.8266940111);
  }
  else if(T < GY25INIT_TIME_MS && T > 0)
  {
  gy25_offset.pitch+=indata.gy25_euler.pitch;
  gy25_offset.roll+=indata.gy25_euler.roll;
     counts++;
  }
  else if(T == GY25INIT_TIME_MS)
  {
  gy25_offset.pitch/=counts;
  gy25_offset.roll /=counts;
  RotReverseInit(&err_rot_matrix, ERROR_YAW,
                 gy25_offset.pitch,
                 gy25_offset.roll);
  }
}

void RotInit(ROT_MATRIX* r, float a, float b, float c)
{
  a *= -DEG_TO_RAD;
  b *= -DEG_TO_RAD;
  c *= -DEG_TO_RAD;
  r->r[0] = cos(c)*cos(a) - sin(c)*sin(a)*sin(b);
  r->r[1] =-sin(a)*cos(c) - sin(c)*cos(a)*sin(b);
  r->r[2] =-sin(c)*cos(b);
  r->r[3] = sin(a)*cos(b);
  r->r[4] = cos(a)*cos(b);
  r->r[5] =-sin(b);
  r->r[6] = sin(c)*cos(a) + cos(c)*sin(a)*sin(b);
  r->r[7] =-sin(c)*sin(a) + cos(c)*cos(a)*sin(b);
  r->r[8] = cos(c)*cos(b);  
}


void RotCoordinate(ROT_MATRIX* error, ROT_MATRIX* measure, POSITION* p)
{
  int j;
  float norm;
  p->x = 0;
  p->y = 0;
  p->z = 0;
  for (j = 0; j < 3; j++)
  {
       p->x += measure->r[j]   * error->r[j+6];
       p->y += measure->r[3+j] * error->r[j+6];
       p->z += measure->r[6+j] * error->r[j+6];
  }   
  norm = invSqrt(p->x * p->x + p->y * p->y + p->z * p->z);
  float temp;
  temp = p->x;
  p->x = norm * p->y;
  p->y =-norm * temp;
  p->z =-norm * p->z;
}

void AngleCalibrate(MPU6050_EULER_STRUCT * e, POSITION * p)
{
  RotInit(&mea_rot_matrix, ERROR_YAW, e->pitch, e->roll);
  RotCoordinate(&err_rot_matrix, &mea_rot_matrix, p);
}
/*************/
