#include "init.h"
#include "usmart.h"
#include "usart.h"
#include "flash.h"
#include "tim.h"
#include "oled.h"
#include "SDFatfs.h"
#include "OLEDUI.h"
#include "adc.h"
#include "data.h"

#define ERROR_YAW 2.204615824775566f

void TimInit()
{
  HAL_TIM_Base_Start_IT(&htim9);
}

void InputDecoder()
{
  //tim4-------------->电机编码器
  //tim8-------------->角度编码器
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  //初始化角度传感器0°
  TIM8->CNT = 512;
}

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

void Gy25ErrInit ()
{
  GetGY_25(&indata.gy25_euler);
  RotReverseInit(&err_rot_matrix, ERROR_YAW, indata.gy25_euler.pitch, indata.gy25_euler.roll);
}


void PWMStart()
{
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_1);//known bug:cannot use TIM_CHANNEL_ALL!!!
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
  
  HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_4);
}

void PWMStop()
{
  HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_4);
                
  HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_4);
}

void UartInit()
{
  HAL_UART_Receive_IT(&huart2, (uint8_t*)uart2_rx_buff, 1);
}

void GpioInit()
{
  PEout(0) = 1;
  PEout(2) = 1;
  PEout(4) = 1;
  PEout(6) = 1;
}

void InitAll()
{
  __disable_irq();
  TimInit();
  PWMStart();
  InputDecoder();
  UartInit();
  HAL_ADC_Start(&hadc1);
  
  FlashInit();
  SDFatFSInit();
  
  UIInit();
  Gy25ErrInit();
  sys.status = READY;
  printf("init finish!\r\n");
  __enable_irq();
}
