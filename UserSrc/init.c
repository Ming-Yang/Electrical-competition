#include "init.h"
#include "usmart.h"
#include "usart.h"
#include "flash.h"
#include "tim.h"
#include "oled.h"
#include "SDFatfs.h"
#include "OLEDUI.h"
#include "mpu6050.h"
#include "mpu6050_process.h"
#include "adc.h"

void TimInit()
{
  HAL_TIM_Base_Start_IT(&htim9);
}

void InputDecoder()
{
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  TIM8->CNT = 512;
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
//  HAL_UART_Receive(&huart1, (uint8_t*)uart1_rx_buff, 8);
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
  
  FlashInit();
  SDFatFSInit();
  
  printf("mpu6050 id:0x%x\r\n",MPU6050_Init());
  IMU_Init();
  
  UIInit();
  sys.status = READY;
  HAL_ADC_Start(&hadc1);
  printf("init finish!\r\n");
  __enable_irq();
}
