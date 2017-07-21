#include "init.h"
#include "usmart.h"
#include "usart.h"
#include "flash.h"
#include "tim.h"
#include "oled.h"
#include "SDFatfs.h"
#include "OLEDUI.h"

void TimInit()
{
  HAL_TIM_Base_Start_IT(&htim9);
}

void PWMStart()
{
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_1);
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
#if UART_IT1
  HAL_UART_Receive_IT(&huart1, (uint8_t*)uart1_rx_buff, 1);
#endif
}

void InitAll()
{
  __disable_irq();
  TimInit();
  UartInit();
  
  printf("mpu6050 id:0x%x\r\n",MPU6050_Init());
   
  SDFatFSInit();
  FlashInit();
  
  UIInit();
  sys.status = READY;
  printf("init finish!\r\n");
  __enable_irq();
}
