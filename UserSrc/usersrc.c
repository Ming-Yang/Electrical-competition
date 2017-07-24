#include "userinc.h"
#include "usart.h"

uint8_t uart1_rx_buff[UART_BUFF_SIZE];
uint8_t uart2_rx_buff[UART_BUFF_SIZE];


u32 systick_getus()
{
  int cur_val[2];
  int cur_tick[2];
  cur_val[0] = SysTick->VAL;
  cur_tick[0] = HAL_GetTick();
  cur_val[1] = SysTick->VAL;
  cur_tick[1] = HAL_GetTick();
  if(cur_val[0]<cur_val[1])
    return cur_val[0]/SystemCoreClock/100000+cur_tick[0]*1000;
  else if(cur_tick[0]<cur_tick[1])
    return cur_val[0]/SystemCoreClock/100000+cur_tick[0]*1000;
  else
    return cur_val[0]/SystemCoreClock/100000+(cur_tick[0]-1)*1000;
}

/***微秒延时函数***/
void systick_delayus(int delay_us)
{
  int cur_us;
  cur_us = systick_getus();
  while(systick_getus()-cur_us < delay_us);
}

/***printf重定向函数***/
/**
* @brief  Retargets the C library printf function to the USART.
* @param  None
* @retval None
*/
int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  
  return ch;
}

/***PWM 占空比***/
#if TIM_IT4||TIM_IT3||TIM_IT2||TIM_IT1
void PwmChangeDuty(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t Duty)
{
  switch (Channel)
  {
  case TIM_CHANNEL_1:htim->Instance->CCR1 = Duty;break;
  case TIM_CHANNEL_2:htim->Instance->CCR2 = Duty;break;
  case TIM_CHANNEL_3:htim->Instance->CCR3 = Duty;break;
  case TIM_CHANNEL_4:htim->Instance->CCR4 = Duty;break;
  default:break;
  }
}
#endif