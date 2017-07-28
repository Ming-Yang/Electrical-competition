#include "interrupt.h"
#include "SDFatfs.h"
#include "OLEDUI.h"
#include "data.h"

volatile uint32_t T;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  static char buff;
  static uint8_t count = 0;
  static uint8_t string_flag = 0;
  
  if(huart->Instance == huart2.Instance)
  {
    if(buff == '\r' || buff == '\n' || buff == '\0' ||
       count == UART_BUFF_SIZE)
    {
      uart2_rx_buff[count] = '\0';
      string_flag = 1;
      if(buff == '\r')
        usmart_scan();
    }
    else
    {
      sys.osc_suspend = 1;
      if(string_flag == 1)
      {
        count = 0;
        memset(&uart2_rx_buff,0,sizeof(uart2_rx_buff));
        string_flag = 0;
      }
      uart2_rx_buff[count++] = buff;
    }
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&buff, 1);
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  
  if(htim->Instance == htim9.Instance)
  {
    T += T_PERIOD_MS;
    
    if(T %500 == 0)
    {
      HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
    }
    
    SysCheck();
    DataInput();
    DataProcess();
    DataSave();  
  }       
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  uint32_t pwm;
  
  if(htim->Instance == htim2.Instance)
    switch(htim->Channel)
    {
    case TIM_CHANNEL_1:pwm = outdata.tim2.channel1;break;
    case TIM_CHANNEL_2:pwm = outdata.tim2.channel2;break;
    case TIM_CHANNEL_3:pwm = outdata.tim2.channel3;break;
    case TIM_CHANNEL_4:pwm = outdata.tim2.channel4;break;
    default:break;
    }
  
  else if(htim->Instance == htim3.Instance)
    switch(htim->Channel)
    {
    case TIM_CHANNEL_1:pwm = outdata.tim3.channel1;break;
    case TIM_CHANNEL_2:pwm = outdata.tim3.channel2;break;
    case TIM_CHANNEL_3:pwm = outdata.tim3.channel3;break;
    case TIM_CHANNEL_4:pwm = outdata.tim3.channel4;break;
    default:break;
    }
  
  PwmChangeDuty(htim,htim->Channel,pwm);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  switch(GPIO_Pin)
  {
  case GPIO_PIN_10:button = PUSH;
    break;
  case GPIO_PIN_11:button = UP;
    break;
  case GPIO_PIN_12:
    if(!PRESS_IN)
      button = PRESS;
    break;
  case GPIO_PIN_13:button = DOWN;
    break;

  case GPIO_PIN_15:
    if(DIRECTION_IN)
      button = CCW;
    else 
      button = CW;
    break;
    
  default:break;
  }
}
