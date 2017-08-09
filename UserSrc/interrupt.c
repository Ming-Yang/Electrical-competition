#include "interrupt.h"
#include "SDFatfs.h"
#include "OLEDUI.h"
#include "data.h"
#include "usmart.h"

#define OV7725_W            80                                    //定义摄像头图像宽度
#define OV7725_H            60                                    //定义摄像头图像高度
#define CAMERA_SIZE         (OV7725_W * OV7725_H/8 )  //图像占用空间大小

volatile uint32_t T;
uint8_t imgbuff[CAMERA_SIZE];

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
  else if(huart->Instance == huart1.Instance)
  {
    indata.ball_position.x = ((uart1_rx_buff[0]) | (uart1_rx_buff[1] << 8))/100.0;
    indata.ball_position.y = ((uart1_rx_buff[2]) | (uart1_rx_buff[3] << 8))/100.0;
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
    TIME_TEST;
    SysCheck();
    DataInput();
    DataProcess();
    DataOutput();
    DataSave();  
    TIME_TEST;
  }       
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  uint32_t pwm,hz;
  
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  HAL_NVIC_DisableIRQ(TIM3_IRQn);
  
  if(htim->Instance == htim2.Instance)
    switch(htim->Channel)
    {
    case HAL_TIM_ACTIVE_CHANNEL_1:
      hz = (int)(1000000/outdata.step_motor1.frequency-1);
      hz = limit(hz, 100, 100000);
      
      if(hz == 100000)
      {
        htim2.Instance->CCR1 = 0;
      }
      else
      {
        htim2.Instance->ARR = hz;
        if(htim2.Instance->CNT > htim2.Instance->ARR)
          htim2.Instance->CNT = 0;
        htim2.Instance->CCR1 = (hz/2);
      }
      break;
      
    default:break;
    }
  
  else if(htim->Instance == htim3.Instance)
    switch(htim->Channel)
    {
    case HAL_TIM_ACTIVE_CHANNEL_1:
      hz = (int)(1000000/outdata.step_motor2.frequency-1);
      hz = limit(hz, 100, 100000);

      if(hz == 100000)
        htim3.Instance->CCR1 = 0;
      else
      {
        htim3.Instance->ARR = hz;
        if(htim3.Instance->CNT > htim3.Instance->ARR)
          htim3.Instance->CNT = 0;
        htim3.Instance->CCR1 = (hz/2);
      }
    break;

    default:break;
    }
  
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


void CameraShow()
{


}