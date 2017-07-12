#include "interrupt.h"
#include "SDFatfs.h"
#include "OLEDUI.h"
#include "data.h"
uint16_t in_count;
uint32_t T;

#ifdef UART_IT1
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  HAL_UART_Receive_IT(&huart1, (uint8_t*)uart1_buff, 1);
  
  usmart_scan();
}
#endif


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  if(htim->Instance == htim9.Instance)
  {
    T++;
    if(T %100 == 0)
      HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
    
    DataInput();
    
    

    DataSave();
  }       
}
