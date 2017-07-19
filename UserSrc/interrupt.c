#include "interrupt.h"
#include "SDFatfs.h"
#include "OLEDUI.h"
#include "data.h"

uint16_t in_count;
uint32_t T;

#if UART_IT1
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  static char buff;
  static uint8_t count = 0;
  static uint8_t string_flag = 0;
  
  if(buff == '\r' || buff == '\n' || buff == '\0' ||
                      count == UART_BUFF_SIZE)
  {
    uart1_rx_buff[count] = '\0';
    string_flag = 1;
    if(buff == '\r')
      usmart_scan();
  }
  else
  {
    if(string_flag == 1)
    {
      count = 0;
      memset(&uart1_rx_buff,0,sizeof(uart1_rx_buff));
      string_flag = 0;
    }
    uart1_rx_buff[count++] = buff;
    
  }
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&buff, 1);

  

}

#endif


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  if(htim->Instance == htim9.Instance)
  {
    T += T_PERIOD_MS;
    
    if(T %500 == 0)
      HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
    
    SysCheck();
    DataInput();
    DataProcess();
    if(sys.status == RUNNING)
      DataOutput();
  }       
}
