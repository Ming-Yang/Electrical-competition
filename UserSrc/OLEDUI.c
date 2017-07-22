#include "OLEDUI.h"
#include "oled.h"
#include "SDFatfs.h"
#include "flash.h"
#include "data.h"
#include "interrupt.h"

#define BUFF_TIME_MS 2000
PARA_LIST_STRUCT setpara = {0};
OLED_STRUCT oled = {0};;
STATUS_BUTTON_STRUCT button;
SYS_STATUS_STRUCT sys_status;
SYS_STRUCT sys;
uint8_t f_sd_num;

void ShowUnder();
//parameters to be saved in flash should be listed here in order
int32_t* para_table[MAX_PARA_SIZE]={
  &setpara.run_counts,
  &setpara.set_time,
  &setpara.steer.mid,
  &setpara.steer.max,
  
  {0}
};
//parameters to be shown on the screen should be listed here in order
PARA_SHOW_STRUCT para_show_table[MAX_PARA_SIZE]=      
{
  {&setpara.set_time,"SetTime",1},
  {&setpara.steer.mid,"DetectTime",1},
  {&setpara.steer.max,"SteerMid",1},
  
  {0}
};

//data to be saved in sd card should be listed here in order
#define F_PRINTF_D(data) f_printf(&fil,"%d\t",(data))
#define F_PRINTF_N(data) f_printf(&fil,#data"\t")
void DataNameWriteFatfs()
{
  F_PRINTF_N(T);
           
  F_PRINTF_N(indata.mpu6050.acc_x);
  F_PRINTF_N(indata.mpu6050.acc_y);
  F_PRINTF_N(indata.mpu6050.acc_z);
  F_PRINTF_N(indata.mpu6050.gyr_x);
  F_PRINTF_N(indata.mpu6050.gyr_y);
  F_PRINTF_N(indata.mpu6050.gyr_z);
  
  f_printf(&fil,"\n");
}

void DataWriteFatfs()
{
  F_PRINTF_D(T);
  
  F_PRINTF_D(indata.mpu6050.acc_x);
  F_PRINTF_D(indata.mpu6050.acc_y);
  F_PRINTF_D(indata.mpu6050.acc_z);
  F_PRINTF_D(indata.mpu6050.gyr_x);
  F_PRINTF_D(indata.mpu6050.gyr_y);
  F_PRINTF_D(indata.mpu6050.gyr_z);
  
  f_printf(&fil,"\n");
}
//data to be sent through uart oscilloscope should be listed here in order
void SendOscilloscope()
{
  printf("%d,",indata.mpu6050.gyr_x);
  printf("%d,",indata.mpu6050.gyr_y);
  printf("%d,",indata.mpu6050.gyr_z);
  
  printf("%d,",indata.decoder1.raw);
  printf("%d,",indata.decoder2.raw);
  
  printf("\r");
}

void ShowUpper(int8 page)
{
  static int lastpage2;
  if(lastpage2!=page)
  {
    for(int i=0;i<5;LCD_ClearLine(i++));        
    lastpage2=page;
  }
  
  switch(page)
  {
  case 0:       
    oledprintf(0,0,"ACC X:%3d Y:%3d Z:%3d",indata.mpu6050.acc_x,indata.mpu6050.acc_y,indata.mpu6050.acc_z);
    oledprintf(1,0,"GYR X:%3d Y:%3d Z:%3d",indata.mpu6050.gyr_x,indata.mpu6050.gyr_y,indata.mpu6050.gyr_z);
    break;
    
  case 1:
    
    break;
    
  case 2:
    
    break;
    
  default:
    break;
  }
}

void ForceParaChange()
{

}

void SysCheck()
{
  switch(sys.status)
  {
    case READY:break;
    case RUNNING:
      sys.T_RUN += T_PERIOD_MS;
      if(sys.T_RUN >= setpara.set_time || sys.force_stop == 1)
        sys.status = BLOCKED;
    
    
    
      break;
    case BLOCKED:
      sys.T_RUN += T_PERIOD_MS;
      if(sys.T_RUN >= setpara.set_time + BUFF_TIME_MS)
      {
        sys.status = READY;
        SDFatFsClose();
        DataNoPut();
      }
      break;
    case TIMEOUT:break;
    default:break;
  }
}

void SysRun()
{
  char filename[5] = {0};
  uint32_t t_last = T;
  if(sys.status == READY)
  {
    __disable_irq();
    memset(&sys,0,sizeof(SYS_STRUCT));
    
    sys.sd_write = 1;
    setpara.run_counts++;
    
    Para2Flash();
    __enable_irq();
    
    sprintf(filename,"%d",setpara.run_counts);
    SDFatFSOpen(strcat(filename,".txt"));       //用到HAL_Delay() 不能关中断
    DataNameWriteFatfs();
    
    while(T - t_last < 1000);
    sys.status = RUNNING;
    DataOutput();
  }
  else
  {
    printf("Not Ready!\r\n");
  }
}

void SysStop()
{
  if(sys.status == RUNNING)
    sys.status = BLOCKED;
}

/*************short*************long****************pro_long***/
/*press********确认*************运行****************停止运行**/
/*push*********改变精度****************************************/
/*up***********翻页*************保存参数************使用自定义参数*/
/*down*********翻页*************发送SD卡*************************/
/****************************************************************/
void CheckKey()
{
  uint32_t pushtime = T;
  
  if(button==PRESS||button==PUSH)
    OLED_Init();        
  
  switch(button)
  {
  case PRESS:                
    while(!PRESS_IN);
    if(T-pushtime<500)       
    {
      oled.changepara ^= 1;  
    }
    else if(T-pushtime<5000)
    {
      SysRun();
    }
    else
    {
      sys.force_stop = 1;
    }
    break;
  case PUSH:
    while(!PUSH_IN);
    if(T-pushtime<500)                              
    {
      oled.precision *= 10;
      if(oled.precision == 1000)
        oled.precision = 1;
    }
    else if(T-pushtime<5000)  
    {
      
    }
    else
    {
      
    }
    break;
  case UP:
    while(!UP_IN);
    if(T-pushtime<500)
    {
      if(oled.changepara)   
      {
        if(oled.para_select >0)
          oled.para_select --;
        else
          oled.para_select = oled.para_num-1;
      }
      else                 
      {
        if(oled.showpage > oled.showpage_min)
          oled.showpage --;
        else
          oled.showpage = 0;
      }
    }
    else if(T-pushtime<2000)
    {
      Para2Flash();
    }
    else
    {
      ForceParaChange();
    }
    break;  
  case DOWN:
    while(!DOWN_IN);
    if(T-pushtime<500)
    {
      if(oled.changepara)   
      {
        if(oled.para_select <oled.para_num-1)
          oled.para_select ++;
        else
          oled.para_select = 0;
      }
      else                 
      {
        if(oled.showpage < oled.showpage_max)
          oled.showpage ++;
        else
          oled.showpage = 0;
      }
    }
    else if(T-pushtime<2000)
    {
      if(sys.status == READY)
      {
        char filename[5] = {0};
        sprintf(filename,"%d",setpara.run_counts);
        SDFatFSRead(strcat(filename,".txt"));
        delay_ms(5000);
      }
    }
    else
    {
      
    }
    break;
  default:
    break;
  }
  button = NONE;
}

void OledShow()
{
  static int lastpage1;
  if(lastpage1!=oled.showpage)
  {
    LCD_CLS();
    lastpage1=oled.showpage;
    oled.para_select = 0;
  }   
  if(1) 
  {
    if(oled.showpage >= 0 && oled.showpage <= oled.showpage_max)
    {
      ShowUpper(oled.showpage);
      ShowUnder();
    }
    if(oled.showpage == -1)
    {
      
    } 
    if(oled.showpage == -2)
    { 
      
    }
  }
}

void ShowUnder()
{
  int temp_para_select = oled.para_select;      
  if(temp_para_select>0)
  {
    oledprintf(5,0,"%02d.%-13s",temp_para_select-1,para_show_table[temp_para_select-1].label);
    oledprintf(5,96,"%5d",*para_show_table[temp_para_select-1].para);
  }
  else
  {
    LCD_ClearLine(5);
  }
  
  if(oled.changepara)
  {
    oledprintf(6,0,"%02d.%-13s",temp_para_select,para_show_table[temp_para_select].label);
    oledprintfw(6,96,"%5d",*para_show_table[temp_para_select].para);
  }
  else
  {  
    oledprintfw(6,0,"%02d.%-13s",temp_para_select,para_show_table[temp_para_select].label);
    oledprintf(6,96,"%5d",*para_show_table[temp_para_select].para);
  }
  
  if(temp_para_select<oled.para_num-1)
  {
    oledprintf(7,0,"%02d.%-13s",temp_para_select+1,para_show_table[temp_para_select+1].label);
    oledprintf(7,96,"%5d",*para_show_table[temp_para_select+1].para);
  }  
  else
  {
    LCD_ClearLine(7);
  }
}

void ChangePara(char event) 
{    
  if(oled.showpage >= 0)
  {
    switch(event)
    {                
    case 1:
      *para_show_table[oled.para_select].para += para_show_table[oled.para_select].precision*oled.precision;
      break;
    case 2:
      *para_show_table[oled.para_select].para -= para_show_table[oled.para_select].precision*oled.precision;
      break;
    default:
      break;
    }
  }
} 

void Para2Flash()
{
  int32_t para_buff[MAX_PARA_SIZE];
  printf("flash save begin:\r\n");
  __disable_irq();
  for(int i=0;i<MAX_PARA_SIZE;i++)
  {
    para_buff[i] = *para_table[i];
  }
  
  WriteFlash(para_buff, FLASH_USER_START_ADDR_1, MAX_PARA_SIZE);
  __enable_irq();
  printf("flash save finish!\r\n");
}

void UIInit()
{
  int32_t para_buff[MAX_PARA_SIZE];
  
  OLED_Init();
  while(para_show_table[oled.para_num].precision)
    oled.para_num++;
  
  ReadFlash(para_buff, FLASH_USER_START_ADDR_1, MAX_PARA_SIZE);
  for(int i=0;i<MAX_PARA_SIZE;i++)
  {
    *para_table[i] = para_buff[i];
  }
  
  oled.showpage_max = 3;
  oled.showpage_min = -2;
}