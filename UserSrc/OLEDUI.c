#include "OLEDUI.h"
#include "oled.h"
#include "SDFatfs.h"
#include "flash.h"
#include "data.h"
#include "interrupt.h"
#include "PIDController.h"

#define BUFF_TIME_MS 1000
#define LED_SYS_RUN     PEout(6)=0
#define LED_SYS_STOP    PEout(6)=1

PARA_LIST_STRUCT setpara = {0};
OLED_STRUCT oled = {0};
STATUS_BUTTON_STRUCT button;
SYS_STATUS_STRUCT sys_status;
SYS_STRUCT sys;

static void ChangePara(char event);
static void ShowUnder();
static void SysStop();
static void SendParaTable(uint8_t mode);
extern void SysRun();

//parameters to be saved in flash should be listed here in order
int32_t* para_table[MAX_PARA_SIZE]={
  &setpara.run_counts,
  &setpara.set_time,
  &setpara.test_x,
  &setpara.test_y,
  
  &setpara.x_pid.kp,
  &setpara.x_pid.ki,
  &setpara.x_pid.kd,
  &setpara.x_pid.bound,
  &setpara.x_pid.death,
  
  &setpara.y_pid.kp,
  &setpara.y_pid.ki,
  &setpara.y_pid.kd,
  &setpara.y_pid.bound,
  &setpara.y_pid.death,
  
  &setpara.x_error_pid.kp,
  &setpara.x_error_pid.ki,
  &setpara.x_error_pid.kd,
  &setpara.x_error_pid.bound,
  &setpara.x_error_pid.death,
             
  &setpara.y_error_pid.kp,
  &setpara.y_error_pid.ki,
  &setpara.y_error_pid.kd,
  &setpara.y_error_pid.bound,
  &setpara.y_error_pid.death,
  
  &setpara.x_err_err_pid.kp,
  &setpara.x_err_err_pid.ki,
  &setpara.x_err_err_pid.kd,
  &setpara.x_err_err_pid.bound,
  &setpara.x_err_err_pid.death,
                
  &setpara.y_err_err_pid.kp,
  &setpara.y_err_err_pid.ki,
  &setpara.y_err_err_pid.kd,
  &setpara.y_err_err_pid.bound,
  &setpara.y_err_err_pid.death,
  
  &setpara.x_stop_pid.kp,
  &setpara.x_stop_pid.ki,
  &setpara.x_stop_pid.kd,
  &setpara.x_stop_pid.bound,
  &setpara.x_stop_pid.death,
          
  &setpara.y_stop_pid.kp,
  &setpara.y_stop_pid.ki,
  &setpara.y_stop_pid.kd,
  &setpara.y_stop_pid.bound,
  &setpara.y_stop_pid.death,
  
  &setpara.x_circle_pid.kp,
  &setpara.x_circle_pid.ki,
  &setpara.x_circle_pid.kd,
  &setpara.x_circle_pid.bound,
  &setpara.x_circle_pid.death,

  &setpara.y_circle_pid.kp,
  &setpara.y_circle_pid.ki,
  &setpara.y_circle_pid.kd,
  &setpara.y_circle_pid.bound,
  &setpara.y_circle_pid.death,
  
  &setpara.x_energy_pid.kp,
  &setpara.x_energy_pid.ki,
  &setpara.x_energy_pid.kd,
  &setpara.x_energy_pid.bound,
  &setpara.x_energy_pid.death,

  &setpara.y_energy_pid.kp,
  &setpara.y_energy_pid.ki,
  &setpara.y_energy_pid.kd,
  &setpara.y_energy_pid.bound,
  &setpara.y_energy_pid.death,
  
  &setpara.task_num,
  &setpara.theta,
  
//  &setpara,
  
  {0}
};
//parameters to be shown on the screen should be listed here in order
PARA_SHOW_STRUCT para_show_table[MAX_PARA_SIZE]=      
{
  {&setpara.task_num,"Task",1},
  {&setpara.set_time,"SetTime",1},
  {&setpara.run_counts,"Counts",1},
  
  {&setpara.test_x,"x",1},
  {&setpara.test_y,"y",1},
  
  {&setpara.x_pid.kp,"x_P",1},
  {&setpara.x_pid.ki,"x_I",1},
  {&setpara.x_pid.kd,"x_D",1},
             
  {&setpara.y_pid.kp,"y_P",1},
  {&setpara.y_pid.ki,"y_I",1},
  {&setpara.y_pid.kd,"y_D",1},

  {&setpara.x_error_pid.kp,"x_EP",1},
  {&setpara.x_error_pid.ki,"x_EI",1},
  {&setpara.x_error_pid.kd,"x_ED",1},
             
  {&setpara.y_error_pid.kp,"y_EP",1},
  {&setpara.y_error_pid.ki,"y_EI",1},
  {&setpara.y_error_pid.kd,"y_ED",1},

  {&setpara.x_err_err_pid.kp,"x_EEP",1},
  {&setpara.x_err_err_pid.ki,"x_EEI",1},
  {&setpara.x_err_err_pid.kd,"x_EED",1},
                         
  {&setpara.y_err_err_pid.kp,"y_EEP",1},
  {&setpara.y_err_err_pid.ki,"y_EEI",1},
  {&setpara.y_err_err_pid.kd,"y_EED",1},

  {&setpara.x_energy_pid.kp,"x_EEP",1},
  {&setpara.x_energy_pid.ki,"x_EEI",1},
  {&setpara.x_energy_pid.kd,"x_EED",1},
          
  {&setpara.y_energy_pid.kp,"y_EEP",1},
  {&setpara.y_energy_pid.ki,"y_EEI",1},
  {&setpara.y_energy_pid.kd,"y_EED",1},

  {&setpara.theta,"Th",1},
  
//  {&setpara,"",1},
  
  {0}
};

//data to be saved in sd card should be listed here in order
#define F_PRINTF_D(data) f_printf(&fil,"%d\t",(data))
#define F_PRINTF_S(data) f_printf(&fil,#data"\t")
void DataNameWriteFatfs()
{
  F_PRINTF_S(sys.T_RUN);
           
  F_PRINTF_S(outdata.gy25_euler.pitch);
  F_PRINTF_S(outdata.gy25_euler.roll);
  F_PRINTF_S(outdata.gy25_euler.yaw);
  
  F_PRINTF_S(outdata.pwm);
  F_PRINTF_S(outdata.speed);
  F_PRINTF_S(indata.decoder1.angle_v);
  
  f_printf(&fil,"\r\n");
}

void DataWriteFatfs()
{
  F_PRINTF_D(sys.T_RUN);
  
  F_PRINTF_D((int)(100*indata.gy25_euler.pitch));
  F_PRINTF_D((int)(100*indata.gy25_euler.roll));
  F_PRINTF_D((int)(100*indata.gy25_euler.yaw));
  
//  F_PRINTF_D(outdata.pwm);
  F_PRINTF_D((int)(100*outdata.speed));
  F_PRINTF_D((int)(100*indata.decoder1.ang_v));
  
  f_printf(&fil,"\r\n");
}
//data to be sent through uart oscilloscope should be listed here in order
void SendOscilloscope()
{
//  printf("%d,",(int)(((int)setpara.Ek)*100));
//  printf("%d,",(int)(((int)setpara.Ep)*100));
//  printf("%d,",(int)((axis_x_energy.current_point)*100));
//  printf("%d,",(int)((axis_y_energy.sum_con)*10));
//  printf("%d,",(int)((axis_y_energy.set_point-axis_x_energy.current_point)*100));
//  printf("%d,",(int)((axis_y_energy.set_point)*100));
                           
  printf("%d,",(int)((axis_y_circle.set_point)*1000));
  printf("%d,",(int)((axis_y_circle.current_point)*1000));
  printf("%d,",(int)((axis_y_circle.sum_con)*10));
  printf("%d,",(int)((axis_x_circle.set_point)*1000));
  printf("%d,",(int)((axis_x_circle.current_point)*1000));
  printf("%d,",(int)((axis_x_circle.sum_con)*10));
//  printf("%d,",(int)((axis_x.set_point)*1000));
//  printf("%d,",(int)((axis_x.current_point)*1000));
//  printf("%d,",(int)((axis_x.sum_con)*10));
//  printf("%d,",(int)((axis_x_error.set_point)*1000));
//  printf("%d,",(int)((axis_x_error.current_point)*1000));
//  printf("%d,",(int)((axis_x_error.sum_con)*1));
//  printf("%d,",(int)((axis_y_error.set_point)*1000));
//  printf("%d,",(int)((axis_y_error.current_point)*1000));
//  printf("%d,",(int)((axis_y_error.sum_con)*1));
  
//  printf("%d,",(int)((axis_y.current_point)*100));
//  printf("%d,",(int)((axis_y.set_point)*100));
//  printf("%d,",(int)((axis_y.sum_con)*10));
  
  printf("%d,",(int)((axis_x.last_error)*1000));
  printf("%d,",(int)((axis_x.last_con)*1000));
  printf("%d,",(int)((sys.T_RUN)*1));
  
  
//  
//  
//  printf("%d,",(int)((euler2speed.prev_error - euler2speed.last_error)*100));
//  printf("%d,",(int)(euler2speed.current_point*100));
//  printf("%d,",(int)(euler2speed.sum_con));
  
  printf("\r\n");
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
    oledprintf(0,0,"G:%4.2fE:%4.2f",indata.gy25_euler.roll ,indata.global_euler.roll );
    oledprintf(1,0,"G %4.2fE %4.2f",indata.gy25_euler.pitch,indata.global_euler.pitch);
    oledprintf(2,0,"G %4.2fE %4.2f",indata.gy25_euler.yaw  ,indata.global_euler.yaw  );
    oledprintf(3,0,"x:%6d,y:%6d",outdata.pwm_x,outdata.pwm_y);
    oledprintf(4,0,"AD:%5d,T:%4.1f",indata.adc10,T/1000.0f);
    break;
    
  case 1:
    oledprintf(0,0,"A X%4d,Y%4d,Z%4d",indata.mpu6050.acc_x>>8,indata.mpu6050.acc_y>>8,indata.mpu6050.acc_z>>8);
    oledprintf(1,0,"G X%4d,Y%4d,Z%4d",indata.mpu6050.gyr_x>>8,indata.mpu6050.gyr_y>>8,indata.mpu6050.gyr_z>>8);
    oledprintf(2,0,"E R%4d,P%4d,Y%4d",(int)outdata.euler.roll,(int)outdata.euler.pitch,(int)outdata.euler.yaw);
//    oledprintf(0,0,"",);
//    oledprintf(1,0,"",);
//    oledprintf(2,0,"",);
//    oledprintf(3,0,"",);
//    oledprintf(4,0,"",);
    
    break;
    
  case 2:
//    oledprintf(0,0,"",);
//    oledprintf(1,0,"",);
//    oledprintf(2,0,"",);
//    oledprintf(3,0,"",);
//    oledprintf(4,0,"",);
    
    break;
    
  default:
    break;
  }
}

void ForceParaChange()
{
  for(int i=0;i<oled.para_num;i++)
  {
    if(abs(*para_show_table[i].para) > 500000000)
      *para_show_table[i].para = 0;
  }
}

void SysCheck()
{
  switch(sys.status)
  {
  case READY:
    break;
  case RUNNING:
    sys.T_RUN += T_PERIOD_MS;
    if(sys.T_RUN >= setpara.set_time*100 || 
                                              sys.force_stop == 1)
      sys.status = BLOCKED;

    break;
  case BLOCKED:
    sys.T_RUN += T_PERIOD_MS;
    if(sys.T_RUN >= setpara.set_time*100 + BUFF_TIME_MS ||
                                              sys.force_stop == 1)
    {
        SysStop();
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
    memset(&sys,0,sizeof(SYS_STRUCT)); 
    memset(&indata,0,sizeof(DATA_IN_STRUCT));
    setpara.run_counts++;
    
    ClearPIDCach(&axis_x);
    ClearPIDCach(&axis_y);
    ClearPIDCach(&axis_x_error);
    ClearPIDCach(&axis_y_error);
    ClearPIDCach(&axis_x_err_err);
    ClearPIDCach(&axis_y_err_err);
    ClearPIDCach(&axis_x_stop);
    ClearPIDCach(&axis_y_stop);
    ClearPIDCach(&axis_x_circle);
    ClearPIDCach(&axis_y_circle);
    ClearPIDCach(&axis_x_energy);
    ClearPIDCach(&axis_y_energy);
    
    Para2Flash();
    
    sprintf(filename,"%d",setpara.run_counts);
    SDFatFSOpen(strcat(filename,".txt"));       //用到HAL_Delay() 不能关中断
    DataNameWriteFatfs();
    

    while(T - t_last < BUFF_TIME_MS);
    LCD_CLS();
    sys.sd_write = 1;
    LED_SYS_RUN;
    sys.status = RUNNING;
  }
  else
  {
    printf("Not Ready!\r\n");
  }
}

void SysStop()
{
  sys.force_stop = 0;
  sys.status = READY;
  sys.sd_write = 0;
  
  SendParaTable(0);
  
  SDFatFsClose();
  LED_SYS_STOP;
}

/*************short*************long****************pro_long***/
/*press********确认*************运行****************停止运行**/
/*push*********改变精度*********发送参数表*********************/
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
    else if(T-pushtime<2000)
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
    else if(T-pushtime<2000)  
    {
      SendParaTable(1);
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
        LCD_CLS();
        oledprintf(3,3,"Sending SD");
        sys.osc_suspend = 1;
        sprintf(filename,"%d",setpara.run_counts);
        SDFatFSRead(strcat(filename,".txt"));
        LCD_CLS();
      }
    }
    else
    {
      
    }
    break;
    
  case CW:
    if(oled.showpage >= 0)
      {
        if(oled.changepara)
          ChangePara(1);
        else 
        {
          if(oled.para_select <oled.para_num-1)
            oled.para_select ++;
          else
            oled.para_select = 0;
        }
      }
    break;

  case CCW:
    if(oled.showpage >= 0)
      {
        if(oled.changepara)
          ChangePara(2);
        else
        {
          if(oled.para_select >0)
            oled.para_select --;
          else
            oled.para_select = oled.para_num-1;
        }
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

static void ShowUnder()
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

static void ChangePara(char event) 
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
  
  LCD_CLS();
  oledprintf(3,3,"Saving Flash");
  printf("flash save begin:\r\n");
  
  for(int i=0;i<MAX_PARA_SIZE;i++)
  {
    para_buff[i] = *para_table[i];
  }
  
  WriteFlash(para_buff, FLASH_USER_START_ADDR_1, MAX_PARA_SIZE);
  printf("flash save finish!\r\n");
  delay_ms(100);
  LCD_CLS();
}

void SendParaTable(uint8_t mode)
{
  for(int i=0;i<oled.para_num;i++)
  {
    if(mode)
      printf("%s = %d\r\n",para_show_table[i].label, *(para_show_table[i].para));
    else
      f_printf(&fil, "%s = %d;\r\n",para_show_table[i].label, *(para_show_table[i].para));
  }
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
  
  oled.precision = 1;
  oled.showpage_max = 3;
  oled.showpage_min = -2;
}