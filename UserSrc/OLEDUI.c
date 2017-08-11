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

SYS_STRUCT sys;
OLED_STRUCT oled = {0};
PARA_LIST_STRUCT setpara;
STATUS_BUTTON_STRUCT button;
SYS_STATUS_STRUCT sys_status;

uint8_t frame[60][80];

static void ChangePara(char event);
static void ShowUnder();
static void SysStop();
static void SendParaTable(uint8_t mode);
static void ShowFrame(uint8_t xPos,uint8_t yPos);
extern void SysRun();

//parameters to be saved in flash should be listed here in order
int32_t* para_table[MAX_PARA_SIZE]={
  &setpara.run_counts,
  &setpara.set_time,
  &setpara.task_num,
  
  &setpara.test_1,
  &setpara.test_2,
  &setpara.test_3,
  &setpara.test_4,
  
  &setpara.pid_x.bound,
  &setpara.pid_x.death,
  &setpara.pid_x.kd,
  &setpara.pid_x.ki,
  &setpara.pid_x.kp,
  
  &setpara.pid_y.bound,
  &setpara.pid_y.death,
  &setpara.pid_y.kd,
  &setpara.pid_y.ki,
  &setpara.pid_y.kp,
  
//  &setpara,
  
  {0}
};
//parameters to be shown on the screen should be listed here in order
PARA_SHOW_STRUCT para_show_table[MAX_PARA_SIZE]=      
{
  {&setpara.task_num,"Task",1},
  {&setpara.set_time,"SetTime",1},
  {&setpara.run_counts,"Counts",1},
  
  {&setpara.test_1,"T1",1},
  {&setpara.test_2,"T2",1},
  {&setpara.test_3,"T3",1},
  {&setpara.test_4,"T4",1},
  
  {&setpara.pid_x.kp,"X_P",1},
  {&setpara.pid_x.ki,"X_I",1},
  {&setpara.pid_x.kd,"X_D",1},
  
  {&setpara.pid_y.kp,"Y_P",1},
  {&setpara.pid_y.ki,"Y_I",1},
  {&setpara.pid_y.kd,"Y_D",1},
  
//  {&setpara,"",1},
  
  {0}
};

//data to be saved in sd card should be listed here in order
#define F_PRINTF_D(data) f_printf(&fil,"%d\t",(data))
#define F_PRINTF_S(data) f_printf(&fil,#data"\t")
void DataNameWriteFatfs()
{
  F_PRINTF_S(sys.T_RUN);
           
//  F_PRINTF_S(outdata.gy25_euler.pitch);
//  F_PRINTF_S(outdata.gy25_euler.roll);
//  F_PRINTF_S(outdata.gy25_euler.yaw);
//  
//  F_PRINTF_S(outdata.pwm);
//  F_PRINTF_S(outdata.speed);
//  F_PRINTF_S(indata.decoder1.angle_v);
  
  f_printf(&fil,"\r\n");
}

void DataWriteFatfs()
{
  F_PRINTF_D(sys.T_RUN);
  
//  F_PRINTF_D((int)(100*indata.gy25_euler.pitch));
//  F_PRINTF_D((int)(100*indata.gy25_euler.roll));
//  F_PRINTF_D((int)(100*indata.gy25_euler.yaw));
  
//  F_PRINTF_D(outdata.pwm);
//  F_PRINTF_D((int)(100*outdata.speed));
//  F_PRINTF_D((int)(100*indata.decoder1.ang_v));
  
  f_printf(&fil,"\r\n");
}
//data to be sent through uart oscilloscope should be listed here in order
void SendOscilloscope()
{
//  printf("%d, ",((int)(indata.ball_position.x*10)));
//  printf("%d, ",((int)(indata.ball_position.y*10)));
  printf("%d, ",((int)(pid_x.current_point*10)));
  printf("%d, ",((int)(pid_x.set_point*10)));
  printf("%d, ",((int)(pid_x.sum_con*10)));

  printf("%d, ",((int)(pid_y.current_point*10)));
  printf("%d, ",((int)(pid_y.set_point*10)));
  printf("%d, ",((int)(pid_y.sum_con*10)));
  
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
    oledprintf(0,0,"X:%4.2f Y:%4.2f",indata.ball_position.x,indata.ball_position.y);
//    oledprintf(1,0,"G %4.2fE %4.2f",indata.global_euler.pitch,indata.gy25_euler.pitch);
//    oledprintf(2,0,"G %4.2fE %4.2f",indata.global_euler.yaw,indata.gy25_euler.yaw);
//    oledprintf(3,0,"x:%6d,y:%6d",outdata.pwm_x,outdata.pwm_y);
    oledprintf(4,0,"T_R:%4.1f T:%4.1f",sys.T_RUN/1000.0f,T/1000.0f);
    break;
    
  case 1:
//    oledprintf(0,0,"A X%4d,Y%4d,Z%4d",indata.mpu6050.acc_x>>8,indata.mpu6050.acc_y>>8,indata.mpu6050.acc_z>>8);
//    oledprintf(1,0,"G X%4d,Y%4d,Z%4d",indata.mpu6050.gyr_x>>8,indata.mpu6050.gyr_y>>8,indata.mpu6050.gyr_z>>8);
//    oledprintf(2,0,"E R%4d,P%4d,Y%4d",(int)outdata.euler.roll,(int)outdata.euler.pitch,(int)outdata.euler.yaw);
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
    if(abs(*para_show_table[i].para) > 500000000 || *para_show_table[i].para == -1)
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
    if(sys.T_RUN >= setpara.set_time*100)
      sys.status = BLOCKED;

    break;
  case BLOCKED:
    sys.T_RUN += T_PERIOD_MS;
    if(sys.T_RUN >= setpara.set_time*100 + BUFF_TIME_MS)
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
    memset(&outdata,0,sizeof(DATA_OUT_STRUCT));
    setpara.run_counts++;
    
    ClearPIDCach(&pid_x);
    ClearPIDCach(&pid_y);
    
    Para2Flash();
#if     SD_ENABLE
    sprintf(filename,"%d",setpara.run_counts);
    SDFatFSOpen(strcat(filename,".txt"));       //用到HAL_Delay() 不能关中断
    DataNameWriteFatfs();
#endif
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
#if     SD_ENABLE
  SendParaTable(0);  
  SDFatFsClose();
#endif
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
    else if(T-pushtime<1000)
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
#if     SD_ENABLE
        char filename[5] = {0};
        LCD_CLS();
        oledprintf(3,3,"Sending SD");
        sys.osc_suspend = 1;
        sprintf(filename,"%d",setpara.run_counts);
        SDFatFSRead(strcat(filename,".txt"));
        LCD_CLS();
#endif
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

void LiveDebug()
{
  if(sys.force_start == 1)
  {
    sys.force_start = 0;
    SysRun();
  }
  if(sys.force_stop == 1)
  {
    sys.force_stop = 0;
    SysStop();
  }
  if(sys.force_flash == 1)
  {
    sys.force_flash = 0;
    SendParaTable(0);
    Para2Flash();
  }
  if(sys.force_sd == 1)
  {
    sys.force_sd = 0;
    
    char filename[5] = {0};
    LCD_CLS();
    oledprintf(3,3,"Sending SD");
    sys.osc_suspend = 1;
    sprintf(filename,"%d",setpara.run_counts);
    SDFatFSRead(strcat(filename,".txt"));
    LCD_CLS();
  }
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
      ShowFrame(((uint8_t)(indata.ball_position.x/2)),((uint8_t)(indata.ball_position.y/2)));
      oledprintf(0,83,"x:%.1f",indata.ball_position.x);
      oledprintf(1,83,"y:%.1f",indata.ball_position.y);
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

void ShowFrame(uint8_t xPos,uint8_t yPos)
{
  for(int y=0;y<60;y++)
  {
    for(int x=0;x<80;x++)
    {
      frame[y][x]=1;
      frame[y][0] = 0;
      frame[y][79] = 0;
      frame[0][x] = 0;
      frame[59][x] = 0;
    }
  }
  frame[xPos][yPos]=0;
  uint8_t oled[1024];
  for(int y=0;y<8;y++)
    for(int x=0;x<128;x++)
    {
      oled[128*y+x]=0;
      for(int i=0;i<8;i++)
      {
        oled[128*y+x]>>=1;
        if(frame[y*8+i][x]==0x00) 
        {
          if(x<80 && y*8+i<60)
          {
            oled[128*y+x]+=0x80;
          }
        }
      }
    }
  full_bmp(oled);
}