#include "OLEDUI.h"
#include "oled.h"
#include "SDFatfs.h"
#include "flash.h"
#include "data.h"
#include "interrupt.h"
/****PIDtest****/  
#include "PIDController.h"
PID euler_speed ;
PID speed_pwm;
/****\PIDtest****/  

#define BUFF_TIME_MS 2000
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

//parameters to be saved in flash should be listed here in order
int32_t* para_table[MAX_PARA_SIZE]={
  &setpara.run_counts,
  &setpara.set_time,
  &setpara.steer.mid,
  &setpara.steer.max,
  &setpara.test,
  &setpara.test2,
  &setpara.speed_pid.kp,
  &setpara.speed_pid.ki,
  &setpara.speed_pid.kd,
  &setpara.angle_pid.kp,
  &setpara.angle_pid.ki,
  &setpara.angle_pid.kd,
  &setpara.pid_para.speed_kp,
  &setpara.pid_para.speed_ki,
  &setpara.pid_para.speed_kd,
  &setpara.pid_para.angle_kp,
  &setpara.pid_para.angle_ki,
  &setpara.pid_para.angle_kd,
  
//  &setpara,
  
  {0}
};
//parameters to be shown on the screen should be listed here in order
PARA_SHOW_STRUCT para_show_table[MAX_PARA_SIZE]=      
{
  {&setpara.set_time,"SetTime",1},
  {&setpara.run_counts,"Counts",1},
  {&setpara.test,"Test",1},
  {&setpara.test2,"Test2",1},
  {&setpara.speed_pid.kp,"Pspeed",1},
  {&setpara.speed_pid.ki,"Ispeed",1},
  {&setpara.speed_pid.kd,"Dspeed",1},
  {&setpara.angle_pid.kp,"Pangle",1},
  {&setpara.angle_pid.ki,"Iangle",1},
  {&setpara.angle_pid.kd,"Dangle",1},
  {&setpara.pid_para.speed_kp,"Pspeed",1},
  {&setpara.pid_para.speed_ki,"Ispeed",1},
  {&setpara.pid_para.speed_kd,"Dspeed",1},
  {&setpara.pid_para.angle_kp,"Pangle",1},
  {&setpara.pid_para.angle_ki,"Iangle",1},
  {&setpara.pid_para.angle_kd,"Dangle",1},
  
//  {&setpara,"",1},
  
  {0}
};

//data to be saved in sd card should be listed here in order
#define F_PRINTF_D(data) f_printf(&fil,"%d\t",(data))
#define F_PRINTF_S(data) f_printf(&fil,#data"\t")
void DataNameWriteFatfs()
{
  F_PRINTF_S(sys.T_RUN);
           
  F_PRINTF_S(indata.mpu6050.acc_x);
  F_PRINTF_S(indata.mpu6050.acc_y);
  F_PRINTF_S(indata.mpu6050.acc_z);
  F_PRINTF_S(indata.mpu6050.gyr_x);
  F_PRINTF_S(indata.mpu6050.gyr_y);
  F_PRINTF_S(indata.mpu6050.gyr_z);
           
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
  
  F_PRINTF_D(indata.mpu6050.acc_x);
  F_PRINTF_D(indata.mpu6050.acc_y);
  F_PRINTF_D(indata.mpu6050.acc_z);
  F_PRINTF_D(indata.mpu6050.gyr_x);
  F_PRINTF_D(indata.mpu6050.gyr_y);
  F_PRINTF_D(indata.mpu6050.gyr_z);
  
  F_PRINTF_D((int)(100*outdata.gy25_euler.pitch));
  F_PRINTF_D((int)(100*outdata.gy25_euler.roll));
  F_PRINTF_D((int)(100*outdata.gy25_euler.yaw));
  
  F_PRINTF_D(outdata.pwm);
  F_PRINTF_D((int)(100*outdata.speed));
  F_PRINTF_D((int)(100*indata.decoder1.ang_v));
  
  f_printf(&fil,"\r\n");
}
//data to be sent through uart oscilloscope should be listed here in order
void SendOscilloscope()
{
//  printf("%d,",(int)(outdata.speed*10));
//  printf("%d,",outdata.pwm);
  
//  printf("%d,",(int)(outdata.euler.roll *100));
//  printf("%d,",(int)(outdata.euler.pitch*100));
//  printf("%d,",(int)(outdata.euler.yaw  *100));  
//  printf("\r\n");
  
  printf("%d,",(int)(speed_pwm.set_point)*1);
  printf("%d,",(int)(speed_pwm.current_point));
  printf("%d,",(int)(speed_pwm.sum_con));
  
  
  printf("%d,",(int)(euler_speed.set_point));
  printf("%d,",(int)(euler_speed.current_point));
  printf("%d,",(int)(euler_speed.sum_con)/10);
  
//  printf("%d,",indata.mpu6050.acc_x);
//  printf("%d,",indata.mpu6050.acc_y);
//  printf("%d,",indata.mpu6050.acc_z);
//  printf("%d,",indata.mpu6050.gyr_x);
//  printf("%d,",indata.mpu6050.gyr_y);
//  printf("%d,",pid_test.current_point);
//  printf("%d,",pid_test.set_point); 
//  printf("%d,",(int)pid_test.last_con);
//  printf("%d,",(int)(pid_test.proportion  ));
//  printf("%d,",(int)(pid_test.integral    ));
//  printf("%d,",(int)(pid_test.differential));
//  printf("%d,",0);   
//  printf("%d,",pwm_con);
//  printf("%d,",pid_test.last_error);
//  printf("%d,",indata.mpu6050.gyr_z);
  

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
    oledprintf(0,0,"g1:%3.2f,s:%3.2f",indata.decoder1.ang_v,outdata.speed);
    oledprintf(1,0,"PWM1:%4d,WPM2:%4d",outdata.tim2.channel1,outdata.tim2.channel2);
    oledprintf(2,0,"E R%4d,P%4d,Y%4d",(int)outdata.gy25_euler.roll,(int)outdata.gy25_euler.pitch,(int)outdata.gy25_euler.yaw);
    oledprintf(3,0,"c1:%6d,c2:%6d",indata.decoder1.raw,indata.decoder2.raw);
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
  
}

void SysCheck()
{
  switch(sys.status)
  {
  case READY:break;
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
    
    Para2Flash();
    
    sprintf(filename,"%d",setpara.run_counts);
    SDFatFSOpen(strcat(filename,".txt"));       //用到HAL_Delay() 不能关中断
    DataNameWriteFatfs();
    
    /****PIDtest****/     
  memset(&euler_speed,0,sizeof(euler_speed));
  euler_speed.proportion   =    setpara.pid_para.speed_kp;
  euler_speed.integral     =    setpara.pid_para.speed_ki;
  euler_speed.differential =    setpara.pid_para.speed_kd;
  euler_speed.upper_bound = 2000.0;
  euler_speed.lower_bound = - euler_speed.upper_bound;
  euler_speed.err_up_limit = 1000;
  euler_speed.err_up_infinitesimal = 0.1;
  euler_speed.err_low_infinitesimal = - euler_speed.err_up_infinitesimal;
  euler_speed.err_low_limit = - euler_speed.err_up_limit;
  
  
  memset(&speed_pwm,0,sizeof(speed_pwm));
  speed_pwm.proportion   =    setpara.pid_para.angle_kp;
  speed_pwm.integral     =    setpara.pid_para.angle_ki;
  speed_pwm.differential =    setpara.pid_para.angle_kd;
  speed_pwm.upper_bound = 10000.0;
  speed_pwm.lower_bound = -speed_pwm.upper_bound;
  speed_pwm.err_up_limit = 24;
  speed_pwm.err_up_infinitesimal = 25.0;
  speed_pwm.err_low_infinitesimal = - speed_pwm.err_up_infinitesimal;
  speed_pwm.err_low_limit = - speed_pwm.err_up_limit;
  //初始化结束
    /********/
    
    while(T - t_last < BUFF_TIME_MS);
    LCD_CLS();
    sys.sd_write = 1;
    LED_SYS_RUN;
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
  sys.status = READY;
  sys.sd_write = 0;
  SDFatFsClose();
  LED_SYS_STOP;
//  
//  char filename[5];
//  sys.osc_suspend = 1;
//  sprintf(filename,"%d",setpara.run_counts);
//  SDFatFSRead(strcat(filename,".txt"));
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
      extern uint8_t bImuReady;
      bImuReady = 0;
      InitOffset6050(&indata.mpu6050,&mpu6050_offset);
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
        sys.osc_suspend = 1;
        sprintf(filename,"%d",setpara.run_counts);
        SDFatFSRead(strcat(filename,".txt"));
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