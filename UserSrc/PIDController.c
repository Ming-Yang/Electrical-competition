#include "PIDController.h"
#include "OLEDUI.h"

PID pid_x;
PID pid_y;

void IncPIDCalc(PID * pid_ptr)
{
        float proportion  = (float)pid_ptr->proportion  /100.0;
        float integral    = (float)pid_ptr->integral    /100.0;
        float differential= (float)pid_ptr->differential/100.0;
        //当前误差
#if   PIDLowPassFilter
        pid_ptr->set_point = 0.1 * pid_ptr->set_point + 0.8 * pid_ptr->last_set_point + 0.1 * pid_ptr->prev_set_point;
#endif  //PIDLowPassFilter
        float current_error  = pid_ptr->set_point - pid_ptr->current_point;
        float const_A = proportion  + integral + differential;
        float const_B = proportion  + differential * 2;
        float const_C = differential;

        //增量计算
        float current_control = const_A * current_error //E[k]项
                - const_B * pid_ptr->last_error //E[k－1]项
                + const_C * pid_ptr->prev_error; //E[k－2]项
        //存储误差，用于下次计算
        pid_ptr->prev_error = pid_ptr->last_error;
        pid_ptr->last_error = current_error;
        //存储设置值
        pid_ptr->prev_set_point = pid_ptr->last_set_point;
        pid_ptr->last_set_point = pid_ptr->set_point;

        //累计增量值
#if PIDDeadZone
        //存储控制量，输出中间量，也可用于调试
        pid_ptr->last_con = current_control;
        if (current_error >= pid_ptr->err_up_infinitesimal
                        ||current_error <= pid_ptr->err_low_infinitesimal)
                pid_ptr->sum_con += current_control;
#else
        pid_ptr->sum_con += current_control;
#endif //PIDDeadZone

#if PIDBound
        if (pid_ptr->sum_con>=pid_ptr->upper_bound)
                pid_ptr->sum_con = pid_ptr->upper_bound;
        else if (pid_ptr->sum_con<=pid_ptr->lower_bound)
                pid_ptr->sum_con = pid_ptr->lower_bound;
#endif //PIDBound

        return;
}

void GetPIDPara(PID * pid, PID_PARA_STRUCT *para)
{
  pid->upper_bound = para->bound;
  pid->lower_bound = -para->bound;
  pid->err_up_infinitesimal = para->death;
  pid->err_low_infinitesimal = - para->death;
  
  pid->proportion   = para->kp;
  pid->integral     = para->ki;
  pid->differential = para->kd;
}

void ClearPIDCach(PID *pid)
{
  memset(pid,0,sizeof(PID));
}