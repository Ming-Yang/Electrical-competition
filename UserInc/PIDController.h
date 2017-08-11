#ifndef __PIDBASIC_H
#define __PIDBASIC_H
#include "userinc.h"
#include "OLEDUI.h"
typedef struct PIDs
{
        float set_point;
        float last_set_point;
        float prev_set_point;
        float current_point;
        float last_point;
        float proportion;
        float integral;
        float differential;
        float last_con;
        float sum_con;
        float last_error;
        float prev_error;
        float err_up_infinitesimal;
        float err_low_infinitesimal;
        float upper_bound;
        float lower_bound;
}PID;
#define PIDLowPassFilter 0
#define PIDBound 1
#define PIDDeadZone 1

void IncPIDCalc(PID * pid_ptr);
void GetPIDPara(PID * pid, PID_PARA_STRUCT *para);
void ClearPIDCach(PID *pid);

extern PID pid_x;
extern PID pid_y;
#endif /*__PIDBASIC_H*/
