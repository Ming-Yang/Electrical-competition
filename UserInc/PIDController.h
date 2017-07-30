#ifndef __PIDBASIC_H
#define __PIDBASIC_H
typedef struct PIDs
{
        int set_point;
        int last_set_point;
        int prev_set_point;
        int current_point;
        int last_point;
        int proportion;
        int integral;
        int differential;
        float last_con;
        float sum_con;
        int last_error;
        int prev_error;
        int err_up_infinitesimal;
        int err_low_infinitesimal;
        int upper_bound;
        int lower_bound;
}PID;
#define PIDLowPassFilter 1
#define PIDBound 1
#define PIDDeadZone 1
void IncPIDCalcSum(PID * pid_ptr);
void IncPIDCalc(PID * pid_ptr);
#endif /*__PIDBASIC_H*/
