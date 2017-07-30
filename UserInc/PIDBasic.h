#ifndef __PIDBASIC_H
#define __PIDBASIC_H
typedef struct PIDs
{
    int set_point;
    int last_set_point;
    int prev_set_point;
    int current_point;
    int last_point;
    float gamma;
    int proportion;
    int integral;
    int differential;
    float delta;
    float last_dif_con;
    float last_con;
    float sum_con;
    long sum_error;
    int current_error;
    int last_error;
    int prev_error;
    int err_up_limit;
    int err_up_infinitesimal;
    int err_low_infinitesimal;
    int err_low_limit;
    int upper_bound;
    int lower_bound;
}PID;
#define PIDKalmanFilter 0
#define PIDLowPassFilter 1
#define PIDBound 1
#define PIDDeadZone 1
#define PIDIntegrationSaturation 1
#define PIDIntegrationSeparation 1
#define PIDPartialDifferential 1
void PIDConClear(PID* pid_ptr);
void IncPIDCalc(PID * pid_ptr);
void LocPIDCalc(PID * pid_ptr);
void ImpPIDCalc(PID * pid_ptr);
#endif /*__PIDBASIC_H*/
