#include "PIDController.h"

#if PIDKalmanFilter
typedef struct KALMAN_FILTERs
{
        int init;
        float x_last;
        float p_last;
        float Q;
        float R;
        float kg;
        float x_mid;
        float x_now;
        float p_mid;
        float p_now;
        float resrc_data;
}KALMAN_FILTER;
KALMAN_FILTER kf;
void KalmanFilter(KALMAN_FILTER* kf)
{
        kf->x_mid = kf->x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        kf->p_mid = kf->p_last + kf->Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
        kf->kg = kf->p_mid / (kf->p_mid + kf->R); //kg为kalman filter，R为噪�
        kf->x_now = kf->x_mid + kf->kg*(kf->resrc_data - kf->x_mid);//估计出的最优�
        kf->p_now = (1 - kf->kg)*kf->p_mid;//最优值对应的协方�
        kf->p_last = kf->p_now;   //更新covariance�
        kf->x_last = kf->x_now;   //更新系统状态�
}
#endif //PIDKalmanFilter

void IncPIDCalc(PID * pid_ptr)
{
#if PIDKalmanFilter
        //滤波器初始化
        if (kf.init != 1)
        {
          kf.x_last = pid_ptr->current_point;
          kf.p_last = 0.02;
          kf.Q = 20;
          kf.R = 1;
          kf.init = 1;
          return;
        }
        //输入值滤�
        kf.resrc_data = pid_ptr->current_point;
        KalmanFilter(&kf);
        pid_ptr->current_point = kf.x_now;        
#endif //PIDKalmanFilter

        float proportion  = (float)pid_ptr->proportion  /1000.0;
        float integral    = (float)pid_ptr->integral    /1000.0;
        float differential= (float)pid_ptr->differential/1000.0;
        //当前误差
#if   PIDLowPassFilter
        pid_ptr->set_point = 0.1 * pid_ptr->set_point + 0.8 * pid_ptr->last_set_point + 0.1 * pid_ptr->prev_set_point;
#endif  //PIDLowPassFilter
        float current_error  = pid_ptr->set_point - pid_ptr->current_point;
        float const_A = proportion  + integral + differential;
        float const_B = proportion  + differential * 2;
        float const_C = differential;

        //增量计算
        float current_control = const_A * current_error //E[k]�
                - const_B * pid_ptr->last_error //E[k�]�
                + const_C * pid_ptr->prev_error; //E[k�]�
        //存储误差，用于下次计�
        pid_ptr->prev_error = pid_ptr->last_error;
        pid_ptr->last_error = current_error;
        //存储设置�
        pid_ptr->prev_set_point = pid_ptr->last_set_point;
        pid_ptr->last_set_point = pid_ptr->set_point;

        //累计增量�
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

void LocPIDCalc(PID * pid_ptr)
{
        float proportion  = (float)pid_ptr->proportion  /1000.0;
        float integral    = (float)pid_ptr->integral    /1000.0;
        float differential= (float)pid_ptr->differential/1000.0;
        
        int current_error, diff_error;
        current_error  = pid_ptr->set_point - pid_ptr->current_point; //偏差
        diff_error = current_error - pid_ptr->last_error; //微分
        pid_ptr->last_error = current_error;
        
#if PIDIntegrationSaturation
         //抗饱�
        int alpha;//抗饱和系�
        if (pid_ptr->last_con >= pid_ptr->upper_bound)
        {
            if (current_error > 0) 
              alpha = 0;
            else            
              alpha = 1;
        }
        else if (pid_ptr->last_con <= pid_ptr->lower_bound)
        {
            if (current_error > 0) 
              alpha = 1;
            else            
              alpha = 0;
        }
        else
          alpha = 1;
        pid_ptr->sum_error += current_error * alpha; //抗饱和积�
#else        
        pid_ptr->sum_error += current_error; //普通积�
#endif // IntegrationSaturation
        
        float int_con;        
#if PIDIntegrationSeparation
    float beta;
    if (current_error>=pid_ptr->err_up_limit||current_error<=pid_ptr->err_low_limit)
        beta=0.0;
    else if ((current_error>=(pid_ptr->err_up_limit*0.66)&&current_error<=pid_ptr->err_up_limit)//积分分离
           ||(current_error<=(pid_ptr->err_low_limit*0.66)&&current_error>=pid_ptr->err_low_limit))
        beta=0.6;
    else if ((current_error>=(pid_ptr->err_up_limit*0.33) &&current_error<=(pid_ptr->err_up_limit*0.66))
           ||(current_error<=(pid_ptr->err_low_limit*0.33)&&current_error>=(pid_ptr->err_low_limit*0.66)))
        beta=0.9;
    else
        beta=1.0;
    int_con = integral * pid_ptr->sum_error * beta;
#else
    int_con = integral   * pid_ptr->sum_error;
#endif //PIDIntegrationSeparation
    
    float dif_con;
    float delta = 0.2;
#if PIDPartialDifferential
    dif_con = differential * diff_error*(1- delta)
             + delta *  pid_ptr->last_dif_con; //微分
    pid_ptr->last_dif_con = dif_con;    
#else
    dif_con = differential * diff_error;
#endif // PIDPartialDifferential
            
                //累计增量�
#if PIDDeadZone
        if (current_error >= pid_ptr->err_up_infinitesimal
          ||current_error <= pid_ptr->err_low_infinitesimal)
          pid_ptr->sum_con+= proportion * current_error //比例�
                           + int_con                    //积分�
                           + differential * diff_error; //微分�
        else
          pid_ptr->sum_con = pid_ptr->last_con;
#else
        pid_ptr->sum_con = proportion * current_error //比例�
                         + int_con//积分�
                         + differential * diff_error; //微分�
#endif //PIDDeadZone
                
#if PIDBound
        if (pid_ptr->sum_con>=pid_ptr->upper_bound)
          pid_ptr->sum_con = pid_ptr->upper_bound;
        else if (pid_ptr->sum_con<=pid_ptr->lower_bound)
          pid_ptr->sum_con = pid_ptr->lower_bound;
#endif //PIDBound

        
        pid_ptr->last_con = pid_ptr->sum_con;
        
        return;
}

