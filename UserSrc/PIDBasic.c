#include "PIDBasic.h"
void PIDConClear(PID * pid_ptr)
{
        pid_ptr->set_point = 0;
        pid_ptr->last_set_point = 0;
        pid_ptr->prev_set_point = 0;
        pid_ptr->current_point = 0;
        pid_ptr->last_point = 0;
        pid_ptr->gamma = 0;
        pid_ptr->proportion = 0;
        pid_ptr->integral = 0;
        pid_ptr->differential = 0;
        pid_ptr->delta = 0;
        pid_ptr->last_dif_con = 0;
        pid_ptr->sum_con = 0;
        pid_ptr->last_con = 0;
        pid_ptr->sum_error = 0;
        pid_ptr->last_error = 0;
        pid_ptr->prev_error = 0;
        pid_ptr->err_up_limit = 0;
        pid_ptr->err_up_infinitesimal = 0;
        pid_ptr->err_low_infinitesimal = 0;
        pid_ptr->err_low_limit = 0;
        pid_ptr->upper_bound  = 0;
        pid_ptr->lower_bound  = 0;
}

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
        kf->p_mid = kf->p_last + kf->Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
        kf->kg = kf->p_mid / (kf->p_mid + kf->R); //kgΪkalman filter��RΪ����
        kf->x_now = kf->x_mid + kf->kg*(kf->resrc_data - kf->x_mid);//���Ƴ�������ֵ
        kf->p_now = (1 - kf->kg)*kf->p_mid;//����ֵ��Ӧ��Э����
        kf->p_last = kf->p_now;   //����covarianceֵ
        kf->x_last = kf->x_now;   //����ϵͳ״ֵ̬
}
#endif //PIDKalmanFilter

void IncPIDCalc(PID * pid_ptr)
{
#if PIDKalmanFilter
        //�˲�����ʼ��
        if (kf.init != 1)
        {
          kf.x_last = pid_ptr->current_point;
          kf.p_last = 0.02;
          kf.Q = 20;
          kf.R = 1;
          kf.init = 1;
          return;
        }
        //����ֵ�˲�
        kf.resrc_data = pid_ptr->current_point;
        KalmanFilter(&kf);
        pid_ptr->current_point = kf.x_now;        
#endif //PIDKalmanFilter
        
        float proportion  = (float)pid_ptr->proportion  /1000.0;
        float integral    = (float)pid_ptr->integral    /1000.0;
        float differential= (float)pid_ptr->differential/1000.0;
        //��ǰ���
        int current_error  = pid_ptr->set_point - pid_ptr->current_point;
        float const_A = proportion  + integral + differential;
        float const_B = proportion  + differential * 2;
        float const_C = differential;

        //��������
        float current_control = const_A * current_error //E[k]��
                - const_B * pid_ptr->last_error //E[k��1]��
                + const_C * pid_ptr->prev_error; //E[k��2]��
        //�洢�������´μ���
        pid_ptr->prev_error = pid_ptr->last_error;
        pid_ptr->last_error = current_error;
        //�洢�����������ڵ���
        pid_ptr->last_con = current_control;
        
        //�ۼ�����ֵ
#if DEADZONE
        if (current_error >= pid_ptr->err_up_infinitesimal
          ||current_error <= pid_ptr->err_low_infinitesimal)
          pid_ptr->sum_con += current_control;
#else
          pid_ptr->sum_con += current_control;
#endif //DEADZONE
        
#if BOUND
        if (pid_ptr->sum_con>=pid_ptr->upper_bound)
          pid_ptr->sum_con = pid_ptr->upper_bound;
        else if (pid_ptr->sum_con<=pid_ptr->lower_bound)
          pid_ptr->sum_con = pid_ptr->lower_bound;
#endif //BOUND

        return;
}

void LocPIDCalc(PID * pid_ptr)
{
        float proportion  = (float)pid_ptr->proportion  /1000.0;
        float integral    = (float)pid_ptr->integral    /1000.0;
        float differential= (float)pid_ptr->differential/1000.0;
        
        int current_error, diff_error;
        current_error  = pid_ptr->set_point - pid_ptr->current_point; //ƫ��
        diff_error = current_error - pid_ptr->last_error; //΢��
        pid_ptr->last_error = current_error;
        
#if PIDIntegrationSaturation
         //������
        int alpha;//������ϵ��
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
        pid_ptr->sum_error += current_error * alpha; //�����ͻ���
#else        
        pid_ptr->sum_error += current_error; //��ͨ����
#endif // IntegrationSaturation
        
        float int_con;        
#if PIDIntegrationSeparation
    float beta;
    if (current_error>=pid_ptr->err_up_limit||current_error<=pid_ptr->err_low_limit)
        beta=0.0;
    else if ((current_error>=(pid_ptr->err_up_limit*0.66)&&current_error<=pid_ptr->err_up_limit)//���ַ���
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
#if PIDPartialDifferential
    dif_con = differential * diff_error*(1-pid_ptr->delta)
             + pid_ptr->delta *  pid_ptr->last_dif_con; //΢��
    pid_ptr->last_dif_con = dif_con;    
#else
    dif_con = differential * diff_error;
#endif // PIDPartialDifferential
            
                //�ۼ�����ֵ
#if PIDDeadZone
        if (current_error >= pid_ptr->err_up_infinitesimal
          ||current_error <= pid_ptr->err_low_infinitesimal)
          pid_ptr->sum_con+= proportion * current_error //������
                           + int_con                    //������
                           + differential * diff_error; //΢����
        else
          pid_ptr->sum_con = pid_ptr->last_con;
#else
        pid_ptr->sum_con = proportion * current_error //������
                         + int_con//������
                         + differential * diff_error; //΢����
#endif //DeadZone
                
#if PIDBound
        if (pid_ptr->sum_con>=pid_ptr->upper_bound)
          pid_ptr->sum_con = pid_ptr->upper_bound;
        else if (pid_ptr->sum_con<=pid_ptr->lower_bound)
          pid_ptr->sum_con = pid_ptr->lower_bound;
#endif //Bound

        
        pid_ptr->last_con = pid_ptr->sum_con;
        
        return;
}
