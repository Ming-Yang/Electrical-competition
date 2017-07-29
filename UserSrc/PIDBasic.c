#include "PIDBasic.h"
typedef struct PIDs
{
    int LastSetPoint;
    int PrevSetPoint;
    int LastPoint;
    float Gamma;
    float Proportion;
    float Integral;
    float Differential;
    float Delta;
    float LastDifCon;
    float LastCon;
    long SumError;
    int LastError;
    int PrevError;
    int ErrUpLimit;
    int ErrUInfinitesimal;
    int ErrDInfinitesimal;
    int ErrLowLimit;
    int UpperBound;
    int LowerBound;
}PID;
static PID pid;

void KPIDInit(PIDC* pidc)
{
    pid.Proportion  = (float)pidc->Kp1000/1000.0; //�������� Proportional Const
    pid.Integral    = (float)pidc->Ki1000/1000.0; //���ֳ��� Integral Const
    pid.Differential= (float)pidc->Kd1000/1000.0; //΢�ֳ��� Differential Const
}

void AllPIDCClear(PIDC* pidc)
{
    pid.LastSetPoint = 0;
    pid.PrevSetPoint = 0;
    pid.LastPoint = 0;
    pid.Gamma      = 0;
    pid.Proportion  = 0;
    pid.Integral    = 0;
    pid.Differential= 0;
    pid.Delta       = 0;
    pid.LastDifCon = 0;
    pid.LastCon = 0;
    pid.SumError = 0;
    pid.LastError = 0;
    pid.PrevError = 0;
    pid.ErrUpLimit = 0;
    pid.ErrUInfinitesimal = 0;
    pid.ErrUInfinitesimal = 0;
    pid.ErrLowLimit = 0;
    pid.UpperBound = 0;
    pid.LowerBound = 0;
}
void IncPIDinit(PIDC* pidc)
{
     AllPIDCClear(pidc);
     KPIDInit(pidc);
}

//increase pid calaulate
int IncPIDCalc(int set_point, int next_point)
{
        int ierror;
        float const_A, const_B, const_C, incpid;
        //��ǰ���
        ierror  = set_point - next_point;
        const_A = (float)(pid.Proportion + pid.Integral + pid.Differential);
        const_B = (float)(pid.Proportion + pid.Differential * 2);
        const_C = (float)(pid.Differential);

        //��������
        incpid = const_A * ierror //E[k]��
               - const_B * pid.LastError //E[k��1]��
               + const_C * pid.PrevError; //E[k��2]��
        //�洢�������´μ���
        pid.PrevError = pid.LastError;
        pid.LastError = ierror;
        //��������ֵ
        return((int)(incpid));
}
//increase pid calaulate
int LocPIDCalc(int set_point, int next_point)
{
    int ierror,derror;
    ierror = set_point - next_point; //ƫ��
    pid.SumError += ierror; //����
    derror = ierror - pid.LastError; //΢��
    pid.LastError = ierror;
    return (int)(pid.Proportion * ierror //������
         + pid.Integral   * pid.SumError //������
         + pid.Differential * derror); //΢����

}


void BasicPIDInit(PIDC* pidc)
{
    AllPIDCClear(pidc);
    #if DifferentialAdvance
    pid.Gamma       = (float)pidc->Gamma1000/1000.0;//΢�����е�ͨ�˲�������
    #endif // DifferentialAdvance 

    pid.ErrUpLimit        = pidc->ErrLimit;
    pid.ErrUInfinitesimal = pidc->DeathZone;
    pid.ErrDInfinitesimal =-pidc->DeathZone;
    pid.ErrLowLimit       =-pidc->ErrLimit;
    pid.UpperBound        = pidc->Threshold;
    pid.LowerBound        =-pidc->Threshold;
}





//����PID�㷨���ں��������źŵ�ͨ�˲��������ͻ��֡����ַ��롢
//�������ơ�������΢������6�ָĽ��Ŀ��Ʒ��������ܷǳ����á�
int BasicPIDCalc(int set_point, int next_point)
{
    int ierror;
    float dcontrol, icontrol, beta;
    


#if LowPass
    //��ͨ�˲���Ӧ�������ź�
    set_point = (int)(0.1 * set_point + 0.8 * pid.LastSetPoint + 0.1 * pid.PrevSetPoint);
#endif // LowPass
    //��ǰ���
    ierror = set_point - next_point;
    //�洢���ֲ���
    pid.PrevSetPoint = pid.LastSetPoint;
    pid.LastSetPoint = set_point;
    pid.LastPoint = next_point;
    pid.LastError = ierror;

    int alpha;//������ϵ��
#if IntegrationSaturation
    //������
    if (pid.LastCon >= pid.UpperBound){
        if (ierror > 0) alpha = 0;
        else            alpha = 1;
    }else if (pid.LastCon <= pid.LowerBound){
        if (ierror > 0) alpha = 1;
        else            alpha = 0;
    }else
#endif // IntegrationSaturation
        alpha = 1;


#if IntegrationSeparation
#if BANGBANG
    if (ierror>=pid.ErrUpLimit){//��������    
        pid.LastCon = pid.UpperBound;
        return pid.UpperBound;
    }
    else if (ierror<=pid.ErrLowLimit){
        pid.LastCon = pid.LowerBound;
        return pid.LowerBound;
    }
#else
    if (ierror>=pid.ErrUpLimit||ierror<=pid.ErrLowLimit)
        beta=0.0;
#endif // BANGBANG
    else if ((ierror>=(pid.ErrUpLimit*0.66)&&ierror<=pid.ErrUpLimit)//���ַ���
           ||(ierror<=(pid.ErrLowLimit*0.66)&&ierror>=pid.ErrLowLimit))
        beta=0.6;
    else if ((ierror>=(pid.ErrUpLimit*0.33) &&ierror<=(pid.ErrUpLimit*0.66))
           ||(ierror<=(pid.ErrLowLimit*0.33)&&ierror>=(pid.ErrLowLimit*0.66)))
        beta=0.9;
    else
#if DeadZone
        if ((ierror>=pid.ErrUInfinitesimal&&ierror<=(pid.ErrUpLimit*0.05))
           ||(ierror<=pid.ErrDInfinitesimal&&ierror>=(pid.ErrLowLimit*0.05)))
        beta=1.0;
    else
        return (int)pid.LastCon;//����������
#else // DeadZone
        beta=1.0;
#endif // DeadZone
#else // IntegrationSeparation
        beta=1.0;
#if BANGBANG
    if (ierror>=pid.ErrUpLimit){//��������    
        pid.LastCon = pid.UpperBound;
        return pid.UpperBound;
    }
    else if (ierror<=pid.ErrLowLimit){
        pid.LastCon = pid.LowerBound;
        return pid.LowerBound;
    }
#endif // BANGBANG
#if DeadZone
    if (ierror<=pid.ErrUInfinitesimal&&ierror>=pid.ErrDInfinitesimal)
        return (int)pid.LastCon;//����������
#endif // DeadZone
#endif // IntegrationSeparation



    pid.SumError += ierror * alpha; //���ֿ�����
    
#if DifferentialAdvance
    float c1, c2, c3, temp, den;
    temp = pid.Differential / pid.Proportion;
    den = pid.Gamma * temp + 1;
    c1 = (den - 1)/den;
    c2 = (temp + 1)/den;
    c3 = temp/den;
    //΢������
    dcontrol = c1 * pid.LastDifCon
             + c2 * next_point
             - c3 * pid.LastPoint; //΢��
#else
    dcontrol = pid.Differential * (ierror - pid.LastError);
#endif // DifferentialAdvance
    icontrol = pid.Proportion * ierror //������
             + pid.Integral   * pid.SumError * beta //���ַ���
             + dcontrol; //΢����
    
    //�����Ͻ�
    if (icontrol > pid.UpperBound)
      icontrol = pid.UpperBound;
    else if (icontrol < pid.LowerBound)
      icontrol = pid.LowerBound;

  
    //�洢�´���Ҫ�õ�����
    pid.LastDifCon = dcontrol;
    pid.LastCon = icontrol;
        
  printf("%d,",(int)pid.LastCon);
  printf("%d,",pid.LastPoint);
  printf("%d,",pid.LastSetPoint); 
  printf("%d,",(int)(pid.Proportion  ));
  printf("%d,",(int)(pid.Integral    ));
  printf("%d,",(int)(pid.Differential));
  
  printf("%d,",pid.LastError);

    return ((int)icontrol);

}

