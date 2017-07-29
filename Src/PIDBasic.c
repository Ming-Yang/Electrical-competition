typedef struct PIDs
{
    int SetPoint;
    int LastSetPoint;
    int PrevSetPoint;
    int LastPoint;
    double Gamma;
    double Proportion;
    double Integral;
    double Differential;
    double Delta;
    double LastDifCon;
    double LastCon;
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

void incPIDInit(PIDC* pidc)
{
    pid.SetPoint = 0;
    pid.LastSetPoint = 0;
    pid.PrevSetPoint = 0;
    pid.LastPoint = 0;
    pid.Gamma      = 0;
    pid.Proportion = (double)pidc.Kp1000/1000.0; //�������� Proportional Const
    pid.Integral   = (double)pidc.Ki1000/1000.0; //���ֳ��� Integral Const
    pid.Differential = (double)pidc.Kd1000/1000.0; //΢�ֳ��� Differential Const
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

//increase pid calaulate
int IncPIDCalc(int next_point)
{
        register int ierror, incpid;
        register int const_A, const_B, const_C;
        //��ǰ���
        ierror = pid.SetPoint - next_point;
        const_A = pid.Proportion + pid.Integral + pid.Derivative;
        const_B = pid.Proportion + pid.Derivative * 2;
        const_C = pid.Derivative;

        //��������
        incpid = const_A * ierror //E[k]��
               - const_B * pid.LastError //E[k��1]��
               + const_C * pid.PrevError; //E[k��2]��
        //�洢�������´μ���
        pid.PrevError = pid.LastError;
        pid.LastError = ierror;
        //��������ֵ
        return(incpid);
}




void BasicPIDInit(PIDC* pidc)
{
    pid.SetPoint = 0;
    pid.LastSetPoint = 0;
    pid.PrevSetPoint = 0;
    pid.LastPoint = 0;
    #if DifferentialAdvance
    pid.Gamma       = (double)pidc.Gamma1000/1000.0;//΢�����е�ͨ�˲�������
    #else
    pid.Gamma       = 0;
    #endif // DifferentialAdvance
    pid.Proportion  = (double)pidc.Kp1000/1000.0; //�������� Proportional Const
    pid.Integral    = (double)pidc.Ki1000/1000.0; //���ֳ��� Integral Const
    pid.Differential= (double)pidc.Kd1000/1000.0; //΢�ֳ��� Differential Const
    pid.Delta       = 0;
    pid.LastDifCon = 0;
    pid.LastCon = 0;
    pid.SumError = 0;
    pid.LastError = 0;
    pid.PrevError = 0;
    pid.ErrUpLimit = pidc.ErrLimit;
    pid.ErrUInfinitesimal = pidc.DeathZone;
    pid.ErrUInfinitesimal = - pidc.DeathZone;
    pid.ErrLowLimit = - pidc.ErrLimit;
    pid.UpperBound = pidc.Threshold;
    pid.LowerBound = - pidc.Threshold;
}

//����PID�㷨���ں��������źŵ�ͨ�˲��������ͻ��֡����ַ��롢
//�������ơ�������΢������6�ָĽ��Ŀ��Ʒ��������ܷǳ����á�
int BasicPIDCalc(int set_point, int next_point)
{
    register int ierror, beta;
    register double dcontrol, icontrol;
#if DifferentialAdvance
    register double c1, c2, c3, temp, den;
    temp = pid.Differential / pid.Proportion;
    den = pid.Gamma * temp + 1;
    c1 = (den - 1)/den;
    c2 = (temp + 1)/den;
    c3 = temp/den;
#endif // DifferentialAdvance

#if LowPass
    //��ͨ�˲���Ӧ�������ź�
    set_point = 0.1 * set_point + 0.8 * pid.LastSetPoint + 0.1 * pid.PrevSetPoint;
    //��ǰ���
    ierror = set_point - next_point;
#endif // LowPass

#if IntegrationSaturation
    //������
    int alpha;//������ϵ��
    if (pid.LastCon >= pid.UpperBound){
        if (ierror > 0) alpha = 0;
        else            alpha = 1;
    }
    else if (pid.LastCon <= pid.LowerBound){
        if (ierror > 0) alpha = 1;
        else            alpha = 0;
    }
    else
#endif // IntegrationSaturation
        alpha = 1;


#if IntegrationSeparation
#if BANGBANG
    if (ierror>=pid.ErrUpLimit)//��������
        return pid.UpperBound;
    else if (ierror<=pid.ErrLowLimit))
        return pid.LowerBound;
#else
    if (ierror>=pid.ErrUpLimit||ierror<=pid.ErrLowLimit)
        return beta=0.0;
#endif // BANGBANG
    else if ((ierror>=(pid.ErrUpLimit*0.66) &&ierror<=pid.ErrUpLimit)//���ַ���
           ||(ierror<=(pid.ErrLowLimit*0.66)&&ierror>=pid.ErrLowLimit)
        beta=0.6;
    else if ((ierror>=(pid.ErrUpLimit*0.33) &&ierror<=(pid.ErrUpLimit*0.66)
           ||(ierror<=(pid.ErrLowLimit*0.33)&&ierror>=(pid.ErrLowLimit*0.66))
        beta=0.9;
    else
#if DeadZone
        if ((ierror>=pid.ErrUInfinitesimal&&ierror<=(pid.ErrUpLimit*0.05)
           ||(ierror<=pid.ErrDInfinitesimal&&ierror>=(pid.ErrLowLimit*0.05))
        beta=1.0;
    else
        return pid.LastCon;//����������
#else // DeadZone
        beta=1.0;
#endif // DeadZone
#else // IntegrationSeparation
        beta=1.0;
#if BANGBANG
    if (ierror>=pid.ErrUpLimit)//��������
        return pid.UpperBound;
    else if (ierror<=pid.ErrLowLimit))
        return pid.LowerBound;
#endif // BANGBANG
#if DeadZone
    if (ierror<=pid.ErrUInfinitesimal||ierror>=pid.ErrDInfinitesimal)
        return pid.LastCon;//����������
#endif // DeadZone
#endif // IntegrationSeparation



    pid.SumError += ierror * alpha; //���ֿ�����
#if DifferentialAdvance
    //΢������
    dcontrol = c1 * pid.LastDifCon
             + c2 * next_point
             - c3 * pid.LastPoint; //΢��
#else
    dcontrol = pid.Differential * (ierror - pid.LastError)
#endif // DifferentialAdvance
    icontrol = pid.Proportion * ierror //������
             + pid.Integral   * pid.SumError * beta //���ַ���
             + dcontrol; //΢����

    //�洢�´���Ҫ�õ�����
    pid.LastError = ierror;
    pid.LastDifCon = dcontrol;
    pid.LastCon = icontrol;
    pid.PrevSetPoint = pid.LastSetPoint;
    pid.LastSetPoint = set_point;
    pid.LastPoint = next_point;

    return icontrol;

}

