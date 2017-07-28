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
    pid.Proportion = (double)pidc.Kp1000/1000.0; //比例常数 Proportional Const
    pid.Integral   = (double)pidc.Ki1000/1000.0; //积分常数 Integral Const
    pid.Differential = (double)pidc.Kd1000/1000.0; //微分常数 Differential Const
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
        //当前误差
        ierror = pid.SetPoint - next_point;
        const_A = pid.Proportion + pid.Integral + pid.Derivative;
        const_B = pid.Proportion + pid.Derivative * 2;
        const_C = pid.Derivative;

        //增量计算
        incpid = const_A * ierror //E[k]项
               - const_B * pid.LastError //E[k－1]项
               + const_C * pid.PrevError; //E[k－2]项
        //存储误差，用于下次计算
        pid.PrevError = pid.LastError;
        pid.LastError = ierror;
        //返回增量值
        return(incpid);
}




void BasicPIDInit(PIDC* pidc)
{
    pid.SetPoint = 0;
    pid.LastSetPoint = 0;
    pid.PrevSetPoint = 0;
    pid.LastPoint = 0;
    #if DifferentialAdvance
    pid.Gamma       = (double)pidc.Gamma1000/1000.0;//微分先行低通滤波器参数
    #else
    pid.Gamma       = 0;
    #endif // DifferentialAdvance
    pid.Proportion  = (double)pidc.Kp1000/1000.0; //比例常数 Proportional Const
    pid.Integral    = (double)pidc.Ki1000/1000.0; //积分常数 Integral Const
    pid.Differential= (double)pidc.Kd1000/1000.0; //微分常数 Differential Const
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

//基本PID算法，融合了输入信号低通滤波、抗饱和积分、积分分离、
//棒棒控制、死区、微分先行6种改进的控制方法，可能非常难用。
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
    //低通滤波对应的输入信号
    set_point = 0.1 * set_point + 0.8 * pid.LastSetPoint + 0.1 * pid.PrevSetPoint;
    //当前误差
    ierror = set_point - next_point;
#endif // LowPass

#if IntegrationSaturation
    //抗饱和
    int alpha;//抗饱和系数
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
    if (ierror>=pid.ErrUpLimit)//棒棒控制
        return pid.UpperBound;
    else if (ierror<=pid.ErrLowLimit))
        return pid.LowerBound;
#else
    if (ierror>=pid.ErrUpLimit||ierror<=pid.ErrLowLimit)
        return beta=0.0;
#endif // BANGBANG
    else if ((ierror>=(pid.ErrUpLimit*0.66) &&ierror<=pid.ErrUpLimit)//积分分离
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
        return pid.LastCon;//死区不控制
#else // DeadZone
        beta=1.0;
#endif // DeadZone
#else // IntegrationSeparation
        beta=1.0;
#if BANGBANG
    if (ierror>=pid.ErrUpLimit)//棒棒控制
        return pid.UpperBound;
    else if (ierror<=pid.ErrLowLimit))
        return pid.LowerBound;
#endif // BANGBANG
#if DeadZone
    if (ierror<=pid.ErrUInfinitesimal||ierror>=pid.ErrDInfinitesimal)
        return pid.LastCon;//死区不控制
#endif // DeadZone
#endif // IntegrationSeparation



    pid.SumError += ierror * alpha; //积分抗饱和
#if DifferentialAdvance
    //微分先行
    dcontrol = c1 * pid.LastDifCon
             + c2 * next_point
             - c3 * pid.LastPoint; //微分
#else
    dcontrol = pid.Differential * (ierror - pid.LastError)
#endif // DifferentialAdvance
    icontrol = pid.Proportion * ierror //比例项
             + pid.Integral   * pid.SumError * beta //积分分离
             + dcontrol; //微分项

    //存储下次需要用到的量
    pid.LastError = ierror;
    pid.LastDifCon = dcontrol;
    pid.LastCon = icontrol;
    pid.PrevSetPoint = pid.LastSetPoint;
    pid.LastSetPoint = set_point;
    pid.LastPoint = next_point;

    return icontrol;

}

