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
    pid.Proportion  = (float)pidc->Kp1000/1000.0; //比例常数 Proportional Const
    pid.Integral    = (float)pidc->Ki1000/1000.0; //积分常数 Integral Const
    pid.Differential= (float)pidc->Kd1000/1000.0; //微分常数 Differential Const
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
        //当前误差
        ierror  = set_point - next_point;
        const_A = (float)(pid.Proportion + pid.Integral + pid.Differential);
        const_B = (float)(pid.Proportion + pid.Differential * 2);
        const_C = (float)(pid.Differential);

        //增量计算
        incpid = const_A * ierror //E[k]项
               - const_B * pid.LastError //E[k－1]项
               + const_C * pid.PrevError; //E[k－2]项
        //存储误差，用于下次计算
        pid.PrevError = pid.LastError;
        pid.LastError = ierror;
        //返回增量值
        return((int)(incpid));
}
//increase pid calaulate
int LocPIDCalc(int set_point, int next_point)
{
    int ierror,derror;
    ierror = set_point - next_point; //偏差
    pid.SumError += ierror; //积分
    derror = ierror - pid.LastError; //微分
    pid.LastError = ierror;
    return (int)(pid.Proportion * ierror //比例项
         + pid.Integral   * pid.SumError //积分项
         + pid.Differential * derror); //微分项

}


void BasicPIDInit(PIDC* pidc)
{
    AllPIDCClear(pidc);
    #if DifferentialAdvance
    pid.Gamma       = (float)pidc->Gamma1000/1000.0;//微分先行低通滤波器参数
    #endif // DifferentialAdvance 

    pid.ErrUpLimit        = pidc->ErrLimit;
    pid.ErrUInfinitesimal = pidc->DeathZone;
    pid.ErrDInfinitesimal =-pidc->DeathZone;
    pid.ErrLowLimit       =-pidc->ErrLimit;
    pid.UpperBound        = pidc->Threshold;
    pid.LowerBound        =-pidc->Threshold;
}





//基本PID算法，融合了输入信号低通滤波、抗饱和积分、积分分离、
//棒棒控制、死区、微分先行6种改进的控制方法，可能非常难用。
int BasicPIDCalc(int set_point, int next_point)
{
    int ierror;
    float dcontrol, icontrol, beta;
    


#if LowPass
    //低通滤波对应的输入信号
    set_point = (int)(0.1 * set_point + 0.8 * pid.LastSetPoint + 0.1 * pid.PrevSetPoint);
#endif // LowPass
    //当前误差
    ierror = set_point - next_point;
    //存储部分参数
    pid.PrevSetPoint = pid.LastSetPoint;
    pid.LastSetPoint = set_point;
    pid.LastPoint = next_point;
    pid.LastError = ierror;

    int alpha;//抗饱和系数
#if IntegrationSaturation
    //抗饱和
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
    if (ierror>=pid.ErrUpLimit){//棒棒控制    
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
    else if ((ierror>=(pid.ErrUpLimit*0.66)&&ierror<=pid.ErrUpLimit)//积分分离
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
        return (int)pid.LastCon;//死区不控制
#else // DeadZone
        beta=1.0;
#endif // DeadZone
#else // IntegrationSeparation
        beta=1.0;
#if BANGBANG
    if (ierror>=pid.ErrUpLimit){//棒棒控制    
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
        return (int)pid.LastCon;//死区不控制
#endif // DeadZone
#endif // IntegrationSeparation



    pid.SumError += ierror * alpha; //积分抗饱和
    
#if DifferentialAdvance
    float c1, c2, c3, temp, den;
    temp = pid.Differential / pid.Proportion;
    den = pid.Gamma * temp + 1;
    c1 = (den - 1)/den;
    c2 = (temp + 1)/den;
    c3 = temp/den;
    //微分先行
    dcontrol = c1 * pid.LastDifCon
             + c2 * next_point
             - c3 * pid.LastPoint; //微分
#else
    dcontrol = pid.Differential * (ierror - pid.LastError);
#endif // DifferentialAdvance
    icontrol = pid.Proportion * ierror //比例项
             + pid.Integral   * pid.SumError * beta //积分分离
             + dcontrol; //微分项
    
    //超过上界
    if (icontrol > pid.UpperBound)
      icontrol = pid.UpperBound;
    else if (icontrol < pid.LowerBound)
      icontrol = pid.LowerBound;

  
    //存储下次需要用到的量
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

