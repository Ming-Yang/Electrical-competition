#ifndef __PIDBASIC_H
#define __PIDBASIC_H


typedef struct PIDCs
{
    int Gamma1000;//Gamma控制微分先行中低通滤波器的频率上限，Gamma值越高上限频率越低
    int Kp1000; //比例常数 Proportional Const
    int Ki1000; //积分常数 Integral Const
    int Kd1000; //微分常数 Differential Const
    int Delta1000; //Tf是对微分项低通滤波器的通过频率，Ts是时间间隔。$\delta = \fraction{T_f}{T_s + T_f}$
    int Threshold;//表示抗饱和时控制量的极限（认为控制速度的上限和下限是对称的）
    int ErrLimit;//表示使用积分分离中PID控制的error上限
    int DeathZone;//表示死区的大小，小于这个部分的不作控制。
} PIDC;
void KPIDInit(PIDC* pidc);
void incPIDInit(PIDC* pidc);
int IncPIDCalc(int set_point, int next_point);//increase pid calaulate
int LocPIDCalc(int set_point, int next_point);//increase pid calaulate

#define LowPass 0                   //低通滤波
#define IntegrationSaturation 0     //积分抗饱和
#define IntegrationSeparation 0     //积分分离
#define BANGBANG 1                  //棒棒控制
#define DeadZone 1                  //死区
#define DifferentialAdvance 1       //微分先行
void BasicPIDInit(PIDC* pidc);
int BasicPIDCalc(int set_point, int next_point);//基本PID算法

#endif /*__PIDBASIC_H*/