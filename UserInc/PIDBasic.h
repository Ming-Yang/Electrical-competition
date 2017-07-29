#ifndef __PIDBASIC_H
#define __PIDBASIC_H


typedef struct PIDCs
{
    int Gamma1000;//Gamma����΢�������е�ͨ�˲�����Ƶ�����ޣ�GammaֵԽ������Ƶ��Խ��
    int Kp1000; //�������� Proportional Const
    int Ki1000; //���ֳ��� Integral Const
    int Kd1000; //΢�ֳ��� Differential Const
    int Delta1000; //Tf�Ƕ�΢�����ͨ�˲�����ͨ��Ƶ�ʣ�Ts��ʱ������$\delta = \fraction{T_f}{T_s + T_f}$
    int Threshold;//��ʾ������ʱ�������ļ��ޣ���Ϊ�����ٶȵ����޺������ǶԳƵģ�
    int ErrLimit;//��ʾʹ�û��ַ�����PID���Ƶ�error����
    int DeathZone;//��ʾ�����Ĵ�С��С��������ֵĲ������ơ�
} PIDC;
void KPIDInit(PIDC* pidc);
void incPIDInit(PIDC* pidc);
int IncPIDCalc(int set_point, int next_point);//increase pid calaulate
int LocPIDCalc(int set_point, int next_point);//increase pid calaulate

#define LowPass 0                   //��ͨ�˲�
#define IntegrationSaturation 0     //���ֿ�����
#define IntegrationSeparation 0     //���ַ���
#define BANGBANG 1                  //��������
#define DeadZone 1                  //����
#define DifferentialAdvance 1       //΢������
void BasicPIDInit(PIDC* pidc);
int BasicPIDCalc(int set_point, int next_point);//����PID�㷨

#endif /*__PIDBASIC_H*/