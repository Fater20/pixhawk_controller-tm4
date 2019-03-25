#include"pid.h"
//#include"sonic.h"
#include"UART1.h"

//PID����
typedef struct PID
{
long SumError; //����ۼ�
double Proportion; //�������� Proportional Const
double Integral; //���ֳ��� Integral Const
double Derivative; //΢�ֳ��� Derivative Const
int LastError; //Error[-1]
int PrevError; //Error[-2]
} PID;

PID sptr={0,0.6,0,0.05,0,0};

//calculate the increment,input current value ,output increment(/rpm)
int calcuIncre(int iError)
{
    register int iIncpid;

    //��������
//    iIncpid = sptr.Proportion * iError- sptr.Integral * sptr.LastError + sptr.Derivative * sptr.PrevError;

    iIncpid = sptr.Proportion * (iError - sptr.LastError) + sptr.Integral * iError + sptr.Derivative * (iError - 2*sptr.LastError + sptr.PrevError);
    //�洢�������´μ���
    sptr.PrevError = sptr.LastError;
    sptr.LastError = iError;



    //��������ֵ
    return iIncpid;

}
