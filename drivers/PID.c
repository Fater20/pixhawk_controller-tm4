#include"pid.h"
//#include"sonic.h"
#include"UART1.h"

//PID参数
typedef struct PID
{
long SumError; //误差累计
double Proportion; //比例常数 Proportional Const
double Integral; //积分常数 Integral Const
double Derivative; //微分常数 Derivative Const
int LastError; //Error[-1]
int PrevError; //Error[-2]
} PID;

PID sptr={0,0.6,0,0.05,0,0};

//calculate the increment,input current value ,output increment(/rpm)
int calcuIncre(int iError)
{
    register int iIncpid;

    //增量计算
//    iIncpid = sptr.Proportion * iError- sptr.Integral * sptr.LastError + sptr.Derivative * sptr.PrevError;

    iIncpid = sptr.Proportion * (iError - sptr.LastError) + sptr.Integral * iError + sptr.Derivative * (iError - 2*sptr.LastError + sptr.PrevError);
    //存储误差，用于下次计算
    sptr.PrevError = sptr.LastError;
    sptr.LastError = iError;



    //返回增量值
    return iIncpid;

}
