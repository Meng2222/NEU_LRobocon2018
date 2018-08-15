#include "PID.h"


/**
 * [PIDCtrl 用来进行PID控制]
 * @param PIDStructure [用来传入PID控制一些关键变量的结构体]
 */
 
float PIDCtrl1(PIDCtrlStructure * PIDStructure)
{
    err1 = (*PIDStructure).ExOut - (*PIDStructure).GetVar();
    /****************************************************
    这是为了修复小车螺旋升天而加入的代码
    ****************************************************/
    if(err1 > 180)
    {
        err1 -= 360;
    }
    else if(err1 < -180)
    {
        err1 += 360;
    }
    iSum1 += err1;
    dDiff1 = err1 - lasterr1;
    lasterr1 = err1;
    UOut1 = (*PIDStructure).KP * err1;
    UOut1 += (*PIDStructure).KI * iSum1;
    UOut1 += (*PIDStructure).KD * dDiff1;
    return UOut1;
}

void PIDCtrlInit1(void)
{
    UOut1 = 0;
    err1 = 0;
    iSum1 = 0;
    lasterr1 = 0;
    dDiff1 = 0;
}
float PIDCtrl2(PIDCtrlStructure * PIDStructure)
{
    err2 = (*PIDStructure).ExOut - (*PIDStructure).GetVar();
    iSum2 += err2;
    dDiff2 = err2 - lasterr2;
    lasterr2 = err2;
    UOut2 = (*PIDStructure).KP * err2;
    UOut2 += (*PIDStructure).KI * iSum2;
    UOut2 += (*PIDStructure).KD * dDiff2;
    return UOut2;
}

void PIDCtrlInit2(void)
{
    UOut2 = 0;
    err2 = 0;
    iSum2 = 0;
    lasterr2 = 0;
    dDiff2 = 0;
}
