#include "PID.h"

typedef struct
{
    float err;
    float iSum;
    float lasterr;
    float dDiff;
}PIDCtrlTemp;

static PIDCtrlTemp Temp1 = {0, 0, 0, 0};
static PIDCtrlTemp Temp2 = {0, 0, 0, 0};

/**
 * [PIDCtrl 用来进行PID控制]
 * @param PIDStructure [用来传入PID控制一些关键变量的结构体]
 */
 
float PIDCtrl1(const PIDCtrlStructure PIDStructure)
{
    float UOut = 0;
    Temp1.err = PIDStructure.ExOut - PIDStructure.GetVar();
    /****************************************************
    这是为了修复小车螺旋升天而加入的代码QAQ
    ****************************************************/
    if(Temp1.err > 180)
    {
        Temp1.err-= 360;
    }
    else if(Temp1.err < -180)
    {
        Temp1.err += 360;
    }
    Temp1.iSum += Temp1.err;
    Temp1.dDiff = Temp1.err - Temp1.lasterr;
    Temp1.lasterr = Temp1.err;
    UOut = PIDStructure.KP * Temp1.err;
    UOut += PIDStructure.KI * Temp1.iSum;
    UOut += PIDStructure.KD * Temp1.dDiff;
    return UOut;
}

void PIDCtrlInit1(void)
{
    Temp1.err = 0;
    Temp1.dDiff = 0;
    Temp1.lasterr = 0;
    Temp1.iSum = 0;
}
float PIDCtrl2(const PIDCtrlStructure PIDStructure)
{
    float UOut = 0;
    Temp2.err = PIDStructure.ExOut - PIDStructure.GetVar();
    /****************************************************
    这是为了修复小车螺旋升天而加入的代码QAQ
    ****************************************************/
    if(Temp2.err > 180)
    {
        Temp2.err-= 360;
    }
    else if(Temp2.err < -180)
    {
        Temp2.err += 360;
    }
    Temp2.iSum += Temp2.err;
    Temp2.dDiff = Temp2.err - Temp2.lasterr;
    Temp2.lasterr = Temp2.err;
    UOut = PIDStructure.KP * Temp2.err;
    UOut += PIDStructure.KI * Temp2.iSum;
    UOut += PIDStructure.KD * Temp2.dDiff;
    return UOut;
}

void PIDCtrlInit2(void)
{
    Temp2.err = 0;
    Temp2.dDiff = 0;
    Temp2.lasterr = 0;
    Temp2.iSum = 0;
}
