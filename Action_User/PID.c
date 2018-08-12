#include "PID.h"


/**
 * [PIDCtrl 用来进行PID控制]
 * @param PIDStructure [用来传入PID控制一些关键变量的结构体]
 */
 
void PIDCtrl(PIDCtrlStructure * PIDStructure)
{
    varGetted = (*PIDStructure).GetVar();
    /****************************************************
    这是为了修复小车螺旋升天而加入的代码
    ****************************************************/
    (*PIDStructure).ExOut = ExRep((*PIDStructure).ExOut, varGetted);
    err = (*PIDStructure).ExOut - varGetted;
    iSum += err;
    dDiff = err - lasterr;
    lasterr = err;
    UOut = (*PIDStructure).KP * err;
    UOut += (*PIDStructure).KI * iSum;
    UOut += (*PIDStructure).KD * dDiff;
    (*(*PIDStructure).Ctrl)(UOut);
}

void PIDCtrlInit(void)
{
    UOut = 0;
    err = 0;
    iSum = 0;
    lasterr = 0;
    dDiff = 0;
}

/****************************************************
这是为了修复小车螺旋升天而加入的代码
****************************************************/

int32_t ExRep(int32_t ExAngle, int32_t GetA)
{
    if(ExAngle != 180 && ExAngle != -180)
    {
        return ExAngle;
    }
    else if(GetA > 0)
    {
        return 180;
    }
    else
    {
        return -180;
    }
}
