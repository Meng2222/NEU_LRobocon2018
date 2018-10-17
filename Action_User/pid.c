#include "pid.h"

static float outMax=1500;
static float outMin=-1500;
float outMax2=90;
float outMin2=-90;

float outMax3=800;
float outMin3=800; 

static struct PIDPara_{
	float aKp;
	float aKi;
	float aKd;
	float dKp;
	float dKi;
	float dKd;
	float sKp;
	float sKi;
	float sKd;

}pid_Para;


/**
* @brief 角度PID
* @param valueSet：角度设定值
* @param valueNow：当前角度值
* @retval none
* @attention 
*/

float AnglePid(float valueSet,float valueNow)
{
	
	float err=0;
	float valueOut=0;
	static float errLast=0;
	static float iTerm=0;
	err=valueSet-valueNow;
	if(err > 180)
	{
		err=err-360;
	}
	else if(err < -180)
	{
		err=360+err;
	}
	iTerm+=(pid_Para.aKi*err);

	if(iTerm > outMax) iTerm=outMax;
	if(iTerm < outMin) iTerm=outMin;
	
	valueOut=(pid_Para.aKp*err)+iTerm+(pid_Para.aKd*(err-errLast));
	
	if(valueOut > outMax) valueOut=outMax;
	if(valueOut < outMin) valueOut=outMin;
	errLast=err;
	return valueOut;
}

/**
* @brief 距离PID
* @param valueSet：距离设定值
* @param valueNow：当前距离值
* @retval none
* @attention 
*/
float DistancePid(float valueSet,float valueNow)
{
	
	float err=0;
	float valueOut=0;
	static float errLast=0;
	static float iTerm=0;

	err=valueSet-valueNow;
	iTerm+=(pid_Para.dKi*err);
	if(err > 1 || err < -1)
	{
	if(iTerm > 10) iTerm=10;
	if(iTerm < -10) iTerm=-10;
	}
	
	valueOut=(pid_Para.dKp*err)+iTerm+(pid_Para.dKd*(err-errLast));
	
	if(valueOut > outMax2) valueOut=outMax2;
	if(valueOut < outMin2) valueOut=outMin2;
	errLast=err;
	return valueOut;
}

/**
* @brief 角度PID系数设置
* @param fKp：比例项系数
* @param fKi：积分项系数
* @param fKd：微分项系数
* @retval none
* @attention 
*/
void Angle_PidPara(float fKp,float fKi,float fKd)
{
	pid_Para.aKp=fKp;
	pid_Para.aKi=fKi;
	pid_Para.aKd=fKd;
}

/**
* @brief 距离PID系数设置
* @param fKp：比例项系数
* @param fKi：积分项系数
* @param fKd：微分项系数
* @retval none
* @attention 
*/
void Distance_PidPara(float fKp,float fKi,float fKd)
{
	pid_Para.dKp=fKp;
	pid_Para.dKi=fKi;
	pid_Para.dKd=fKd;
}



