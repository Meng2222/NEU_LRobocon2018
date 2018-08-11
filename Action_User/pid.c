#include "pid.h"

static float outMax;
static float outMin;
float kp;
float ki;
float kd;

float Pid(uint16_t valueNow,uint16_t valueSet,uint8_t swt)
{
	static float valueOut=0;
	float err=0;
	static float errLast=0;
	static float iTerm=0;
	if(PidSwitch(swt) == 0) return valueOut;
	else if(PidSwitch(swt) == 2)
	{
		iTerm=valueOut; 
		if(iTerm> outMax) iTerm= outMax; 
		else if(iTerm< outMin) iTerm= outMin; 
	}
	
	err=valueSet-valueNow;
	iTerm+=ki*err;
	
	valueOut=(kp*err)+iTerm+(kd*(err-errLast));
	if(valueOut > outMax) valueOut=outMax;
	if(valueOut < outMin) valueOut=outMin;
	
	
	errLast=err;
	
	return valueOut;
}


uint8_t PidSwitch(uint8_t sw)
{
	static uint8_t swLast;
	if(!sw) 
	{
		swLast=sw; 
		return 0;
	}
	if(sw != swLast) 
	{
		swLast=sw; 
		return 2;
	}
	else
	{
		swLast=sw; 
		return 1;
	}
	
}

void PidPara(float fKp,float fKi,float fKd)
{
	kp=fKp;
	ki=fKi;
	kd=fKd;
}
