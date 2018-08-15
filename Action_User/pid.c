#include "pid.h"

static float outMax=800;
static float outMin=-800;
static float kp;
static float ki;
static float kd;


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
	iTerm+=(ki*err);

	if(iTerm > outMax) iTerm=outMax;
	if(iTerm < outMin) iTerm=outMin;
	
	valueOut=(kp*err)+iTerm+(kd*(err-errLast));
	
	if(valueOut > outMax) valueOut=outMax;
	if(valueOut < outMin) valueOut=outMin;
	errLast=err;
	return valueOut;
}

float DisplacementPid(float valueSet,float valueNow)
{
	
	float err=0;
	float valueOut=0;
	static float errLast=0;
	static float iTerm=0;

	err=valueSet-valueNow;
	iTerm+=(ki*err);

	if(iTerm > outMax) iTerm=outMax;
	if(iTerm < outMin) iTerm=outMin;
	
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
	if((sw == 1) && (sw != swLast)) 
	{
		swLast=sw; 
		return 2;
	}
	else if ((sw == 1) && (swLast == sw))
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
