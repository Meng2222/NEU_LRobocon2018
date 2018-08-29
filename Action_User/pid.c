#include "pid.h"

static float outMax=800;
static float outMin=-800;
float outMax2=90;
float outMin2=-90;

static float outMax3=2000;
static float outMin3=1500; 

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


//角度pid
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

//距离pid
float DistancePid(float valueSet,float valueNow)
{
	
	float err=0;
	float valueOut=0;
	static float errLast=0;
	static float iTerm=0;

	err=valueSet-valueNow;
	iTerm+=(pid_Para.dKi*err);

	if(iTerm > 10) iTerm=8;
	if(iTerm < -10) iTerm=-8;
	
	valueOut=(pid_Para.dKp*err)+iTerm+(pid_Para.dKd*(err-errLast));
	
	if(valueOut > outMax2) valueOut=outMax2;
	if(valueOut < outMin2) valueOut=outMin2;
	errLast=err;
	return valueOut;
}

//速度pid
float SpeedPid(float valueSet,float valueNow)
{
	
	float err=0;
	float valueOut=0;
	static float errLast=0;
	static float iTerm=0;

	err=valueSet-valueNow;
	iTerm+=(pid_Para.sKi*err);

	if(iTerm > 100) iTerm=100;
	if(iTerm < -100) iTerm=-100;
	
	valueOut=(pid_Para.sKp*err)+iTerm+(pid_Para.sKd*(err-errLast));
	
	if(valueOut > outMax3) valueOut=outMax3;
	else if(valueOut < outMin3) valueOut=outMin3;
	
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




void Angle_PidPara(float fKp,float fKi,float fKd)
{
	pid_Para.aKp=fKp;
	pid_Para.aKi=fKi;
	pid_Para.aKd=fKd;
}

void Distance_PidPara(float fKp,float fKi,float fKd)
{
	pid_Para.dKp=fKp;
	pid_Para.dKi=fKi;
	pid_Para.dKd=fKd;
}

void Speed_PidPara(float fKp,float fKi,float fKd)
{
	pid_Para.sKp=fKp;
	pid_Para.sKi=fKi;
	pid_Para.sKd=fKd;
}


