#include "pid.h"
/**
* @brief  直线函数
* @param  vel: 速度，单位：mm每秒，范围：最小速度限制到最大速度限制
*/
float setAngle = 0;
float shouldX = 0 ,shouldY = 0 ;
float errorAngle = 0;
float errorDis = 0;
float Uout = 0;
float phase1 = 0,phase2 = 0,uOutAngle = 0,uOutDis = 0;
char updownFlag = 0;
extern Pos_t Pos;
extern PId_t Angle_PId;
extern PId_t	Dis_PId;
//PosNow_t PosNow;
void WalkLine(float vel)
{
	float phase = 0;
	phase = 4096.f * vel / (WHEEL_DIAMETER * PI);
	VelCrl(CAN2,1,phase);
	VelCrl(CAN2,2,-phase);
}

/**
* @brief  直线函数用PID调节
* @param  vel: 速度，单位：mm每秒，范围：最小速度限制到最大速度限制
*/
void WalkLine2PID(float vel,float setAngle)
{
	errorAngle = setAngle - Pos.angle;
	if(errorAngle > 180.0f)
		errorAngle = errorAngle - 360.0f;
	else if(errorAngle < -180.0f)
		errorAngle = 360.0f + errorAngle;		
	uOutAngle = PID_Compentate(errorAngle,&Angle_PId);
	phase1 = 4096.f * (vel + uOutAngle)/ (WHEEL_DIAMETER * PI);
	phase2 = 4096.f * (vel - uOutAngle)/ (WHEEL_DIAMETER * PI);
	VelCrl(CAN2,1,phase1);
	VelCrl(CAN2,2,-phase2);
}
/**
* @brief  圆形函数
* @param  vel: 速度，单位：mm每秒，范围：最小速度限制到最大速度限制
* @param  radius: 半径，单位：mm，范围：最小限制到最大限制
* @param  side: 半径，1圆心在右，0 圆心正左
*/
void WalkRound(float vel,float radius,char side)
{
	float phase1 = 0,phase2 = 0;
	float v1 = 0,v2 = 0;
	v1 = (radius - WHEEL_TREAD / 2) / radius * vel;
	v2 = (radius + WHEEL_TREAD / 2) / radius * vel;
	phase1 = 4096.f *v1 / (WHEEL_DIAMETER * PI);
	phase2 = 4096.f *v2 / (WHEEL_DIAMETER * PI);
	if(side == 1)
	{
		VelCrl(CAN2,1,phase1);
		VelCrl(CAN2,2,-phase2);
	}
	else if(side == 0)
	{
		VelCrl(CAN2,1,phase2);
		VelCrl(CAN2,2,-phase1);
	}
}

float PID_Compentate(float err,PId_t* PId_tTYPE)
{
	
	static float lastErr = 0;
	static float sumErr = 0;
	sumErr += err;
	Uout = PId_tTYPE->Kp * err + PId_tTYPE->Ki * sumErr + PId_tTYPE->Kd *(err - lastErr);
	lastErr = err;
	return Uout;
}
/**
* @brief  直线位置闭环函数
* @param  Ax+by+C = 0
* @param  Dir 定义沿直线的方向 1 为沿正方向，2为沿负方向
*/

void KownedLinePID(float a,float b,float c,char Dir)
{
	
	errorDis = fabsf(a * Pos.x + b * Pos.y + c) / sqrtf(a *a + b * b); 
	//y轴正半轴
	if (b == 0 && Dir == 1)
	{
		setAngle = 0;
		
	}
	//y轴负半轴
	else if (b == 0 && Dir == 2)
	{
		setAngle = -180;
		
	}
	//x正半轴
	else if (a == 0 && Dir == 1)
	{
		setAngle = -90;
		
	}
	//x负半轴
	else if (a == 0 && Dir == 2)
	{
		setAngle = 90;
	
	}
//	//一相限
	else if(Dir == 1 && a * b < 0)
	{
		setAngle = atan((-a)/b) * 180 / PI - 90;// 一相限-90
		
	}
	//三相限
	else if(Dir == 2 && a * b < 0)
	{
		setAngle = (atan((-a)/b) * 180 / PI) + 90;//三相限+90
		
	}
	// 二相限
	else if(Dir == 1 && a * b > 0)
	{
		setAngle = atan((-a)/b) * 180 / PI + 90;// 二相限+90
		
	}
	// 四相限
	else
	{
		setAngle = atan((-a)/b) * 180 / PI - 90;// 四相限-90
		
	}

	//updownFlag 1右侧2 左侧 
	if(a == 0)
	{
		if(Pos.y > (-c)/b)
			updownFlag = 1;
		else
			updownFlag = 2;
	}
	else
		{
			shouldX  = (- b * Pos.y - c)/a;
			if(shouldX < Pos.x)
				updownFlag = 1;
			else
				updownFlag = 2;
		}
	if(updownFlag == 2 && Dir == 1) //在线的左侧,正方向
	{
		errorDis = -errorDis;
	}
	else if(updownFlag == 2 && Dir == 2) //在线的右侧，反方向
	{
		errorDis = errorDis;
	}
	else if (updownFlag == 1&& Dir == 1)//在线的右侧,正方向
	{
		errorDis = errorDis;
	}
	else if (updownFlag == 1&& Dir == 2)//在线的左侧,反方向
	{
		errorDis = -errorDis;
	}
	if(Dis_PID(errorDis)>90)
		Angle_PID(SPEED,setAngle + 90);
	else if(Dis_PID(errorDis)<-90)
		Angle_PID(SPEED,setAngle - 90);
	else
		Angle_PID(SPEED,setAngle + Dis_PID(errorDis));
			
}
//角度环PID
void Angle_PID(float vel,float value)
{		
	errorAngle = value - Pos.angle;
	if(errorAngle > 180.0f)
		errorAngle = errorAngle - 360.0f;
	else if(errorAngle < -180.0f)
		errorAngle = 360.0f + errorAngle;
	uOutAngle = PID_Compentate(errorAngle,&Angle_PId);
	phase1 = 4096.f * (vel + uOutAngle)/ (WHEEL_DIAMETER * PI);
	phase2 = 4096.f * (vel - uOutAngle)/ (WHEEL_DIAMETER * PI);
	VelCrl(CAN2,1,phase1);
	VelCrl(CAN2,2,-phase2);
}

float Dis_PID(float error)
{	
	uOutDis = PID_Compentate(error,&Dis_PId);
	return uOutDis;
}
