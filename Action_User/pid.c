#include "pid.h"
#include "pps.h"
/**
* @brief  直线函数
* @param  vel: 速度，单位：mm每秒，范围：最小速度限制到最大速度限制
*/
float setAngle = 0;
float setAngle_R = 0;
float shouldX = 0 ,shouldY = 0 ;
float errorAngle = 0;
float errorDis = 0;//直线距离
float errorRadius = 0;//圆心距离
float Uout = 0;
float phase1 = 0,phase2 = 0,uOutAngle = 0,uOutDis = 0;
char updownFlag = 0;
extern PId_t Angle_PId;
extern PId_t	Dis_PId;
float compareAngle = 0;
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
	errorAngle = setAngle - GetAngle();
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
/**
* @brief  圆形闭环函数
* @param  vel: 速度，单位：mm每秒，范围：最小速度限制到最大速度限制
* @param  radius: 半径，单位：mm，范围：最小限制到最大限制
* @param  x,y: 圆心坐标，单位：mm，范围：最小限制到最大限制
* @param  side: 顺时针-1 逆时针1
*/
void WalkRoundPID(Round_t* PID_RndTYPE)
{	
	if(PID_RndTYPE->side == 1)
		setAngle_R = atan2f((GetX()-PID_RndTYPE->x),-(GetY()-PID_RndTYPE->y)) * 180 / PI - 90;
	else if(PID_RndTYPE->side == -1)
		setAngle_R = atan2f(-(GetX()-PID_RndTYPE->x),(GetY()-PID_RndTYPE->y)) * 180 / PI - 90;
	errorRadius = sqrtf(powf((GetX()-PID_RndTYPE->x),2) + powf((GetY()-PID_RndTYPE->y),2)) - PID_RndTYPE->radius;
	if(PID_RndTYPE->side == -1) //圆外顺时针
		errorRadius = -errorRadius;
	else if (PID_RndTYPE->side == 1)//圆内逆时针
		errorRadius = errorRadius;
	setAngle_R = setAngle_R + (PID_RndTYPE->side * PID_RndTYPE->vel ) / (PID_RndTYPE->radius * 100) * 180 / PI;
	Angle_PID(PID_RndTYPE->vel,setAngle_R + Dis_PID(errorRadius));
}

float PID_Compentate(float err,PId_t* PId_tTYPE)
{
	
	PId_tTYPE->sumErr += err;
	if(PId_tTYPE->Ki * PId_tTYPE->sumErr >= 10)
		Uout = PId_tTYPE->Kp * err + 10 + PId_tTYPE->Kd *(err - PId_tTYPE->lastErr);
	else if(PId_tTYPE->Ki * PId_tTYPE->sumErr <= -10)
		Uout = PId_tTYPE->Kp * err - 10 + PId_tTYPE->Kd *(err - PId_tTYPE->lastErr);
	else
		Uout = PId_tTYPE->Kp * err + PId_tTYPE->Ki * PId_tTYPE->sumErr + PId_tTYPE->Kd *(err - PId_tTYPE->lastErr);
	PId_tTYPE->lastErr = err;
	if(Uout > 800)
		Uout = 800;
	else if(Uout < -800)
		Uout = -800;
	return Uout;
}
/**
* @brief  直线位置闭环函数
* @param  Ax+by+C = 0
* @param  Dir 定义沿直线的方向 1 为沿正方向，2为沿负方向
*/

void KownedLinePID(float a,float b,float c,char Dir)
{
	
	errorDis = fabsf(a * GetX() + b * GetY() + c) / sqrtf(a *a + b * b); 
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
		if(GetY() > (-c)/b)
			updownFlag = 2;
		else
			updownFlag = 1;
	}
	
	else 
		{
			shouldX  = (- b * GetY() - c)/a;
			if(shouldX < GetX())
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
		Angle_PID(SPEED,setAngle + Dis_PID(errorDis));
			
}
//角度环PID
void Angle_PID(float vel,float value)
{		
	if(value > 180.0f)
		value = value -360.0f;
	else if(value < -180.0f)
		value = value + 360.0f;
	errorAngle = value - GetAngle();
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
	if(uOutDis > 90)
		uOutDis = 90;
	else if(uOutDis < -90)
		uOutDis = -90;
	return uOutDis;
}
