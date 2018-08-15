/**
  ******************************************************************************
  * @file	  moveBase.c
  * @author	  Action
  * @version   V1.0.0
  * @date	  2018/08/09
  * @brief	 2018省赛底盘运动控制部分
  ******************************************************************************
  * @attention
  *			None
  ******************************************************************************
  */
/* Includes -------------------------------------------------------------------------------------------*/

#include "moveBase.h"
#include "timer.h"
#include "usart.h"
#include "math.h"
/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/

/**
  * @brief  
  * @note
  * @param  
  * @retval None
  */

uint8_t LocationFlag = 0;
double SlopeSetLine = 0;  //设定直线斜率
float  InterceptSetLine = 0;	//设定直线截距
float  InterceptActual = 0;		//实际截距

double InverseTangentSlopeSetLineVaule = 0;
float  Distance = 0;
float  PID_SetAngle = 0;
float  AngleSetLine = 0;

float  AngleError = 0;
float  AngleControl = 0;
float  LocationError = 0;
float  LocationControl = 0;

float  xpos = 0;
float  ypos = 0;
extern float angle;

void CircleAround(float radius, float speed) //(半径(mm)), 速度(mm/s))
{
	float PulseR = 0;
	float PulseL = 0; 
	PulseR = ((((2.f*radius - WHEEL_TREAD - WHEEL_WIDTH) * speed)/(2.f*radius)) * COUNTS_PER_ROUND)/(WHEEL_DIAMETER * 3.14f);
	PulseL = ((radius+((WHEEL_WIDTH+WHEEL_TREAD)/2))/(radius-((WHEEL_WIDTH+WHEEL_TREAD)/2.0f))) * PulseR;
	VelCrl(CAN2, 1, (int)PulseR);	//右轮(正为向前)
	VelCrl(CAN2, 2, (int)-PulseL);	//左轮(负为向前)
}

void RectangleAround(float length, float width, float BasicSpeed) //(长(mm), 宽(mm), 基础速度(mm/s))
{
	if(GetXpos() <= 400 && GetYpos() <= width-100)
	{
		LocationFlag = 1;
	}
	if(GetXpos() <= length-100 && GetYpos() > width-100)
	{
		LocationFlag = 2;
	}
	if(GetXpos() > length-100 && GetYpos() >= 100)
	{
		LocationFlag = 3;
	}
	if(GetXpos() > 400 && GetYpos() <100)
	{
		LocationFlag = 4;
	}

	if(LocationFlag == 1)
	{
		PID_SetAngle = 0;
		Move(BasicSpeed - AnglePID(P_Angle,I_Angle,D_Angle,PID_SetAngle,GetAngle()), BasicSpeed + AnglePID(P_Angle,I_Angle,D_Angle,PID_SetAngle,GetAngle()));
	}
	if(LocationFlag == 2)
	{
		PID_SetAngle = -90;
		Move(BasicSpeed - AnglePID(P_Angle,I_Angle,D_Angle,PID_SetAngle,GetAngle()), BasicSpeed + AnglePID(P_Angle,I_Angle,D_Angle,PID_SetAngle,GetAngle()));
	}
	if(LocationFlag == 3)
	{
		PID_SetAngle = -179;
		Move(BasicSpeed - AnglePID(P_Angle,I_Angle,D_Angle,PID_SetAngle,GetAngle()), BasicSpeed + AnglePID(P_Angle,I_Angle,D_Angle,PID_SetAngle,GetAngle()));
		if(GetAngle() >= -180 && GetAngle() <= 0)
		{
			PID_SetAngle = -179;
			Move(BasicSpeed - AnglePID(P_Angle,I_Angle,D_Angle,PID_SetAngle,GetAngle()), BasicSpeed + AnglePID(P_Angle,I_Angle,D_Angle,PID_SetAngle,GetAngle()));
		}	
		if(GetAngle() >= 0 && GetAngle() <= 180)
		{
			PID_SetAngle = 179;
			Move(BasicSpeed - AnglePID(P_Angle,I_Angle,D_Angle,PID_SetAngle,GetAngle()), BasicSpeed + AnglePID(P_Angle,I_Angle,D_Angle,PID_SetAngle,GetAngle()));
		}
	}
	if(LocationFlag == 4)
	{
		PID_SetAngle = 90;
		Move(BasicSpeed - AnglePID(P_Angle,I_Angle,D_Angle,PID_SetAngle,GetAngle()), BasicSpeed + AnglePID(P_Angle,I_Angle,D_Angle,PID_SetAngle,GetAngle()));
	}
}

void LockLineMove(double k, float b, float BasicSpeed, uint8_t direction) //(设定直线斜率, 设定直线截距, 基础速度, 方向(1为沿X轴正向, 0为负向))
{
	SlopeSetLine = k; 	//设定直线斜率
	InterceptSetLine = b;
	InverseTangentSlopeSetLineVaule = atan(SlopeSetLine);	//反正切输出为弧度制
	AngleSetLine = ((InverseTangentSlopeSetLineVaule*180)/3.141) - 90;	//转换为角度并旋转坐标系
	InterceptActual = GetYpos() - SlopeSetLine*GetXpos();
	Distance =  (InterceptSetLine - InterceptActual)/sqrt(SlopeSetLine*SlopeSetLine + 1);
	if(direction == 1)
	{
		Move(BasicSpeed - (AnglePID(P_Angle, I_Angle, D_Angle, AngleSetLine, GetAngle())/2) + (LocationPID(P_Location, I_Location, D_Location, 0, Distance)/2), BasicSpeed + (AnglePID(P_Angle, I_Angle, D_Angle, AngleSetLine, GetAngle())/2) - (LocationPID(P_Location, I_Location, D_Location, 0, Distance)/2));	//向设定直线方向前进
	}
	if(direction == 0)
	{
		Move(BasicSpeed - (AnglePID(P_Angle, I_Angle, D_Angle, AngleSetLine+180, GetAngle())/2) + (LocationPID(P_Location, I_Location, D_Location, 0, Distance)/2), BasicSpeed + (AnglePID(P_Angle, I_Angle, D_Angle, AngleSetLine+180, GetAngle())/2) - (LocationPID(P_Location, I_Location, D_Location, 0, Distance)/2));	//
	}
} 

float AnglePID(float Kp, float Ki, float Kd, float AngleSet, float AngleActual)
{
	static float AngleLastError =0;
	static float AngleIntegralError = 0;
	AngleError = AngleSet - AngleActual;
	if(AngleError <= -180)
	{
		AngleError += 360;
	}
	if(AngleError >= 180)
	{
		AngleError -= 360;
	}
	AngleIntegralError += AngleError;
	AngleControl = Kp*AngleError + Ki*AngleIntegralError + Kd*(AngleError - AngleLastError);
	AngleLastError = AngleError;
	return AngleControl;
}

float LocationPID(float Kp, float Ki, float Kd, float LocationSet, float LocationActual)
{
	static float LocationLastError =0;
	static float LocationIntegralError = 0;
	LocationError = LocationSet - LocationActual;
	LocationIntegralError += LocationError;
	LocationControl = Kp*LocationError + Ki*LocationIntegralError + Kd*(LocationError - LocationLastError);
	LocationLastError = LocationError;
	return LocationControl;
}
void Move(float SpeedL, float  SpeedR)
{
	static int PulseR = 0;
	static int PulseL = 0;
	PulseR = (int)(SpeedR*COUNTS_PER_ROUND)/(3.14f*WHEEL_DIAMETER);
	PulseL = (int)(SpeedL*COUNTS_PER_ROUND)/(3.14f*WHEEL_DIAMETER);
	VelCrl(CAN2, 1, PulseR);	//右轮(正为向前)
	VelCrl(CAN2, 2, -PulseL);	//左轮(负为向前)
}

void SetAngle(float val)
{
	angle = val;
}

void SetXpos(float val)
{
	xpos = val;
}

void SetYpos(float val)
{
	ypos = val;
}

float AbsoluteValue(float value)
{
	if(value >= 0)
	{
		value = value;
	}
	if(value < 0)
	{
		value = -value;
	}
	return value;
}



/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
