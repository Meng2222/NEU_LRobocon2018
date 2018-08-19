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
float  AngleSetLine = 0;
float  InterceptSetLine = 0;	//设定直线截距
float  InterceptActual = 0;		//实际截距

float AngleTangentLine = 0;

double InverseTangentSlopeSetLineVaule = 0;
float  DistanceActual = 0;
float  PID_SetAngle = 0;

float  AngleControl = 0;

float  xpos = 0;
float  ypos = 0;
extern float angle;

void CircleAround(float CenterXpos, float CenterYpos,float radius, float BasicSpeed, uint8_t direction) //(圆心横坐标(mm), 圆心纵坐标(mm), 半径(mm)), 线速度(mm/s), 绕行方向(1为顺时针, 2为逆时针))
{
	float SlopeTangentLine = 0;
	
	if(direction == 1)
	{
		SlopeTangentLine = atan(-((GetXpos() - CenterXpos)/(GetYpos() - CenterYpos)));
		AngleTangentLine = ((SlopeTangentLine*180)/3.141f) - 90;
		if(GetYpos() >= CenterYpos)
		{
			DistanceActual = sqrt((GetXpos() - CenterXpos)*(GetXpos() - CenterXpos) + (GetYpos() - CenterYpos)*(GetYpos() - CenterYpos));
			PID_SetAngle = CircleDistancePID(P_CircleDistance, I_CircleDistance, D_CircleDistance, radius, DistanceActual) + AngleTangentLine;
		}
		if(GetYpos() < CenterYpos)
		{
			DistanceActual = sqrt((GetXpos() - CenterXpos)*(GetXpos() - CenterXpos) + (GetYpos() - CenterYpos)*(GetYpos() - CenterYpos));
			PID_SetAngle = CircleDistancePID(P_CircleDistance, I_CircleDistance, D_CircleDistance, radius, DistanceActual)+ AngleTangentLine + 180;
		}
		//角度PID控制器输入量限幅
		if(PID_SetAngle > 180)
		{
			PID_SetAngle -= 360;
		}
		if(PID_SetAngle < -180)
		{
			PID_SetAngle += 360;
		}
		Move(BasicSpeed - (AnglePID(P_Angle, I_Angle, D_Angle, PID_SetAngle, GetAngle())/2), \
		BasicSpeed + (AnglePID(P_Angle, I_Angle, D_Angle, PID_SetAngle, GetAngle())/2));	
	}		
}

void RectangleAround(float length, float width, float BasicSpeed) //(长(mm), 宽(mm), 基础速度(mm/s))
{
	//位置判断程序
	if(GetXpos() <= 700 && GetYpos() <= width-700)
	{
		LocationFlag = 1;	//直线 x = 0
	}
	else if(GetXpos() <= length-700 && GetYpos() > width-700)
	{
		LocationFlag = 2;	//直线 y = width
	}
	else if(GetXpos() > length-700 && GetYpos() >= 700)
	{
		LocationFlag = 3;	//直线 x= length
	}
	else if(GetXpos() > 700 && GetYpos() <700)
	{
		LocationFlag = 4;	//直线 y= 0
	}
	//执行程序
	if(LocationFlag == 1)
	{
		LockLineMove(0, 0, 0, 0, BasicSpeed, 1);	//直线 x = 0
	}
	if(LocationFlag == 2)
	{
		LockLineMove(1, 0, width, 0, BasicSpeed, 1);	//直线 y = width
	}
	if(LocationFlag == 3)
	{
		LockLineMove(0, 0, 0, length, BasicSpeed, 0);	//直线 x= length
	}
	if(LocationFlag == 4)
	{
		LockLineMove(1, 0, 0, 0, BasicSpeed, 0);	//直线 y= 0
	}
}

//(是否存在斜率(1为存在, 0为不存在), 设定直线斜率, 设定直线截距, 若不存在斜率则填写设定直线横坐标, 基础速度, 方向(1为正向, 0为负向))
void LockLineMove(uint8_t ExistSlope, double k, float b, float SetXpos, float BasicSpeed, uint8_t direction) 
{
	if(ExistSlope)
	{
		SlopeSetLine = k; 	//设定直线斜率
		InterceptSetLine = b;
		InverseTangentSlopeSetLineVaule = atan(SlopeSetLine);	//反正切(弧度制)
		AngleSetLine = ((InverseTangentSlopeSetLineVaule*180)/3.141f) - 90;	//转换为角度制并进行坐标变换
		DistanceActual =  (SlopeSetLine*GetXpos() - GetYpos() + InterceptSetLine)/sqrt(SlopeSetLine*SlopeSetLine + 1);	//计算到目标直线距离
		if(direction == 1)
		{
			PID_SetAngle = LocationPID(P_Location, I_Location, D_Location, 0, -DistanceActual) + AngleSetLine;
			if(PID_SetAngle > 180)
			{
				PID_SetAngle -= 360;
			}
			if(PID_SetAngle < -180)
			{
				PID_SetAngle += 360;
			}
			Move(BasicSpeed - (AnglePID(P_Angle, I_Angle, D_Angle, PID_SetAngle, GetAngle())/2), \
			BasicSpeed + (AnglePID(P_Angle, I_Angle, D_Angle, PID_SetAngle, GetAngle())/2));		
		}
		if(direction == 0)
		{
			PID_SetAngle = LocationPID(P_Location, I_Location, D_Location, 0, DistanceActual) + AngleSetLine + 180;
			if(PID_SetAngle > 180)
			{
				PID_SetAngle -= 360;
			}
			if(PID_SetAngle < -180)
			{
				PID_SetAngle += 360;
			}
			Move(BasicSpeed - (AnglePID(P_Angle, I_Angle, D_Angle, PID_SetAngle, GetAngle())/2), \
			BasicSpeed + (AnglePID(P_Angle, I_Angle, D_Angle, PID_SetAngle, GetAngle())/2));		
		}
	}
	if(!ExistSlope)
	{
		AngleSetLine = 0;
		if(direction == 1)
		{
			PID_SetAngle = -LocationPID(P_Location, I_Location, D_Location, SetXpos, GetXpos()) + AngleSetLine;
			if(PID_SetAngle > 180)
			{
				PID_SetAngle -= 360;
			}
			if(PID_SetAngle < -180)
			{
				PID_SetAngle += 360;
			}
			Move(BasicSpeed - (AnglePID(P_Angle, I_Angle, D_Angle, PID_SetAngle, GetAngle())/2), \
			BasicSpeed + (AnglePID(P_Angle, I_Angle, D_Angle, PID_SetAngle, GetAngle())/2));	
		}
		if(direction == 0)
		{
			PID_SetAngle = LocationPID(P_Location, I_Location, D_Location, SetXpos, GetXpos()) + AngleSetLine + 180;
			if(PID_SetAngle > 180)
			{
				PID_SetAngle -= 360;
			}
			if(PID_SetAngle < -180)
			{
				PID_SetAngle += 360;
			}
			Move(BasicSpeed - (AnglePID(P_Angle, I_Angle, D_Angle, PID_SetAngle, GetAngle())/2), \
			BasicSpeed + (AnglePID(P_Angle, I_Angle, D_Angle, PID_SetAngle, GetAngle())/2));
		}
	}
} 

float AnglePID(float Kp, float Ki, float Kd, float AngleSet, float AngleActual)
{
	float  AngleError = 0;
	static float AngleLastError =0;
	static float AngleIntegralError = 0;
	AngleError = AngleSet - AngleActual;
	if(AngleError < -180)
	{
		AngleError += 360;
	}
	if(AngleError > 180)
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
	float  LocationError = 0;
	float  LocationControl = 0;
	static float LocationLastError =0;
	static float LocationIntegralError = 0;
	LocationError = LocationSet - LocationActual;
	LocationIntegralError += LocationError;
	LocationControl = Kp*LocationError + Ki*LocationIntegralError + Kd*(LocationError - LocationLastError);
	LocationLastError = LocationError;
	if(LocationControl >= 90)
	{
		LocationControl = 90;
	}
	if(LocationControl <= -90)
	{
		LocationControl = -90;
	}
	return LocationControl;
}

float CircleDistancePID(float Kp, float Ki, float Kd, float RadiusSet, float DistanceToCenterActual)
{
	float CircleDistanceError = 0;
	float CircleDistanceControl = 0;
	static float CircleDistanceLastError =0;
	static float CircleDistanceIntegralError = 0;
	CircleDistanceError = RadiusSet - DistanceToCenterActual;
	CircleDistanceIntegralError += CircleDistanceError;
	CircleDistanceControl = Kp*CircleDistanceError + Ki*CircleDistanceIntegralError + Kd*(CircleDistanceError - CircleDistanceLastError);
	CircleDistanceLastError = CircleDistanceError;
	if(CircleDistanceControl > 180)
	{
		CircleDistanceControl = 180;			
	}
	if(CircleDistanceControl < -180)
	{
		CircleDistanceControl = -180;
	}
	return CircleDistanceControl;
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
