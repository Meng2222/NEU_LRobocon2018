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
float   PID_SetAngle = 0;
float Error = 0;
float AngleControl = 0;
static float angle = 0;
static float xpos = 0;
static float ypos = 0;

void CircleAround(float radius, float speed) //(半径(mm)), 速度(mm/s))
{
	float PulseR = 0;
	float PulseL = 0; 
	PulseR = ((((2.f*radius - WHEEL_TREAD - WHEEL_WIDTH) * speed)/(2.f*radius)) * COUNTS_PER_ROUND)/(WHEEL_DIAMETER * 3.14f);
	PulseL = ((radius+((WHEEL_WIDTH+WHEEL_TREAD)/2))/(radius-((WHEEL_WIDTH+WHEEL_TREAD)/2.0f))) * PulseR;
	VelCrl(CAN2, 1, (int)PulseR);	//右轮(正为向前)
	VelCrl(CAN2, 2, (int)-PulseL);	//左轮(负为向前)
}

void RectangleAround(float length, float width, float BasicSpeed) //(长(mm), 宽(mm), 速度(mm/s))
{
	if(GetXpos() <= 300 && GetYpos() <= width-100)
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
	if(GetXpos() > 300 && GetYpos() <100)
	{
		LocationFlag = 4;
	}

	if(LocationFlag == 1)
	{
		PID_SetAngle = 0;
		Move(BasicSpeed - AnglePID(P,I,D,PID_SetAngle,GetAngle()), BasicSpeed + AnglePID(P,I,D,PID_SetAngle,GetAngle()));
	}
	if(LocationFlag == 2)
	{
		PID_SetAngle = -90;
		Move(BasicSpeed - AnglePID(P,I,D,PID_SetAngle,GetAngle()), BasicSpeed + AnglePID(P,I,D,PID_SetAngle,GetAngle()));
	}
	if(LocationFlag == 3)
	{
		PID_SetAngle = -179;
		Move(BasicSpeed - AnglePID(P,I,D,PID_SetAngle,GetAngle()), BasicSpeed + AnglePID(P,I,D,PID_SetAngle,GetAngle()));
		if(GetAngle() >= -180 && GetAngle() <= 0)
		{
			PID_SetAngle = -179;
			Move(BasicSpeed - AnglePID(P,I,D,PID_SetAngle,GetAngle()), BasicSpeed + AnglePID(P,I,D,PID_SetAngle,GetAngle()));
		}	
		if(GetAngle() >= 0 && GetAngle() <= 180)
		{
			PID_SetAngle = 179;
			Move(BasicSpeed - AnglePID(P,I,D,PID_SetAngle,GetAngle()), BasicSpeed + AnglePID(P,I,D,PID_SetAngle,GetAngle()));
		}
	}
	if(LocationFlag == 4)
	{
		PID_SetAngle = 90;
		Move(BasicSpeed - AnglePID(P,I,D,PID_SetAngle,GetAngle()), BasicSpeed + AnglePID(P,I,D,PID_SetAngle,GetAngle()));
	}
	
}
float AnglePID(float Kp, float Ki, float Kd, float AngleSet, float AngleActual)
{
	static float LastError =0;
	static float IntegralError = 0;
	Error = AngleSet - AngleActual;
	if(Error <= -180)
	{
		Error += 360;
	}
	if(Error >= 180)
	{
		Error -= 360;
	}
	IntegralError += Error;
	AngleControl = Kp*Error + Ki*IntegralError + Kd*(Error - LastError);
	LastError = Error;
	return AngleControl;
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

float GetAngle(void)
{
	return angle;
}

float GetXpos(void)
{
	return xpos;
}

float GetYpos(void)
{
	return ypos;
}



/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
