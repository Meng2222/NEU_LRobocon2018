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
  
static float angle = 0;
static float xpos = 0;
static float ypos = 0;

void CircleAround(float radius, float speed) //(半径(mm)), 速度(mm/s))
{
	float PulseR = 0;
	float PulseL = 0;
	PulseR = ((((2.f*radius - WHEEL_TREAD - WHEEL_WIDTH) * speed)/(2.f*radius)) * COUNTS_PER_ROUND)/(WHEEL_DIAMETER * 3.14f);
	PulseL = (float)((radius+((WHEEL_WIDTH+WHEEL_TREAD)/2))/(radius-((WHEEL_WIDTH+WHEEL_TREAD)/2.0f))) * PulseR;
	VelCrl(CAN2, 1, (int)PulseR);	//右轮(正为向前)
	VelCrl(CAN2, 2, (int)-PulseL);	//左轮(负为向前)
}

void RectangleAround(float length, float width, float speed) //(长(mm), 宽(mm), 速度(mm/s))
{
	float PulseR = 0;
	float PulseL = 0; 
	if(GetAngle() == 0)
	{
	PulseR = (float)(speed*COUNTS_PER_ROUND)/(3.14f*WHEEL_DIAMETER);
	PulseL = (speed*COUNTS_PER_ROUND)/(3.14f*WHEEL_DIAMETER);
	VelCrl(CAN2, 1, (int)PulseR);	//右轮(正为向前)
	VelCrl(CAN2, 2, (int)-PulseL);	//左轮(负为向前)
	}
}
	
float AnglePID(float Kp, float Ki, float Kd, float AngleSet, float AngleActual)
{
	uint8_t Error = 0;
	uint8_t AngleControl;
	static uint8_t LastError =0;
	static uint8_t IntegralError;
	Error = AngleSet - AngleActual;
	IntegralError += Error;
	AngleControl = Kp*Error + Ki*IntegralError + Kd*(Error - LastError);
	LastError = Error;
	return AngleControl;
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

void RevolveAngle(float angle)
{
	for(uint8_t i = 1; i <= angle; i++ )
	{
		VelCrl(CAN2, 1, 1138);	//右轮(正为向前)
		VelCrl(CAN2, 2, -1138);	//左轮(负为向前)
	}
}

/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
