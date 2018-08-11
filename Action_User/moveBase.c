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

void RevolveAround(uint16_t radius, uint16_t speed)
{
	static int PulseR = 0;
	static int PulseL = 0;
	PulseR = ((((2*radius - WHEEL_TREAD - WHEEL_WIDTH) * speed)/(2*radius)) * COUNTS_PER_ROUND)/(WHEEL_DIAMETER * 3.141);
	PulseL = ((radius+((WHEEL_WIDTH+WHEEL_TREAD)/2))/(radius-((WHEEL_WIDTH+WHEEL_TREAD)/2))) * PulseR;
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
