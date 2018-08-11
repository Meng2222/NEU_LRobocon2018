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

#include "elmo.h"

/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/

/**
  * @brief  走直线
  * @note	
  * @param  speed：给定速度
  * @retval None
  */

void Straight(float speed)
{
	int32_t pulseNum=(speed*4095*1000)/(PI*WHEEL_DIAMETER);
	
	VelCrl(CAN2, 0x01,pulseNum);
	VelCrl(CAN2, 0x02,-pulseNum);
}

/**
  * @brief  转圈
  * @note	
* @param  rightSpeed：给定右轮速度,正数为逆时针转,为外侧速度；负数顺时针转，为内侧速度
  * @param  radius：给定半径
  * @retval None
  */

void Round(float rightSpeed,float radius)
{
	float leftSpeed;
	int32_t rightPulseNum;
	int32_t leftPulseNum;
	//逆时针转
	if(rightSpeed > 0.0)
	{
		rightPulseNum=(rightSpeed*4095*1000)/(PI*WHEEL_DIAMETER);
		leftSpeed=rightSpeed*(radius-(WHEEL_TREAD/1000))/(radius+(WHEEL_TREAD/1000));
		leftPulseNum=-(leftSpeed*4095*1000)/(PI*WHEEL_DIAMETER);
		
		VelCrl(CAN2, 0x01,rightPulseNum);
		VelCrl(CAN2, 0x02,leftPulseNum);
		
	}
	//顺时针转
	else
	{
		rightPulseNum=-(rightSpeed*4095*1000)/(PI*WHEEL_DIAMETER);
		leftSpeed=rightSpeed*(radius+(WHEEL_TREAD/1000))/(radius-(WHEEL_TREAD/1000));
		leftPulseNum=(leftSpeed*4095*1000)/(PI*WHEEL_DIAMETER);
		
		VelCrl(CAN2, 0x01,rightPulseNum);
		VelCrl(CAN2, 0x02,leftPulseNum);
	}
}

/**
  * @brief  走直线
  * @note	
  * @param  speed：给定速度
  * @retval None
  */

void Turn(uint8_t dir,float angle)
{
	int32_t pulseNum;
	float getAngle;
	float speed;
	PidPara(1.0,0.1,0.1);
	getAngle=GetAngle();
	speed=Pid(getAngle,angle,ON);
	if(dir)
	{
		pulseNum=(speed*4095*1000)/(PI*WHEEL_DIAMETER);
		VelCrl(CAN2, 0x01,pulseNum);
	}
	else
	{
		pulseNum=-(speed*4095*1000)/(PI*WHEEL_DIAMETER);
		VelCrl(CAN2, 0x02,-pulseNum);
	}
}

/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
