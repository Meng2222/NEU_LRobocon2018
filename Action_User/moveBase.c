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
  * @brief  PID 转弯
  * @note	
* @param  angle：给定角度,为正左转，为负右转
  * @param  getAngle：现在的角度
  * @retval None
  */

void Turn(float angle)
{
	int32_t pulseNum=0;
	int32_t bPulseNum=(700*4095)/(PI*WHEEL_DIAMETER);
	float getAngle=0;
	float speed=0;
	getAngle=GetAngle();
	speed=Pid(angle,getAngle);	
	
	pulseNum=(speed*4095)/(PI*WHEEL_DIAMETER);
	VelCrl(CAN2, 0x01,bPulseNum+pulseNum);
	VelCrl(CAN2, 0x02,pulseNum-bPulseNum);
//	if(err > 0.0)
//	{
//		VelCrl(CAN2, 0x01,pulseNum);
//		VelCrl(CAN2, 0x02,0);
//	}
//	else
//	{
//		VelCrl(CAN2, 0x01,0);
//		VelCrl(CAN2, 0x02,pulseNum);
//	}


}	
 
/**
  * @brief  PID 回正
  * @note	
  * @param  angle：给定角度
  * @param  getAngle：现在的角度
  * @retval None
  */

void BTP(float angle)
{
	int32_t pulseNum=0;
	int32_t bPulseNum=(700*4095)/(PI*WHEEL_DIAMETER);
	float getAngle=0;
	float speed=0;
	getAngle=GetAngle();
	
	speed=Pid(angle,getAngle);
	pulseNum=(speed*4095)/(PI*WHEEL_DIAMETER);
	
	VelCrl(CAN2, 0x01,bPulseNum+pulseNum);
	VelCrl(CAN2, 0x02,pulseNum-bPulseNum);

}

/**
  * @brief  走方形
  * @note	
  * @param  
  * @retval None
  */

void Square(void)
{
	float x;
	float y;
	
	static uint8_t flg=0;
	x=GetPosX();
	y=GetPosY();
	
	if(flg == 0)
	{
		if(y > 1600.0)
		{
			flg++;
		}
		else
		{
			Turn(0.0);
		}
	}
	else if(flg == 1)
	{
		if(x > 1600.0)
		{
			flg++;
		}
		else
		{
			Turn(-90.0);
		}
	}
	else if(flg == 2)
	{
		if(y < 400.0)
		{
			flg++;
		}
		else
		{
			Turn(-180.0);
		}
	}
	else if(flg == 3)
	{
		if(x < 400.0)
		{
			flg=0;
		}
		else
		{
			Turn(90.0);
		}
	}
	
}

/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
