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
void Round(float speed,float radius)
{
	WheelSpeed(((speed/radius)*(radius-(WHEEL_TREAD/2))),1);
	WheelSpeed(((speed/radius)*(radius-(WHEEL_TREAD/2))),2);
}
void WheelSpeed(float speed,int Num)
{
	int frequency=0;
	if(Num==1)
	{
		frequency=(int)((COUNTS_PER_ROUND*speed)/(PI*WHEEL_DIAMETER));
	}
	else if(Num==2)
	{
		frequency=(int)(-1*(COUNTS_PER_ROUND*speed)/(PI*WHEEL_DIAMETER));
	}
	VelCrl(CAN2,Num,frequency);
}	
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



/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
