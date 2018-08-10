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
void GoStraight(float speed)
{
    WheelSpeed(speed, 1);
    WheelSpeed(speed, 2);
}    

void MakeCircle(float speed, float radius)
{
    WheelSpeed(((speed / radius) * (radius - (WHEEL_TREAD / 2))), 1);
    WheelSpeed(((speed / radius) * (radius + (WHEEL_TREAD / 2))), 2);
}

void WheelSpeed(float speed, uint8_t ElmoNum)
{
    int32_t frequency = 0;
    frequency = (int32_t)((COUNTS_PER_ROUND * speed)/(PI * WHEEL_DIAMETER));
    VelCrl(CAN2, ElmoNum, frequency);
}

/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
