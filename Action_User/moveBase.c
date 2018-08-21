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
void MainWheelSpeed(float speed)
{
    int32_t frequency = 0;
    frequency = (int32_t)((NEW_CAR_COUNTS_PER_ROUND * speed)/(PI * WHEEL_DIAMETER) * REDUCTION_RATIO);
    VelCrl(CAN2, BACK_WHEEL_ID, frequency);
}

void TurnWheelSpeed(float speed)
{
    int32_t frequency = 0;
    frequency = (int32_t)((NEW_CAR_COUNTS_PER_ROUND * speed) / (PI * TURN_AROUND_WHEEL_DIAMETER) * REDUCTION_RATIO);
    VelCrl(CAN2, TURN_AROUND_WHEEL_ID, frequency);
}
/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
