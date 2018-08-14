/**
  ******************************************************************************
  * @file    .h
  * @author  ACTION_2017
  * @version V0.0.0._alpha
  * @date    2017//
  * @brief   This file contains all the functions prototypes for 
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOVEBASE_H
#define __MOVEBASE_H



/* Includes ------------------------------------------------------------------*/

#include "elmo.h"

/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  
  */


 
/* Exported constants --------------------------------------------------------*/



/** @defgroup 
  * @{
  */

//#define 

//电机旋转一周的脉冲数
#define COUNTS_PER_ROUND (4096)
//轮子直径（单位：mm）
#define WHEEL_DIAMETER (106.8f)
//调试小车车长（单位：mm）
#define MOVEBASE_LENGTH (500.0f)
//调试小车车宽(单位：mm)
#define MOVEBASE_WIDTH (403.0f)
//轮子宽度（单位：mm）
#define WHEEL_WIDTH (46.0f)
//两个轮子中心距离（单位：mm）
#define WHEEL_TREAD (355.4f)
#define PI (3.14f)


/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void WheelSpeed(float speed,int Num);
void Round(float speed,float radius);


#endif /* ___H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

