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

#include <stdint.h>
#include "pid.h"
#include "stm32f4xx_it.h"
#include "usart.h"

/* Includes ------------------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  
  */


 
/* Exported constants --------------------------------------------------------*/



/** @defgroup 
  * @{
  */

#define PI (3.141593)
#define TURN_RIGHT 0
#define TURN_LEFT  1

//电机旋转一周的脉冲数
#define COUNTS_PER_ROUND (4096.0f)
//轮子直径（单位：mm）
#define WHEEL_DIAMETER (120.0f)
//调试小车车长（单位：mm）
#define MOVEBASE_LENGTH (492.0f)
//调试小车车宽(单位：mm)
#define MOVEBASE_WIDTH (490.0f)
//轮子宽度（单位：mm）
#define WHEEL_WIDTH (40.0f)
//两个轮子中心距离（单位：mm）
#define WHEEL_TREAD (424.0f)




/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


void Straight(float speed);
void Round(float rightSpeed,float radius);
void Turn(float angle,float gospeed);
void BackTurn(float angle,float gospeed);
void BTP(float angle);
void Square(void);
void straightLine(float A1,float B1,float C1,uint8_t dir);
uint8_t BackstraightLine(float A2,float B2,float C2,uint8_t dir);
void BiggerSquareOne(void);
void BiggerSquareTwo(void);
void SquareTwo(void);
void RoundTwo(float centerX,float centerY,float r,uint8_t o,float speed);
float Speed_X(void);
float Speed_Y(void);
#endif /* ___H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

