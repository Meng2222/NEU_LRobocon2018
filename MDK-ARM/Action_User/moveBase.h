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
#include "stdint.h"


/* Includes ------------------------------------------------------------------*/



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
#define COUNTS_PER_ROUND (32768)
//轮子直径（单位：mm）
#define WHEEL_DIAMETER (120.0f)
//调试小车车长（单位：mm）
#define MOVEBASE_LENGTH (492.0f)
//调试小车车宽(单位：mm)
#define MOVEBASE_WIDTH (490.0f)
//轮子宽度（单位：mm）
#define WHEEL_WIDTH (40.0f)
//两个轮子中心距离（单位：mm）
#define WHEEL_TREAD (457.0f)

// 宏定义棍子收球电机ID
#define COLLECT_BALL1_ID (5)//左
#define COLLECT_BALL2_ID (6)//右
// 宏定义推球电机ID
#define PUSH_BALL_ID (7)
// 宏定义送弹机构送弹时电机应该到达位置：单位位脉冲
#define PUSH_POSITION (32768)
// 宏定义航向角减速比
#define YAW_REDUCTION_RATIO (4.0f)
float YawTransform(float yawAngle);
void YawAngleCtr(float yawAngle);
void Straight(float v);
void Spin(float R,float v);
void TurnRight(float angle,float v);
void AnglePID(float setAngle,float feedbackAngle);
void GetFunction(float x1,float y1,float x2,float y2);
void linePID(float x1,float y1,float x2,float y2,float v);
void CirclePID(float x0,float y0,float R,float v,int status);
void GetYawangle(uint8_t StdId);
void GetDistance(uint8_t StdId);
void BingoJudge(uint8_t StdId);
void GetShootSituation(uint8_t StdId);
int FirstshootJudge(void);
void Rchange(int Rchange);
float getLingtAngle(float xi,float yi,int tragetCnt);
#define pi 3.141592f
//后轮电机的CAN ID号
#define BACK_WHEEL_ID             1
//前轮转向电机的CAN ID号
#define TURN_AROUND_WHEEL_ID      2

//前轮转向电机到后轮两轮轴中心间距
#define TURN_AROUND_WHEEL_TO_BACK_WHEEL                               (266.55f)

//定位系统到后轮两轮轴中心间距
#define OPS_TO_BACK_WHEEL                                             (180.47f)
//前轮后轮都是3508转一周脉冲都为8192
#define CAR_WHEEL_COUNTS_PER_ROUND                                      (8192)
//转向轮子直径
#define TURN_AROUND_WHEEL_DIAMETER                                    (50.72f)
//3508电机减速比，相当于给出去的脉冲要多乘上减速比
#define REDUCTION_RATIO                                               (19.2f)
//电机与轮子减速比
#define WHEEL_REDUCTION_RATIO										  (1.004f)
//己方的球(1是白球,2是黑球)
#define MY_BALL_COLOR													(1)
/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/





#endif /* ___H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

