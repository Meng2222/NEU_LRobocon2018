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
#define WHEEL_DIAMETER (120.0f)
//调试小车车长（单位：mm）
#define MOVEBASE_LENGTH (492.0f)
//调试小车车宽(单位：mm)
#define MOVEBASE_WIDTH (490.0f)
//轮子宽度（单位：mm）
#define WHEEL_WIDTH (40.0f)
//两个轮子中心距离（单位：mm）
//#define WHEEL_TREAD (434.0f)

// 宏定义棍子收球电机ID
#define COLLECT_BALL_ID (8)
// 宏定义推球电机ID
#define PUSH_BALL_ID (6)
// 宏定义送弹机构送弹时电机应该到达位置：单位位脉冲
#define PUSH_POSITION (4500)
// 宏定义送弹机构收回时电机位置
#define PUSH_RESET_POSITION (5)
//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
// 宏定义发射机构航向电机ID
#define GUN_YAW_ID (7)
// 电机旋转一周的脉冲数
#define COUNT_PER_ROUND (4096.0f)
// 宏定义每度对应脉冲数
#define COUNT_PER_DEGREE  (COUNT_PER_ROUND/360.0f)
// 宏定义航向角减速比
#define YAW_REDUCTION_RATIO (4.0f)
// 发射航向角转换函数 由度转换为脉冲
// yawAngle为角度，范围180到-180之间，初始位置为0度。
float YawTransform(float yawAngle);
void YawAngleCtr(float yawAngle);
void Straight(float v);
void Spin(float R,float v);
void TurnRight(float angle,float v);
void AnglePID(float setAngle,float feedbackAngle);
void GetFunction(float x1,float y1,float x2,float y2);
void linePID(float x1,float y1,float x2,float y2,float v);
void CirclePID(float x0,float y0,float R,float v,int status);
#define WHEEL_TREAD (355.4f)

//后轮电机的CAN ID号
#define BACK_WHEEL_ID             5
//前轮转向电机的CAN ID号
#define TURN_AROUND_WHEEL_ID      6

//新三轮底盘  前轮转向电机到后轮两轮轴中心间距
#define TURN_AROUND_WHEEL_TO_BACK_WHEEL                               (286.f)

//定位系统到后轮两轮轴中心间距
#define OPS_TO_BACK_WHEEL                                             (116.5f)
//前轮后轮都是3508转一周脉冲都为8192
#define NEW_CAR_COUNTS_PER_ROUND                                      (8192)
//转向轮子直径
#define TURN_AROUND_WHEEL_DIAMETER                                    (50.8f)
//3508电机减速比，相当于给出去的脉冲要多乘上减速比
#define REDUCTION_RATIO                                               (19.2f)

/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/





#endif /* ___H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

