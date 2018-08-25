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
#define WHEEL_DIAMETER (106.8f)
//调试小车车长（单位：mm）
#define MOVEBASE_LENGTH (500.0f)
//调试小车车宽(单位：mm)
#define MOVEBASE_WIDTH (403.0f)
//轮子宽度（单位：mm）
#define WHEEL_WIDTH (46.0f)
//两个轮子中心距离（单位：mm）
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
//
/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/





#endif /* ___H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

