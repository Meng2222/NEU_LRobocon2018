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


 
/* Exported constants ----- ---------------------------------------------------*/



/** @defgroup 
  * @{
  */

//#define 

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
#define WHEEL_TREAD (434.0f)

#define Pi 3.141592

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






//航向电机150ms延迟，5ms转1度，提前转好
//发射电机150ms延迟，5ms加速1圈，提前设定好
//推球电机500ms延迟，成功率低，提前发球，调提前的时间
//底盘电机400ms延迟，提前发命令可解决，已解决
//炮口距后挡板160+-5mm
//定位系统距后挡板95mm



/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/





#endif /* ___H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

