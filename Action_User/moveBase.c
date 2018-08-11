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
#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "moveBase.h"
  #define PAI 3.14 
/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/



/**
  * @brief  转圈
  * @note
  * @param  V: 单位m/s 绕圈速度   R：单位m 绕圈半径 
  * @param  motor1_value：右轮电机给定值（脉冲/s） motor2_value：左轮电机给定值（脉冲/s）负值
  * @param  ElmoNum1:1号电机ID号，ElmoNum2：2号电机ID号
  * @retval None
  */
void  go_round(float V,float R,int ElmoNum1,int ElmoNum2 ,CAN_TypeDef* CANx)
{
	float v1=0,v2=0;
	float motor1_value=0;
	float motor2_value=0;
	v1=(2*0.001*WHEEL_TREAD+4*R)*V/(4*R);
	v2=(4*R-2*0.001*WHEEL_TREAD)*V/(4*R);
	motor1_value=(int)v1*4095/(PAI*0.001*WHEEL_DIAMETER);
	motor2_value=(int)-v2*4095/(PAI*0.001*WHEEL_DIAMETER);
	
	VelCrl(CANx, ElmoNum1,motor1_value);
	VelCrl(CANx, ElmoNum2,motor2_value);
}



/**
  * @brief  走直线
  * @note
  * @param  V: 单位m/s 绕圈速度 
  * @param  motor1_value：右轮电机给定值（脉冲/s） motor2_value：左轮电机给定值（脉冲/s）负值
  * @param  ElmoNum1:1号电机ID号，ElmoNum2：2号电机ID号
  * @retval None
  */
void  go_straight(float V,int ElmoNum1,int ElmoNum2 ,CAN_TypeDef* CANx)
{
	int motor1_value=(int)V*4095/(PAI*0.001*WHEEL_DIAMETER);
	int motor2_value=(int)(-V*4095/(PAI*0.001*WHEEL_DIAMETER));
	VelCrl(CAN2, 01,motor1_value);
	VelCrl(CAN2, 02,motor2_value);
}	
void correct_direction()
{
	
	
//	VelCrl(CAN2, 01,motor1_value);
//	VelCrl(CAN2, 02,motor2_value);
	
}


#define K1p  2//
#define K1i  2//
#define K1d  2//

//指定角度转弯
float turn(float setValue,float feedbackValue)
{
	float Ierr=0;
    float pre_err=0;
    float Derr=0;
    float err_value;
    static int Uk;
    float last_angle=0;
	int motor1_value=0;
	int motor2_value=0;
	err_value=setValue-feedbackValue;
	Ierr=err_value+Ierr;
	Derr=err_value-pre_err;
	Uk=err_value*K1p +Ierr*K1i+Derr*K1d ;//Uk是我要更改的角度
	motor1_value=0;
	motor2_value=(int)(-(Uk*WHEEL_TREAD/0.005)*4095/(PAI*0.001*WHEEL_DIAMETER)); 
	VelCrl(CAN2, 01,motor1_value);
	VelCrl(CAN2, 02,motor2_value);
    pre_err=err_value;
}
//直行
float straight(float setValue,float feedbackValue)
{
	float Ierr=0;
    float pre_err=0;
    float Derr=0;
    float err_value;
    static int Uk;
    float last_angle=0;
	int motor1_value=0;
	int motor2_value=0;
    err_value=setValue-feedbackValue;
	Ierr=err_value+Ierr;
	Derr=err_value-pre_err;
	pre_err=err_value;
	Uk=err_value*K1p +Ierr*K1i+Derr*K1d ;//Uk是我要调整的前进的距离
	motor1_value=(int)(Uk/0.005)*4095/(PAI*0.001*WHEEL_DIAMETER);
	motor2_value=(int)(-(Uk/0.005)*4095/(PAI*0.001*WHEEL_DIAMETER));
	VelCrl(CAN2, 01,motor1_value);
	VelCrl(CAN2, 02,motor2_value);
}




/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
