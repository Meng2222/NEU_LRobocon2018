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
float K1p=100;


float K1i=0.002;
float K1d=0.002;
float K2p=50;
float K2i=0.003;
float K2d=0.002;
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
  #define duty_1 20
  #define duty_2 1 
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

	float Ierr_1=0;
    float pre_err_1=0;
    float Derr_1=0;
    float err_value_1;
    float Uk_1=0;
	int motor1_value_1=0;
	int motor2_value_1=0;
 
    float Ierr_2=0;
    float pre_err_2=0;
    float Derr_2=0;
    float err_value_2;
     float Uk_2=0;
	int motor1_value_2=0;
	int motor2_value_2=0;




//指定角度转弯
void turn(float setValue,float feedbackValue)
{
    
	err_value_1=setValue-feedbackValue;
	//处理突变
	if(err_value_1>180)
	err_value_1=err_value_1-360;
	if(err_value_1<-180)
	err_value_1=360+err_value_1;
	
	
	Ierr_1=err_value_1+Ierr_1;
	Derr_1=err_value_1-pre_err_1;
	Uk_1=err_value_1*K1p;//Uk是我要更改的角度            P调节
	
//	惯性过冲限制
	if(Uk_1>10000)
	Uk_1=10000;
	if(Uk_1<-10000)
	Uk_1=-10000;
	
	
//	if(Uk_1<-180)
//	Uk_1=360+Uk_1;
	
	//Uk_1=-Uk_1;//一号车角度反向
	
	motor1_value_1=(int)Uk_1;
	motor2_value_1=(int)Uk_1; 
	VelCrl(CAN2, 01,motor1_value_1);
	VelCrl(CAN2, 02,motor2_value_1);
	//USART_OUT( UART4, (uint8_t*)"Amotor value: " );
	//USART_OUT( UART4, (uint8_t*)"%d ",motor1_value_1 );
	//USART_OUT( UART4, (uint8_t*)"%d ",motor2_value_1 );
    pre_err_1=err_value_1;
}





//直行
void straight(float setValue,float feedbackValue)
{
	extern int mission;
	extern int car;
    err_value_2=setValue-feedbackValue;

	Ierr_2=err_value_2+Ierr_2;
	Derr_2=err_value_2-pre_err_2;
	pre_err_2=err_value_2;
	Uk_2=err_value_2*K2p;//Uk是我要调整的前进的距离
    if(Uk_2>=10000)
		Uk_2=10000;
	if(Uk_2<=-10000)
		Uk_2=-10000;
	if((mission==4||mission==7)&&car==4)
	{
		Uk_2=-Uk_2;
	}
		if((mission==1||mission==4)&&car==1)
	{
		Uk_2=-Uk_2;
	}
		motor1_value_2=(int)Uk_2;
	    motor2_value_2=(int)-Uk_2;
	
	VelCrl(CAN2, 01,motor1_value_2);
	VelCrl(CAN2, 02,motor2_value_2);
  //  USART_OUT( UART4, (uint8_t*)"Vmotor value: " );
  //	USART_OUT( UART4, (uint8_t*)"%d ",motor1_value_2 );
//	USART_OUT( UART4, (uint8_t*)"%d ",motor2_value_2 );
}




/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
