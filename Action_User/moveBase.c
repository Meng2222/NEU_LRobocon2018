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
//j距离的P
float k3p=50;
//角度的P
float k4p=100;

extern struct pos_t{
	float x;
	float y;
	float angle;	
}action;
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
#include "math.h"
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


 

void turn(float setValue,float feedbackValue,float direc)//原地单轮转到固定角度
{
//    float Ierr_1=0;
    float err_value_1;
    float Uk_1=0;
	int motor1_value_1=0;
	int motor2_value_1=0;
	err_value_1=setValue-feedbackValue;
	//处理突变
	if(err_value_1>180)
	err_value_1=err_value_1-360;
	if(err_value_1<-180)
	err_value_1=360+err_value_1;
	Uk_1=err_value_1*K1p;//P调节
	
	
//	惯性过冲限制
	if(Uk_1>10000)
	Uk_1=10000;
	if(Uk_1<-10000)
	Uk_1=-10000;
	motor1_value_1=(int)Uk_1; 
	motor2_value_1=(int)Uk_1;
if(direc==1)//逆时针	
{
	VelCrl(CAN2, 01,0); 
	VelCrl(CAN2, 02,-motor1_value_1); 
}
if(direc==-1)//顺时针
{
	VelCrl(CAN2, 01,motor2_value_1); 
	VelCrl(CAN2, 02,0);
}	
	//USART_OUT( UART4, (uint8_t*)"Amotor value: " );
	//USART_OUT( UART4, (uint8_t*)"%d ",motor1_value_1 );
	//USART_OUT( UART4, (uint8_t*)"%d ",motor2_value_1 );
}








//直行
 float Uk_2=0;
void straight(float setValue,float feedbackValue)
{

    float err_value_2;
    
	int motor1_value_2=0;
	int motor2_value_2=0;

	extern int mission;
	extern int car;
    err_value_2=setValue-feedbackValue;
	
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


	





    


 


    float distance;	
	float d_value=0;
	float d_err=0;
	float d_Uk=0;
	float alpha=0;
    float alpha_set=0;
    float distance_set=0;
	float a_value=0;
	float a_err=0;
	float a_Uk=0;
	float dx=0;
	float dy=0;
	float D_flag=0;
	float motor1_value_1;
	float motor2_value_1;
float go_to(float x0, float y0 ,float x, float y)//由当前点（x0，y0）走向目标点（x，y)
	{

		//距离设定值
		distance_set=sqrt((x-x0)*(x-x0)+(y-y0)*(y-y0));
		//距离feedback
		distance=sqrt((action.x-x)*(action.x-x)+(action.y-y)*(action.y-y));
		//角度设定值
		alpha_set=atan((y-y0)/(x-x0))/PAI*180;
		//角度feedback
		alpha=atan((action.y-y)/(action.x-x))/PAI*180;

	
	
	
	//距离调整
	d_err=distance_set-distance;
	d_Uk=d_err*k3p;
	motor1_value_1=d_Uk;
	motor2_value_1=-d_Uk;
	
		
	//角度feedback'以防直线走偏
	a_err=(alpha_set-alpha);
	if(a_err>180)
		a_err=a_err-360;
	if(a_err<-180)
		a_err=a_err+360;
	a_Uk=a_err*k4p;
	
	
	//偏差不太大的话，差速调整角度，保持车速
	if(a_Uk>2&&a_Uk<45)
		motor2_value_1=motor2_value_1-a_Uk;
	if(a_Uk<-2&&a_Uk>=-45)
		motor1_value_1=motor1_value_1-a_Uk;
	
	
	if(a_Uk>45||a_Uk<-45)//偏差过大，停下来纠正偏差
	{	
		motor1_value_1=5*a_Uk;
	    motor2_value_1=5*a_Uk;
	}
	if(motor1_value_1>8000)
		motor1_value_1=8000;
	if(motor2_value_1>8000)
		motor2_value_1=8000;
	if(motor1_value_1<-8000)
		motor1_value_1=-8000;
	if(motor2_value_1<-8000)
		motor2_value_1=-8000;
	//电机做动作
	VelCrl(CAN2, 01,motor1_value_1);
	VelCrl(CAN2, 02,motor2_value_1);
	if(x-x0>0)
		dx=x-x0;
	if(y-y0>0)
		dy=y-y0;
	if(x0-x>0)
		dx=x0-x;
	if(y0-y>0)
		dy=y0-y;
	if(dx<80&&dy<80)
		D_flag=1;
	else D_flag=0;
	return D_flag;
	}





   




	float Ierr_1=0;
    float err_value_1;
    float Uk_1=0;
	float d_angle=0;
	int motor1_value_2=0;
	int motor2_value_2=0;
	int A_flag=0;
	float adjust_angle(float a)//由当前角度调整到目标角度a
{

	err_value_1=a-action.angle;
	//处理突变
	if(err_value_1>180)
	err_value_1=err_value_1-360;
	if(err_value_1<-180)
	err_value_1=360+err_value_1;
	Uk_1=err_value_1*K1p;//Uk是我要更改的角度            P调节
	
//	惯性过冲限制
	if(Uk_1>10000)
	Uk_1=10000;
	if(Uk_1<-10000)
	Uk_1=-10000;
	motor1_value_2=(int)Uk_1;
	motor2_value_2=(int)Uk_1; 
	VelCrl(CAN2, 01,motor1_value_2);
	VelCrl(CAN2, 02,motor2_value_2);
	//USART_OUT( UART4, (uint8_t*)"Amotor value: " );
	//USART_OUT( UART4, (uint8_t*)"%d ",motor1_value_1 );
	//USART_OUT( UART4, (uint8_t*)"%d ",motor2_value_1 );
	if(a-action.angle>=0)
		d_angle=a-action.angle;
	if(action.angle-a<0)
		d_angle=action.angle-a;
     if(d_angle<5)
		 A_flag=1;
	 else A_flag=0;
	return A_flag;
}
	
	
	
	
	


int arrive_flag=0;
int patrolline(float x,float y,int V)//输入要去的点的坐标
{
	VelCrl(CAN2, 01,V);
	VelCrl(CAN2, 02,V);
	if(((action.x-x<20)||(action.x>-20))&&((action.y-y<20)||(action.y-y>-20)))
		arrive_flag=1;
    else arrive_flag=0;
	return arrive_flag;
}






/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
