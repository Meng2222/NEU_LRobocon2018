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
#include "math.h"

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
#define pi 3.141592f
float Kp=200,Ki=0.01,Kd=1,err=0,lastErr=0,Sumi=0,Output=0,Vk=0,errl,lastErr1,Vkl=0,Sumli=0;
void Straight(float v)
{
	VelCrl(CAN2,1,v*4096/(pi*WHEEL_DIAMETER)+Vk);
	VelCrl(CAN2,2,-v*4096/(pi*WHEEL_DIAMETER)+Vk);
}	
void Spin(float R,float v)
{
	VelCrl(CAN2,1,v*4096/(pi*WHEEL_DIAMETER));
	VelCrl(CAN2,2,-v*4096*(R+(WHEEL_TREAD-WHEEL_WIDTH)/2)/((R-(WHEEL_TREAD-WHEEL_WIDTH)/2)*pi*WHEEL_DIAMETER));
}
void TurnRight(float angle,float v)
{
	VelCrl(CAN2,1,0);
	VelCrl(CAN2,2,-v*4096*WHEEL_TREAD*angle*2/((WHEEL_DIAMETER)*360));
}	

void AnglePID(float setAngle,float feedbackAngle)
{
	if(setAngle>180)
		err=-(360-setAngle);
	if(setAngle<-180)
		err=360+setAngle;
	err=setAngle-feedbackAngle;
	if(err>180)
		err=-(360-err);
	if(err<-180)
		err=360+err;
	Sumi+=Ki*err;
	if((int)err==0)
		Sumi=0;
	Vk=Kp*err+Sumi+Kd*(err-lastErr);
	lastErr=err;
}	
float k,b,lAngle,setAngle,x,y;
static int flag;
void GetFunction(float x1,float y1,float x2,float y2)
{
		if(x1-0.1<x2&&x2<x1+0.1)
		{
			if(y2-y1>0)
				lAngle=0;
			else
				lAngle=180;
			flag=0;
		}	
	  	else
		{
			k=(y2-y1)/(x2-x1);
			b=y1-k*x1;
			if(k>0)
			{	
				if(y2-y1>=0)
					lAngle=(atan(k)*180/pi)-90;
				else 
					lAngle=(atan(k)*180/pi)+90;
			}	
			else if(k<0)
			{
				if(y2-y1>=0)
					lAngle=(atan(k)*180/pi)+90;
				else 
					lAngle=(atan(k)*180/pi)-90;
			}	
			else
			{	
				if(x2-x1>0)
					lAngle=-90;
				else
					lAngle=90;
			}	
			flag=1;
		}
}	
void linePID(float x1,float y1,float x2,float y2,float v)
{
		GetFunction(x1,y1,x2,y2);
		#if CAR_NUM==1
		x=-GetYpos();
		y=GetXpos();
		#elif CAR_NUM==4
		x=GetXpos();
		y=GetYpos();
		#endif
		if(flag)
		{
			if(k>0)
				if(y>k*x+b)
					setAngle=lAngle-90*(1-500/(y-k*x-b+500));
				else
					setAngle=lAngle+90*(1-500/(-y+k*x+b+500));
			else
				if(y>k*x+b)
					setAngle=lAngle+90*(1-500/(y-k*x-b+500));
				else
					setAngle=lAngle-90*(1-500/(-y+k*x+b+500));
		}	
	 	else
		{	
			if(y2-y1>0)
				if(x<x1)
					setAngle=lAngle-90*(1-1000/(fabs(x-x1)+1000));
				else
					setAngle=lAngle+90*(1-1000/(fabs(x-x1)+1000));	
			else 	
				if(x<x1)	
					setAngle=lAngle+90*(1-1000/(fabs(x-x1)+1000));		
				else
					setAngle=lAngle-90*(1-1000/(fabs(x-x1)+1000));
		}
		AnglePID(setAngle,GetAngle());
		Straight(v);
}	
/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
