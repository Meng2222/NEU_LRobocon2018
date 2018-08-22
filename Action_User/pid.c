#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "pid.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "moveBase.h"
#include "adc.h"
#include "pps.h"
#include "fort.h"
#include <math.h>
#define Kp1 (98)   //角度PID参数
#define Ki1 (0)
#define Kd1 (0) 
#define Kp2 500.0/1000*7 //位置PID参数
#define Ki2 (0)
#define Kd2 (0)
#define speed (1000)  //小车速度
#define pointX  (0.0f)  //pointX，pointY：给定圆形的坐标
#define pointY  (2200.0f)
float nowAngle,R=2000,motorState;
float u1,u2;
/**
* @brief  圆形的角度闭环
*/
void uAng(void)
{
	static float lastErr = 0;
	static float sumErr = 0;
	float err,pointAngle;
	if(motorState==0)
		pointAngle=atan2(pointX-GetX(),GetY()-pointY)*180/PI;
	if(motorState==1)
		pointAngle=atan2(GetX()-pointX,pointY-GetY())*180/PI;
	nowAngle=GetAngle()+90;
	if(nowAngle>180)
	 nowAngle=nowAngle-360;
     err=pointAngle-nowAngle+u2;
	if(err>180)
		err=err-360;
	if(err<-180)
		err=err+360;
	sumErr += err;
	u1= -Kp1 * err + Ki1 * sumErr +Kd1 *(err - lastErr);
	lastErr = err;
}
/**
* @brief  圆形的半径闭环
*/
void uPlace(void)
{
	static float lastErr = 0;
	static float sumErr = 0;
	float err,distance;
    distance=sqrt((pointX-GetX())*(pointX-GetX())+(pointY-GetY())*(pointY-GetY()));
	err=R-distance;
	if(motorState==1)
		err=-err;
	sumErr += err;
	u2= Kp2*0.01*err + Ki2 * sumErr +Kd2 *(err - lastErr);
	if(u2>90)
		u2=90;
	if(u2<-90)
		u2=-90;
	lastErr = err;
}
/**
* @brief  圆形闭环主程序
*/
void walkRound(void)
{
		static float buff1,buff2,pulse,distance1,distance2,rState;
		static float lastX,lastY,errState=0,errCount,buffstate,buffR;
		static int switchState=0;
        distance1=Get_Adc_Average(14,10)*(4400/4096);			//设置ADC1通道14,15，平均值获取次数为10
		distance2=Get_Adc_Average(15,10)*(4400/4096);	
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)nowAngle);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetX());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetY());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)R);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)u2);
		USART_OUT(UART4,(uint8_t*)"%d\r\n",(int)u1);
		switch(switchState)
		{
		 case 0:
				if(distance2<=100)
				{
					motorState=0;
					switchState=1;
				}
			    else if(distance1<=100)
				{
					motorState=1;
					switchState=1;
				}
		 break;
		 case 1:
			if(fabs(lastX-GetX())<2&&fabs(lastY-GetY())<2)
			{
				errCount++;
				if(errCount>=60)
				{
					errState=1;
					errCount=0;
				}
			}
			lastX=(int)GetX();
			lastY=(int)GetY();
			if(R>=2000)
				rState=0;
			if(R<=500)
				rState=1;
			if(rState==0)
			{
				if(nowAngle>=170&&nowAngle<=180&&GetX()<=10&&GetX()>=2)
					R-=250;
			}
			if(rState==1)
			{
				if(nowAngle>=170&&nowAngle<=180&&GetX()<=10&&GetX()>=2)
					R+=250;
			}
			uPlace();
			uAng();
		   if(errState==0)
		   {
			   pulse=4096*speed/(WHEEL_DIAMETER*PI);  //mm/s转化为脉冲/s
			   buff1=(1+0.5*WHEEL_TREAD/R)*pulse+u1;  //左轮       u+右， u-左，
			   buff2=(1-0.5*WHEEL_TREAD/R)*pulse-u1;   //右轮
			   VelCrl(CAN2,1,buff2);
			   VelCrl(CAN2,2,-buff1);
		   }
		   if(errState>=1)
		   {
			   VelCrl(CAN2,1,-13000);
			   VelCrl(CAN2,2,13000);
			   errState++;
			   buffR=R;
			   if(errState>=100)
			   {
				   errState=0;
				   buffstate=1;
				   if(R<=1400)
					 R+=900;
				   else
					 R-=900;	   
			   }
		   }
		   if(buffstate>=1)
		  {
			   buffstate++;
			   if(buffstate>=180)
			   {
				   buffstate=0;
				   R=buffR;
			   }
		   } 
       break;		  
	 }
}


