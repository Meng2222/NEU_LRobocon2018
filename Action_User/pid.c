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
#define Kp1 (20000)   //角度PID参数
#define Ki1 (0)
#define Kd1 (0) 
#define Kp2 500.0/800*7 //位置PID参数
#define Ki2 (0)
#define Kd2 (0)
#define speed (800)  //小车速度
#define pointX  (0.0f)  //pointX，pointY：给定圆形的坐标
#define pointY  (2200.0f)
float nowAngle,R=2000,motorState,nowX,nowY;
float u1,u2;
extern FortType fort;
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
* @brief  推球
*/
void Pos(void)
{
	static float timeCount;
	timeCount++;
	if(timeCount>=50)
	PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);      // 推球电机推球
	if(timeCount>=100)
	{
	   PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION); // 推球电机复位
	   timeCount=0;
	}
}
/**
* @brief  航向电机转向角控制
*/
void YawPos(float circleX,float circleY)
{
	static float lineAngle,gunAngle;
	lineAngle=atan2(circleY-GetY(),circleX-GetX())*180/PI;
	if(motorState==0)
	{
	   gunAngle=270-GetAngle()+lineAngle;
		if(gunAngle>=360)
			gunAngle=gunAngle-360;
	   YawPosCtrl(170-gunAngle-18);
	}
	if(motorState==1)
	{
	   gunAngle=90+GetAngle()-lineAngle;
	   YawPosCtrl(170+gunAngle-18 );
 
	}
}
/**
* @brief  发射枪电机转速控制
*/
void ShooterVel(float circleX,float circleY)
{
	static float lineDistance,velSpeed;
	lineDistance=sqrt((circleX-GetX())*(circleX-GetX())+(circleY-GetY())*(circleY-GetY()));
	velSpeed=40.0/3102.505*lineDistance+35.7;
	ShooterVelCtrl(velSpeed);
}
/**
* @brief  圆形闭环主程序       
*/
#if  CARNUM == 1                      //旧车走形
void walkRound(void)
{
		static float buff1,buff2,pulse,distanceL,distanceR,rState;
		static float lastX,lastY,errState=0,errCount,buffstate,buffR;
		static int switchState=0,switchCount=0;
        distanceL=Get_Adc_Average(14,10)*(4400/4096);			//设置ADC1通道14,15，平均值获取次数为10
		distanceR=Get_Adc_Average(15,10)*(4400/4096);	
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)nowAngle);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetX());
		USART_OUT(UART4,(uint8_t*)"%d\r\t",(int)GetY());
		switch(switchState)
		{
		 case 0:
				if(distanceR<=100)                //检测激光判断顺时针还是逆时针圆
				{
					motorState=0;
					switchState=1;
				}
			    else if(distanceL<=100)
				{
					motorState=1;
					switchState=1;
				}
		 break;
		 case 1:                         
			if(motorState==0)                   //顺时针投球
		   {
		      switch(switchCount)
	         { 
			   case 0:
				 if(GetX()<=-600&&GetY()<=1600)
			    {
				   YawPos(-2200,0);
				   ShooterVel(-2200,0);
				   Pos();
		        }			   
			   if(GetY()>2200)
				   switchCount=1;
			   break;
			 case 1:
			    if(GetX()<=-600&&GetY()>=2800)
				{
				   YawPos(-2200,4400);
                   ShooterVel(-2200,4400);
                   Pos();					
			   }
			   if(GetX()>0)
				   switchCount=2;
			   break;
			 case 2:
				if(GetX()>=600&&GetY()>=2800)
				{					
				   YawPos(2200,4400);
                   ShooterVel(2200,4400);
                   Pos();					
		       }
			   if(GetY()<2200)
				   switchCount=3;
			   break;
			  case 3:
				if(GetX()>=600&&GetY()<=1600)
				{
				   YawPos(2200,0);
				   ShooterVel(2200,0);
					Pos();
				}					
				if(GetX()<0)
					switchCount=0;
			    break;
	          }
	       }
		   if(motorState==1)                    //逆时针投球
		   {
		      switch(switchCount)
	         { 
				 case 0:
					 if(GetX()>=600&&GetY()<=1600)
					 {
					    YawPos(2200,0);
					    ShooterVel(2200,0);
					    Pos();					 
					 }
					  if(GetY()>2200)
					  switchCount=1;
			     break;
				 case 1:
					if(GetX()>=600&&GetY()>=2800)
					{					
					   YawPos(2200,4400);
                       ShooterVel(2200,4400);
                       Pos();						
				   }
				   if(GetX()<0)
					  switchCount=2;
			    break;
				case 2:
					if(GetX()<=-600&&GetY()>=2800)
					{
					   YawPos(-2200,4400);
                       ShooterVel(-2200,4400);
                       Pos();						
				    }
				   if(GetY()<2200)
					   switchCount=3;
				   break;
			   case 3:
				   if(GetX()<=-600&&GetY()<=1600)
			      {
					   YawPos(-2200,0);
					   ShooterVel(-2200,0);
					   Pos();
		          }			   
			      if(GetX()>0)
				   switchCount=0;
			   break;
	         }
	       }
			if(fabs(lastX-GetX())<2&&fabs(lastY-GetY())<2)  //检测小车卡住
			{
				errCount++;
				if(errCount>=100)
				{
					errState=1;
					errCount=0;
				}
			}
			lastX=(int)GetX();
			lastY=(int)GetY();
			if(R>=2000)              //判断何时半径增大，半径减小
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
		   if(errState>=1)          //遇到卡住先后退1s
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
		   if(buffstate>=1)      //后退1s后再以半径减小或者增大的圆走1.8s
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
#elif CARNUM == 2                  //新车走形
void walkRound(void)
{
		static float pulse1,pulse2,rState;
		static float lastX,lastY,errState=0,errCount,buffstate,buffR;
        nowX=GetX()-OPS_TO_BACK_WHEEL*cos(nowAngle*PI/180);
	    nowY=GetY()-OPS_TO_BACK_WHEEL*sin(nowAngle*PI/180)+OPS_TO_BACK_WHEEL;
		USART_OUT(USART1,(uint8_t*)"%d\t",(int)nowAngle);
		USART_OUT(USART1,(uint8_t*)"%d\t",(int)nowX);
		USART_OUT(USART1,(uint8_t*)"%d\t",(int)nowY);
		USART_OUT(USART1,(uint8_t*)"%d\t",(int)u2);
		USART_OUT(USART1,(uint8_t*)"%d\r\n",(int)u1);
		if(fabs(lastX-nowX)<2&&fabs(lastY-nowY)<2)        //检测小车卡住
		{
			errCount++;
			if(errCount>=100)
			{
				errState=1;
				errCount=0;
			}
		}
		lastX=(int)nowX;
		lastY=(int)nowY;
		if(R>=2000)                              //判断何时半径增大，半径减小
			rState=0;
		if(R<=500)
			rState=1;
		if(rState==0)
		{
			if(nowAngle>=170&&nowAngle<=180&&nowX<=10&&nowX>=2)
				R-=250;
		}
		if(rState==1)
		{
			if(nowAngle>=170&&nowAngle<=180&&nowX<=10&&nowX>=2)
				R+=250;
		}
		uPlace();
		uAng();
		if(errState==0)
		{
		    pulse1=8192*REDUCTION_RATIO*speed/(WHEEL_DIAMETER*PI);  //mm/s转化为脉冲/s
			pulse2=8192*REDUCTION_RATIO*speed*TURN_AROUND_WHEEL_TO_BACK_WHEEL/R/(TURN_AROUND_WHEEL_DIAMETER*PI);
		    VelCrl(CAN2,5,pulse1);
		    VelCrl(CAN2,6,pulse2+u1);
		}
		  if(errState>=1)                              //遇到卡住先后退1s
		   {
			   VelCrl(CAN2,5,-130000);
			   VelCrl(CAN2,6,0);
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
		   if(buffstate>=1)                             //后退1s后再以半径减小或者增大的圆走1.8s
		  {
			   buffstate++;
			   if(buffstate>=180)
			   {
				   buffstate=0;
				   R=buffR;
			   }
		   } 		  
}
#endif


