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
#include "pps.h"
#include "fort.h"
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

float Kp=85,Ki=0,Kd=0,err=0,lastErr=0,Sumi=0,Vk=0,errl,lastErr1;
extern int R;

void Straight(float v)
{
	VelCrl(CAN1,1,-v*CAR_WHEEL_COUNTS_PER_ROUND*REDUCTION_RATIO*WHEEL_REDUCTION_RATIO/(pi*WHEEL_DIAMETER));
	VelCrl(CAN1,2,-Vk*CAR_WHEEL_COUNTS_PER_ROUND*REDUCTION_RATIO*WHEEL_REDUCTION_RATIO/(pi*TURN_AROUND_WHEEL_DIAMETER));
}	
void Spin(float R,float v)
{
	VelCrl(CAN1,1,-v*CAR_WHEEL_COUNTS_PER_ROUND*REDUCTION_RATIO*WHEEL_REDUCTION_RATIO/(pi*WHEEL_DIAMETER));
	VelCrl(CAN1,2,v*(TURN_AROUND_WHEEL_TO_BACK_WHEEL/R)*CAR_WHEEL_COUNTS_PER_ROUND*REDUCTION_RATIO*WHEEL_REDUCTION_RATIO/(pi*TURN_AROUND_WHEEL_DIAMETER));
}
void TurnRight(float angle,float v)
{
	VelCrl(CAN2,1,0);
	VelCrl(CAN2,2,-v*4096*WHEEL_TREAD*angle*2/((WHEEL_DIAMETER)*360));
}	
void AnglePID(float setAngle,float feedbackAngle)
{
	err=setAngle-feedbackAngle;
	if(err>180)
		err=-(360-err);
	if(err<-180)
		err=360+err;
	Sumi+=Ki*err;
	Vk=Kp*err+Sumi+Kd*(err-lastErr);
	if(Vk>1200)
		Vk=1200;
	if(Vk<-1200)
		Vk=-1200;
	lastErr=err;																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																									
}	
float k,b,lAngle,setAngle,d;
int flag;
extern float yawAngle,x,y,lastX,lastY;
extern int status;
float min(float d1,float d2)
{
	if(d1>d2)
		return d2;
	else
		return d1;
}
extern int scanCnt[10];
int findMostGroup()
{
	int a=0;
	for(int i=0;i<=9;i++)
	{
		if(scanCnt[a]<scanCnt[i])
			a=i;
	}
	return a;
}
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
			else 
			{
				if(y2-y1>=0)
					lAngle=(atan(k)*180/pi)+90;
				else 
					lAngle=(atan(k)*180/pi)-90;
			}	
			if(y1-0.5<y2&&y2<y1+0.5)
			{	
				if(x2-x1>0)
					lAngle=-90;
				if(x2<x1)
					lAngle=90;
			}	
			flag=1;
		}
}	
void linePID(float x1,float y1,float x2,float y2,float v)
{
		
		GetFunction(x1,y1,x2,y2);
		if(flag)
		{
			if(k>0)
				if((y-k*x-b)*(y2-y1)>0)
					setAngle=lAngle-90*(1-1000/(y-k*x-b+1000));
				else
					setAngle=lAngle+90*(1-1000/(-y+k*x+b+1000));
			else
				if((y-k*x-b)*(y2-y1)>0)
					setAngle=lAngle+90*(1-1000/(y-k*x-b+1000));
				else
					setAngle=lAngle-90*(1-1000/(-y+k*x+b+1000));
			//斜率为0	
			if(y1-0.5<y2&&y2<y1+0.5)
			{	
				if(x2>x1)
					if(y>y1)
						setAngle=lAngle-90*(1-1000/(y-b+1000));
					else
						setAngle=lAngle+90*(1-1000/(-y+b+1000));
				else
					if(y>y1)
						setAngle=lAngle+90*(1-1000/(y-b+1000));
					else
						setAngle=lAngle-90*(1-1000/(-y+b+1000));	
			}	
		}	
		//斜率不存在
	 	else
		{	
			if((y2-y1)*(x-x1)<0)
					setAngle=lAngle-90*(1-1000/(fabs(x-x1)+1000));
			else
					setAngle=lAngle+90*(1-1000/(fabs(x-x1)+1000));	
		}
		AnglePID(setAngle,GetAngle());
		Straight(v);
		
}	
void CirclePID(float x0,float y0,float R,float v,int status)
{
	x=GetX();
	y=GetY();
	GetFunction(x,y,x0,y0);
	d=(x-x0)*(x-x0)+(y-y0)*(y-y0);
	//逆时针
	if(status==0)
	{	
		if(sqrtf(d)>R)
			setAngle=lAngle-90*(600/(fabs(sqrtf(d)-R)+600));	
		else
			setAngle=lAngle-180+90*(600/(fabs(sqrtf(d)-R)+600));		
	}	 
	//顺时针
	if(status==1)
	{	
		if(sqrtf(d)>R)
			setAngle=lAngle+90*(600/(fabs(sqrtf(d)-R)+600));
		else
			setAngle=lAngle+180-90*(600/(fabs(sqrtf(d)-R)+600));
	}	
	AnglePID(setAngle,GetAngle());
	Straight(v);
}	
/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
extern float Distance,shootX,shootY,angle,antiRad,location[4][2],speed;
extern int bingoFlag[4][2],haveShootedFlag,errTime,throwFlag,RchangeFlag,shakeShootFlag,banFirstShoot,FindBallModel,StdId;
void GetYawangle(uint8_t StdId)
{
	if(shakeShootFlag||FindBallModel)
	{	antiRad=0;
		shootX=x;
		shootY=y;
	}	
	if(status==0)
	{	
		GetFunction(shootX,shootY,location[StdId][0],location[StdId][1]);
		if((GetAngle()+antiRad*180/pi)<-90&&lAngle>90)
			yawAngle=360+(GetAngle()+antiRad*180/pi)-lAngle;
		else
			yawAngle=(GetAngle()+antiRad*180/pi)-lAngle;		
	}
	else
	{
		GetFunction(shootX,shootY,location[StdId][0],location[StdId][1]);
		if(lAngle<-90&&(GetAngle()-antiRad*180/pi)>90)
			yawAngle=-360+(GetAngle()-antiRad*180/pi)-lAngle;
		else
			yawAngle=(GetAngle()-antiRad*180/pi)-lAngle;	
	}	
}
void GetDistance(uint8_t StdId)
{
	Distance=sqrtf((shootX-location[StdId][0])*(shootX-location[StdId][0])+(shootY-location[StdId][1])*(shootY-location[StdId][1]));
}	
void GetDistance1(uint8_t StdId)
{
	Distance=sqrtf((x-location[StdId][0])*(x-location[StdId][0])+(y-location[StdId][1])*(y-location[StdId][1]));
}	
void BingoJudge(uint8_t StdId)
{
	if(haveShootedFlag==1&&bingoFlag[StdId][0]==0)
	{	
		bingoFlag[StdId][0]=3-errTime;
		haveShootedFlag=0;
	}	
	else if(haveShootedFlag==1&&bingoFlag[StdId][1]==0)
	{	
		bingoFlag[StdId][1]=3-errTime;
		haveShootedFlag=0;
	}					
	if(bingoFlag[StdId][1]!=0)
		throwFlag=0;
	else if(bingoFlag[StdId][0]!=0)
	{
		for(uint8_t i=0;i<4;i++)
		{
			if(bingoFlag[i][0]==0&&i!=StdId)
			{
				throwFlag=0;
				break;
			}	
		}
	}
	if(FindBallModel==0)
	{
		if(speed<1300||speed>1800||(!FindBallModel&&errTime>1))
		{	
			throwFlag=0;
			banFirstShoot=100;
		}	
		GetFunction(x,y,0,2400);
		if(((status==0&&fabs(GetAngle()-lAngle+90)>12)||(status==0&&fabs(GetAngle()-lAngle+90)>12)))
		{
			throwFlag=0;
			banFirstShoot=100;
		}	
	}	
}

void GetShootSituation(uint8_t StdId)
{
	GetYawangle(StdId);
	GetDistance(StdId);
	BingoJudge(StdId);
}	
int FirstshootJudge(void)
{
	int StdId=0,maxPriority=0,i=0;
	maxPriority=bingoFlag[0][0];
	for(i=0;i<=3;i++)
	{
		if(bingoFlag[i][0]<bingoFlag[0][0])
		{
			maxPriority=bingoFlag[i][0];
			StdId=i;
		}	
	}	
	return StdId;
}	
int RchangeTime=250; 
void Rchange(int Rchange)
{
	if(--RchangeTime>=0)
		R+=Rchange/250;
	else
	{
		RchangeFlag=0;
		RchangeTime=250;
	}
}
void IncreaseR(int Radium)
{	
	if(status==0)
	{
		if(x<-100&&lastX>-100&&y>2400&&R<Radium)
			RchangeFlag=1;
		if(R>=Radium)
		{
			RchangeFlag=0;
			RchangeTime=250;
		}	
	}	
	else
	{
		if(x>100&&lastX<100&&y>2400&&R<Radium)
			RchangeFlag=1;
		if(R>=Radium)
		{
			RchangeFlag=0;
			RchangeTime=250;
		}	
	}		
	if(RchangeFlag)
		Rchange(500);	
}	
void DecreaseR(int Radium)
{	
	if(status==0)
	{
		if(x<-100&&lastX>-100&&y>2400&&R>Radium)
			RchangeFlag=1;
		if(R<=Radium)
		{
			RchangeFlag=0;
			RchangeTime=250;
		}			
	}	
	else
	{
		if(x>100&&lastX<100&&y>2400&&R>Radium)
			RchangeFlag=1;
		if(R<=Radium)
		{
			RchangeFlag=0;
			RchangeTime=250;
		}	
	}		
	if(RchangeFlag)
		Rchange(-500);	
}	
extern int errFlag,count,shootCnt,shakeShootCnt,ballColor,rDecreaseFlag,circleCnt,semiPushCount,pushBallFlag;
extern float angle,speed,speedY,speedX,T0,T1,rps,realR,v;
int errSituation1,errSituation2,shakeShootOff=200,shutOffCnt=0;
void Avoidance()
{
		static int errSituation3,statusFlag,lastTime=0,time=0,backwardCount;
		static float changeAngle,speedAngle;
		if(errFlag==1)
		{	
			backwardCount++;
			if(errSituation1)
			{
				Kp=10;
				AnglePID(changeAngle,GetAngle());
				switch(circleCnt)
				{
					case 1: 
						R=1100;
						break;
					case 2: 
						R=1600;
						break;
					case 3:
						R=1800;
					break;
				}	
				Straight(-1100);
				if(errTime%2==0&&statusFlag)
				{
					status=1-status;
					statusFlag=0;
				}	
			}
			if(errSituation2)
			{
				AnglePID(changeAngle,GetAngle());
				Straight(1200);
			}	
			if(backwardCount>=100)	
			{
				errFlag=0;
				backwardCount=0;
				if(R>=1600)
					banFirstShoot=50;
			}	
			time=0;
			lastTime=0;
			throwFlag=0;
		}	
		else
		{
			time++;
			GetFunction(lastX,lastY,x,y);
			speedAngle=lAngle;
			//与对手相持
			if(fabs(speedAngle-GetAngle())<40&&speed>1200)
				lastTime=time;
			else				
			{
				//情况2：被对方侧面推着跑，此时有一定速度，车身角度与速度角度不一致
				if(fabs(speedAngle-GetAngle())>40&&speed>500)
				{
					errSituation2=1;
					errSituation1=0;
					changeAngle=speedAngle;
				}	
				//情况1：与对方正面相撞
				else if(speed<1000)
				{	
					errSituation1=1;
					errSituation2=0;
					GetFunction(x,y,0,2400);
					if(R<=1100)
						changeAngle=lAngle;
					else
						changeAngle=lAngle+180;
				}
				if(FindBallModel)
				{
					errSituation1=1;
					errSituation2=0;
//					rDecreaseFlag=1;
				}	
			}				
			if(time-lastTime>=150)
			{	
				errFlag=1;
				statusFlag=1;
				errTime++;
			}		
		}	
}	
/*凯化*/
void ShakeShoot(void)
{
	VelCrl(CAN1,1,0);				
	VelCrl(CAN1,2,0);
	if(status==0)
	{
		if(x>-400&&x<400&&y<2400)
			GetShootSituation(0);
		if(y<2800&&y>2000&&x>0)
			GetShootSituation(1);
		if(x>-400&&x<400&&y>2400)
			GetShootSituation(2);			
		if(y<2800&&y>2000&&x<0)
			GetShootSituation(3);		
	}
	else
	{
		if(x>-400&&x<400&&y<2400)
			GetShootSituation(3);
		if(y<2800&&y>2000&&x>0)
			GetShootSituation(0);
		if(x>-400&&x<400&&y>2400)
			GetShootSituation(1);			
		if(y<2800&&y>2000&&x<0)
			GetShootSituation(2);		
	}				
	shutOffCnt++;
	if(shakeShootCnt>0)
		shakeShootOff--;
	if(shakeShootOff==0||(ballColor!=MY_BALL_COLOR&&shakeShootCnt==0))
	{
		banFirstShoot=200;
		errFlag=0;
		shakeShootCnt=0;
		shakeShootFlag=0;
		shakeShootOff=100;
	}	
	else if(fabs(rps-ReadRps())<1)
		throwFlag=1;
	else
		throwFlag=0;
}	
void BorderSweeping(void)
{
	if(status==0)
	{
		if(x>1100&&y<3500)
		{	linePID(2100,0,2100,100,v);
			StdId=1;
		}	
		if(y>3500&&x>-1100)
		{	linePID(2200,4500,0,4500,v);
			StdId=2;
		}	
		if(x<-1100&&y>3500)
		{	linePID(-2100,100,-2100,0,v);
			StdId=3;
		}	
		if(y<1300&&x<1100)
		{	linePID(-2200,300,0,300,v);
			StdId=0;
		}	
	}	
	else
	{
		if(x>1100&&y>1300)
		{	linePID(2100,100,2100,0,v);
			StdId=0;
		}	
		if(y<1300&&x>-1100)
		{	linePID(2200,300,0,300,v);
			StdId=3;
		}
		if(x<-1100&&y<3500)
		{	linePID(-2100,-100,-2100,0,v);
			StdId=2;
		}	
		if(y>3500&&x<1100)
		{	linePID(-2200,4500,0,4500,v);
			StdId=1;
		}	
		if((x>1000&&x<-1000)||(y>3400&&y<1400))
			throwFlag=1;
		GetShootSituation(StdId);
		YawPosCtrl(yawAngle);
		rps=((Distance*9800/(sqrtf(4*4900*(sqrt(3)*Distance-650)+3*speed*speed)-sqrt(3)*speed)-speed)-166.59)/39.574+(Distance-3500)*0.0045;
		ShooterVelCtrl(rps);
	}	
}	
/*激光模式*/
float getLingtAngle(float xi,float yi,int tragetCnt)
{
	static float angii=0;	
	if(tragetCnt==0)
		angii=GetAngle()+90-180/pi*atan((0-yi)/(2400-xi));
	if(tragetCnt==1)
		angii=GetAngle()+90-180/pi*atan((4800-yi)/(2400-xi));
	if(tragetCnt==2)
		angii=GetAngle()-90-180/pi*atan((4800-yi)/(-2400-xi));
	if(tragetCnt==3)
		angii=GetAngle()-90-180/pi*atan((0-yi)/(-2400-xi));
	return angii;
}