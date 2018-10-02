#include <includes.h>
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
#include "adc.h"
#include "pps.h"
#include "fort.h"
/*
===============================================================
						信号量定义
===============================================================
*/

OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
void App_Task()
{	
	CPU_INT08U os_err;
	os_err = os_err; /*防止警告...*/
	/*创建信号量*/
	PeriodSem = OSSemCreate(0);
	/*创建任务*/
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, /*初始化任务*/
						  (void *)0,
						  (OS_STK *)&App_ConfigStk[Config_TASK_START_STK_SIZE - 1],
						  (INT8U)Config_TASK_START_PRIO);
	os_err = OSTaskCreate((void (*)(void *))WalkTask,
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  (INT8U)Walk_TASK_PRIO);				
	OSTaskSuspend(OS_PRIO_SELF);
}
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	delay_s(3);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM_Init(TIM2,999,83,1,3);
	KeyInit();
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	USART3_Init(115200);
	UART4_Init(921600);
	UART5_Init(921600);
	delay_s(2);
	ElmoInit(CAN1);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,COLLECT_BALL1_ID,500000,500000);
	VelLoopCfg(CAN2,COLLECT_BALL2_ID,500000,500000);
	PosLoopCfg(CAN2,PUSH_BALL_ID,5000000,5000000,10000000);
	VelLoopCfg(CAN1,1,50000000,50000000);
	VelLoopCfg(CAN1,2,50000000,50000000);
	MotorOn(CAN2,COLLECT_BALL1_ID);
	MotorOn(CAN2,COLLECT_BALL2_ID);
	MotorOn(CAN1,1);
	MotorOn(CAN1,2);
	delay_s(2);
	WaitOpsPrepare();
	OSTaskSuspend(OS_PRIO_SELF);
}
float yawAngle=0,T=0.03,v=1500,angle,Distance,antiRad,realR,shootX,shootY,futureX,futureY,lastX=0,lastY=0,x,y,LastCount;
int status=1,throwFlag=0,R=600,bingoFlag[4][2]={0},haveShootedFlag=0,errTime=0,StdId,laserModel=0,changeLightTime=0,RchangeFlag,FindBallModel=0,lastTime=0,time=0;
float location[4][2]={{2200,200},{2200,4600},{-2200,4600},{-2200,200}};
extern float Vk,Kp,lAngle;
extern int ballColor,backwardCount;
extern Msg_t frontwheelspeedBuffer;
void WalkTask(void)
{
	static int PE1=0,PE0=0,cycleCnt=0,staticShootFlag_1=0,staticShootFlag_2=0,staticShootFlag_3=0,E=1,push_Ball_Count=0,noPushCnt=0,noPushFlag=1;
	static float staticShootAngle_1=0,timeAngle=0,DistanceA=0,DistanceB=0,lightAngle=0,staticShootAngle_2=0;
	static int errFlag=0,pushBallCount=0,statusFlag,count=0,semiPushCount=0,noBallCount=0,pushBallFlag=1,staticPushBallFlag=1,Cnt=0,laserModelCount,FindBallModelCount,banFirstShoot=150,realCount=0;
	static float dLeft=0,dRight=0,changeAngle,rps=50,Vx,Vy,V,shootAngle;
	do
	{
		PE0=GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0);
		VelCrl(CAN2,COLLECT_BALL1_ID,-100*32768); 
		VelCrl(CAN2,COLLECT_BALL2_ID,100*32768);	
	}
	while(PE0==1);
	VelCrl(CAN2,COLLECT_BALL1_ID,80*32768); 
	VelCrl(CAN2,COLLECT_BALL2_ID,-80*32768);	
	do
	{		
		//炮台激光
		dLeft=ReadLaserAValue()*2.48+24.8;
		dRight=ReadLaserBValue()*2.48+24.8;	
	}
	while((dLeft>1500&&dRight>1500)||dLeft==0||dRight==0);
	//战术选择
	if(dRight<=1500)
	{	status=0;
//		PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--semiPushCount)*PUSH_POSITION/2+count*PUSH_POSITION);
		if(dRight>100&&dRight<500)
			R=600;
		if(dRight>500&&dRight<1000)
			R=1100;
	}	
	else 
	{
//		PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--semiPushCount)*PUSH_POSITION/2+count*PUSH_POSITION);
		status=1;
		if(dLeft>100&&dLeft<500)
			R=600;
		if(dLeft>500&&dLeft<1000)
			R=1100;
	}
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem,0,&os_err);
		x=GetX();
		y=GetY();
		if(laserModel)
		{
			Kp=75;
			Cnt++;
			CirclePID(0,2400,R,1000,status);
			if(Cnt<=300)
				Rchange(-750);
			else if(Cnt<=600)
				Rchange(-750);
			else if(Cnt<650)
			{ 
				VelCrl(CAN1,1,0);				
				VelCrl(CAN1,2,0);
			}	
			else if(Cnt<700)
			{
				MotorOff(CAN1,1);
				MotorOff(CAN1,2);
				StdId=FirstshootJudge();
			}	
			else 
			{
				if(ReadLaserAValue()>=25)
				DistanceA=ReadLaserAValue()*2.480+24.8;
				if(ReadLaserBValue()>=25)
				DistanceB=ReadLaserBValue()*2.480+24.8;
				if(staticShootFlag_3!=1)
				{
					cycleCnt=(StdId+1)/4;
					lightAngle=getLingtAngle(GetX(),GetY(),(StdId+1)%4)-cycleCnt*360;
					YawPosCtrl(lightAngle-timeAngle);
					if(timeAngle>15&&E==1)
					{
						E=-1;
					}
					if(timeAngle<-15&&E==-1)
					{
						E=1;
					}
					timeAngle+=E*0.07;
				}

				if(changeLightTime>800)
				{
					if(DistanceA>4500&&DistanceB<4500&&fabs(DistanceA-DistanceB)>2000)
					{
						staticShootAngle_1=lightAngle-timeAngle;
						staticShootFlag_1=1;
					}
					if(DistanceB>4500&&DistanceA<4500&&fabs(DistanceA-DistanceB)>2000)
					{
						staticShootAngle_2=lightAngle-timeAngle;
						staticShootFlag_2=1;
					}
					if(fabs(lightAngle-(staticShootAngle_1+staticShootAngle_2)/2)<=5&&staticShootFlag_1==1&&staticShootFlag_2==1)
					{
						staticShootFlag_3=1;
						YawPosCtrl((staticShootAngle_1+staticShootAngle_2)/2);
					}
				}
				if(staticShootFlag_3==1)
				{
						push_Ball_Count++;
						if(fabs(DistanceA-DistanceB)<300)
						{
							rps=40/3102.505*(DistanceA+DistanceB)/2+35.7;
							if(rps>85)
								rps=85;
							ShooterVelCtrl(rps);
						}
						if(ballColor==0)
						{
							staticPushBallFlag=1;
						}
						if(push_Ball_Count%100==0)
						{
							noPushFlag=0;
							if(ballColor==1&&staticPushBallFlag==1)
							{
								semiPushCount+=1;
								bingoFlag[StdId][0]+=5;
								noPushFlag=1;
								staticPushBallFlag=0;
								push_Ball_Count=410;
							}
							if(ballColor==2&&staticPushBallFlag==1)
							{
								semiPushCount-=1;
								noPushFlag=1;
								staticPushBallFlag=0;
							}
							if(noPushFlag==0)
								noPushCnt++;
							if(noPushCnt>=4)
							{
								semiPushCount-=1;
								noPushCnt=0;
							}
							PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,semiPushCount*PUSH_POSITION/2+count*PUSH_POSITION);
						}
						if(push_Ball_Count>=490)
						{
//							PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,(--count)*PUSH_POSITION);
							push_Ball_Count=0;
							staticShootFlag_1=0;
							staticShootFlag_2=0;
							staticShootFlag_3=0;
							StdId++;
							changeLightTime=0;
						}
						USART_OUT(UART4,(uint8_t *)"%d\t%d\t%d\t%d\t%d\t%d\t\r\n",(int)staticPushBallFlag,(int)ballColor,noPushFlag,(int)noPushCnt,(int)LastCount,count);
					}
				}
			} //cnt>250终点	
		else if(FindBallModel)
		{
			Kp=40;
			IncreaseR(1950);
			laserModelCount++;
			if(R>1950&&x>0&&lastX<0)
			{if(status==0)
			{
				if(x>1400&&y<3800)
					linePID(2000,400,2000,4400,1000);
				if(y>3800&&x>-1400)
					linePID(2000,4400,-2000,4400,1000);
				if(x<-1400&&y>1000)
					linePID(-2000,4400,-2000,400,1000);
				if(y<1000&&x<1400)
					linePID(-2000,400,2000,400,1000);
			}	
			else
			{
				if(x>1400&&y>1000)
					linePID(2000,4400,2000,400,1000);
				if(y>3800&&x<1400)
					linePID(-2000,4400,2000,4400,1000);
				if(x<-1400&&y<3800)
					linePID(-2000,400,-2000,4400,1000);
				if(y<1000&&x>-1400)
					linePID(2000,400,-2000,400,1000);
			}	
			if(laserModelCount>3000)
			{
				FindBallModel=0;
				laserModel=1;
			}
			}
		}			
		else{
		if(v==1600)
			Kp=100;
		if(v==2000)
			Kp=110;
		angle=GetAngle()+90;
		realR=sqrtf(x*x+(y-2400)*(y-2400));
		antiRad=T*sqrtf(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY())/realR;
		if(errFlag==1)
		{	
			Kp=40;
			GetFunction(x,y,0,2400);
			if(R<=1100)
				changeAngle=lAngle;
			else
				changeAngle=lAngle+180;
			AnglePID(changeAngle,GetAngle());
			Straight(-1500);
			if(errTime%2==0&&statusFlag)
			{
				status=1-status;
				statusFlag=0;
				PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--semiPushCount)*PUSH_POSITION/2+count*PUSH_POSITION);
			}	
			if(Get_Time_Flag())	
				errFlag=0;
			time=0;
			lastTime=0;
			throwFlag=0;
		}	
		else
		{
			time++;
			CirclePID(0,2400,R,v,status);
			IncreaseR(1600);
			if((x-lastX)*(x-lastX)+(y-lastY)*(y-lastY)>80)
				lastTime=time;
			if(time-lastTime>=80)
			{	
				errFlag=1;
				statusFlag=1;
				errTime++;
				backwardCount=0;
			}	
			if(status==0)
			{
				//预判Ts后的坐标（适用于走圆形）
				futureX=realR*cos((angle-90)*pi/180+antiRad);
				futureY=realR*sin((angle-90)*pi/180+antiRad)+2400;
				//预判Ts后炮台坐标
				shootX=futureX-68*sin(GetAngle()*pi/180+antiRad);
				shootY=futureY+68*cos(GetAngle()*pi/180+antiRad);
				if(shootX<=800&&shootY<1600)
				{	
					if(shootX>-600&&shootX<600)
						throwFlag=1;
					else
						throwFlag=0;
					GetShootSituation(0);
				}
				if(shootY<=3200&&shootX>=800)
				{	
					if(shootY<3000&&shootY>1800)
						throwFlag=1;
					else
						throwFlag=0;
					GetShootSituation(1);
				}	
				if(shootX>-800&&shootY>=3200)
				{	
					if(shootX>-600&&shootX<600)
						throwFlag=1;
					else
						throwFlag=0;
					GetShootSituation(2);
				}			
				if(shootY>1600&&shootX<-800)
				{	
					if(shootY<3000&&shootY>1800)
						throwFlag=1;
					else
						throwFlag=0;
					GetShootSituation(3);		
				}
			}
			else
			{
				//Ts后定位系统坐标
				futureX=realR*cos((angle+90)*pi/180-antiRad);
				futureY=realR*sin((angle+90)*pi/180-antiRad)+2400;
				//预判Ts后炮台坐标
				shootX=futureX-68*sin(GetAngle()*pi/180-antiRad);
				shootY=futureY+68*cos(GetAngle()*pi/180-antiRad);
				if(shootX>=800&&shootY>1600)
				{
					if(shootY<3000&&shootY>1800)
						throwFlag=1;
					else
						throwFlag=0;
					GetShootSituation(0);					
				}	
				if(shootX<=800&&shootY>=3200)
				{
					if(shootX>-600&&shootX<600)
						throwFlag=1;
					else
						throwFlag=0;
					GetShootSituation(1);				
				}	
				if(shootX<-800&&shootY<=3200)
				{	
					if(shootY<3000&&shootY>1800)
						throwFlag=1;
					else
						throwFlag=0;
					GetShootSituation(2);						
				}
				if(shootX>-800&&shootY<1600)
				{	
					if(shootX>-600&&shootX<600)
						throwFlag=1;
					else
						throwFlag=0;
					GetShootSituation(3);
				}
			}
		}	
		
		//圆筒方向速度
		Vx=cos(yawAngle*pi/180)*sqrtf(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY());
		//圆筒垂直方向速度
		Vy=sin(yawAngle*pi/180)*sqrtf(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY());
		//应给的速度
		V=-Vx+Distance*9800/(sqrtf(4*4900*(sqrt(3)*Distance-650)+3*Vx*Vx)-sqrt(3)*Vx);
		shootAngle=yawAngle+atan(Vy/V)*180/pi;
		YawPosCtrl(shootAngle);
		rps=(sqrtf(V*V+Vy*Vy)-166.59)/39.574+(Distance-3000)*0.003;
		ShooterVelCtrl(rps);
		if(ballColor==0)
			noBallCount++;
		if(noBallCount%25==0)	
			pushBallFlag=1;
		//一边球仓无球一段时间后后换仓
		if(noBallCount>=500)
		{	
			PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--semiPushCount)*PUSH_POSITION/2+count*PUSH_POSITION);
			noBallCount=0;
		}	
		if(R>=1600)
		{	
			if(throwFlag&&ballColor==MY_BALL_COLOR&&pushBallFlag)
			{
				//当刚切换到1600半径稳定1.5s
				if(--banFirstShoot>0)
					goto label;	
				PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,semiPushCount*PUSH_POSITION/2+(++count)*PUSH_POSITION);				
				pushBallFlag=0;
				haveShootedFlag=1;
				noBallCount=1;
			}		
			if(pushBallCount%100==0)
			{	
				//当拿到对方的球，倒转180度	
				if(ballColor!=MY_BALL_COLOR&&ballColor!=0&&pushBallFlag)
				{
					PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,semiPushCount*PUSH_POSITION/2+(--count)*PUSH_POSITION);
					pushBallFlag=0;
					noBallCount=1;
				}
			}	
		}
		label:pushBallCount++;
		lastX=x;
		lastY=y;
		if((bingoFlag[0][1]!=0&&bingoFlag[1][1]!=0&&bingoFlag[2][1]!=0&&bingoFlag[3][1]!=0)||errTime>4||pushBallCount>4000)
		{
			laserModelCount++;
			if(laserModelCount>=100)
				laserModel=1;
		}	
		if(semiPushCount<-5)
			FindBallModel=1;
		ReadActualVel(CAN1,2);
		USART_OUT(UART4,(uint8_t *)"%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)GetX(),(int)GetY(),ballColor,count,throwFlag,(int)(frontwheelspeedBuffer.data32[1]/(CAR_WHEEL_COUNTS_PER_ROUND*REDUCTION_RATIO*WHEEL_REDUCTION_RATIO)*(pi*TURN_AROUND_WHEEL_DIAMETER)));
		}
	}	
}
