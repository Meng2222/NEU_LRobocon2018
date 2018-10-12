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
int PE0=0;
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	delay_s(2);
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
	do
	{
		PE0=GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0);
		VelCrl(CAN2,COLLECT_BALL1_ID,-80*32768); 
		VelCrl(CAN2,COLLECT_BALL2_ID,80*32768);	
	}
	while(PE0==1);
	PosLoopCfg(CAN2,PUSH_BALL_ID,5000000,5000000,10000000);
	VelLoopCfg(CAN1,1,50000000,50000000);
	VelLoopCfg(CAN1,2,50000000,50000000);
	MotorOn(CAN2,COLLECT_BALL1_ID);
	MotorOn(CAN2,COLLECT_BALL2_ID);
	MotorOn(CAN1,1);
	MotorOn(CAN1,2);
	YawPosCtrl(90);
	delay_s(2);
	WaitOpsPrepare();
	OSTaskSuspend(OS_PRIO_SELF);
}
float yawAngle=0,T1=0,T0=0,v=1500,angle,Distance,antiRad,realR=0,shootX,shootY,futureX,futureY,lastX=0,lastY=0,x,y,LastCount;
int pushBallFlag=1,borderSweepFlag=0,count=0,shakeShootFlag=0,shakeShootCnt=0,errFlag=0,status=1,semiPushCount=0,throwFlag=0,R=500,bingoFlag[4][2]={0},haveShootedFlag=0,errTime=0,StdId,laserModel=0,changeLightTime=0,RchangeFlag,rDecreaseFlag=0,FindBallModel=0,banFirstShoot=0,shootCnt=0,circleCnt=1;
float location[4][2]={{2200,200},{2200,4600},{-2200,4600},{-2200,200}},speedX,speedY,speed,rps=50;
extern float Vk,Kp,lAngle,CollectRightVel,CollectLeftVel;
int scanCnt[20]={0},touchLaserTime,lastTouchLaserTime,quickLaserModel=0;
float scanAngle[20][300]={0};
float scanDistance[20][300]={0};
extern int ballColor,backwardCount,errSituation1,errSituation2;
extern Msg_t frontwheelspeedBuffer;
extern Msg_t collectball1speedBuffer;
extern Msg_t collectball2speedBuffer;
extern Msg_t pushballmotorPosBuffer;
void WalkTask(void)
{
	static int PE1=0,cycleCnt=0,staticShootFlag_1=0,staticShootFlag_2=0,staticShootFlag_3=0,E=1,push_Ball_Count=0,noPushCnt=0,noPushFlag=1,notFoundcnt=0,stuckTime=0;
	static int foundRange=15,collectBallSpeed1=0,collectBallSpeed2=0,lastCollectBallSpeed1=0,lastCollectBallSpeed2=0,pushBallMotorPos=0;
	static float D1=0,D2=0,distance,lastDistanceA=0,lastDistanceB=0,realAngle=0,realDistance=0,deflectAngle=0;
	static int LastStdId=0,squareNode=0,noBallPushCnt=0,findBallCnt=0;
	static int waitCnt=0,goalCnt=0,scanFlag_1=0,scanFlag_2=0,myCnt=0,notCnt=0,goalFlag=0,mostGroup=0,OnlyFlag=1,scanErrCnt=0,scanErrFlag=0,scanFlag_Ok=0;
	static float staticShootAngle_1=0,timeAngle=16,DistanceA=0,DistanceB=0,lightAngle=0,staticShootAngle_2=0;
	static int pushBallCount=0,statusFlag=0,noBallCount=0,staticPushBallFlag=1,Cnt=0,laserModelCount=0,FindBallModelCount=0,realCount=0,lastSemiPushCount,lastCount,time=0;
	static float dLeft=0,dRight=0,Vx,Vy,V,shootAngle,realRps,lastRps;
//	USART_OUT(UART4,(uint8_t *)"%d",1);
	VelCrl(CAN2,COLLECT_BALL1_ID,80*32768); 
	VelCrl(CAN2,COLLECT_BALL2_ID,-80*32768);
	do
	{		
		//炮台激光
		dLeft=ReadLaserAValue()*2.48+24.8;
		dRight=ReadLaserBValue()*2.48+24.8;	
		if(dLeft>800&&dRight>800)
			lastTouchLaserTime=touchLaserTime;
		USART_OUT(UART4,(uint8_t *)"%d\t%d\r\n",(int)dLeft,(int)dRight);
	}
	while(touchLaserTime-lastTouchLaserTime<=100);
	PE0=GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0);
	if(PE0==1)
		R=1100;
	//战术选择
	if(dRight<=300)
	{
		status=0;
		if(PE0==1)
		{
			T0=0.095;
			T1=0.056;
			circleCnt+=1;
		}	
		else
		{
			T0=0.096;
			T1=0.046;
		}	
		PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--semiPushCount)*PUSH_POSITION/2+count*PUSH_POSITION);
	}	
	else if(dLeft<=300)
	{
		status=1;
		if(PE0==1)
		{	T0=0.086;
			T1=0.056;
			circleCnt+=1;
		}	
		else
		{
			T0=0.096;
			T1=0.046;
		}	
	}
	else if(dRight>300)
	{
		status=0;
		quickLaserModel=1;
	}	
	else
	{
		status=1;
		quickLaserModel=1;
	}	
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet (PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend (PeriodSem,0,&os_err);
		x = GetX();
		y = GetY();
		speedX = GetSpeedX();
		speedY = GetSpeedY();
		speed = sqrtf(speedX*speedX+speedY*speedY);
		realRps = ReadRps();
		realR=sqrtf(x*x+(y-2400)*(y-2400));
		angle=GetAngle()+90;
		collectBallSpeed1 = collectball1speedBuffer.data32[1]/32768;
		collectBallSpeed2 = collectball2speedBuffer.data32[1]/32768;
		pushBallMotorPos = pushballmotorPosBuffer.data32[1];
		if(v == 1800)
			Kp = 120;
		if(v == 1500)
			Kp = 85;
		if(laserModel)
		{
			Cnt++;
			v = 1500;	
			if(Cnt>1000&&realR>900)
				Cnt=0;
			if(Cnt<250)
			{	
				if(errFlag==0)
					CirclePID (0,2400,R,v,status);
				Avoidance();
			}	
			else if(Cnt<300)
			{ 
				VelCrl (CAN1,1,0);				
				VelCrl (CAN1,2,0);
				StdId = FirstshootJudge();	
			}	
			else
			{
				if(staticShootFlag_3 != 1)
				{
					//if(timeAngle<=25)
					deflectAngle=0;
					foundRange=foundRange-deflectAngle;
					if(LastStdId-StdId==3)
						cycleCnt++;
					if(LastStdId-StdId==-3)
						cycleCnt--;
					lightAngle = getLingtAngle(GetX(),GetY(),StdId)-cycleCnt*360;
					LastStdId=StdId;
					YawPosCtrl (lightAngle-timeAngle);
					if(timeAngle > -foundRange && fabs(ReadyawPos()-(lightAngle-timeAngle)) < 20)
						timeAngle -= 0.32;
					if(timeAngle <= -foundRange)
					{
						scanFlag_Ok=1;
//						USART_OUT(UART4,(uint8_t *)"scanFlag_OK\r\n");
					}
				}
				if(ReadLaserAValue() >= 100)
				DistanceA = ReadLaserAValue()*2.461+55.43;
				if(ReadLaserBValue() >= 100)
				DistanceB = ReadLaserBValue()*2.467+60.87;
				GetDistance1 (StdId);
				if(staticShootFlag_3 != 1)
				{
					if(scanFlag_Ok == 1)
					{
//						USART_OUT(UART4,(uint8_t *)"OK\r\n");
						scanFlag_Ok=0;
						scanFlag_1 = 0;
						scanFlag_2 = 0;//0.0145 38
						int t1 = 0,t2=0;
						mostGroup=findMostGroup();
						for(int i = 0;i <= 299;i++)
						{
							if(scanDistance[mostGroup][t1] < scanDistance[mostGroup][i])
								t1 = i;
						}
						for(int i = 0;i <= 299;i++)
						{
							if(scanDistance[mostGroup][i]!=0)
								t2 = i;
							else
								break;
						}
//						realAngle = (scanAngle[mostGroup][t2]+scanAngle[mostGroup][0])/2;
//						float angleBack=0;
//						if(scanDistance[mostGroup][t2]>scanDistance[mostGroup][0])
//						{
//							angleBack=0.2;
//						}
//						else
//							angleBack=1.6;
						if(rps>88)
							realAngle = (scanAngle[mostGroup][t2]+scanAngle[mostGroup][0])/2+1.6;
						else
							realAngle = (scanAngle[mostGroup][t2]+scanAngle[mostGroup][0])/2+1.6+0.6;
						realDistance = scanDistance[mostGroup][t1];
						if(realDistance != 0)
						{
							deflectAngle=(realAngle - lightAngle)/2;
							staticShootFlag_3 = 1;
							YawPosCtrl(realAngle);
						}	
						else
						{
							scanFlag_Ok = 0;
							if(foundRange<25)
								foundRange+=5;			
							timeAngle=foundRange;
						}
//						USART_OUT(UART4,(uint8_t *)"Angle=%d\tGroupCnt=%d\tCnt=%d\r\n",(int)realAngle,(int)mostGroup,(int)t);
//						staticShootFlag_3 = 1;
						goalCnt=0;
						for(int i = 0;i <= 9;i++)
						{ 
							scanCnt[i]= 0;
							for(int a = 0;a <= 299;a++)
							{
								scanAngle[i][a] = 0;
								scanDistance[i][a] = 0;
							}
						}
					}
					else
					{
//						USART_OUT(UART4,(uint8_t *)"GO1\r\n");
						if(fabs(ReadyawPos()-(lightAngle-timeAngle)) < 10 && staticShootFlag_3 != 1)
						{
//							USART_OUT(UART4,(uint8_t *)"GO2\r\n");
							if(fabs(DistanceA-DistanceB) < 200 && fabs(DistanceA-Distance)<800 && fabs(DistanceA-DistanceB)<800 && DistanceA < 4500 && DistanceB < 4500 && fabs(lastDistanceA-DistanceA) < 50 && fabs(lastDistanceB-DistanceB) < 50 && fabs(fabs(lastDistanceA-DistanceA)-fabs(lastDistanceB-DistanceB)) < 20)
							{
								goalFlag = 1;
								scanAngle[goalCnt][scanCnt[goalCnt]] = ReadyawPos();
								scanDistance[goalCnt][scanCnt[goalCnt]] = (DistanceA+DistanceB)/2;
//								USART_OUT(UART4,(uint8_t *)"%d\t%d\r\n",(int)scanAngle[goalCnt][scanCnt[goalCnt]],(int)scanDistance[goalCnt][scanCnt[goalCnt]]);
								if(scanCnt[goalCnt]<=298)
								scanCnt[goalCnt]++;
							}
							else
							{
								if(goalFlag == 1 && DistanceA>1200 && DistanceB > 1200)
								{
//									USART_OUT(UART4,(uint8_t *)"Next\r\n");
									if(goalCnt<=19)
									goalCnt++;
									goalFlag = 0;
								}
							}
							lastDistanceA = DistanceA;
							lastDistanceB = DistanceB;
						}
					}
				}
				if(staticShootFlag_3 == 1)
				{
					YawPosCtrl(realAngle);
					/*00000000000000000000000000000000000000000*/
					if(fabs(ReadyawPos() - realAngle)<=1)
					{
						if(fabs(DistanceA-DistanceB) <= 500 && DistanceB < 4500 && DistanceB < 4500 && fabs(realDistance-(DistanceA+DistanceB)/2) <=300)			
						{
							foundRange=15;
							push_Ball_Count++;
//							rps = -0.00000154*(DistanceA+DistanceB)*(DistanceA+DistanceB)/4+0.02197*(DistanceA+DistanceB)/2+22.97;
							rps=-0.000001104*(DistanceA+DistanceB)*(DistanceA+DistanceB)/4+0.01965*(DistanceA+DistanceB)/2+26.71;
							if(rps > 100)
								rps = 100;
							ShooterVelCtrl (rps);
							if(ballColor == MY_BALL_COLOR && pushBallFlag == 1 && OnlyFlag == 1 && fabs(rps-ReadRps())<=1)
							{
									OnlyFlag = 0;
									USART_OUT(UART4,(uint8_t *)"++\r\n");
									semiPushCount += 1;
									noBallCount=0;
									bingoFlag[StdId][0] += 5;
									pushBallFlag = 0;
									noBallPushCnt=0;
									push_Ball_Count = 9900;
							}
							if(push_Ball_Count >= 9990)
							{
								OnlyFlag=1;
								timeAngle=foundRange;
								push_Ball_Count = 0;
								staticShootFlag_3 = 0;	
								StdId=FirstshootJudge();
	//							USART_OUT(UART4,(uint8_t *)"PushOver and pushcnt=%d\r\n",push_Ball_Count);
							}
						}
						else
						{
							scanErrCnt++;
							if(scanErrCnt>=120)
							{
								timeAngle=foundRange;
								scanErrCnt = 0;
								scanFlag_Ok = 0;
								scanFlag_1 = 0;
								scanFlag_2 = 0;
								staticShootFlag_3 = 0;
								if(scanErrFlag==1)
								{
									bingoFlag[StdId][0] += 5;
									scanErrFlag=0;
									StdId=FirstshootJudge();
	//								USART_OUT(UART4,(uint8_t *)"errOver\r\n");
								}
								else
								{
									scanErrFlag = 1;
								}
							}
						}
					}

				}
				if(ballColor != MY_BALL_COLOR && ballColor != 0 && pushBallFlag == 1&&pushBallCount%50==0)
				{
					semiPushCount -= 1;
					noBallCount=0;
					noBallPushCnt=0;
				}	
				pushBallCount++;
				if(noBallPushCnt >= 3)
				{
					laserModel = 0;
					FindBallModel = 1;
					R = 800;
					Cnt = 0;
					staticShootFlag_3=0;
					noBallPushCnt=0;
					push_Ball_Count = 0;
				}
				PosCrl (CAN2, PUSH_BALL_ID,ABSOLUTE_MODE, semiPushCount*PUSH_POSITION/2 + count*PUSH_POSITION);
				USART_OUT(UART4,(uint8_t *)"laserModel\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)realR,StdId,(int)DistanceA,(int)DistanceB,(int)rps,(int)ReadRps(),(int)staticShootFlag_3,(int)realAngle,(int)ReadyawPos(),(int)noBallPushCnt,(int)semiPushCount,(int)realAngle,staticShootFlag_3,ballColor,pushBallFlag,pushBallMotorPos);
//				USART_OUT(UART4,(uint8_t *)"laserModel\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)GetX(),(int)GetY(),(int)GetAngle(),(int)GetWZ(),(int)GetSpeedX(),(int)GetSpeedY(),ballColor,pushBallFlag,semiPushCount);
			}
		} //cnt>250终点										
		else if(FindBallModel)
		{
			scanFlag_Ok=0;
			scanFlag_1 = 0;
			scanFlag_2 = 0;
			if(errFlag==0)
			{
				if(borderSweepFlag)
				{
					Kp=40;
					if((x>1200||x<-1200)&&(y>3600||y<1200))
						v=1200;	
					else
						v=1800;
					if(x*lastX<0||(y-2400)*(lastY-2400)<0)
						squareNode+=1;
				}	
				else
				{	
					v=1800;
					CirclePID(0,2400,R,v,status);
				}	
				if(R>=1800)
					borderSweepFlag=1;
				if(borderSweepFlag)
					BorderSweeping();
				if(squareNode>3)
				{	
					rDecreaseFlag=1;	
					borderSweepFlag=0;
				}	
				if(rDecreaseFlag==0&&R<1800)			
					IncreaseR(1800);
				if(rDecreaseFlag)
					DecreaseR(1600);
				if(throwFlag&&ballColor==MY_BALL_COLOR&&pushBallFlag)
				{
					//当刚切换到1600半径稳定1s
					PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,semiPushCount*PUSH_POSITION/2+(++count)*PUSH_POSITION);				
					pushBallFlag=0;
					haveShootedFlag=1;
					shootCnt++;
					noBallCount=0;
					time=0;
				}	
			}
			if(pushBallCount%50==0)
			{	
				//当拿到对方的球，倒转180度	
				if(ballColor!=MY_BALL_COLOR&&ballColor!=0&&pushBallFlag)
				{
					PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,semiPushCount*PUSH_POSITION/2+(--count)*PUSH_POSITION);
					pushBallFlag=0;
					noBallCount=1;
					time=0;
				}
			}	
			Avoidance();
			laserModelCount++;
			if((R<=1600&&rDecreaseFlag)||laserModelCount>3000)
			{
				FindBallModel=0;
				laserModel=1;
				squareNode=0;
				rDecreaseFlag=0;
				errTime=0;
				laserModelCount=0;
				R=700;
			}
			USART_OUT(UART4,(uint8_t *)"FindBallModel\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)GetX(),(int)GetY(),(int)GetAngle(),(int)GetWZ(),(int)GetSpeedX(),(int)GetSpeedY(),R,rDecreaseFlag,pushBallMotorPos,squareNode);
		}	
		else{
//		if(R<=1100)
//			v=1800;
//		else 
//			v=1500;	
		v=1500;	
		if(status==0)	
			antiRad=T0*speed/realR;
		else 
			antiRad=T1*speed/realR;
		if(errFlag==0)
		{
			CirclePID(0,2400,R,v,status);
			if(x>-100&&lastX<-100&&y>2400&&status==1)
			{	
				circleCnt++;
				if(R>=1600)
				{
					if(PE0==1)
					{	T1-=0.007;
						T0+=0.007;
					}	
					else
					{	T1-=0.007;
						T0+=0.007;
					}	
				}	
			}	
			if(x<100&&lastX>100&&y>2400&&status==0)
			{	
				circleCnt++;
				if(R>=1600)
				{
					if(PE0==1)
					{
						T1+=0.006;
						T0-=0.006;
					}	
					else
					{
						T1+=0.006;
						T0-=0.006;
					}
				}		
			}	
			IncreaseR(1600);
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
				StdId=0;				
			}	
			if(shootX<=800&&shootY>=3200)
			{
				if(shootX>-600&&shootX<600)
					throwFlag=1;
				else
					throwFlag=0;
				StdId=1;				
			}	
			if(shootX<-800&&shootY<=3200)
			{	
				if(shootY<3000&&shootY>1800)
					throwFlag=1;
				else
					throwFlag=0;
				StdId=2;						
			}
			if(shootX>-800&&shootY<1600)
			{	
				if(shootX>-600&&shootX<600)
					throwFlag=1;
				else
					throwFlag=0;
				StdId=3;
			}
			GetShootSituation(StdId);
		}				
		//圆筒方向速度
		Vx=cos(yawAngle*pi/180)*speed;
		//圆筒垂直方向速度
		Vy=sin(yawAngle*pi/180)*speed;
		//应给的速度
		V=-Vx+Distance*9800/(sqrtf(4*4900*(sqrt(3)*Distance-650)+3*Vx*Vx)-sqrt(3)*Vx);
		shootAngle=yawAngle+atan(Vy/V)*180/pi;
		YawPosCtrl(shootAngle);
		rps=(sqrtf(V*V+Vy*Vy)-166.59)/39.574+(Distance-3000)*0.0048;
		if(R==1100&&StdId==2&&PE0==1)
			rps=(sqrtf(V*V+Vy*Vy)-166.59)/39.574+(Distance-3300)*0.003+4;
		if(StdId==3&&R<1600&&PE0==1)
			rps=(sqrtf(V*V+Vy*Vy)-166.59)/39.574+(Distance-3300)*0.003-3;
		ShooterVelCtrl(rps);
		Avoidance();
		if(R>=1100&&circleCnt<=4)
		{	
			if(throwFlag&&ballColor==MY_BALL_COLOR&&pushBallFlag)
			{
				//当刚切换到1600半径稳定1s
				if(--banFirstShoot>0)
					goto label;	
				PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,semiPushCount*PUSH_POSITION/2+(++count)*PUSH_POSITION);				
				pushBallFlag=0;
				haveShootedFlag=1;
				shootCnt++;
				noBallCount=0;
			}		
		}
		if(pushBallCount%50==0)
		{	
			//当拿到对方的球，倒转180度	
			if(ballColor!=MY_BALL_COLOR&&ballColor!=0&&pushBallFlag)
			{
				PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,semiPushCount*PUSH_POSITION/2+(--count)*PUSH_POSITION);
				pushBallFlag=0;
				noBallCount=0;
			}
		}	
		label:pushBallCount++;
		if(quickLaserModel&&circleCnt>1)
		{
			R=500;
			laserModel=1;
			noBallPushCnt=0;
		}	
		if(circleCnt>4)
		{
			R=500;
			errTime=0;
			laserModel=1;
			noBallPushCnt=0;
		}	
		if(noBallPushCnt>2)
			FindBallModel=1;
		ReadActualVel(CAN1,2);
		ReadActualVel(CAN2,5);
		ReadActualVel(CAN2,6);
		ReadActualPos(CAN2,7);
//		USART_OUT(UART4,(uint8_t *)"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)GetX(),(int)GetY(),(int)GetAngle(),(int)GetWZ(),ballColor,pushBallFlag,pushBallMotorPos,count,semiPushCount,errSituation1,errSituation2);//(int)(frontwheelspeedBuffer.data32[1]/(CAR_WHEEL_COUNTS_PER_ROUND*REDUCTION_RATIO*WHEEL_REDUCTION_RATIO)*(pi*TURN_AROUND_WHEEL_DIAMETER)));
		USART_OUT(UART4,(uint8_t *)"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",quickLaserModel,(int)GetX(),(int)GetY(),(int)GetAngle(),(int)GetWZ(),(int)GetSpeedX(),(int)GetSpeedY(),R,squareNode,ballColor,pushBallFlag,pushBallMotorPos);
		}
//		if(abs(pushBallMotorPos-(semiPushCount*PUSH_POSITION/2+count*PUSH_POSITION))>1200)
//		{
//			stuckTime++;
//			if(stuckTime>=200)
//			{	
//				semiPushCount=lastSemiPushCount;
//				count=lastCount;
//				PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,semiPushCount*PUSH_POSITION/2+count*PUSH_POSITION);
//			}	
//		}	
//		else
//		{	
//			stuckTime=0;
//			lastCount=count;
//			lastSemiPushCount=semiPushCount;
//			
//		}	
		if(ballColor==0)
			noBallCount++;
		else
		{	
			noBallCount=0;
			time++;
			if((time%20==0&&!laserModel)||(time%50==0&&laserModel))
				pushBallFlag=1;
		}	
		if(pushBallFlag)
			time=0;
		//一边球仓无球一段时间后后换仓
		if((noBallCount>=200&&!laserModel)||((noBallCount>=300&&laserModel)))
		{	
			PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--semiPushCount)*PUSH_POSITION/2+count*PUSH_POSITION);
			noBallCount=0;
			if(laserModel||circleCnt>2)
				noBallPushCnt++;
		}	
		lastX=x;
		lastY=y;
		lastRps=realRps;
		lastCollectBallSpeed1=collectBallSpeed1;
		lastCollectBallSpeed2=collectBallSpeed2;
		lastCount=count;
		lastSemiPushCount=semiPushCount;
	}
}
