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
	//比赛结束后吐球程序段
	do
	{
		//读取按键
		PE0=GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0);
		VelCrl(CAN2,COLLECT_BALL1_ID,-80*32768); 
		VelCrl(CAN2,COLLECT_BALL2_ID,80*32768);	
	}
	while(PE0==1);
	PosLoopCfg(CAN2,PUSH_BALL_ID,10000000,10000000,900000);
	VelLoopCfg(CAN1,1,50000000,50000000);
	VelLoopCfg(CAN1,2,50000000,50000000);
	MotorOn(CAN2,COLLECT_BALL1_ID);
	MotorOn(CAN2,COLLECT_BALL2_ID);
	MotorOn(CAN1,1);
	MotorOn(CAN1,2);
	//开电后航向电机右偏90度防止照射对方激光被反射或对方抢跑造成误触发
	YawPosCtrl(90);
	delay_s(2);
	WaitOpsPrepare();
	OSTaskSuspend(OS_PRIO_SELF);
}
float yawAngle=0,T1=0,T0=0,v=1500,angle,Distance,antiRad,realR=0,shootX,shootY,futureX,futureY,lastX=0,lastY=0,x,y;
int pushBallFlag=1,borderSweepFlag=0,count=0,shakeShootFlag=0,shakeShootCnt=0,errFlag=0,status=1,semiPushCount=0,throwFlag=0,R=500,bingoFlag[4][2]={0},haveShootedFlag=0,errTime=0,StdId,laserModel=0,changeLightTime=0,RchangeFlag,rDecreaseFlag=0,findBallModel=0,banFirstShoot=0,shootCnt=0,circleCnt=1;
float location[4][2]={{2200,200},{2200,4600},{-2200,4600},{-2200,200}},speedX,speedY,speed,rps=50;
int scanCnt[20]={0},touchLaserTime,lastTouchLaserTime,quickLaserModel=0,Cnt=0;
float scanAngle[20][300]={0},scanDistance[20][300]={0};
extern int ballColor,backwardCount,errSituation1,errSituation2;
extern float Vk,Kp,lAngle;
extern Msg_t frontwheelspeedBuffer,collectball1speedBuffer,collectball2speedBuffer,pushballmotorPosBuffer;
void WalkTask(void)
{
	int PE1=0,cycleCnt=0,staticShootFlag_1=0,staticShootFlag_2=0,staticShootFlag_3=0,push_Ball_Count=0,noPushCnt=0,noPushFlag=1,notFoundCnt=0,stuckTime=0;
	int foundRange=15,collectBallSpeed1=0,collectBallSpeed2=0,lastCollectBallSpeed1=0,lastCollectBallSpeed2=0,pushBallMotorPos=0;
	float D1=0,D2=0,distance,lastDistanceA=0,lastDistanceB=0,realAngle=0,realDistance=0,deflectAngle=0,errDistance=0,RA=0,RB=0;
	int lastStdId=0,squareNode=0,noBallPushCnt=0;
	int goalCnt=0,scanFlag_1=0,scanFlag_2=0,goalFlag=0,mostGroup=0,OnlyFlag=1,scanErrCnt=0,scanErrFlag=0,scanFlag_Ok=0;
	float staticShootAngle_1=0,timeAngle=16,DistanceA=0,DistanceB=0,lightAngle=0,staticShootAngle_2=0;
	int pushBallCount=0,statusChangeFlag=0,noBallCount=0,staticPushBallFlag=1,laserModelCount=0,findBallModelCount=0,realCount=0,lastSemiPushCount,lastCount,time=0;
	float dLeft=0,dRight=0,Vx,Vy,V,shootAngle,realRps;
	VelCrl(CAN2,COLLECT_BALL1_ID,70*32768); 
	VelCrl(CAN2,COLLECT_BALL2_ID,-70*32768);
	do
	{		
		//炮台激光读取距离
		dLeft=ReadLaserAValue1()*2.461+55.43;
		dRight=ReadLaserBValue1()*2.467+60.87;
		//800mm内挡住激光100ms后触发
		if(dLeft>800&&dRight>800)
		{
			lastTouchLaserTime=touchLaserTime;
		}	
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
		switch(PE0)
		{
			//逆时针内圈
			case 0:
				T0=0.076;
				T1=0.043;
				break;
			//逆时针中圈
			case 1:
				T0=0.078;
				T1=0.056;
				circleCnt+=1;
				break;
		}	
		//推球转盘倒转180度优先使用内舱
		PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--semiPushCount)*PUSH_POSITION/2+count*PUSH_POSITION);
	}	
	else if(dLeft<=300)
	{
		status=1;
		switch(PE0)
		{
			//顺时针内圈
			case 0:
				T0=0.076;
				T1=0.043;
				break;
			//顺时针中圈
			case 1:
				T0=0.078;
				T1=0.056;
				circleCnt+=1;
				break;
		}	
	}
	//远处遮挡激光启动快速定点模式
	else if(dRight>300&&dRight<800)
	{
		status=0;
		quickLaserModel=1;
	}	
	else if(dLeft>300&&dLeft<800)
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
		RA=ReadLaserAValue();
		RB=ReadLaserBValue();
		if(RA >= 100)
		DistanceA = RA*2.461+55.43;
		if(RB >= 100)
		DistanceB = RB*2.467+60.87;
		x = GetX();
		y = GetY();
		speedX = GetSpeedX();
		speedY = GetSpeedY();
		speed = sqrtf(speedX*speedX+speedY*speedY);
		realRps = ReadRps();
		realR=sqrtf(x*x+(y-2400)*(y-2400));
		//将定位系统返回的角度转化为以出发区为原点的直角坐标系的角度
		angle=GetAngle()+90;
		//读取收球辊子速度
		collectBallSpeed1 = collectball1speedBuffer.data32[1]/32768;
		collectBallSpeed2 = collectball2speedBuffer.data32[1]/32768;
		//读取推球转盘位置
		pushBallMotorPos = pushballmotorPosBuffer.data32[1];
		//不同速度用不同Kp调节
		if(v == 1800)
			Kp = 120;
		if(v == 1500)
			Kp = 85;
		//激光模式
		if(laserModel)
		{
			Cnt++;
			v = 1500;	
			//当被对方推出中心区自动归位
			if(Cnt>1000&&realR>900)
				Cnt=-100;
			if(Cnt<250)
			{	
				if(errFlag==0)
					CirclePID (0,2400,R,v,status);
				Avoidance();
			}	
			//停住
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
					deflectAngle=0;
					foundRange=foundRange-deflectAngle;
					//让航向电机沿劣弧旋转
					if(lastStdId-StdId==3)
						cycleCnt++;
					if(lastStdId-StdId==-3)
						cycleCnt--;
					//通过定位系统得到航向角的大致角度
					lightAngle = getLingtAngle(GetX(),GetY(),StdId)-cycleCnt*360;
					lastStdId=StdId;
					//航向角在定位系统给出的大概角度左右20度扫描，扫描速度32度/s
					YawPosCtrl (lightAngle-timeAngle);
					if(timeAngle > -foundRange && fabs(ReadyawPos()-(lightAngle-timeAngle)) < 20)
						timeAngle -= 0.32;
					if(timeAngle <= -foundRange)
						scanFlag_Ok=1;
				}
				//通过定位系统得到桶的大致坐标从而算出距离
				GetDistance1 (StdId);
				//激光扫描到桶条件未满足
				if(staticShootFlag_3 != 1)
				{
					if(scanFlag_Ok == 1)
					{
						scanFlag_Ok=0;
						scanFlag_1 = 0;
						scanFlag_2 = 0;
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
						//枪口与激光有一定角度误差，给出一定的补偿角度
						realAngle=scanAngle[mostGroup][t1]+1.6;
						USART_OUT(UART4,(uint8_t *)"laserModel\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",StdId,(int)DistanceA,(int)DistanceB,(int)(scanAngle[mostGroup][t2]*100),(int)(scanAngle[mostGroup][0]*100),(int)(realAngle*100),(int)scanAngle[mostGroup][t2]*100,(int)realDistance);
						//激光返回的精确距离
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
							if(foundRange>=25)
								errDistance+=500;
							if(errDistance==1500)
							{
								bingoFlag[StdId][0] += 5;
								errDistance=0;
								StdId=FirstshootJudge();
							}
							timeAngle=foundRange;
						}
						goalCnt=0;
						for(int i = 0;i <= 19;i++)
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
						if(fabs(ReadyawPos()-(lightAngle-timeAngle)) < 10 && staticShootFlag_3 != 1)
						{
							if(fabs(DistanceA-DistanceB) < 200 && fabs(DistanceA-Distance)<800 && fabs(DistanceA-DistanceB)<800 && DistanceA < 4500 && DistanceB < 4500 && fabs(lastDistanceA-DistanceA) < 50 && fabs(lastDistanceB-DistanceB) < 50 && fabs(fabs(lastDistanceA-DistanceA)-fabs(lastDistanceB-DistanceB)) < 20)
							{
								goalFlag = 1;
								scanAngle[goalCnt][scanCnt[goalCnt]] = ReadyawPos();
								scanDistance[goalCnt][scanCnt[goalCnt]] = (DistanceA+DistanceB)/2;
								if(scanCnt[goalCnt]<=298)
								scanCnt[goalCnt]++;
							}
							else
							{
								if(goalFlag == 1 && DistanceA>1200 && DistanceB > 1200)
								{
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
				//激光扫描到桶
				if(staticShootFlag_3 == 1)
				{
					YawPosCtrl(realAngle);
					//航向电机转到给定角度
					if(fabs(ReadyawPos() - realAngle)<=1)
					{
						if(fabs(DistanceA-DistanceB) <= 500 && DistanceB < 4500 && DistanceB < 4500 && fabs(realDistance-(DistanceA+DistanceB)/2) <=300)			
						{
							foundRange=15;
							push_Ball_Count++;
							rps=-0.000001104*(DistanceA+DistanceB)*(DistanceA+DistanceB)/4+0.01964*(DistanceA+DistanceB)/2+26.71+((DistanceA+DistanceB)/2-3700)/1300;
							if(2200<DistanceA/2+DistanceB/2&&DistanceA/2+DistanceB/2<3200)
								rps+=(DistanceA/2+DistanceB/2-3200)*0.0037;
							//转速限幅
							if(rps > 100)
								rps = 100;
							ShooterVelCtrl (rps);
							if(ballColor == MY_BALL_COLOR && pushBallFlag == 1 && OnlyFlag == 1 && fabs(rps-ReadRps())<=1)
							{
									OnlyFlag = 0;
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
				//无球反推三下开始找球
				if(noBallPushCnt >= 3)
				{
					laserModel = 0;
					findBallModel = 1;
					R = 800;
					Cnt = 0;
					staticShootFlag_3=0;
					noBallPushCnt=0;
					push_Ball_Count = 0;
				}
				PosCrl (CAN2, PUSH_BALL_ID,ABSOLUTE_MODE, semiPushCount*PUSH_POSITION/2 + count*PUSH_POSITION);
				USART_OUT(UART4,(uint8_t *)"laserModel\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",count,(int)realR,StdId,(int)DistanceA,(int)DistanceB,(int)rps,(int)ReadRps(),(int)staticShootFlag_3,(int)realAngle,(int)ReadyawPos(),(int)noBallPushCnt,(int)semiPushCount,(int)realAngle,staticShootFlag_3,ballColor,pushBallFlag,pushBallMotorPos);
//				USART_OUT(UART4,(uint8_t *)"laserModel\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)GetX(),(int)GetY(),(int)GetAngle(),(int)GetWZ(),(int)GetSpeedX(),(int)GetSpeedY(),ballColor,pushBallFlag,semiPushCount);
			}
		} 									
		else if(findBallModel)
		{
			scanFlag_Ok=0;
			scanFlag_1 = 0;
			scanFlag_2 = 0;
			if(errFlag==0)
			{
				if(rDecreaseFlag==0&&R<1800)			
					IncreaseR(1800);
				if(R>=1800)
					borderSweepFlag=1;
				if(borderSweepFlag)
				{
					Kp=40;
					//过弯时减速
					if((x>1200||x<-1200)&&(y>3600||y<1200))
						v=1200;	
					else
						v=1800;
					//方形扫边节点计数
					if(x*lastX<0||(y-2400)*(lastY-2400)<0)
						squareNode+=1;
				}	
				else
				{	
					CirclePID(0,2400,R,v,status);
					v=1800;
				}	
				if(borderSweepFlag)
					BorderSweeping();
				//扫边一圈后降半径准备定点
				if(squareNode>3)
				{	
					rDecreaseFlag=1;	
					borderSweepFlag=0;
				}	
				if(rDecreaseFlag)
				{
					DecreaseR(1600);
				}	
				if(throwFlag&&ballColor==MY_BALL_COLOR&&pushBallFlag)
				{
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
				//当拿到对方的球，倒转360度	
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
				findBallModel=0;
				laserModel=1;
				squareNode=0;
				rDecreaseFlag=0;
				errTime=0;
				laserModelCount=0;
				R=550;
				noBallPushCnt=0;
			}
			USART_OUT(UART4,(uint8_t *)"findBallModel\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)GetX(),(int)GetY(),(int)GetAngle(),(int)GetWZ(),R,semiPushCount,count,pushBallMotorPos,throwFlag);
		}	
		else{
		v=1500;	
		switch(status)
		{
			case 0:
				antiRad=T0*speed/realR;
			case 1:
				antiRad=T1*speed/realR;
			default:
				break;
		}	
		if(errFlag==0)
		{
			CirclePID(0,2400,R,v,status);
			if(x>-100&&lastX<-100&&y>2400&&status==1)
			{	
				//走过圈数计数
				circleCnt++;
				if(R>=1600)
				{
					switch(PE0)
					{
						case 0:
							T1-=0.0065;
							T0+=0.0065;
							break;
						case 1:
							T1-=0.007;
							T0+=0.007;
							break;
					}	
				}	
			}	
			if(x<100&&lastX>100&&y>2400&&status==0)
			{	
				circleCnt++;
				if(R>=1600)
				{
					switch(PE0)
					{
						case 0:
							T1+=0.006;
							T0-=0.006;
							break;
						case 1:
							T1+=0.006;
							T0-=0.006;
							break;
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
		//航向电机应给角度
		shootAngle=yawAngle+atan(Vy/V)*180/pi;
		YawPosCtrl(shootAngle);
		//包胶轮应给转速
		rps=(sqrtf(V*V+Vy*Vy)-166.59)/39.574+(Distance-3200)*0.0051;
		//逆时针内圈时应给转速
		if(status==0&&PE0==0)
			rps=(sqrtf(V*V+Vy*Vy)-166.59)/39.574+(Distance-3000)*0.0043;
		ShooterVelCtrl(rps);
		Avoidance();
		if(R>=1100&&circleCnt<=5)
		{	
			if(throwFlag&&ballColor==MY_BALL_COLOR&&pushBallFlag)
			{
				//被撞后稳定一段时间
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
		if(circleCnt>5||(errTime>2&&circleCnt>3))
		{
			R=500;
			errTime=0;
			laserModel=1;
			noBallPushCnt=0;
			throwFlag=0;
		}	
		//空转三次后进入找球模式
		if(noBallPushCnt>2)
		{
			findBallModel=1;
			throwFlag=0;
		}	
		USART_OUT(UART4,(uint8_t *)"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)GetX(),(int)GetY(),(int)GetAngle(),(int)GetWZ(),semiPushCount,count,R,squareNode,ballColor,pushBallFlag,circleCnt,pushBallMotorPos);
		}
		ReadActualVel(CAN1,2);
		ReadActualVel(CAN2,5);
		ReadActualVel(CAN2,6);
		ReadActualPos(CAN2,7);
		//防卡球
		if(abs(pushBallMotorPos-(semiPushCount*PUSH_POSITION/2+count*PUSH_POSITION))>1200)
		{
			stuckTime++;
			if(stuckTime>=200)
			{	
				semiPushCount=lastSemiPushCount;
				count=lastCount;
				PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,semiPushCount*PUSH_POSITION/2+count*PUSH_POSITION);
			}	
		}	
		else
		{	
			stuckTime=0;
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
			if(noBallCount>=200)
			{	
				PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--semiPushCount)*PUSH_POSITION/2+count*PUSH_POSITION);
				noBallCount=0;
				if(laserModel||circleCnt>2)
					noBallPushCnt++;
			}	
			lastCount=count;
			lastSemiPushCount=semiPushCount;
		}	
		lastX=x;
		lastY=y;
		//试图通过收球辊子转速变化数球（未完成）
		lastCollectBallSpeed1=collectBallSpeed1;
		lastCollectBallSpeed2=collectBallSpeed2;
	}
}
