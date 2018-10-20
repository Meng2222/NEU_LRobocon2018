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
#define SWITCH_ON  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)
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
	//比赛结束后打开行程开关吐球程序段
	do
	{
		VelCrl(CAN2,COLLECT_BALL1_ID,-80*32768); 
		VelCrl(CAN2,COLLECT_BALL2_ID,80*32768);	
	}
	while(SWITCH_ON==1);
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
//航向电机角度
float yawAngle=0;
//T：发出命令到航向电机响应的延迟时间，1表示顺时针，0表示逆时针  antiRad：圆形走投时延迟时间内走过的弧度
float T1=0,T0=0,antiRad=0;
//一些走行的基本参数。angle：现实直角坐标系下的角度  realR：圆形闭环的实际半径，也是车距场中心的距离  shootx(y):炮台的坐标  x(y):后轮轴中心点坐标 lastx(y):10ms前后轮轴中心坐标
float v=1500,angle,Distance,realR=0,shootX,shootY,futureX,futureY,lastX=0,lastY=0,x,y;
//桶的ID号，从0到3
int StdId;
//行号为桶的ID号，对应四个桶的中心坐标
float location[4][2]={{2200,200},{2200,4600},{-2200,4600},{-2200,200}};
//rps:给定的包胶轮转速
float speedX,speedY,speed,rps=50;
//定义数组用来存放扫描得到的角度和距离参数
float scanAngle[20][300]={0},scanDistance[20][300]={0};
//给定的圆形闭环半径 status：顺逆时针，1为顺，0为逆
int R=500,status=1;
//一些标志位，望文生义，表面意思。bingoFlag：用于投球优先级的判断
int pushBallFlag=1,borderSweepFlag=0,errFlag=0,throwFlag=0,bingoFlag[4][2]={0},haveShootedFlag=0,RchangeFlag,rDecreaseFlag=0,banFirstShoot=0;
//一些计数。shootCnt:射出球的次数  circleCnt:走过的圈数 Cnt:定点模式的计时器
int count=0,semiPushCount=0,errTime=0,changeLightTime=0,shootCnt=0,circleCnt=1,touchLaserTime,lastTouchLaserTime,Cnt=0;
//模式标志位
int laserModel=0,findBallModel=0,quickLaserModel=0;
int scanCnt[20]={0};
extern int ballColor,backwardCount,errSituation1,errSituation2;
extern float Vk,Kp,lAngle;
//读取一些电机转速
extern Msg_t frontwheelspeedBuffer,collectball1speedBuffer,collectball2speedBuffer,pushballmotorPosBuffer;
void WalkTask(void)
{
	int cycleCnt=0,staticShootFlag=0,push_Ball_Count;
	//定点扫描的左右角度范围
	int foundRange=20;
	//电机转速或位置
	int collectBallSpeed1=0,collectBallSpeed2=0,lastCollectBallSpeed1=0,lastCollectBallSpeed2=0,pushBallMotorPos=0;
	//距离和角度
	float DistanceA=0,DistanceB=0,lastDistanceA=0,lastDistanceB=0,realAngle=0,realDistance=0,deflectAngle=0,errDistance=0;
	//lastStdId:上次扫描的桶号  squareNode：方形扫边的节点，四个节点说明扫过一圈 nopushCnt：无球反推计数
	int lastStdId=0,squareNode=0,noBallPushCnt=0;
	//定点时一些计数和标志位。onlyFlag：只射一颗球的标志位。
	int goalCnt=0,goalFlag=0,mostGroup=0,OnlyFlag=1,scanErrCnt=0,scanErrFlag=0,scanOverFlag=0;
	//lightAngle:定点扫描时定位系统算出航向电机应给的角度  timeAngle：范围角度
	float timeAngle=16,lightAngle=0;
	//计数
	int pushBallCount=0,noBallCount=0,lastSemiPushCount,lastCount,time=0,stuckTime=0;
	float Vx,Vy,V,shootAngle;
	VelCrl(CAN2,COLLECT_BALL1_ID,70*32768); 
	VelCrl(CAN2,COLLECT_BALL2_ID,-70*32768);
	do
	{		
		//左激光读取距离
		DistanceA=ReadLaserAValue1()*2.461+55.43;
		//右激光读取距离
		DistanceB=ReadLaserBValue1()*2.467+60.87;
		//800mm内挡住激光100ms后触发
		if(DistanceA>800&&DistanceB>800)
		{
			lastTouchLaserTime=touchLaserTime;
		}	
		USART_OUT(UART4,(uint8_t *)"%d\t%d\r\n",(int)DistanceA,(int)DistanceB);
	}
	while(touchLaserTime-lastTouchLaserTime<=100);
	if(SWITCH_ON==1)
		R=1100;
	//战术选择
	if(DistanceB<=300)
	{
		status=0;
		switch(SWITCH_ON)
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
	else if(DistanceA<=300)
	{
		status=1;
		switch(SWITCH_ON)
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
	else if(DistanceB>300&&DistanceB<800)
	{
		status=0;
		quickLaserModel=1;
	}	
	else if(DistanceA>300&&DistanceA<800)
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
		DistanceA = ReadLaserAValue()*2.461+55.43;
		DistanceB = ReadLaserBValue()*2.467+60.87;
		x = GetX();
		y = GetY();
		speedX = GetSpeedX();
		speedY = GetSpeedY();
		speed = sqrtf(speedX*speedX+speedY*speedY);
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
				//先扫优先级高的桶
				StdId = FirstshootJudge();	
			}	
			else
			{
				if(staticShootFlag != 1)
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
						scanOverFlag=1;
				}
				//通过定位系统得到桶的大致坐标从而算出距离
				GetDistance1 (StdId);
				//激光扫描到桶条件未满足
				if(staticShootFlag != 1)
				{
					//如果扫描已结束，记下最大距离处和此时航向电机的角度
					if(scanOverFlag == 1)
					{
						scanOverFlag=0;
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
							staticShootFlag = 1;
							YawPosCtrl(realAngle);
						}	
						else
						{
							scanOverFlag = 0;
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
					//扫描中
					else
					{
						if(fabs(ReadyawPos()-(lightAngle-timeAngle)) < 10 )
						{
							if(fabs(DistanceA-DistanceB) < 200 && fabs(DistanceA-Distance)<800 && fabs(DistanceA-DistanceB)<800 && DistanceA < 4500 && DistanceB < 4500 && fabs(lastDistanceA-DistanceA) < 50 && fabs(lastDistanceB-DistanceB) < 50 && fabs(fabs(lastDistanceA-DistanceA)-fabs(lastDistanceB-DistanceB)) < 20)
							{
								goalFlag = 1;
								//记录扫描所得到的角度和距离
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
							//记录上10ms的激光返回距离，从而扫到突变
							lastDistanceA = DistanceA;
							lastDistanceB = DistanceB;
						}
					}
				}
				//激光扫描到桶
				if(staticShootFlag == 1)
				{
					YawPosCtrl(realAngle);
					//当航向电机转到给定角度
					if(fabs(ReadyawPos() - realAngle)<=1)
					{
						//满足是桶的判断条件则准备射球
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
								staticShootFlag = 0;	
								StdId=FirstshootJudge();
							}
						}
						//否则扫描错误，重新扫描
						else
						{
							scanErrCnt++;
							if(scanErrCnt>=120)
							{
								timeAngle=foundRange;
								scanErrCnt = 0;
								scanOverFlag = 0;
								staticShootFlag = 0;
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
				//无球反推三下开始进入找球模式
				if(noBallPushCnt >= 3)
				{
					laserModel = 0;
					findBallModel = 1;
					R = 800;
					Cnt = 0;
					staticShootFlag=0;
					noBallPushCnt=0;
					push_Ball_Count = 0;
				}
				PosCrl (CAN2, PUSH_BALL_ID,ABSOLUTE_MODE, semiPushCount*PUSH_POSITION/2 + count*PUSH_POSITION);
				USART_OUT(UART4,(uint8_t *)"laserModel\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",count,(int)realR,StdId,(int)DistanceA,(int)DistanceB,(int)rps,(int)ReadRps(),(int)staticShootFlag,(int)realAngle,(int)ReadyawPos(),(int)noBallPushCnt,(int)semiPushCount,(int)realAngle,staticShootFlag,ballColor,pushBallFlag,pushBallMotorPos);
			}
		} 									
		else if(findBallModel)
		{
			scanOverFlag=0;
			if(errFlag==0)
			{
				//走圆半径小时先扩大半径
				if(rDecreaseFlag==0&&R<1800)			
					IncreaseR(1800);
				//圆形半径到达边缘时进入扫边
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
			//每0.5s检测一次是否有黑球
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
			//由扫边变为走圆后开始进入定点投
			if(R<=1600&&rDecreaseFlag)
			{
				findBallModel=0;
				laserModel=1;
				squareNode=0;
				rDecreaseFlag=0;
				errTime=0;
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
				break;
			case 1:
				antiRad=T1*speed/realR;
				break;
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
				//每走一圈，定位系统会有一定的飘移，提前量做出一些补偿
				if(R>=1600)
				{
					switch(SWITCH_ON)
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
					switch(SWITCH_ON)
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
			//划分四个投球区域，每个区域对应投一个桶
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
		if(status==0&&SWITCH_ON==0)
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
		//快速定点模式一圈后进入定点
		if(quickLaserModel&&circleCnt>1)
		{
			R=500;
			laserModel=1;
			noBallPushCnt=0;
		}	
		//圆形走五圈后开始进入定点
		if(circleCnt>5||(errTime>2&&circleCnt>3))
		{
			R=500;
			laserModel=1;
			noBallPushCnt=0;
			throwFlag=0;
		}	
		//如果外圈空转三次后进入找球模式
		if(noBallPushCnt>2)
		{
			findBallModel=1;
			throwFlag=0;
		}	
		USART_OUT(UART4,(uint8_t *)"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)GetX(),(int)GetY(),(int)GetAngle(),(int)GetWZ(),semiPushCount,count,R,squareNode,ballColor,pushBallFlag,circleCnt,pushBallMotorPos);
		}
		//读取各电机的速度或位置
		ReadActualVel(CAN1,2);
		ReadActualVel(CAN2,5);
		ReadActualVel(CAN2,6);
		ReadActualPos(CAN2,7);
		//防卡球
		if(abs(pushBallMotorPos-(semiPushCount*PUSH_POSITION/2+count*PUSH_POSITION))>1200)
		{
			//当推球转盘不在给定位置，开始计时，2s后恢复倒转回到给定命令之前的状态
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
			//当推球转盘未卡住，开始识别球色
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
