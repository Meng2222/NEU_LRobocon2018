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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM_Init(TIM2,999,83,1,3);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	USART3_Init(115200);
	UART4_Init(921600);
	UART5_Init(921600);
	ElmoInit(CAN1);
	ElmoInit(CAN2);
	delay_s(2);
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
float yawAngle=0,T=0.112,v=1600,angle,Distance,antiRad,realR,shootX,shootY,futureX,futureY;
int status=1,throwFlag=0,R=1600,bingoFlag[4][2]={0},haveShootedFlag=0,errTime=0,StdId,laserModel=0;
float location[4][2]={{2200,200},{2200,4600},{-2200,4600},{-2200,200}};
extern float x,y,Vk,Kp;
extern int time,Cnt,ballColor;
extern Msg_t frontwheelspeedBuffer;
void WalkTask(void)
{
	static int lastTime=0,errFlag=0,pushBallCount=0,statusFlag,count=0,noBallCount=0,pushBallFlag=1,Cnt=0,firstShootTime=2,LaserModelCount;
	static float lastX=0,dLeft,dRight,lastY,lastRps,changeAngle,rps=50,Vx,Vy,V,shootAngle;
	VelCrl(CAN2,COLLECT_BALL1_ID,100*32768); 
	VelCrl(CAN2,COLLECT_BALL2_ID,-100*32768);
	do
	{
		//炮台激光
		dLeft=ReadLaserAValue()*2.48+24.8;
		dRight=ReadLaserBValue()*2.48+24.8;
	}	
	while((dLeft>1500&&dRight>1500)||dLeft==0||dRight==0);
	//战术选择
	if(dRight<=1500)
		status=1;	
	else 
	{
		status=0;
		PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--count)*PUSH_POSITION/2);
	}
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	time=0;
	while (1)
	{
		OSSemPend(PeriodSem,0,&os_err);
		if(laserModel)
		{
			Cnt++;
			CirclePID(0,2400,600,1000,status);
			if(Cnt>150&&Cnt<200)
			{
				VelCrl(CAN1,1,0);				
				VelCrl(CAN1,2,0);
			}	
			if(Cnt>200&&Cnt<250)
			{
				MotorOff(CAN1,1);
				MotorOff(CAN1,2);
			}	
			if(Cnt>250)
			{
				if(StdId==4)
					StdId=0;
				pushBallCount++;
				antiRad=0;
				if(firstShootTime>0)
					StdId=FirstshootJudge();
				GetYawangle(StdId);
				GetDistance(StdId);
				YawPosCtrl(yawAngle);
				rps=(Distance+2255.3)/70.84;
				ShooterVelCtrl(rps);
				if(pushBallCount%200==0)
					pushBallFlag=1;
				if(ballColor==MY_BALL_COLOR&&pushBallFlag&&ReadyawPos()-yawAngle>-0.5&&ReadyawPos()-yawAngle<0.5)
				{	PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(++count)*PUSH_POSITION/2);	
					pushBallFlag=0;
					StdId++;
					firstShootTime--;
				}	
				if(ballColor!=MY_BALL_COLOR&&ballColor!=0&&pushBallFlag)
				{	PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--count)*PUSH_POSITION/2);	
					pushBallFlag=0;	
				}	
			}	
		}	
		else{
		if(v==1600)
			Kp=75;
		if(v==2000)
			Kp=110;
		x=GetX();
		y=GetY();
		angle=GetAngle()+90;
		realR=sqrtf(x*x+(y-2400)*(y-2400));
		antiRad=T*sqrtf(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY())/realR;
		if(errFlag==1)
		{	
			AnglePID(changeAngle,GetAngle());
			Straight(-1200);
			if(errTime%2==0&&statusFlag)
			{
				status=1-status;
				statusFlag=0;	
			}	
			if(Get_Time_Flag())	
				errFlag=0;
			time=0;
		}	
		else
		{
			CirclePID(0,2400,R,v,status);
			if(status==0)
			{					
				if(x>-100&&lastX<-100&&y<2400&&R<1500)
					R+=500;
			}	
			else
			{
				if(x<100&&lastX>100&&y<2400&&R<1500)
					R+=500;
			}		
			if((x-lastX)*(x-lastX)+(y-lastY)*(y-lastY)>50)
				lastTime=time;
			if(time-lastTime>=100)
			{	
				errFlag=1;
				statusFlag=1;
				errTime++;
				changeAngle=GetAngle()+45;
				Cnt=0;
			}	
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
		//圆筒方向速度
		Vx=cos(yawAngle*pi/180)*sqrtf(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY());
		//圆筒垂直方向速度
		Vy=sin(yawAngle*pi/180)*sqrtf(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY());
		//应给的速度
		V=-Vx+Distance*9800/(sqrtf(4*4900*(sqrt(3)*Distance-650)+3*Vx*Vx)-sqrt(3)*Vx);
		shootAngle=yawAngle+atan(Vy/V)*180/pi;
		YawPosCtrl(shootAngle);
		rps=(sqrtf(V*V+Vy*Vy)-166.59)/39.574+(Distance-3500)*0.0041;
		if(rps>74)
			throwFlag=0;
		ShooterVelCtrl(rps);
		if(ballColor==0)
			noBallCount++;	
		if(noBallCount%36==0)	
			pushBallFlag=1;
		//一段时间无球后反转防卡球
//		if(noBallCount>=220)
//		{	
//			PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--count)*PUSH_POSITION/2);	
//			noBallCount=0;
//		}	
		if(R>1500)
		{
				if(throwFlag&&ballColor==MY_BALL_COLOR&&pushBallFlag)
				{
					PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(++count)*PUSH_POSITION);				
					pushBallFlag=0;
					noBallCount=1;
					haveShootedFlag=1;
				}		
				if(pushBallCount%100==0)
				{	
					//当拿到对方的球，倒转180度	
					if(ballColor!=MY_BALL_COLOR&&ballColor!=0&&pushBallFlag)
					{
						PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--count)*PUSH_POSITION);
						pushBallFlag=0;
						noBallCount=1;
					}	
				}	
		}
		pushBallCount++;
		lastX=x;
		lastY=y;
		if((bingoFlag[0][1]!=0&&bingoFlag[1][1]!=0&&bingoFlag[2][1]!=0&&bingoFlag[3][1]!=0)||errTime>4||pushBallCount>6000)
		{
			LaserModelCount++;
			if(LaserModelCount>=100)
				laserModel=1;
		}	
//		ReadActualVel(CAN1,2);
		USART_OUT(UART4,(uint8_t *)"%d\t%d\t%d\t%d\t%d\t\r\n",(int)GetX(),(int)GetY(),ballColor,(int)count,laserModel);//(int)(frontwheelspeedBuffer.data32[1]/(CAR_WHEEL_COUNTS_PER_ROUND*REDUCTION_RATIO*WHEEL_REDUCTION_RATIO)*(pi*TURN_AROUND_WHEEL_DIAMETER)));
		}	
	}	
}
