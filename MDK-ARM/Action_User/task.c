#include  <includes.h>
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
float yawAngle=0,T=0.168,v=1500,angle,Distance,antiRad,realR,shootX,shootY,futureX,futureY;
int status=0,throwFlag=0,R=600;
extern float x,y,Vk;
extern int time,Cnt,ballColor;
extern Msg_t frontwheelspeedBuffer;
void WalkTask(void)
{
	static int lastTime=0,errFlag=0,pushBallCount=0,errTime=0,statusFlag,count=0,noBallCount=0,pushBallFlag=1,bingoFlag[4]={0},haveShootedFlag=0;
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
	{
		status=1;
		if(dRight<=900&&dRight>100)
			R=1100;
		else if(dRight<=1500&&dRight>900)
			R=1600;		
	}	
	else if(dLeft<=1500)
	{
		status=0;
		if(dLeft<=900&&dLeft>100)
			R=1100;
		else if(dLeft<=1500&&dLeft>900)
			R=1600;
	}	
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	time=0;
	while (1)
	{
		OSSemPend(PeriodSem,0,&os_err);
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
			if(shootX<=600&&shootY<1800)
			{	
				if(shootX>-600)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(2200,200);
				GetDistance(2200,200);
				if(haveShootedFlag==1)
				{	bingoFlag[0]=errTime+1;
					haveShootedFlag=0;
				}	
				if(bingoFlag[0]<4&&bingoFlag[0]>0)
					throwFlag=0;
			}
			if(shootY<=3000&&shootX>=600)
			{	
				if(shootY>1800)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(2200,4600);
				GetDistance(2200,4600);
				if(haveShootedFlag==1)
				{	bingoFlag[1]=errTime+1;
					haveShootedFlag=0;
				}	
				if(bingoFlag[0]<4&&bingoFlag[0]>0)
					throwFlag=0;				
			}	
			if(shootX>-600&&shootY>=3000)
			{	
				if(shootX<600)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(-2200,4600);
				GetDistance(-2200,4600);
				if(haveShootedFlag==1)
				{	bingoFlag[2]=errTime+1;
					haveShootedFlag=0;
				}	
				if(bingoFlag[0]<4&&bingoFlag[0]>0)
					throwFlag=0;				
			}
			if(shootY>1800&&shootX<-600)
			{	
				if(shootY<3000)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(-2200,200);
				GetDistance(-2200,200);
				if(haveShootedFlag==1)
				{	bingoFlag[3]=errTime+1;
					haveShootedFlag=0;
				}	
				if(bingoFlag[0]<4&&bingoFlag[0]>0)
					throwFlag=0;				
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
			if(shootX>=600&&shootY>1800)
			{
				if(shootY<2600)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(2200,200);
				GetDistance(2200,200);
				if(haveShootedFlag==1)
				{
					bingoFlag[0]=errTime+1;
					haveShootedFlag=0;
				}	
				if(bingoFlag[0]<4&&bingoFlag[0]>0)
					throwFlag=0;				
			}	
			if(shootX<=600&&shootY>=3000)
			{
				if(shootX>-200)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(2200,4600);
				GetDistance(2200,4600);
				if(haveShootedFlag==1)
				{	
					bingoFlag[1]=errTime+1;
					haveShootedFlag=0;
				}		
				if(bingoFlag[0]<4&&bingoFlag[0]>0)
					throwFlag=0;				
			}	
			if(shootX<-600&&shootY<=3000)
			{	
				if(shootY>2200)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(-2200,4600);
				GetDistance(-2200,4600);
				if(haveShootedFlag==1)
				{
					bingoFlag[2]=errTime+1;
					haveShootedFlag=0;
				}		
				if(bingoFlag[0]<4&&bingoFlag[0]>0)
					throwFlag=0;				
			}
			if(shootX>-600&&shootY<1800)
			{	
				if(shootX<200)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(-2200,200);
				GetDistance(-2200,200);
				if(haveShootedFlag==1)
				{	bingoFlag[3]=errTime+1;
					haveShootedFlag=0;
				}	
				if(bingoFlag[0]<4&&bingoFlag[0]>0)
					throwFlag=0;				
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
		rps=(sqrtf(V*V+Vy*Vy)-166.59)/39.574-(v-1000)*0.0035;
		if(rps>80)
			rps=53;
		ShooterVelCtrl(rps);
		if(ballColor==0)
			noBallCount++;	
		if(noBallCount%31==0)
		{	
			pushBallFlag=1;
		}	
		//一段时间无球后反转防卡球
//		if(noBallCount>=110)
//		{	
//			PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--count)*PUSH_POSITION/2);	
//			noBallCount=0;
//		}	
		if(R>1000)
		{
				if(throwFlag&&ballColor==MY_BALL_COLOR&&pushBallFlag)
				{
					PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(++count)*PUSH_POSITION);				
					pushBallFlag=0;
					noBallCount=1;
				}		
				if(pushBallCount>=100)
				{	
					//当拿到对方的球，倒转180度	
					if(ballColor!=MY_BALL_COLOR&&ballColor!=0&&pushBallFlag)
					{
						PosCrl(CAN2,PUSH_BALL_ID,ABSOLUTE_MODE,(--count)*PUSH_POSITION);
						pushBallFlag=0;
						noBallCount=1;
					}	
					pushBallCount=0;
				}	
		}
		pushBallCount++;
		lastX=x;
		lastY=y;
		//通过转速突变判断是否射出球
//		if(lastRps/ReadRps()-1>0.2&&lastRps/ReadRps()-1<0.5)
//			haveShootedFlag=1;
		lastRps=ReadRps();
		ReadActualVel(CAN1,2);
		USART_OUT(UART4,(uint8_t *)"%d\t%d\t%d\t%d\t%d\t%d\t\r\n",(int)GetX(),(int)GetY(),ballColor,(int)rps,(int)(-Vk),(int)(frontwheelspeedBuffer.data32[1]/(CAR_WHEEL_COUNTS_PER_ROUND*REDUCTION_RATIO*WHEEL_REDUCTION_RATIO)*(pi*TURN_AROUND_WHEEL_DIAMETER)));
	}
}