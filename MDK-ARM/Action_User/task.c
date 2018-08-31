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
static OS_STK ErrTaskStk[Err_TASK_STK_SIZE];
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
/*	os_err = OSTaskCreate((void (*)(void *))ErrTask,
						  (void *)0,
						  (OS_STK *)&ErrTaskStk[Err_TASK_STK_SIZE - 1],
						  (INT8U)Err_TASK_PRIO);		*/				
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
	Adc_Init();
	ElmoInit(CAN1);
	ElmoInit(CAN2);
	VelLoopCfg(CAN1,COLLECT_BALL_ID,50000,50000);
	PosLoopCfg(CAN1,PUSH_BALL_ID,1000000,1000000,400000);
	VelLoopCfg(CAN2,1,10000000,10000000);
	VelLoopCfg(CAN2,2,10000000,10000000);
	MotorOn(CAN1,PUSH_BALL_ID);
	MotorOn(CAN1,COLLECT_BALL_ID);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	VelCrl(CAN1,COLLECT_BALL_ID,600*4096); 
	/*一直等待定位系统初始化完成*/
	delay_s(2);
	WaitOpsPrepare();
	OSTaskSuspend(OS_PRIO_SELF);
}
float yawAngle=170,T=0.2,v=1500,angle,Distance;
int status,throwFlag=0,R=600;
extern float x,y;
extern int time,Cnt;
void WalkTask(void)
{
	static int flag=0,lastTime=0,errFlag=0,push_Ball_Count=0,yawAngleFlag,yawcompangle,errTime=0,statusFlag;
	static float lastX=0,dLeft,dRight,lastY,changeAngle,rps=50,Vx,Vy,V,shootAngle;
	do
	{
		//右ADC
		dRight=Get_Adc_Average(14,20)*0.92;
		//左ADC
		dLeft=Get_Adc_Average(15,20)*0.92;	
	}	
	while(dLeft>1000&&dRight>1000);
	if(dRight<=1000)
		status=0;
	else if(dLeft<=1000)
		status=1;
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
		if(errFlag==1)
		{	
			AnglePID(changeAngle,GetAngle());
			Straight(-1000);
			if(errTime%2==0&&statusFlag)
			{
				status=1-status;
				statusFlag=0;	
			}	
			if(Get_Time_Flag())	
				errFlag=0;
			time=0;
			throwFlag=0;
		}	
		else
		{
			throwFlag=1;
			CirclePID(0,2400,R,v,status);
			if(status==0)
			{					
				if(x>-200&&lastX<-200&&R<1500&&y<2400)		
					R+=500;
			}	
			else
				if(x<100&&lastX>100&&R<1500&y<2400)
					R+=500;
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
//		DistanceA=ReadLaserAValue()*2.4973+40;
//		DistanceB=ReadLaserBValue()*2.4973+360;	
		if(status==0)
		{
			if(x<=900&&y<1500)
			{	
				if(x>-900)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(2200,200);
				GetDistance(2200,200);
			}	
			if(y<=3300&&x>=900)
			{	
				if(y>1500)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(2200,4600);
				GetDistance(2200,4600);
			}	
			if(x>-900&&y>=3300)
			{	
				if(x<900)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(-2200,4600);
				GetDistance(-2200,4600);
			}
			if(y>1500&&x<-900)
			{	
				if(y<3300)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(-2200,200);
				GetDistance(-2200,200);
			}
		}
		else
		{
			if(x>=900&&y>1500)
			{
				if(y<3300)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(2200,200);
				GetDistance(2200,200);
			}	
			if(x<=900&&y>=3300)
			{
				if(x>-900)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(2200,4600);
				GetDistance(2200,4600);
			}	
			if(x<-900&&y<=3300)
			{	
				if(y>1500)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(-2200,4600);
				GetDistance(-2200,4600);
			}
			if(x>-900&&y<1500)
			{	
				if(x<900)
					throwFlag=1;
				else
					throwFlag=0;
				GetYawangle(-2200,200);
				GetDistance(-2200,200);
			}
		}
//		if(status==0)
			yawcompangle=(yawAngle-170)*pi/180;
//		else
//			yawcompangle=(-yawAngle+170)*pi/180;
		//圆筒方向速度
		Vx=cos(yawcompangle)*sqrtf(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY());
		//圆筒垂直方向速度
		Vy=sin(yawcompangle)*sqrtf(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY());
		//应给的速度
		V=-Vx+Distance*9800/(sqrtf(4*4900*(sqrt(3)*Distance-650)+3*Vx*Vx)-sqrt(3)*Vx);
		if(status==0)
			shootAngle=yawAngle+atan(Vy/V)*180/pi;
		else
			shootAngle=yawAngle+atan(Vy/V)*180/pi;
		YawPosCtrl(shootAngle);
		rps=(sqrtf(V*V+Vy*Vy)-168.94)/39.56+1;
		if(rps>80)
			rps=80;
		USART_OUT(UART4,(uint8_t *)"%d\r\n",(int)rps);
		ShooterVelCtrl(rps);
		if(R>1000)
		{
			if(throwFlag==1)
			{
				if(push_Ball_Count==500)
					PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);		
				if(push_Ball_Count==600)
				{
					PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);
					push_Ball_Count=0;
				}
			}
			else
				push_Ball_Count=495;
		}
		push_Ball_Count++;
		lastX=x;
		lastY=y;
		//USART_OUT(UART4,(uint8_t *)"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)GetAngle(),(int)GetX(),(int)GetY(),(int)rps,(int)GetSpeedX(),(int)GetSpeedY(),(int)Distance,(int)Vx,(int)Vy);
	}
}