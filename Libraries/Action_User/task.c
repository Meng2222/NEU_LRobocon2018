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
	PosLoopCfg(CAN1,PUSH_BALL_ID,500000,500000,50000);
	VelLoopCfg(CAN2,1,10000000,10000000);
	VelLoopCfg(CAN2,2,10000000,10000000);
	MotorOn(CAN1,PUSH_BALL_ID);
	MotorOn(CAN1,COLLECT_BALL_ID);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	/*一直等待定位系统初始化完成*/
	delay_s(2);
	WaitOpsPrepare();
	OSTaskSuspend(OS_PRIO_SELF);
}
float yawAngle=170;
int status;
extern float x,y;
extern int time,Cnt;
void WalkTask(void)
{
	static int flag=0,R=1600,lastTime=0,errFlag=0,push_Ball_Count=0,yawAngleFlag,yawcompangle,errTime=0,statusFlag;
	static float lastX=-100,dLeft,dRight,lastY=-100,changeAngle,rps=50,Vx,Vy,V,Distance,shootAngle;
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
		}	
		else
		{
			CirclePID(0,2400,R,1000,status);
			if(x<-0.5&&lastX>0.5&&R<1500)
				R+=500;
			if((x-lastX)*(x-lastX)+(y-lastY)*(y-lastY)>20)
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
		VelCrl(CAN1,COLLECT_BALL_ID,60*4096); 
		if(status==0)
		{
		if(x>=-400&&y<2000)
		{	GetYawangle(2200,200);
			Distance=sqrtf((x-2400)*(x-2400)+(y-0)*(y-0));
		}	
		if(y>=2000&&x>=400)
		{	GetYawangle(2200,4600);
			Distance=sqrtf((x-2400)*(x-2400)+(y-4800)*(y-4800));
		}	
		if(x<400&&y>=2800)
		{	GetYawangle(-2200,4600);
			Distance=sqrtf((x+2400)*(x+2400)+(y-4800)*(y-4800));
		}	
		if(y<2800&&x<-400)
		{	GetYawangle(-2200,200);
			Distance=sqrtf((x+2400)*(x+2400)+(y-0)*(y-0));
		}
		}
		else
		{
		if(x>=400&&y<2800)
		{	GetYawangle(2200,200);
			Distance=sqrtf((x-2400)*(x-2400)+(y-0)*(y-0));
		}	
		if(x>=-400&&y>=2800)
		{	GetYawangle(2200,4600);
			Distance=sqrtf((x-2400)*(x-2400)+(y-4800)*(y-4800));
		}	
		if(x<-400&&y>=2000)
		{	GetYawangle(-2200,4600);
			Distance=sqrtf((x+2400)*(x+2400)+(y-4800)*(y-4800));
		}	
		if(x<400&&y<2000)
		{	GetYawangle(-2200,200);
			Distance=sqrtf((x+2400)*(x+2400)+(y-0)*(y-0));
		}
		}
		if(status==0)
			yawcompangle=(yawAngle-170)*pi/180;
		else
			yawcompangle=(-yawAngle+170)*pi/180;
		//圆筒方向速度
		Vx=cos(yawcompangle)*sqrtf(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY());
		//圆筒垂直方向速度
		Vy=sin(yawcompangle)*sqrtf(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY());
		//应给的速度
		V=-Vx+(Distance-200)*9800/(sqrtf(4*4900*(sqrt(3)*(Distance-200)-650)+3*Vx*Vx)-sqrt(3)*Vx);
		if(status==0)
			shootAngle=yawAngle+atan(Vy/(V+Vx))*180/pi;
		else
			shootAngle=yawAngle-atan(Vy/(V+Vx))*180/pi;
		YawPosCtrl(shootAngle);
		rps=(sqrtf((V+Vx)*(V+Vx)+Vy*Vy)-104.8)/39.507;
		if(rps>100)
			rps=50;
		ShooterVelCtrl(rps);
		if(R>1500)
		{
			// 推球
//			if(x*lastX<0||(y-2400)*(lastY-2400)<0)
			if(push_Ball_Count==50)
			{	PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);		
				flag=1;
			}		
			// 复位
			if(push_Ball_Count==100)
			{	
				PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);
				push_Ball_Count=0;
				flag=0;
			}
		}
//		if(flag)
			push_Ball_Count++;
		lastX=x;
		lastY=y;
		USART_OUT(UART4,(uint8_t *)"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)GetAngle(),(int)GetX(),(int)GetY(),(int)rps,(int)GetSpeedX(),(int)GetSpeedY(),(int)Distance,(int)Vx,(int)Vy);
	}
}