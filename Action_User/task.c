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
	USART1_Init(115200);
	USART3_Init(115200);
	UART4_Init(921600);
	Adc_Init();
	ElmoInit(CAN2);
	VelLoopCfg(CAN1, 8, 50000, 50000);
	VelLoopCfg(CAN2,1,50000000,50000000);
	VelLoopCfg(CAN2,2,50000000,50000000);
	// 配置位置环
	PosLoopCfg(CAN1, PUSH_BALL_ID, 50000,50000,20000);
	// 推球
	PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);
	// 复位
	PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	/*一直等待定位系统初始化完成*/
	delay_s(2);
	WaitOpsPrepare();
	OSTaskSuspend(12);
	OSTaskSuspend(OS_PRIO_SELF);
}
float xRepair,changeAngle;
int errFlag=0;
extern int Cnt,time;
void WalkTask(void)
{
	static int flag=0,R=2100,status;
	static float lastX=-100,dLeft,dRight,lastY=-100;
	extern float x,y;
	do
	{
		xRepair=dLeft-dRight;
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
			if(Get_Time_Flag())
			{	
				errFlag=0;
				if(x<0&&lastX>0&&R>600)
					R-=450;
				if(x<0&&lastX>0&&R<800)
					R+=450;
			}	
			time=0;
		}	
		else
		{
			CirclePID(0,2400,R,1000,status);
			if(x<0&&lastX>0&&R>600)
				R-=450;
			if(x<0&&lastX>0&&R<800)
				R+=450;
		}	
		if(time>=100)flag=1;
		else flag=0;
		if(time>20000)time=100;
		if(flag&&(x-lastX)*(x-lastX)+(y-lastY)*(y-lastY)<20)			
		{	
			errFlag=1;
			changeAngle=GetAngle()+45;
			Cnt=0;
		}	
		lastX=x;
		lastY=y;	
		USART_OUT(UART4,(uint8_t *)"%d\t%d\t%d\r\n",(int)GetAngle(),(int)GetX(),(int)GetY());
	}
}