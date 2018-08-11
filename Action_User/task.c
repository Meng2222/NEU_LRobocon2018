#include "includes.h"
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
#define PAI 3.14


/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
int cnt=0;
static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
/*
==================================================================================
						自定义添加函数
*/
void  go_straight(float V,int ElmoNum1,int ElmoNum2 ,CAN_TypeDef* CANx);
void  go_round(float V,float R,int ElmoNum1,int ElmoNum2 ,CAN_TypeDef* CANx);
float straight(float setValue,float feedbackValue);
float turn(float setValue,float feedbackValue);
/*
==================================================================================
*/
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
}

/*
   ===============================================================
   初始化任务
   ===============================================================
   */
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM_Init(TIM2, 999, 83, 0x01, 0x03);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	//驱动器初始化
	ElmoInit( CAN2);
	//速度环和位置环初始化
	//右轮
	VelLoopCfg(CAN2, 1, 100, 100);
	PosLoopCfg(CAN2, 1, 100, 100,4095);
	//左轮
	VelLoopCfg(CAN2, 2, 100, 100);
	PosLoopCfg(CAN2, 2, 100, 100,4095);
	//电机使能
	MotorOn(CAN2, 01);
	MotorOn(CAN2, 02);
	OSTaskSuspend(OS_PRIO_SELF);
	//定位系统串口初始化
     USART6_Init(115200);
    delay_s(10);

	
}
int mission=0;

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		//直行
		if(mission==1&&action.angle<=1&&action.angle>=-1)
		{
			straight(2,action.x);
		}
		if(mission==2&&action.angle<=91&&action.angle>=89)
		{
			straight(2,action.x);
		}
		if(mission==3&&action.angle<=180&&action.angle<=-1)
		{
			straight(2,action.x);
		}
		if(mission==4&&action.angle>=-91&&action.angle<=-89)
		{
			straight(2,action.x);
		}
		//转向
		if(action.x==2&&mission==1)
		{
			turn(90,action.angle);
		}
		if(action.x==2&&mission==2)
		{
			turn(180,action.angle);
		}
		if(action.x==2&&mission==3)
		{
			turn(0,action.angle);
		}
		if(action.x==2&&mission==4)
		{
			turn(0,action.angle);
		}

		
		
		
	}
}






