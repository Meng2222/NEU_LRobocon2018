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

/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
static OS_STK Walk_VirerStk[Walk_Virer_TASK_STK_SIZE];

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
	os_err = OSTaskCreate((void (*)(void *))Walk_Virer,
						  (void *)0,
						  (OS_STK *)&Walk_VirerStk[Walk_Virer_TASK_STK_SIZE - 1],
						  (INT8U)Walk_Virer_TASK_PRIO);
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
	TIM_Init(TIM2,999,83,1,0);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,1,1,1);
	VelLoopCfg(CAN2,2,1,1);
	VelCrl(CAN2,1,1);
	VelCrl(CAN2,2,1);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	OSTaskSuspend(OS_PRIO_SELF);
}
float speed1,speed2;
void  Walk_Virer(int radius,int multiple)
{
	speed1=(radius+WHEEL_TREAD/2)*multiple;
	speed2=(radius-WHEEL_TREAD/2)*multiple;
	VelCrl(CAN2,1,speed1);
	VelCrl(CAN2,2,-speed2);
}
void WalkTask(void)
{

	CPU_INT08U os_err;
	os_err = os_err;

	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		Walk_Virer(500,20);
	}
}
