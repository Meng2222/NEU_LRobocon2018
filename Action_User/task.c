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
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	TIM_Init(TIM2,999,83,0,3);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,1,8000,8000);
	VelLoopCfg(CAN2,2,8000,8000);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	OSTaskSuspend(OS_PRIO_SELF);
}
void WalkStright(float mult)
{
	float speed;
	speed=4096.f*mult/(WHEEL_DIAMETER*PI);
	VelCrl(CAN2,1,speed);
	VelCrl(CAN2,2,-speed);
}
void WalkAround(float mult,float radius)
{
	float speed1,speed2,buff1,buff2;
	buff1=(radius+WHEEL_TREAD/2)*mult/radius;
	buff2=(radius-WHEEL_TREAD/2)*mult/radius;
	speed1=COUNTS_PER_ROUND*buff1/(WHEEL_DIAMETER*PI);
	speed2=COUNTS_PER_ROUND*buff2/(WHEEL_DIAMETER*PI);
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
        WalkAround(200,500);
	}
}
