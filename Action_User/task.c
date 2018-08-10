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
#define ACW 0
#define CW 1

/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
/*
===============================================================
							直行
					入口参数：速度,单位mm/s
===============================================================
*/
void WalkStraight(int v)                             
{
	VelCrl(CAN2,1,(4096/378)*v);
	VelCrl(CAN2,2,-((4096/378)*v));
}
/*
===============================================================
						    画圈
入口参数;               前进方向  ACW  or  CW;
                       轴中间的速度 v 单位mm/s
                        圈的半径  r   单位mm
===============================================================
*/
void WalkRound(u8 direction, int v,int r)           //画圈，入口参数
{
	int w = v/r;
	if(direction == CW)
	{
		VelCrl(CAN2,1,(4096/378)*(w*(r-217)));
		VelCrl(CAN2,2,-((4096/378)*(w*(r+217))));
	}
	else
	{
		VelCrl(CAN2,1,(4096/378)*(w*(r+217)));
		VelCrl(CAN2,2,-((4096/378)*(w*(r-217))));
	}
}

void cycle(u8 direction, int v ,int r)
{
	
}
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                  //系统中断优先级分组2
	TIM_Init(TIM2,999,83,0,0);                                       //时钟2初始化，1ms周期
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);                //can1初始化
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);                //can2初始化
	
	ElmoInit(CAN2);                                                  //驱动器初始化
	VelLoopCfg(CAN2,2,40000000,40000000);                            //左电机速度环初始化
	VelLoopCfg(CAN2,1,40000000,40000000);                            //右电机速度环初始化
	MotorOn(CAN2,1);                                                 //右电机使能
	MotorOn(CAN2,2);                                                 //左电机使能
//	MotorOff(CAN2,2);                                                //左电机失能
//	MotorOff(CAN2,1);                                                //右电机失能
	
	OSTaskSuspend(OS_PRIO_SELF);                                     //挂起初始化函数
}

/*
===============================================================
                   WalkTask      初始化后执行
===============================================================
*/
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;                                                 //防报错
	OSSemSet(PeriodSem, 0, &os_err);                                 //信号量归零
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);                            //等信号量，10ms一次
		WalkRound(ACW,1000,1000);                                    //转圈，逆时针，速度1000mm/s,半径1000mm
	}
}



