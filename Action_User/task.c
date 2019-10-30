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
#include "pps.h"
#include "fort.h"
#include  "movebase.h" 


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
void MotorInit(void);

void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	delay_ms(500);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	USART1_Init(921600);//蓝牙
	USART3_Init(115200);//定位系统
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5, GPIO_Pin_6);
	TIM_Init(TIM2, 99, 839, 1, 0);
	/*一直等待定位系统初始化完成*/
	delay_ms(500);
	MotorInit();
	BEEP_ON;
	WaitOpsPrepare();
	OSTaskSuspend(OS_PRIO_SELF);
}

void MotorInit(void)
{
	ElmoInit(CAN2);
	
	VelLoopCfg(CAN2, 1, 2000 * 10, 2000 * 10);
	VelLoopCfg(CAN2, 2, 2000 * 10, 2000 * 10);
	VelLoopCfg(CAN2, 3, 2000 * 10, 2000 * 10);
	
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	MotorOn(CAN2, 3);
	
	VelCrl(CAN2,1,0);
	VelCrl(CAN2,2,0);
	VelCrl(CAN2,3,0);
}

extern FortType fort;

void WalkTask(void)
{

	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
//		MotorOff(CAN2,1);
//		MotorOff(CAN2,2);
//		MotorOff(CAN2,3);

//		OpenLoopLine(vell * 2000 * (91/6), 0 );
//		OpenLoopCircle(1 * 2000 * (91/6),0.0f,1000.0f);
//		CloseLoopLine(0.5,45,0, GetAngle(), GetX(),GetY(),100.0f,100.0f);
//		CloseLoopCircle(0.5,1000.0f,0.0f,90.0f, 89.0f,0.0f,GetAngle(),800.0f);		
		
		ReadActualVel(CAN2, 1);
		ReadActualVel(CAN2, 2);
		ReadActualVel(CAN2, 3);
		
		USART_OUT(USART1,(uint8_t*)"%d\t%d\t%d\t",wheel1Speed,wheel2Speed,wheel3Speed);
		USART_OUT(USART1,(uint8_t*)"X=\t%d\tY=\t%d\tdis=\t%d\t%d\tAng=\t%d\r\n",\
		(int)ppsReturn.ppsX,(int)ppsReturn.ppsY,(int)sqrt(ppsReturn.ppsX * ppsReturn.ppsX + ppsReturn.ppsY * ppsReturn.ppsY),\
			(int)sqrt(ppsReturn.ppsSpeedX * ppsReturn.ppsSpeedX + ppsReturn.ppsSpeedY * ppsReturn.ppsSpeedY),(int)ppsReturn.ppsAngle);
	}
}


