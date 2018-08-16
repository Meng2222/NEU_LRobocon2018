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
#include "math.h"

#if CAR_NUM==1
extern int isOKFlag,beginFlag;
int IsSendOK(void)
{
	return isOKFlag;
}	
void SetOKFlagZero(void)
{
	isOKFlag=0;
}	
void driveGyro(void)
{
	while(!IsSendOK())
	{
		delay_ms(5);
		USART_SendData(USART3,'A');
		USART_SendData(USART3,'T');
		USART_SendData(USART3,'\r');
		USART_SendData(USART3,'\n');
	}	
	SetOKFlagZero();
}	
#endif
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
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,1,10000,10000);
	//PosLoopCfg(CAN2,1,1000,0,4000);
	VelLoopCfg(CAN2,2,10000,10000);
	//PosLoopCfg(CAN2,1,1000,0,4000);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	#if CAR_NUM==1
	delay_s(2);
	driveGyro();
	while(!beginFlag);
	#elif CAR_NUM==4
	delay_s(10);
	delay_s(5);
	#endif
	OSTaskSuspend(OS_PRIO_SELF);
}

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem,0,&os_err);
		linePID(0,1000,0,0,1000);
		USART_OUT(UART4,(uint8_t *)"  %d\t%d\t%d\r\n",(int)GetAngle(),(int)GetXpos(),(int)GetYpos());
	}
}
