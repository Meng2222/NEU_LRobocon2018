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
#include "moveBase.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"

/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];

uint8_t isOKFlag;

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
	Init();
	OSTaskSuspend(OS_PRIO_SELF);
}


//5ms 运行一次；
void WalkTask(void)
{

	CPU_INT08U os_err;
	os_err = os_err;

	float pos_Angle;
	float pos_X;
	float pos_Y;
	
	uint16_t angle;
	uint16_t x;
	uint16_t y;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		pos_Angle=GetAngle();
		pos_X=GetPosX();
		pos_Y=GetPosY();
		angle=pos_Angle*1000;
		x=pos_X*1000;
		y=pos_Y*1000;

		USART_OUT(UART4, "%d\t",angle);
		USART_OUT(UART4, "%d\t",x);
		USART_OUT(UART4, "%d\r\n",y);
	}
}


//初始化函数
void Init(void)
{
	TIM_Init(TIM2, 999, 84, 0x01, 0x03);
//	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
//	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	USART3_Init(115200);
	UART4_Init(921600);
	CarOne();
//	delay_ms(10000);
//	ElmoInit(CAN2);
//	
//	VelLoopCfg(CAN2, 0x01, 2000, 2000);
//	VelLoopCfg(CAN2, 0x02, 2000, 2000);
//	
//	MotorOn(CAN2, 0x01);
//	MotorOn(CAN2, 0x02);
}


//车1定位系统初始化
void CarOne(void)
{
	while(!isOKFlag)
	{
		delay_ms(5);
		USART_SendData(USART3,'A');
		USART_SendData(USART3,'T');
		USART_SendData(USART3,'\r');
		USART_SendData(USART3,'\n');
	}
	USART_SendData(UART4, 'o');
	USART_SendData(UART4, 'k');
	USART_SendData(UART4, '\r');
	USART_SendData(UART4, '\n');
	
	isOKFlag=0;
}
