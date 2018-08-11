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
OS_EVENT *BluetoothSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
static OS_STK BluetoothTaskStk[Bluetooth_TASK_STK_SIZE];

void App_Task()
{
	CPU_INT08U os_err;
	os_err = os_err; /*防止警告...*/

	/*创建信号量*/
	PeriodSem = OSSemCreate(0);
	BluetoothSem = OSSemCreate(0);
	/*创建任务*/
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, /*初始化任务*/
						  (void *)0,
						  (OS_STK *)&App_ConfigStk[Config_TASK_START_STK_SIZE - 1],
						  (INT8U)Config_TASK_START_PRIO);

	os_err = OSTaskCreate((void (*)(void *))WalkTask,
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  (INT8U)Walk_TASK_PRIO);
	os_err = OSTaskCreate((void (*)(void *))BluetoothTask,
						  (void *)0,
						  (OS_STK *)&BluetoothTaskStk[Bluetooth_TASK_STK_SIZE - 1],
						  (INT8U)Bluetooth_TASK_PRIO);
						  
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	TIM_Init(TIM2, 100, 840, 0x01, 0x03);
	USART3_Init(115200);
	UART4_Init(921600);	//蓝牙串口
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2, 1, 8000, 8000);
	VelLoopCfg(CAN2, 2, 8000, 8000);
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	delay_ms(8000);	//延时8s 用于启动定位系统
	OSTaskSuspend(OS_PRIO_SELF);
}

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		RectangleAround(2000, 2000, 750);
	}
}

void BluetoothTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(BluetoothSem, 0, &os_err);
	while (1)
	{
		OSSemPend(BluetoothSem, 0, &os_err);
		
	}
}