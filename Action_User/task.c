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
#include "void.h"
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
	
	USART3_Init(115200);
	//	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);//can1初始化
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	USART3_Init(115200);
	UART4_Init(921600);
	TIM_Init(TIM2,10*1000-1,83,0x01,0x03);
	Adc_Init();
	ElmoInit(CAN2);//驱动初始化
	VelLoopCfg(CAN2,1,2000,2000);//速度环初始化
	VelLoopCfg(CAN2,2,2000,2000);
	MotorOn(CAN2,1);//电机初始化
	MotorOn(CAN2,2);
	/*一直等待定位系统初始化完成*/
	delay_s(2);
	WaitOpsPrepare();
	
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
		Walkline(0,0,Radius(),AdcFlag(),0.7);   ////setx  sety  r  方向  速度
		errdeal();
//////////////////发数测试////////////////////////////		
// 	USART_OUT(UART4,(uint8_t*) "%d	%d	%d\r\n",(int)(GetX()),(int)(GetY()),Radius());
           //////////////////////////test/////////////////////////////
	}
}
