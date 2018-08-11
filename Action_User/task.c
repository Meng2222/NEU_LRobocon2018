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
/*           车的基本信息         */
#define COUNT_PER_ROUND (4096.0f)  //电机旋转一周的脉冲数 
#define WHEEL_DIAMETER (120.0f)    //轮子直径(mm)
#define ROBOT_LENGTH (492.0f)      //调试小车车长(mm)
#define ROBOT_WIDTH (490.0f)       //调试小车车宽(mm)
#define WHEEL_WIDTH (40.0f)        //轮子宽度(mm)
#define WHEEL_TREAD (434.0f)       //两个轮子的中心距离(mm)
#define R (1000.0f)                //单位 mm
#define VEL (4096.0f)              //r/s


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
						  2);

	os_err = OSTaskCreate((void (*)(void *))WalkTask,
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  3);
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
	TIM_Init(TIM2, 1000-1, 83, 0, 2);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,1,1000,0);
	VelLoopCfg(CAN2,2,1000,0);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	
	OSTaskSuspend(OS_PRIO_SELF);
}

void WalkTask(void)
{
	float value(float r);
	float VEL_1;
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
			VEL_1=VEL*value(R);
			VelCrl(CAN2,1,VEL_1);
			VelCrl(CAN2,2,-VEL);
	}
}
float value(float r)
{
	float value,r1,r2;
	r1=r+(WHEEL_TREAD/2);
	r2=r-(WHEEL_TREAD/2)-(ROBOT_LENGTH/2);
	value=r1/r2;
	return value;
}

	
	
