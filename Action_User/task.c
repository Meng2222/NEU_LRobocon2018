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
#include "movebase.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "status.h"

#define Pulse2mm COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi)

//void GetAngPID();
//void GetDirPID();


/*
一个脉冲是4096/(120*Pi)
定义输入速度mm/s和半径mm
*/
float ratio1,ratio2;
void vel_radious(float vel,float radious)
{
	ratio1=(radious+WHEEL_TREAD/2)/radious;
	ratio2=(radious-WHEEL_TREAD/2)/radious;
	VelCrl(CAN2,1,ratio1*vel*Pulse2mm);
	VelCrl(CAN2,2,-ratio2*vel*Pulse2mm);
}


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

//	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);//can1初始化
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);//can2初始化
	USART3_Init(115200);
	UART4_Init(921600);
	TIM_Init(TIM2,10*1000-1,83,0x01,0x03);
	ElmoInit(CAN2);//驱动初始化
	VelLoopCfg(CAN2,1,20000,50000);//速度环初始化
	VelLoopCfg(CAN2,2,20000,50000);
//	SetVelLimit(CAN2,0x001,80000,-80000);//配置驱动速度限制
//	SetVelLimit(CAN2,0x002,80000,-80000);
//	PosLoopCfg(CAN2,0x001,2000,2000,40000);//位置环初始化
//	PosLoopCfg(CAN2,0x002,2000,2000,40000);
//	SetPosLimit(CAN2,0x001,80000,-80000);//配置驱动位置限制
//	SetPosLimit(CAN2,0x002,80000,-80000);
//	SetPosCountingRange(CAN2,0 ,200000,-200000);//配置位置记录范围
//	SetUnitMode(CAN2,0x001, SPEED_CONTROL_MODE);//配置驱动器为速度控制模式
//	SetUnitMode(CAN2,0x002, SPEED_CONTROL_MODE);
//	SetSmoothFactor(CAN2,0x001,100);//设置驱动器平滑输出
	MotorOn(CAN2,1);//电机初始化
	MotorOn(CAN2,2);
	delay_s(3);
	#if	Car==1
	driveGyro();
	delay_s(10);
		delay_s(5);

	#elif Car==4
	delay_s(12);//车4
	#endif
  OSTaskSuspend(OS_PRIO_SELF);
	
}


#define V 1
void WalkTask(void)
{
	float angle=0;
	CPU_INT08U os_err;
	os_err = os_err;

	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
//		switch(AngleChange())
//		{
//			case 0:	line(1,0,-30,1,V);break;
//			case 1:	line(0,1,-1970,1,V);break;
//			case 2:	line(1,0,-1970,2,V);break;
//			case 3:	line(0,1,-30,2,V);break;
//			
//		}
////		line(0,1,-1800,1,V);
//		angle=GetAngle();
		surround(2000,1000,500,0.5,1);
	}
}

