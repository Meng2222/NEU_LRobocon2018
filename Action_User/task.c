#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "pid.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "moveBase.h"
#include "adc.h"
#include "pps.h"
#include "fort.h"
#include <math.h>
extern FortType fort;
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
//电机相关初始化
void Motor_Init(void)
{
	ElmoInit(CAN1);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,1,20000,20000);  //配置两个差速电机速度环
	VelLoopCfg(CAN2,2,20000,20000);
	VelLoopCfg(CAN1,8,50000,50000);	// 配置棍子收球电机速度环
    PosLoopCfg(CAN1, PUSH_BALL_ID, 500000,500000,200000);// 配置推球电机位置环
	PosLoopCfg(CAN1, GUN_YAW_ID, 50000,50000,20000);// 配置航向电机位置环
	MotorOn(CAN2,1);   //两个差速电机使能
	MotorOn(CAN2,2);
	MotorOn(CAN1,8);   //棍子收球电机使能
	MotorOn(CAN1,6);   //推球电机使能
	MotorOn(CAN1,7);   //航向电机使能
}
//新车电机初始化
void Motor_InitNew(void)
{
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,5,10000000,10000000);  //配置前后轮电机速度环
	VelLoopCfg(CAN2,6,10000000,10000000);
	MotorOn(CAN2,5);   //后轮电机使能
	MotorOn(CAN2,6);   //前轮转向电机使能
}
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	
	TIM_Init(TIM2,999,83,0,0);
	
	USART3_Init(115200);   //定位器串口初始化
	UART5_Init(921600);     //电机数据收发串口初始化
	
	#if  CARNUM == 1
	UART4_Init(921600);    //蓝牙串口初始化
	#elif CARNUM == 2
	USART1_Init(921600);    //新车蓝牙串口初始化
	#endif
	
	BEEP_ON;
	delay_ms(2000);
	
	/*一直等待定位系统初始化完成*/
	WaitOpsPrepare();
	
	Adc_Init();
	#if  CARNUM == 1
	Motor_Init();      //旧电机电机初始化
	#elif CARNUM == 2
	Motor_InitNew();     //新车电机初始化
    #endif
	OSTaskSuspend(OS_PRIO_SELF);
}

/*
   ===============================================================
   主函数
   ===============================================================
*/
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);                                        
//        VelCrl(CAN1,COLLECT_BALL_ID,40*4096);                       // 控制棍子收球电机的转速，脉冲。
        walkRound();         //走形程序
	   
   }		
}

