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
#include "pid.h"
#include "adc.h"
#include "pps.h"
#include "gun.h"
#include "fort.h"
/*
===============================================================
						信号量定义
===============================================================
*/
extern char updownFlag;
extern char pposokflag;
extern char updownFlag;//在线左右标志
extern float setAngle;//直线设置角度
extern float setAngle_R;//圆形设置角度
extern float errorRadius;//圆心距离;
extern float errorAngle,errorDis;//等于位置环输出角度-实际角度，位置偏差
extern float phase1,phase2,uOutAngle,uOutDis;//角度PID输出，位置PID输出
extern float Uout;//PID计算的输出值

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
	UART4_Init(921600);
	USART1_Init(115200);
	UART5_Init(921600);
	USART3_Init(115200);
	Adc_Init();
	
	TIM_Init(TIM2, 99, 839, 1, 0);
	TIM_Init(TIM2, 1000-1, 84-1, 0x01, 0x03); //定时器初始化1ms
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	ElmoInit(CAN1);
	VelLoopCfg(CAN2, 1, 500000, 500000);
	VelLoopCfg(CAN2, 2, 500000, 500000);
	//棍子收球电机 配置速度环
	VelLoopCfg(CAN1,COLLECT_BALL_ID, 50000, 50000);
	//推球电机 配置位置环
	PosLoopCfg(CAN1, PUSH_BALL_ID, 50000,50000,20000);
	//航向电机配置位置环
	PosLoopCfg(CAN1, GUN_YAW_ID, 50000,50000,20000);
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	//棍子收球电机
	MotorOn(CAN1, COLLECT_BALL_ID);
	//推球电机
	MotorOn(CAN1, PUSH_BALL_ID);
	//航向电机
	MotorOn(CAN1, GUN_YAW_ID);
	delay_ms(2000);
	/*一直等待定位系统初始化完成*/
	BEEP_ON;
	WaitOpsPrepare();
	OSTaskSuspend(OS_PRIO_SELF);
}
extern FortType fort;
extern num_t u_Num;
void WalkTask(void)
{	
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		//USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t\r\n",(int)GetAngle(),(int)GetX(),(int)GetY());
		WalkWholeRound();
	}
}


		//控制发射枪电机转速
//		ShooterVelCtrl(10);		
		//// 控制电机的转速，脉冲。OK
//		VelCrl(CAN1,COLLECT_BALL_ID,0*4096); 		
		//推球电机,配置位置环 OK
		// 推球
//		PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);
//		// 复位
//		PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);		
		//控制航向电机
//		YawPosCtrl(90);
