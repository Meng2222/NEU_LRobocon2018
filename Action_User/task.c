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

#define Pulse2mm COUNTS_PER_ROUND/(WHEEL_DIAMETER*PI)

#define CAR 4
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
	int32_t cnt=0;

	PidPara(15,0,2.0);
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{

		OSSemPend(PeriodSem, 0, &os_err);

		pos_Angle=GetAngle();
		pos_X=GetPosX();
		pos_Y=GetPosY();
		Square();
		USART_OUT(UART4, "%d\t", (int)pos_X);
		USART_OUT(UART4, "%d\r\n", (int)pos_Y);
		
		
		
	}
}


//初始化函数
void Init(void)
{
	TIM_Init(TIM2, 999, 84, 0x01, 0x03);
	USART3_Init(115200);
	UART4_Init(921600);
//	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2, 0x01, 2000, 2000);
	VelLoopCfg(CAN2, 0x02, 2000, 2000);
	MotorOn(CAN2, 0x01);
	MotorOn(CAN2, 0x02);
	delay_ms(3000);
	delay_ms(10000);
//	CarOne();


	
}


//车1定位系统初始化
uint8_t isOKFlag=0;
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
	isOKFlag=0;
	

}
