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



extern char isOKFlag;
extern char pposokflag;
extern double Input1,Output1,Output2;
extern float Angle_qie;
extern float Angle_w;

void driveGyro(void)
{
	while(!isOKFlag)
	{
		delay_ms(5);
		USART_SendData(USART3,'A');
		USART_SendData(USART3,'T');
		USART_SendData(USART3,'\r');
		USART_SendData(USART3,'\n');
	}
	isOKFlag = 0;
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
	TIM_Init(TIM2, 1000-1, 84-1, 0x00, 0x00);
	USART3_Init(115200);
	UART4_Init(921600);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);			
	ElmoInit(CAN2);		
	MotorOn(CAN2,1);			
	MotorOn(CAN2,2);
	VelLoopCfg(CAN2,1,1000,1000);
	VelLoopCfg(CAN2,2,1000,1000);
	delay_s(2);
	
	#if CARNUM == 1
	driveGyro();
	while(!pposokflag);
	#elif CARNUM == 4
	delay_s(10);
	#endif
	OSTaskSuspend(OS_PRIO_SELF);
}



void WalkTask(void)
{

	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	SetTunings1(70,0,0);
	SetTunings2(0.1,0.5*0.01,0);
	int state=1;
	while (1)
	{
		OSSemPend(PeriodSem,0,&os_err);
		
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)Input1);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)Getposx());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)Getposy());
		
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)Angle_qie);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)Angle_w);
		
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)Output1);		
		USART_OUT(UART4,(uint8_t*)"%d\r\n",(int)Output2);
		
		Double_closed_loop(500,0,500,SPEED,1);
		
		switch(state)
		{
			case 1:
					Double_closed_loop(2400,2400,2000,SPEED,1);
					
					
			
			
			
		}
		
		
	}
	
}








