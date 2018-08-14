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
#define Kp 250
#define Ki 0
#define Kd 0
float u,setAng;
int count;
extern char isOKFlag;
extern char pposokflag;
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
void Motor_Init(void)
{
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,1,8000,8000);
	VelLoopCfg(CAN2,2,8000,8000);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
}
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	
	TIM_Init(TIM2,999,83,0,0);
	
	USART3_Init(115200);   //定位器串口
	UART4_Init(921600);    //蓝牙串口
    
	Motor_Init();
	delay_s(2);
	#if CARNUM == 1
	driveGyro();
	while(!pposokflag);
	#elif CARNUM == 4
	delay_s(10);
	#endif
	
	OSTaskSuspend(OS_PRIO_SELF);
}
int Uk(float err)
{
	static float lastErr = 0;
	static float sumErr = 0;
	sumErr += err;
	u= Kp * err + Ki * sumErr +Kd *(err - lastErr);
	lastErr = err;
}
void WalkStright(float mult)//直线走
{
	float errAng;
	errAng=GetAngle()-setAng;
	if(errAng>180)
		errAng=errAng-360;
	if(errAng<-180)
		errAng=errAng+360;
	Uk(errAng);
	VelCrl(CAN2,1,4096.f*mult);
	VelCrl(CAN2,2,-u-4096.f*mult);
}
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetAngle());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetXpos());
		USART_OUT(UART4,(uint8_t*)"%d\r\n",(int)GetYpos());
		switch(count)
		{
		 case 0:
		     setAng=0;
		    WalkStright(1);
		 if(GetYpos()>1750)
			 count=1;
		 break;
		 case 1:
		     setAng=-90;
		    WalkStright(1);
		 if(GetXpos()>1750)
			 count=2;
		 break;
		 case 2:
		     setAng=-180;
		    WalkStright(1);
		 if(GetYpos()<250)
			 count=3;
		 break;
		 case 3:
		     setAng=90;
		    WalkStright(1);
		 if(GetXpos()<250)
			 count=0;
		 break;
	   }	
	}
}
