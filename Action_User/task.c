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
extern float angle;
extern float x;
extern float y;

/*
===============================================================
						信号量定义
===============================================================
*/
#define Kp 10.0f
#define Ki	1.0f
#define Kd	1.0f
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];



static void ConfigTask(void);
static void  WalkTask(void);

	
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
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	UART4_Init(921600);
	USART3_Init(115200);
	TIM_Init(TIM2,1000-1,84-1,0x00,0x01);
	VelLoopCfg(CAN2,1,4000,0);
	VelLoopCfg(CAN2,2,4000,0);
	
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	ElmoInit(CAN2);
//	IsSendOK();
	
	OSTaskSuspend(OS_PRIO_SELF);
}
//unsigned long lasttime;
//double input,output,setpoint;
//double errsum,lasterr;
//void comput()
//{
//double timeChange = (double)(now -
//lasttime);
// input =GetAngle();

// double error=setpoint-input;
// errsum += (error * timeChange);
//double derr=(error-lasterr)/timeChange;
//output=KP*error+KI*errsum+KD*derr;

//}
void  Walk_Straight(float speed1,float speed2)
{
	VelCrl(CAN2,1,speed1);
	VelCrl(CAN2,2,-speed2);
}
float Angle_Pid(float err)
{
	static float Iterm=0;
	float Dterm=0,errlast=0,Uk=0;
	Iterm +=Ki*err;
	Dterm=Kd*(err-errlast);
	errlast=err;
	Uk=Kp*err+Iterm+Dterm;
	return Uk;
}

	
void WalkTask(void)
{
	
	CPU_INT08U os_err;
	os_err = os_err;
	delay_s(10);
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{	
		
		OSSemPend(PeriodSem, 0, &os_err);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetAngle());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetXpos());
		USART_OUT(UART4,(uint8_t*)"%d\r\n",(int)GetYpos());
		Walk_Straight(3000-Angle_Pid(GetAngle()),3000+Angle_Pid(GetAngle()));
		
		
	}
}
