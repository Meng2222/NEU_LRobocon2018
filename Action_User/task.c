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

#define Pulse2mm COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi)

//typedef struct
//{
//	float x;
//	float y;
//	float angle;
//} Pos;


void Compute(void);
void SetSampleTime(int NewSampleTime);
void SetTuning(float Kp,float Ki,float Kd);
/*
一个脉冲是4096/(120*Pi)
定义输入速度mm/s和半径mm
*/

unsigned long lastTime;
float Input,Output,Setpoint=0;
float errSum,lastInput,lasterror;
float Kp,Ki,Kd;
int SampleTime=1000;
extern float angle,xpos,ypos;
//Pos carpos;


float ratio1,ratio2;
void vel_radious(float vel,float radious)
{
	ratio1=(radious+WHEEL_TREAD/2)/radious;
	ratio2=(radious-WHEEL_TREAD/2)/radious;
	VelCrl(CAN2,1,ratio1*vel*Pulse2mm);
	VelCrl(CAN2,2,-ratio2*vel*Pulse2mm);
}

void walk_stragiht(void)
{
	VelCrl(CAN2,1,-2173);				//2173pulse/s，即200mm/s
	VelCrl(CAN2,2,2173);
}

void walk_around(void)
{
	vel_radious(200,WHEEL_TREAD/2);		//以小车右轮为原点，中心到右轮距离为半径即R=L/2	
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
	
	TIM_Init(TIM2,1000-1,84-1,1,3);	//产生10ms中断，抢占优先级为1，响应优先级为3
	UART4_Init(921600);
	USART3_Init(115200);
	
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	
	VelLoopCfg(CAN2,1, 5000, 5000);				//驱动器速度环初始化
	VelLoopCfg(CAN2,2, 5000, 5000);
	
	ElmoInit(CAN2);								//驱动器初始化
	MotorOn(CAN2,1);							//电机使能（通电）
	MotorOn(CAN2,2);
	
	delay_s(10);								//等待10s挂起
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
		USART_OUT(UART4,(uint8_t*)"angle=%d\t\t",(int)GetAngle());
		USART_OUT(UART4,(uint8_t*)"xpos=%d\t\t",(int)GetXpos());
		USART_OUT(UART4,(uint8_t*)"ypos=%d\r\n",(int)GetYpos());
//		vel_radious(500.0,500.0);			//半径为0.5m，速度为0.5m/s

//		delay_ms(5000);

	}
}

void Compute(void)				//PID控制
{
	Input=GetAngle();
	float error = Setpoint - Input;
	errSum+=error;
	float dErr=error-lasterror;
	Output=Kp*error+Ki*errSum+Kd*dErr;
	lasterror=error;
}
void SetTuning(float Kp,float Ki,float Kd)
{
	double SampleTimeInSec=((double)SampleTime)/1000;
	Kp=Kp;
	Ki=Ki*SampleTimeInSec;
	Kd=Kd*SampleTimeInSec;
}

