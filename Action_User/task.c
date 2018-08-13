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
#define long 100
void controltype1(void);
void controltype2(void);
void controlrightv(void);
void controlleftv(void);
void controlbalancev(void);
void Compute(void);
void SetTunings(double Kp,double Ki,double Kd);
double Input,Output,Setpoint;
double errSum=0,lastErr=0;
double kp,ki,kd,error;
int SampleTime=1000;
//unsigned long lastTime;
int count=0;
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
	delay_s(10);
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
	OSTaskSuspend(OS_PRIO_SELF);
	
}


int state=1;
void WalkTask(void)
{

	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem,0, &os_err);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetAngle());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)Getposx());
		USART_OUT(UART4,(uint8_t*)"%d\r\n",(int)Getposy());
//		USART_OUT(UART4,(uint8_t*)"%d\r\n",(int)Setpoint);
//		USART_OUT(UART4,(uint8_t*)"%d\r\n",(int)error);
		SetTunings(600,0,0);
		Compute();
		switch(state)
		{
			case 1:	Setpoint=0;
					if(GetAngle()>=-1&&GetAngle()<=1)
					{
						VelCrl(CAN2,1,COUNTS_PER_ROUND);
						VelCrl(CAN2,2,-COUNTS_PER_ROUND);
					}
					else
					{
						VelCrl(CAN2,1,COUNTS_PER_ROUND+Output/2);
						VelCrl(CAN2,2,-COUNTS_PER_ROUND+Output/2);
					}
					if(Getposy()>(2000-long)&&Getposy()<2000)
							state=2;
				break;
					
			case 2:	 Setpoint=-90;
					if(GetAngle()>=-91&&GetAngle()<=-89)
					{
						VelCrl(CAN2,1,COUNTS_PER_ROUND);
						VelCrl(CAN2,2,-COUNTS_PER_ROUND);
					}
					else
					{
						VelCrl(CAN2,1,COUNTS_PER_ROUND+Output/2);
						VelCrl(CAN2,2,-COUNTS_PER_ROUND+Output/2);
					}
					if(Getposx()>(2000-long)&&Getposx()<2000)
							state=3;	
				break;
			case 3:	Setpoint=-180;
					if((GetAngle()>=-180&&GetAngle()<=-179)||(GetAngle()>=179&&GetAngle()<=180))
					{
						VelCrl(CAN2,1,COUNTS_PER_ROUND);
						VelCrl(CAN2,2,-COUNTS_PER_ROUND);
					}
					else
					{
						VelCrl(CAN2,1,COUNTS_PER_ROUND+Output/2);
						VelCrl(CAN2,2,-COUNTS_PER_ROUND+Output/2);
					}
					if(Getposy()>0&&Getposy()<long)
							state=4;
				break;
			case 4:	Setpoint=90;
					if(GetAngle()>=89&&GetAngle()<=91)
					{
						VelCrl(CAN2,1,COUNTS_PER_ROUND);
						VelCrl(CAN2,2,-COUNTS_PER_ROUND);
					}
					else
					{
						VelCrl(CAN2,1,COUNTS_PER_ROUND+Output/2);
						VelCrl(CAN2,2,-COUNTS_PER_ROUND+Output/2);
					}
					if(Getposx()>0&&Getposx()<long)
							state=1;
				break;	
		
		}
	}
}

void Compute(void)
{
	Input=GetAngle();
	error=Setpoint-Input;
	if(error > 180)
		error = error - 360;
	else if(error < -180)
		error = error +360;
		
	errSum+=error;
	double dErr=error-lastErr;
	Output=kp*error+ki*errSum+kd*dErr;
	lastErr=error;
}

void SetTunings(double Kp,double Ki,double Kd)
{
	double SampleTimeInSec=((double)SampleTime)/1000;
	kp=Kp;
	ki=Ki*SampleTimeInSec;
	kd=Kd*SampleTimeInSec;

}
