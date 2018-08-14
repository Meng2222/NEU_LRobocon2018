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

/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
//static OS_STK Walk_VirerStk[Walk_Virer_TASK_STK_SIZE];
//static OS_STK Walk_CircleStk[Walk_Circle_TASK_STK_SIZE];
//static OS_STK Walk_StraightStk[Walk_Straight_TASK_STK_SIZE];

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
//	os_err = OSTaskCreate((void (*)(void *))Walk_Virer,
//						  (void *)0,
//						  (OS_STK *)&Walk_VirerStk[Walk_Virer_TASK_STK_SIZE - 1],
//						  (INT8U)Walk_Virer_TASK_PRIO);
//	os_err = OSTaskCreate((void (*)(void *))Walk_Circle,
//						  (void *)0,
//						  (OS_STK *)&Walk_CircleStk[Walk_Circle_TASK_STK_SIZE - 1],
//						  (INT8U)Walk_Circle_TASK_PRIO);
//	os_err = OSTaskCreate((void (*)(void *))Walk_Straight,
//						  (void *)0,
//						  (OS_STK *)&Walk_StraightStk[Walk_Straight_TASK_STK_SIZE - 1],
//						  (INT8U)Walk_Straight_TASK_PRIO);
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
	USART3_Init(115200);
	UART4_Init(921600);
	TIM_Init(TIM2,999,83,1,0);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,1,0,0);
	VelLoopCfg(CAN2,2,0,0);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	//driveGyro();
	delay_s(12);
	OSTaskSuspend(OS_PRIO_SELF);
}

void  Walk_Virer(int radius,int multiple)
{
	float speed1,speed2;
	speed1=(radius+WHEEL_TREAD/2)*multiple;
	speed2=(radius-WHEEL_TREAD/2)*multiple;
//	VelCrl(CAN2,1,speed1);
//	VelCrl(CAN2,2,-speed2);
}
void  Walk_Circle(int speed1,int speed2)
{
//	VelCrl(CAN2,1,speed1);
//	VelCrl(CAN2,2,speed2);
}
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
	int cnt=0,i=0;
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		
		OSSemPend(PeriodSem, 0, &os_err);
		i++;
		if(i>=10)
		{
			i=0;
			//USART_OUT(UART4,(uint8_t*) "%d\t",(int)GetAngle());
			USART_OUT(UART4,(uint8_t*) "%d\t",(int)GetXpos());
			USART_OUT(UART4,(uint8_t*) "%d\r\n",(int)GetYpos());
		//	USART_OUT(UART4,(uint8_t*) "%d\r\n",(int)Angle_Pid(GetAngle()));
		}
		switch(cnt)
		{
			case 0:
				if(GetYpos()<1750)
				{
					Walk_Straight(5000+Angle_Pid(0-GetAngle()),5000-Angle_Pid(0-GetAngle()));
				}
				else
				{
					cnt++;
				}
			break;
			case 1:
				if(GetXpos()>-1750)
				{
					Walk_Straight(5000+Angle_Pid(90-GetAngle()),5000-Angle_Pid(90-GetAngle()));
				}
				else
				{
					cnt++;
				}
			break;
			case 2:
				if(GetAngle()>0&&GetAngle()<=180)
				{
					if(GetYpos()>250)
					{
						Walk_Straight(5000+Angle_Pid(180-GetAngle()),5000-Angle_Pid(180- GetAngle()));
					}
					else
					{
						cnt++;
					}
					break;
				}
				else if(GetAngle()>=-180&&GetAngle()<0)
				{
					if(GetYpos()>250)
					{
						Walk_Straight(5000+Angle_Pid(-180-GetAngle()),5000-Angle_Pid(-180-GetAngle()));
					}
					else
					{
						cnt++;
					}
					break;
				}
			case 3:
				if(GetAngle()>0&&GetAngle()<=180)
				{
					if(GetXpos()<-250)
					{
						Walk_Straight(5000-Angle_Pid(-90-GetAngle()),5000+Angle_Pid(-90-GetAngle()));
					}
					
					break;
				}
				else if(GetAngle()>=-180&&GetAngle()<0)
				{
					if(GetXpos()<-250)
					{
						Walk_Straight(5000+Angle_Pid(-90-GetAngle()),5000-Angle_Pid(-90-GetAngle()));
					}
					else
					{
						cnt=0;
					}
					break;
				}
		}
	}
}
