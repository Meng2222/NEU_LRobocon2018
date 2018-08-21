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
#include "pps.h"
#include "adc.h"

extern char isOKFlag;
extern char pposokflag;
extern double Input1,Output1,Output2;
extern float Angle_qie;
extern float Angle_w;

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
	UART4_Init(921600);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);			
	ElmoInit(CAN2);		
	MotorOn(CAN2,1);			
	MotorOn(CAN2,2);
	Adc_Init();
	VelLoopCfg(CAN2,1,1000,1000);
	VelLoopCfg(CAN2,2,1000,1000);
	delay_s(2);
	
	
	USART3_Init(115200);
	/*一直等待定位系统初始化完成*/
	WaitOpsPrepare();
	
	OSTaskSuspend(OS_PRIO_SELF);
}

float flag_distance1 = 0;
float flag_distance2 = 0;

void WalkTask(void)
{

	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	SetTunings1(130,0,0);
	SetTunings2(0.06,0.7*0.01,0);
	int state=0;
	int i=400;
	static int flag_dir=1;//半径增大减小标志位
	static int flag_angle=0;//走过大半圈标志位
	int flag_adc=0;//激光第一次判断标志位
	int dir;
	while (1)
	{
		OSSemPend(PeriodSem,0,&os_err);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)flag_distance1);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)flag_distance2);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)state);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)Input1);
		
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetX());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetY());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetAngle());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)Angle_qie);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)Angle_w);
		
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)Output1);		
		USART_OUT(UART4,(uint8_t*)"%d\r\n",(int)Output2);
		
		if(flag_adc==0)
		{
//			flag_distance1=(float)Get_Adc_Average(14,10);//左侧距离 
//			flag_distance2=(float)Get_Adc_Average(15,10); //右侧距离
			flag_distance1=(float)Get_Adc_Average(14,10)*4400.f/4096.f;//左侧距离 
			flag_distance2=(float)Get_Adc_Average(15,10)*4400.f/4096.f; //右侧距离
			
			if(flag_distance1<200)
			{
				dir=-1;
				flag_adc=1;
				state = 1;
			}
			
			if(flag_distance2<200)
			{
				dir=1;
				flag_adc=1;
				state = 1;
			}
			
		}
		
		
		
		
		switch(state)
		{
			case 1:
					Double_closed_loop(0,2400,R,SPEED,dir);//双闭环函数（横坐标，纵坐标，半径，速度，时针方向）
					if(dir>0)
					{
						if(GetAngle()>120&&GetAngle()<130)
							flag_angle=1;
						
						if(GetAngle()>35&&GetAngle()<45&&flag_angle==1)
						{
							state=2;
							flag_dir=1;
							flag_angle=0;
						}		
					}
					if(dir<0)
					{
						if(GetAngle()>-130&&GetAngle()<-120)
							flag_angle=1;
						
						if(GetAngle()>-45&&GetAngle()<-35&&flag_angle==1)
						{
							state=2;
							flag_dir=1;
							flag_angle=0;
						}		
					}
			break;
					
			case 2:
					Double_closed_loop(0,2400,R-2*i,SPEED,dir);
			
					if(dir>0)
					{
						if(GetAngle()>120&&GetAngle()<130)
							
							flag_angle=1;
						
						if(GetAngle()>35&&GetAngle()<45&&flag_angle==1)
						{
							if(flag_dir>0)
								state=3;
							
							else if(flag_dir<0)
								state=1;
							
							flag_angle=0;
						}		
					}
					if(dir<0)
					{
						if(GetAngle()>-130&&GetAngle()<-120)
							
							flag_angle=1;
						
						if(GetAngle()>-45&&GetAngle()<-35&&flag_angle==1)
						{
							if(flag_dir>0)
								state=3;
							
							else if(flag_dir<0)
								state=1;
							flag_angle=0;
						}		
					}
			
			break;	
					
			case 3:
					Double_closed_loop(0,2400,R-4*i,SPEED,dir);
					if(dir>0)
					{
						if(GetAngle()>120&&GetAngle()<130)
							
							flag_angle=1;
						
						if(GetAngle()>35&&GetAngle()<45&&flag_angle==1)
						{
							if(flag_dir>0)
								state=4;
							
							else if(flag_dir<0)
								state=2;
							
							flag_angle=0;
						}		
					}
					if(dir<0)
					{
						if(GetAngle()>-130&&GetAngle()<-120)
							
							flag_angle=1;
						
						if(GetAngle()>-45&&GetAngle()<-35&&flag_angle==1)
						{
							if(flag_dir>0)
								state=4;
							
							else if(flag_dir<0)
								state=2;
							flag_angle=0;
						}		
					}
			
			break;
					
			
					
			case 4:
					Double_closed_loop(0,2400,R-5*i,SPEED,dir);
					if(dir>0)
					{
						if(GetAngle()>120&&GetAngle()<130)
							
							flag_angle=1;
						
						if(GetAngle()>35&&GetAngle()<45&&flag_angle==1)
						{
							state=3;
							flag_angle=0;
							flag_dir=-1;
							
						}		
					}
					if(dir<0)
					{
						if(GetAngle()>-130&&GetAngle()<-120)
							
							flag_angle=1;
						
						if(GetAngle()>-45&&GetAngle()<-35&&flag_angle==1)
						{
							state=3;
							flag_angle=0;
							flag_dir=-1;
						}		
					}
			break;	
									
		}	
	
	}
}
