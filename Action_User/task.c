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

/*
1mm所需脉冲数是4096/(120*Pi)
定义输入速度mm/s和半径mm
*/
/////////////////转圈//////////////////////////
float ratio1,ratio2;
void vel_radious(float vel,float radious)
{
	ratio1=(radious+WHEEL_TREAD/2)/radious;
	ratio2=(radious-WHEEL_TREAD/2)/radious;
	VelCrl(CAN2,1,ratio1*vel*Pulse2mm); 
	VelCrl(CAN2,2,-ratio2*vel*Pulse2mm);     
}
int exchange(float v)
{
	int vdanweim=0;
	vdanweim=(int)(v*Pulse2mm/1000);
	return vdanweim;
}


///////////////////////////1号车///////////////////////
int isOKFlag;
void SetOKFlagZero(void)
{
	isOKFlag=0;
}

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
	SetOKFlagZero();
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
static OS_STK controlsrStk[controlsr_TASK_STK_SIZE];
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
//	os_err = OSTaskCreate((void (*)(void *))controlsrTask,
//						  (void *)0,
//						  (OS_STK *)&controlsrStk[controlsr_TASK_STK_SIZE - 1],
//						  (INT8U)controlsr_TASK_PRIO);
////（转圈任务）
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
//	TIM_Init(TIM1,9999,83,0x01,0x03);
  TIM_Init(TIM2,1000-1,84-1,0x00,0x00);	//产生10ms中断，抢占优先级为1，响应优先级为3
	UART4_Init(921600);
	USART3_Init(115200);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	VelLoopCfg(CAN2,1,4095,4095);
	VelLoopCfg(CAN2,2,4095,4095);
	ElmoInit(CAN2);
	//PosLoopCfg(CAN2,1,500,500,500);
	//PosCrl(CAN2,1,RELATIVE_MODE,pos);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	delay_s(5);
//	driveGyro();                    //////1号车
  delay_s(8);
	OSTaskSuspend(OS_PRIO_SELF);
}
extern struct position pos_t;
float setangle=0;
int T=0;
float suduchange=0;
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;

	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		T++;
		if(T>=101)
		{
			T=1;
		}
		if(AngleChange())
		{
			T=0;
			setangle+=90;
			if(setangle>180)
			{
				setangle-=360;
			}
		}
		suduchange=AnglePID(GetAngle(),setangle);
		VelCrl(CAN2,0x01,exchange(0.5)+suduchange);
		VelCrl(CAN2,0x02,-exchange(0.5)+suduchange);
		if(T%10==0)
		{
			USART_OUT(UART4, (uint8_t*)"%d %d\r\n",(int)((int)(GetXpos()),(int)(GetYpos())));
		}
         //////////串口回复位置角度//////////	



	
}}