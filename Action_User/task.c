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
/*
===============================================================
						信号量定义
===============================================================
*/

extern char updownFlag;
extern char pposokflag;
int Sign = 1;
char taskFlag = 0;//切换运动状态标志
float distanceL = 0,distanceR = 0;//定义ADC左右距离
char dirFlag = 1;//直线运动方向
char changeFlag = 0;//切换直线标志
extern char updownFlag;//在线左右标志
char inAngleFlag = 0;//进入规定角度范围
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
PId_t Angle_PId = {
	.Kp = 14,
	.Ki = 0,
	.Kd = 1.2,
	.sumErr = 0,
	.lastErr = 0	
};
PId_t Dis_PId = {
	.Kp = 0.2 * 400 / SPEED,
	.Ki = 0.28 * 0.01 * 800 / SPEED,
	.Kd = 0,
	.sumErr = 0,
	.lastErr = 0
};
//初始化圆形结构体
Round_t Rnd_PID ={
	.x = 0,
	.y = 2400,
	.radius = 1800,
	.vel = SPEED,
	.side = 1 //-1为顺,1为逆
};
char posFlag = 0;//需要存位置标志
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	UART4_Init(921600);
	Adc_Init();
	TIM_Init(TIM2, 1000-1, 84-1, 0x01, 0x03); //定时器初始化1ms
	//CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2, 1, 500000, 500000);
	VelLoopCfg(CAN2, 2, 500000, 500000);
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	USART3_Init(115200);
	delay_ms(2000);
	/*一直等待定位系统初始化完成*/
	WaitOpsPrepare();
	
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
		USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)inAngleFlag,(int)taskFlag);
		USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)Rnd_PID.radius,(int)Sign);
		//USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)distanceL,(int)distanceR);
		//USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)taskFlag,(int)Rnd_PID.radius);
//		USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)errorRadius,(int)setAngle_R);
//		USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)uOutAngle,(int)uOutDis);
//		USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)phase1,(int)phase2);
		USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t\r\n",(int)GetAngle(),(int)GetX(),(int)GetY());
		if(Rnd_PID.side == 1)
		{
			if(GetAngle()<-160&&GetAngle()>-170)
			{
				inAngleFlag = 1;
			}
				
			if(fabsf(4*GetX() +GetY()-2400)<=20 && inAngleFlag == 1)
			{
				if((Rnd_PID.radius>=1800||Rnd_PID.radius<=600) && GetAngle()<0)
					Sign = -Sign;
				Rnd_PID.radius += Sign * 400;
				if(Rnd_PID.radius>1800)
					Rnd_PID.radius=1800;
				taskFlag++;	
				inAngleFlag = 0;
			}
		}
//		else if(Rnd_PID.side == 1)
//		{
//				taskFlag++;
//			if(Rnd_PID.radius>600)
//				Rnd_PID.radius -= 400;
//			else if(Rnd_PID.radius<=600)
//				Rnd_PID.radius += 400;
//		}
//		WalkRoundPID(&Rnd_PID);
//		switch(taskFlag)
//		{
//			case 0:
//			{
//				distanceR = (4400.f/4096.f)*(float)Get_Adc_Average(14,10);
//				distanceL = (4400.f/4096.f)*(float)Get_Adc_Average(15,10);
//				if(distanceR<=20)
//				{
//					Rnd_PID.side = 1;
//					taskFlag = 1;
//				}
//				else if(distanceL<=20)
//				{
//					Rnd_PID.side = -1;
//					taskFlag = 1;
//				}
//			}break;
//			case 1:
//			{
//				WalkRoundPID(&Rnd_PID);
//			}break;		
//		}
WalkRoundPID(&Rnd_PID);
	}
}

