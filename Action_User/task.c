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
#define ACW 0
#define CW 1
#define manual 1
#define Auto 0
/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
/*
===============================================================
							直行
					入口参数：速度,单位mm/s
===============================================================
*/
void WalkStraight(int v)                             
{
	VelCrl(CAN2,1,(4096/378)*v);
	VelCrl(CAN2,2,-((4096/378)*v));
}
/*
===============================================================
						    画圈
入口参数;               前进方向  ACW  or  CW;
                       轴中间的速度 v 单位mm/s
                        圈的半径  r   单位mm
===============================================================
*/
void WalkRound(u8 direction, int v,int r)           //画圈，入口参数
{
	int w = v/r;
	if(direction == CW)
	{
		VelCrl(CAN2,1,(4096/378)*(w*(r-217)));
		VelCrl(CAN2,2,-((4096/378)*(w*(r+217))));
	}
	else
	{
		VelCrl(CAN2,1,(4096/378)*(w*(r+217)));
		VelCrl(CAN2,2,-((4096/378)*(w*(r-217))));
	}
}

/*
===============================================================
                          PID角度控制
===============================================================
*/

int angle_set = 0;
//float angle_change(float angle_1)
//{
//	if(angle_set == 0) return angle_1;
//	if(angle_set == 90)
//	{
//		if(angle_1 < -90) return (360+angle_1);
//		else return angle_1;
//	}
//	if(angle_set == 180)
//	{
//		if(angle_1 < 0) return (360+angle_1);
//		else return angle_1;
//	}
//	if(angle_set == 270)
//	{
//		if(angle_1 < 90) return (360+angle_1);
//		else return angle_1;
//	}
//}

extern u8 isOKFlag;
void driveGyro(void)
{
	while(!isOKFlag)
	{
		TIM_Delayms(TIM4,5);
		USART_SendData(USART3,'A');
		USART_SendData(USART3,'T');
		USART_SendData(USART3,'\r');
		USART_SendData(USART3,'\n');
	}
	isOKFlag = 0;
}

float kp = 20;
float ki = 0.0001;
float kd = 2;
float lastAngle = 0;
float velocity = 0;
float ITerm = 0;
float velocityMax1 = 1800;
float velocityMax2 = -1800;
float velocityMin1 = 100;
float velocityMin2 = -100;
extern float angle;
void SetTunings(float p,float i,float d)
{
	kp = p;
	ki = i;
	kd = d;
}

void Init_PID(float angle)
{
	lastAngle = angle;
	ITerm = velocity;
    if(ITerm > velocityMax1) ITerm= velocityMax1;
    else if(ITerm < velocityMax2) ITerm= velocityMax2;
}

//void PID_Angle(u8 status,float Angle_Set,float Angle,int v)
//{
//	static u8 lastStatus = 0;
//	if(status == manual)
//	{
//		lastStatus = status;
//		return;
//	}
//	if(lastStatus == manual && status == Auto) Init_PID(Angle);
//	float error = Angle_Set - Angle;
//	ITerm += ki*error;
//	if(ITerm > velocityMax1) ITerm= velocityMax1;
//    if(ITerm < velocityMax2) ITerm= velocityMax2;
////	if(ITerm > 0 && ITerm < velocityMin1) ITerm = velocityMin1;
////	if(ITerm < 0 && ITerm > velocityMin2) ITerm = velocityMin2;
//	float DTerm = lastAngle - Angle;
//	velocity = kp*error + ITerm + kd*DTerm;
////	velocity = kp*error + kd*DTerm;
//	if(velocity > velocityMax1) velocity = velocityMax1;
//    if(velocity < velocityMax2) velocity = velocityMax2;
////	if(velocity > 0 && velocity < velocityMin1) velocity = velocityMin1;
////  if(velocity < 0 && velocity > velocityMin2) velocity = velocityMin2;
//	lastAngle = Angle;
//	lastStatus = status;
//	VelCrl(CAN2,1,(((4096/378)*(velocity))+(4096/378)*v));
//	VelCrl(CAN2,2,(((4096/378)*(velocity))-(4096/378)*v));
//}


void PID_Angle(u8 status,float Angle_Set,float Angle,int v)
{
	static u8 lastStatus = 0;
	if(status == manual)
	{
		lastStatus = status;
		return;
	}
	if(lastStatus == manual && status == Auto) Init_PID(Angle);
	
	if(Angle_Set>=0)
	{
		if(Angle>=Angle_Set-180) Angle = Angle-Angle_Set;
		else Angle = Angle + 360 - Angle_Set;
	}
	if(Angle_Set<0)
	{
		if(Angle<=Angle_Set+180) Angle = Angle-Angle_Set;
		else Angle = Angle - 360 - Angle_Set;
	}
	
	float error = 0 - Angle;
	ITerm += ki*error;
	if(ITerm > velocityMax1) ITerm= velocityMax1;
    if(ITerm < velocityMax2) ITerm= velocityMax2;
//	if(ITerm > 0 && ITerm < velocityMin1) ITerm = velocityMin1;
//	if(ITerm < 0 && ITerm > velocityMin2) ITerm = velocityMin2;
	float DTerm = lastAngle - Angle;
	velocity = kp*error + ITerm + kd*DTerm;
//	velocity = kp*error + kd*DTerm;
	if(velocity > velocityMax1) velocity = velocityMax1;
    if(velocity < velocityMax2) velocity = velocityMax2;
//	if(velocity > 0 && velocity < velocityMin1) velocity = velocityMin1;
//  if(velocity < 0 && velocity > velocityMin2) velocity = velocityMin2;
	lastAngle = Angle;
	lastStatus = status;
	VelCrl(CAN2,1,(((4096/378)*(velocity))+(4096/378)*v));
	VelCrl(CAN2,2,(((4096/378)*(velocity))-(4096/378)*v));
}

extern union u8andfloat
{   
	uint8_t data[24];
	float ActVal[6];
}posture;

u8 coordinateCnt = 0;

void PID_Coordinate(float x0,float y0,int v)
{
	if(posture.ActVal[3]>x0 && posture.ActVal[4]>y0) PID_Angle(Auto,(90+((atan((posture.ActVal[4]-y0)/(posture.ActVal[3]-x0)))*(180/3.141592))),posture.ActVal[0],v);
	if(posture.ActVal[3]>x0 && posture.ActVal[4]<y0) PID_Angle(Auto,(90-((atan((y0-posture.ActVal[4])/(posture.ActVal[3]-x0)))*(180/3.141592))),posture.ActVal[0],v);
	if(posture.ActVal[3]<x0 && posture.ActVal[4]>y0) PID_Angle(Auto,(-90-((atan((posture.ActVal[4]-y0)/(x0-posture.ActVal[3])))*(180/3.141592))),posture.ActVal[0],v);
	if(posture.ActVal[3]<x0 && posture.ActVal[4]<y0) PID_Angle(Auto,(-90+((atan((y0-posture.ActVal[4])/(x0-posture.ActVal[3])))*(180/3.141592))),posture.ActVal[0],v);
	if((x0-100)<posture.ActVal[3] && posture.ActVal[3]<(x0+100) && (y0-100)<posture.ActVal[4] && posture.ActVal[4]<(y0+100)) coordinateCnt++;
}


/*
===============================================================
                           APP_Task
===============================================================
*/

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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                  //系统中断优先级分组2
	TIM_Init(TIM2,999,83,0,0);                                       //时钟2初始化，1ms周期
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);                //can1初始化
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);                //can2初始化
	
	ElmoInit(CAN2);                                                  //驱动器初始化
	VelLoopCfg(CAN2,2,60000000,60000000);                            //左电机速度环初始化
	VelLoopCfg(CAN2,1,60000000,60000000);                            //右电机速度环初始化
	MotorOn(CAN2,1);                                                 //右电机使能
	MotorOn(CAN2,2);                                                 //左电机使能
	USART3_Init(115200);
	UART4_Init(921600);
	TIM_Delayms(TIM4,15000);
//	driveGyro();
//	MotorOff(CAN2,2);                                                //左电机失能
//	MotorOff(CAN2,1);                                                //右电机失能
	
	OSTaskSuspend(OS_PRIO_SELF);                                     //挂起初始化函数
}

/*
===============================================================
                   WalkTask      初始化后执行
===============================================================
*/
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;                                                 //防报错
	OSSemSet(PeriodSem, 0, &os_err);                            	 //信号量归零
	int cnt = 0;
	int lasttime = 0;
	int time = 0;
	angle_set = 0;
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);                            //等信号量，10ms一次

		cnt++;

		if(cnt>799) cnt = 0;
		switch (coordinateCnt)
		{
			case 0:
				PID_Coordinate(0,0,1000);
				break;
			case 1:
				PID_Coordinate(0,2000,1000);
				break;
			case 2:
				PID_Coordinate(-2000,2000,1000);
				break;
			case 3:
				PID_Coordinate(-2000,0,1000);
				break;
		}
		if(coordinateCnt == 4) coordinateCnt=0;

		time = cnt/10;
		if(lasttime != time)
		{
			USART_OUT(UART4,(uint8_t*)"angle = %d  ", (int)posture.ActVal[0]);
			USART_OUT(UART4,(uint8_t*)"x = %d  ", (int)posture.ActVal[3]);
			USART_OUT(UART4,(uint8_t*)"y = %d  ", (int)posture.ActVal[4]);
			USART_OUT(UART4,(uint8_t*)"v = %d  ", (int)velocity);
			USART_SendData(UART4,'\r');
			USART_SendData(UART4,'\n');
			lasttime = time;
		}
	}
}



