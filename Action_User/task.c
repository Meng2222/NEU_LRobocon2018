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


float Input,error,Setpoint;
float errSum=0,lasterror=0;
double Output;
float Kp,Ki,Kd;
float ratio1,ratio2;
//typedef struct
//{
//	float x;
//	float y;
//	float angle;
//} Pos;


void Compute(void);
void SetTuning(float kp,float ki,float kd);		//设定PID值
/*
一个脉冲是4096/(120*Pi)
定义输入速度mm/s和半径mm
*/

//void vel_radious(float vel,float radious)
//{
//	ratio1=(radious-WHEEL_TREAD/2)/radious;
//	ratio2=(radious+WHEEL_TREAD/2)/radious;
//	VelCrl(CAN2,1,ratio1*vel*Pulse2mm);			//右轮
//	VelCrl(CAN2,2,-ratio2*vel*Pulse2mm);		//左轮
//}

void walk_stragiht(void)
{
//	VelCrl(CAN2,1,0);										//4346pulse/s，即400mm/s
//	VelCrl(CAN2,2,0);										//左轮是2，右轮是1（顺时针为正）
	VelCrl(CAN2,1,COUNTS_PER_ROUND+Output/2);				//4346pulse/s，即400mm/s
	VelCrl(CAN2,2,-COUNTS_PER_ROUND+Output/2);				//左轮是2，右轮是1（顺时针为正）
}




/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

//Pos carpos;



//void walk_around(void)
//{
//	vel_radious(200,WHEEL_TREAD/2);		//以小车右轮为原点，中心到右轮距离为半径即R=L/2	
//}


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

	USART3_Init(115200);
	UART4_Init(921600);
	
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	
	ElmoInit(CAN2);								//驱动器初始化
	MotorOn(CAN2,1);							//电机使能（通电）
	MotorOn(CAN2,2);
	
	VelLoopCfg(CAN2,1, 5000, 5000);				//驱动器速度环初始化
	VelLoopCfg(CAN2,2, 5000, 5000);
	
	delay_s(10);								//等待10s挂起
	OSTaskSuspend(OS_PRIO_SELF);
	
}

void WalkTask(void)
{

	CPU_INT08U os_err;
	os_err = os_err;
	int state=1;

	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetAngle());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetXpos());
		USART_OUT(UART4,(uint8_t*)"%d\r\n",(int)GetYpos());
		SetTuning(600,0,0);					//PID参数
		Compute();
//		walk_stragiht();
		switch(state)
		{
			case 1:		
				Setpoint=0;
				if(GetYpos()>=1800&&GetYpos()<=2000)
				{
					state++;
				}
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
				break;
			case 2:		
				Setpoint=-90;
				if(GetXpos()>=1800&&GetXpos()<=2000)
				{
					state++;
				}
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
				break;
			case 3:		
				Setpoint=-180;
				if(GetYpos()>=0&&GetYpos()<=200)
				{
					state++;
				}
				if(GetAngle()>=-180&&GetAngle()<=-178)
				{
					VelCrl(CAN2,1,COUNTS_PER_ROUND);
					VelCrl(CAN2,2,-COUNTS_PER_ROUND);
				}
				else
				{
					VelCrl(CAN2,1,COUNTS_PER_ROUND+Output/2);
					VelCrl(CAN2,2,-COUNTS_PER_ROUND+Output/2);
				}
				break;
			case 4:			
				Setpoint=90;
				if(GetXpos()>=0&&GetXpos()<=200)
				{
					state=1;
				}
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
				break;
			default:break;
		}
	}
}

void Compute(void)				//PID控制
{
	Input=GetAngle();
	error = Setpoint - Input;
	if(error<-180)
	{
		error=error+360;
	}
	else if(error>180)
	{
		error=error-360;
	}
	errSum+=error;
	float dErr=error-lasterror;
	Output=Kp*error+Ki*errSum+Kd*dErr;
	lasterror=error;
}


void SetTuning(float kp,float ki,float kd)
{
	Kp=kp;
	Ki=ki;
	Kd=kd;
}

