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

#define Pulse2mm COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi
#define car 1

/*
一个脉冲是4096/(120*Pi)
定义输入速度mm/s和半径mm
*/
/**float ratio1,ratio2;
void vel_radious(float vel,float radious)
{
	ratio1=(radious+WHEEL_TREAD/2)/radious;
	ratio2=(radious-WHEEL_TREAD/2)/radious;
	VelCrl(CAN2,1,ratio1*vel*Pulse2mm);
	VelCrl(CAN2,2,-ratio2*vel*Pulse2mm);
}
**/
extern int isOKFlag;
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
	isOKFlag=0;
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
extern int posokflag;
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//定时器10ms pwm CAN通信 右1左2 轮子CAN2  引脚 串口3/4
	TIM_Init(TIM2,999,83,0x00,0x00);
	USART3_Init(115200);
	UART4_Init(921600);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,1,1000,0);
	VelLoopCfg(CAN2,2,1000,0);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	delay_s(2);
	if(car==1)
	{
		driveGyro();
		while(!posokflag);
	}
	
	else if(car==4)
		delay_s(10);
	OSTaskSuspend(OS_PRIO_SELF);
	
	/**TIM_Init(TIM2,1000-1,84-1,1,3);	//产生10ms中断，抢占优先级为1，响应优先级为3

	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	
	VelLoopCfg(CAN2,1, 5000, 5000);				//驱动器速度环初始化
	VelLoopCfg(CAN2,2, 5000, 5000);
	
	ElmoInit(CAN2);								//驱动器初始化
	MotorOn(CAN2,1);							//电机使能（通电）
	MotorOn(CAN2,2);**/
}
int vel1,vel2,turnflag=0;
int X,Y,angle;
int Out_Pulse;
int flag1,flag2;
extern struct Pos_t position;
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	void Round(float speed,float R);
	void PID(float Agl);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		X=(int)(position.X);
		Y=(int)(position.Y);
		angle=(int)(position.angle);
		USART_OUT(UART4,(uint8_t*)"X=%d  y=%d  angle=%d\r\n",X,Y,angle);
		//Round(0.5,0.2);
		if(X<0&&X>-1950&&flag1==0)
		{
			PID(0);
			flag1=1;
		}
		if(X<=-1950&&X>=-2050&&flag2==0)
			PID(90);
		if(Y>0&&Y<1950&&flag2==0)
		{
			PID(90);
			flag2=1;
		}	
		if(Y>=1950&&Y<=2050&&flag1)
			PID(180);
		if(X<-50&&X>-2000&&flag1)
		{
			PID(180);
		  flag1=0;
		}
		if(X>=-50&&X<=50&&flag2)
			PID(-90);
		if(Y>50&&Y<2000&&flag2)
		{
			PID(-90);
			flag2=0;
		}
		if(Y<=50&&Y>=-50&&flag1)
			PID(0);
		VelCrl(CAN2,1,4096+Out_Pulse);
		VelCrl(CAN2,2,-4096);
		
	}
}
void PID(float Agl)
{
	float err,Last_err,Sum_err;
	err=Agl-position.angle;
	Sum_err=Sum_err+err;
	Out_Pulse=(int)(10*err+10*Sum_err+10*(err-Last_err));
	Last_err=err;
}
void Round(float speed,float R)
{
	vel1=(int)(10865*speed*(R-0.217)/R);
  vel2=-(int)(10865*speed*(R+0.217)/R);
}
