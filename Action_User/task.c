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
#include "math.h"

#define Pulse2mm COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi

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
	//切换车的宏定义在elmo.h里
  #if CAR_CONTRAL==1
		driveGyro();
		while(!posokflag);
	#elif CAR_CONTRAL==4
		delay_s(10);
	#endif
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
// 用position.XY？或者把2000往前提提,179与-179
int vel1,vel2,turnflag=0;
int X,Y,angle;
int Out_Pulse;
int flag=0;
extern struct Pos_t position;
void Round(float speed,float R);
void PID(int Agl_Flag);
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		X=(int)(position.X);
		Y=(int)(position.Y);
		angle=(int)(position.angle);
		USART_OUT(UART4,(uint8_t*)"%d\t%d\r\n",X,Y);
		//Round(0.1,0.2);
		 /**switch(flag)
		{
			case 0:
				PID(0);
			  if(position.X<=-1950)
				  flag=1;
				break;
			case 1:
				PID(90);
			  if(position.Y>=2000)
					flag=2;
				break;
			case 2:
				if(position.angle>0)
				  PID(179);
				else if(position.angle<0)
					PID(-179);
			  if(position.X>=0)
					flag=3;
				break;
			case 3:
				PID(-90);
			  if(position.Y<=0)
				  flag=0; 
				break;
		}
    VelCrl(CAN2,1,7000);
		VelCrl(CAN2,2,-7000-Out_Pulse);**/
		switch(flag)
		{
			case 0:
				PID(0);
			  VelCrl(CAN2,1,7000);
		    VelCrl(CAN2,2,-7000-Out_Pulse);
				if(position.X<-1550)
				  flag=1;
	    	break;
			case 1:
				PID(1);
			  VelCrl(CAN2,1,0);
		    VelCrl(CAN2,2,-Out_Pulse);
			  if(position.angle>89)
					flag=2;
				break;
			case 2:
				PID(1);
			  VelCrl(CAN2,1,7000);
		    VelCrl(CAN2,2,-7000-Out_Pulse);
			  if(position.Y>1550)
					flag=3;
				break;
			case 3:
				PID(2);
			  VelCrl(CAN2,1,0);
		    VelCrl(CAN2,2,-Out_Pulse);
			  if(position.angle>178)
					flag=4;
				break;
			case 4:
				PID(2);
				VelCrl(CAN2,1,7000);
		    VelCrl(CAN2,2,-7000-Out_Pulse);
				if(position.X>-450)
					flag=5;
				break;
			case 5:
				PID(3);
			  VelCrl(CAN2,1,0);
		    VelCrl(CAN2,2,-Out_Pulse);
			  if(position.angle>-90.5&&position.angle<-89.5)
				  flag=6; 
				break;
			case 6:
				PID(3);
			  VelCrl(CAN2,1,7000);
		    VelCrl(CAN2,2,-7000-Out_Pulse);
			  if(position.Y<450)
				  flag=7; 
				break;
			case 7:
				PID(0);
			  VelCrl(CAN2,1,0);
		    VelCrl(CAN2,2,-Out_Pulse);
			  if(position.angle>-1)
				  flag=0; 
				break;
		}
		   
				
      /**switch(flag)
			{
				case 0:
					PID(0);
				  if(Y>2000)
					  flag=1;
				  break; 
				case 1:
					PID(-90);
				  if(X>2000)
					  flag=2;
					break;
				case 2:
		      if(position.angle>0)
					  PID(-179);
		      else if(position.angle<0)
		        PID(179);
				  if(Y<0)
						flag=3;
					break;
				case 4:
					PID(90);
				  if(X<0)
						flag=0;
					break;	
			}**/
		
		/**VelCrl(CAN2,1,4096+Out_Pulse);
		VelCrl(CAN2,2,-4096);**/
	}
}
void PID(int Agl_Flag)
{
	float err,Err1,Err2,Last_err,Sum_err;
	switch(Agl_Flag)
	{
		case 0:
			err=0-position.angle;
		  break;
		case 1:
			Err1=90-position.angle;
		  Err2=-270-position.angle;
		  if(fabs(Err1)<fabs(Err2))
			  err=Err1;
		  else err=Err2;
			break;
		case 2:
			Err1=180-position.angle;
		  Err2=-180-position.angle;
		  if(fabs(Err1)<fabs(Err2))
			  err=Err1;
		  else err=Err2;
			break;
		case 3:
			Err1=-90-position.angle;
		  Err2=270-position.angle;
		  if(fabs(Err1)<fabs(Err2))
			  err=Err1;
		  else err=Err2;
			break;
	}
	Sum_err=Sum_err+err;
	Out_Pulse=(int)(500*err);
		//+5*Sum_err+5*(err-Last_err));
	Last_err=err;
}
void Round(float speed,float R)
{
	vel1=(int)(10865*speed*(R-0.217)/R);
  vel2=-(int)(10865*speed*(R+0.217)/R);
}
