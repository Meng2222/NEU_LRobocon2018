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

/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

pos_t xya;
static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
static float set_x=0;
static float set_y=0;
static float set_angle=0;
int iSOKFlag=0;
static int car=1;
void driveGyro(void)
{
	while(!iSOKFlag)
	{
		delay_ms(5);
		USART_SendData(USART3,'A');
		USART_SendData(USART3,'T');
		USART_SendData(USART3,'\r');
		USART_SendData(USART3,'\n');
	}
	iSOKFlag=0;
}
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
	TIM_Init(TIM2,1000-1,84-1,0x010,0x03);
	USART3_Init(115200);
	UART4_Init(921600);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,1,10000,10000);
	VelLoopCfg(CAN2,2,10000,10000);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	if(car==1)
	{	
     driveGyro();
		
	}else if (car==4)
     delay_s(10);	
	
	OSTaskSuspend(OS_PRIO_SELF);
}
static int Right_cr1;
static int Left_cr2;
void WalkTask(void)
{
    void go(float);
	CPU_INT08U os_err;
	os_err = os_err;
    int x;
	int y;
	int angle;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{		
		OSSemPend(PeriodSem,  0, &os_err);
		x=(int)xya.x;
		y=(int)xya.y;
		angle=(int)xya.angle;
		USART_OUT(UART4,(uint8_t*)"x=%d,y=%d,angle=%d\r\n",x,y,angle);
		go(0.674);		
	}
}
void turn(float);	
void go(float v)
{   
    
	float V=v*1000;
	

	if(fabs(xya.x-set_x)<2000&&fabs(xya.y-set_y)<2000)
	{	
		Right_cr1=4096*V/377;		
	    Left_cr2=-Right_cr1;
		VelCrl(CAN2,1,Right_cr1);
		VelCrl(CAN2,2,Left_cr2);
	}
	else if(fabs(xya.x-set_x)>=2000&&fabs(xya.y-set_y)>=2000)
	{
		turn(v);
	}
		
}
void turn(float v)
{ 
	float V=v*1000;
	
	if(xya.angle==180)
	set_angle=-xya.angle;
	else set_angle=xya.angle;	
    
	while(1)
	{
     if(xya.angle-set_angle<90)		
	 {
		 Right_cr1=V/217*434*4096/377;
	     Left_cr2=0;
		 VelCrl(CAN2,1,Right_cr1);
		 VelCrl(CAN2,2,Left_cr2);
	 }else if(xya.angle-set_angle>=90) 
	 {
		 set_x=xya.x;
		 set_y=xya.y;
		 break;
		 
	 }
    }
}
	

static float setangle;
static float Kp;
static float Ki;
static float Kd;
static float Aout=0;
static float nowerror=0;
static float lasterror=0;
static float adderror=0;
void Anglepid(void)
{   
	lasterror=nowerror;
	nowerror=setangle-xya.angle;
	adderror+=nowerror;
	Aout+=Kp*nowerror;
	Aout+=Ki*adderror;
	Aout+=Kd*(nowerror-lasterror);	
}
