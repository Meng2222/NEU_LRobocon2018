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
#include <math.h>
#define Kp1 120
#define Ki1 0
#define Kd1 0 
#define Kp2 10
#define Ki2 0
#define Kd2 0
#define State1 1
#define State2 0
float u1,u2;
int count;
extern char isOKFlag;
extern char pposokflag;
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
	isOKFlag = 0;
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
void Motor_Init(void)
{
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,1,8000,8000);
	VelLoopCfg(CAN2,2,8000,8000);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
}
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	
	TIM_Init(TIM2,999,83,0,0);
	
	USART3_Init(115200);   //定位器串口
	UART4_Init(921600);    //蓝牙串口
    
	Motor_Init();
	delay_s(2);
	#if CARNUM == 1
	driveGyro();
	while(!pposokflag);
	#elif CARNUM == 4
	delay_s(10);
	#endif
	OSTaskSuspend(OS_PRIO_SELF);
}
void uAng(float u2,float pointAng)//角度闭环
{
	static float lastErr = 0;
	static float sumErr = 0;
	float err,nowAngle;
	if(State1==0)
	{
	  nowAngle=GetAngle()+90;
	  if(nowAngle>180)
	  nowAngle=nowAngle-360;
	  err=u2+pointAng-nowAngle;
	}
	else
	{
		nowAngle=GetAngle()-90;
		if(nowAngle<-180)
		nowAngle=nowAngle+360;
	  err=-u2+pointAng-nowAngle; 
	}
	if(err>180)
		err=err-360;
	if(err<-180)
		err=err+360;
	sumErr += err;
	u1= Kp1 * err + Ki1 * sumErr +Kd1 *(err - lastErr);
	lastErr = err;
}
//void uPlace(float k,float x1,float y1)//位置闭环
//{
//	static float lastErr = 0;
//	static float sumErr = 0;
//	float err;
//	err=fabs(k*GetXpos()-GetYpos()+y1-k*x1)/sqrt(k*k+1);
//	if(k*GetXpos()-k*x1+y1<GetYpos()&&State2==0)
//	err=-err;
//    if(k*GetXpos()-k*x1+y1>GetYpos()&&State2==1)
//	err=-err;	
//	sumErr += err;
//	u2= Kp2*0.01*err + Ki2 * sumErr +Kd2 *(err - lastErr);
//	lastErr = err;
//}
void uPlace()  //沿y轴特殊情况
{
	static float lastErr = 0;
	static float sumErr = 0;
	float err;
	err=GetXpos();
	sumErr += err;
	u2= Kp2*0.01*err + Ki2 * sumErr +Kd2 *(err - lastErr);
	lastErr = err;
}	
void WalkStright(float speed)//直线走
{
	float pulse;
	pulse=4096*speed/(WHEEL_DIAMETER*PI);
	VelCrl(CAN2,1,pulse+u1);
	VelCrl(CAN2,2,-pulse);//左偏，u1+，右偏，u1-
}
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetAngle()+90);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetXpos());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetYpos());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)u2);
		USART_OUT(UART4,(uint8_t*)"%d\r\n",(int)u1);
//		switch(count)
//		{
//		 case 0:
//		    WalkStright(1,0);
//		 if(GetYpos()>1750)
//			 count=1;
//		 break;
//		 case 1:
//		    WalkStright(1,90);
//		 if(GetXpos()>1750)
//			 count=2;
//		 break;
//		 case 2:
//		    WalkStright(1,180);
//		 if(GetYpos()<250)
//			 count=3;
//		 break;
//		 case 3:
//		    WalkStright(1,-90);
//		 if(GetXpos()<250)
//			 count=0;
//		 break;
//		uPlace(1,0,100);
		uPlace();
//		uAng(u2,atan(1)*180/PI+State2*180);
		uAng(u2,90);
	    WalkStright(310);
	}		
}

