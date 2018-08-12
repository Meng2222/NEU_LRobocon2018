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

//extern int t;
extern float X,Y,Ang;  //XY坐标（浮点） 校正角度（浮点）
//int i=1,j=0;
//void delay10(void)
//{
//if(t<10000){i=1;}
//if(t>=10000){i=0;}
//}

//void delay1(void)
//{if(t<100){j=0;}
//if(t>=100){j=1;}}

void Move(int V1,int V2)
{
VelCrl(CAN2,1,V1);//右轮
VelCrl(CAN2,2,V2);//左轮
}
void Move_Mode2()
{

}
int isOKFlag=0;
int IsSendOK(void)
{
return isOKFlag;
}
void SetOKFlagZero(void)
{
isOKFlag=0;
}
void driveGyro(void)
{
while(!IsSendOK())
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

	os_err = OSTaskCreate((void (*)(void *))WalkTask,    /*运动指令*/
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  (INT8U)Walk_TASK_PRIO);
    OSTaskSuspend(OS_PRIO_SELF);
}

/*
   ===============================================================
   初始化任务
   ===============================================================
   */
void ConfigTask(void)        //初始化
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	//TIM4_Pwm_Init (9999,83);//pwm初始化（10ms）
	TIM_Init(TIM2, 999, 83, 0X01, 0X03);//TIM2 1ms中断
	
	USART3_Init(115200);
	UART4_Init(921600);
	
//	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);//CAN1通信
	
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);//CAN2通信
	
	//Elmo驱动器初始化
	ElmoInit(CAN2);
	
	//驱动器速度环初始化
	VelLoopCfg(CAN2,1,2048000,2048000);
	VelLoopCfg(CAN2,2,2048000,2048000);

//	//驱动器位置环初始化
//	PosLoopCfg(CAN2,1,2048000,2048000,1024);
//	PosLoopCfg(CAN2,2,2048000,2048000,1024);
	
	//电机使能（通电）
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	
	delay_s(5);
	driveGyro();
	delay_s(5);
	
	OSTaskSuspend(OS_PRIO_SELF);
}


#define v 0.5      //车身速度
#define r 2        //半径or边长
                   //Mode1：车身旋转半径（填入0则直行）（r>0逆时针运动；r<0顺时针运动）
                   //Mode2：多边形边长
                   //Mode3：（无实际意义）
#define direction 0//方向（0为前进，1为后退）
#define Mode 0     //模式选择：
                   //0调试
                   //1直行（r=0）或圆周运动 前进/后退; 
                   //2多边形运动（此时r为多边形边长）
                   //3直线运动（带自动校正）
#define angle 90   //Mode1：（无实际意义）
                   //Mode2：多边形邻边角度
                   //Mode3：（偏转校正角度）
int v1,v2;         //两轮速度（2左 || 1右）
int x,y,angl;      //XY坐标（整数） 校正角度（整数）

void WalkTask(void)
{
	Move(1024,1024);
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem,0,&os_err);
	OSSemPend(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		
		x=(int)X;
		y=(int)Y;
		angl=(int)Ang;
		
		delay1();
		while(j)
		{
		USART_OUT(UART4,(uint8_t*)"%s%s","X",":");
		USART_OUT(UART4,(uint8_t*)"%d  ",x);
		USART_OUT(UART4,(uint8_t*)"%s%s","Y",":");
		USART_OUT(UART4,(uint8_t*)"%d  ",y);
		USART_OUT(UART4,(uint8_t*)"%s%s","A",":");
		USART_OUT(UART4,(uint8_t*)"%d\r\n",angl);
		delay1();
		t=0;
		j=0;
		}
		
		if(Mode==0)
	    {
		VelCrl(CAN2,1,512);//右轮
		VelCrl(CAN2,2,512);//左轮;
		USART_OUT(UART4,(uint8_t*)"%s%s%s%s%d\r\n","M","o","d","e",0);
		}
		if(Mode==1)                       //Mode1 直行（r=0）或圆周运动 前进/后退
		{
			if(r!=0)                            //圆周运动
			{
				v1=(int)((10865*v)+(217*v*10.8/r));
				v2=(int)((10865*v)-(217*v*10.8/r));
				if(direction==0)                      //圆周运动（前进）
				{Move(v1,-v2);}
				else if(direction==1)                 //圆周运动（后退）
				{Move(-v1,v2);}
			}
			else                                //直行
			{
				v1=(int)10865*v;
				if(direction==0)                      //直行（前进）
				{Move(v1,-v1);}
				if(direction==1)                      //直行（后退）
				{Move(-v1,v1);}
			}
	    }
		if(Mode==2)                       //Mode2 多边形运动（r为多边形边长；angle为多边形邻边角度） 
		{
//			v1=(int)10865*v;
//			if(t<=1000*r/v)                          //直行r(m)
//			{Move(v1,-v1);}
//			if(t>(1000*r/v)&&t<=1000*r/v+3.8*angle/v)//旋转angle度
//			{Move(v1,v1);}
//			if(t>(1000*r/v+3.8*angle/v))                   //t归0
//			{t=0;}
			
		}
		if(Mode==3)                      //Mode3 直行+校正
		{
//			ang=(int)angl*45/512;
			v1=(int)10865*v;
			
			if(angl>=-1&&Ang<=1)
			{Move(v1,-v1);t=0;}//直行
			if(angl>1){Move(v1,0);}
			if(angl<-1){Move(0,-v1);}
			
//			if(ang!=0)
//			{
//			if(t<=3.8*ang/v)//直行校正
//			{Move(v1,v1);}
//			if(t>3.8*ang/v)
//			{angl=0;}
//			}
		}
		OSSemPend(PeriodSem, 0, &os_err);
	}
}
