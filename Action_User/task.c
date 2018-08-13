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
/*           车的基本信息         */
#define COUNT_PER_ROUND (4096.0f)  //电机旋转一周的脉冲数 
#define WHEEL_DIAMETER (120.0f)    //轮子直径(mm)
#define ROBOT_LENGTH (492.0f)      //调试小车车长(mm)
#define ROBOT_WIDTH (490.0f)       //调试小车车宽(mm)
#define WHEEL_WIDTH (40.0f)        //轮子宽度(mm)
#define WHEEL_TREAD (434.0f)       //两个轮子的中心距离(mm)
#define CIRCLE_LENGTH (PI*WHEEL_DIAMETER) //车轮转一圈走的距离
#define Pulse2mm (COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi))
/*
一个脉冲是4096/(120*Pi)
定义输入速度mm/s和半径mm
*/
//float ratio1,ratio2;
//void vel_radious(float vel,float radious)
//{
//	ratio1=(radious+WHEEL_TREAD/2)/radious;
//	ratio2=(radious-WHEEL_TREAD/2)/radious;
//	VelCrl(CAN2,1,ratio1*vel*Pulse2mm);
//	VelCrl(CAN2,2,-ratio2*vel*Pulse2mm);
//}

		

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
	USART3_Init(115200);
	UART4_Init(921600);  
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	TIM_Init(TIM2,1000-1,84-1,0,0);	//产生10ms中断，抢占优先级为1，响应优先级为3

	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	VelLoopCfg(CAN2,1, 5000, 5000);				//驱动器速度环初始化
	VelLoopCfg(CAN2,2, 5000, 5000);
	ElmoInit(CAN2);								//驱动器初始化
	MotorOn(CAN2,1);							//电机使能（通电）
	MotorOn(CAN2,2);
	delay_ms(12000);
	OSTaskSuspend(OS_PRIO_SELF);
}
//void walk_direct(float vel) //左轮速度,单位mm/s
//{
//	VelCrl(CAN2,1,vel*Pulse2mm); //设置右轮速度
//	VelCrl(CAN2,2,-vel*Pulse2mm); //设置左轮速度
//}
extern float posX,posY,angle;
typedef struct{
	float setangle;
	float actualangle;
	float err;
	float last_err;
	float d_err;
	float kp,ki,kd;
	float err_integral;
	float vel;
}PID;
PID pid;
float vel_PID(float setangle,int kp,int ki)
{
	pid.kp=kp;
	pid.ki=ki;
	pid.setangle=setangle;
	pid.actualangle=angle;
	pid.err=pid.setangle-pid.actualangle;
	if(pid.err<-180)
	{
		pid.err+=360;
	}
	if(pid.err>180)
	{
		pid.err=pid.err-360;
	}
	pid.d_err=pid.err-pid.last_err;
	pid.err_integral+=pid.err;
	pid.vel=pid.kp*pid.err+pid.ki*pid.err_integral+pid.kd*pid.d_err;
	pid.last_err=pid.err;
	pid.actualangle=pid.vel;
	return pid.vel;
}
int x,y,agl,erro;
		
void WalkTask(void)
{ 
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem,0, &os_err);
	float vel=500.0;
	while (1)
	{
		OSSemPend(PeriodSem, 0,&os_err);
		x=(int)(posX);
		y=(int)(posY);
		agl=(int)(angle);
		erro=(int)(pid.err);
		USART_OUT(UART4,(uint8_t*)"x=%d  y=%d  agl=%d  err=%d\r\n",x,y,agl,erro);
		if(x>-200&&x<200&&y>=0&&y<=1800)
		{
			VelCrl(CAN2,1,(vel+vel_PID(0,90,0))*Pulse2mm );
			VelCrl(CAN2,2,-vel*Pulse2mm ); 
		}
		if(x>=-200&&x<=1800&&y>1800&&y<2200)
		{
			VelCrl(CAN2,1,(vel+vel_PID(-90,90,0))*Pulse2mm );
			VelCrl(CAN2,2,-vel*Pulse2mm ); 
		}
		if(x>1800&&x<2200&&y>200&&y<=2200)
		{
				VelCrl(CAN2,1,(vel+vel_PID(-180,90,0))*Pulse2mm );
				VelCrl(CAN2,2,-vel*Pulse2mm ); 
		}	
		if(x>=200&&x<2200&&y<200&&y>-200)
		{
				VelCrl(CAN2,1,(vel+vel_PID(90,90,0))*Pulse2mm );
				VelCrl(CAN2,2,-vel*Pulse2mm ); 
		}		
		erro=(int)(pid.err);
		USART_OUT(UART4,(uint8_t*)"x=%d  y=%d  agl=%d  erro=%d\r\n",x,y,agl,erro);		
	}
}


	
	
