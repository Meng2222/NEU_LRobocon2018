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
//extern int isOKFlag;
//void driveFyro(void)
//{
//	while(!isOKFlag){
//		delay_ms(5);
//		USART_SendData(USART3,'A');
//		USART_SendData(USART3,'T');
//		USART_SendData(USART3,'\r');
//		USART_SendData(USART3,'\n');
//	}
//	isOKFlag=0;
//}
//		
/*
   ===============================================================
   初始化任务
   ===============================================================
   */
//int car=1;
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
	delay_ms(10000);
	delay_ms(5000);
	OSTaskSuspend(OS_PRIO_SELF);
}
//void walk_direct(float vel) //左轮速度,单位mm/s
//{
//	VelCrl(CAN2,1,vel*Pulse2mm); //设置右轮速度
//	VelCrl(CAN2,2,-vel*Pulse2mm); //设置左轮速度
//}
extern float posX,posY,Angle;
struct PID{
	float set;
	float actual;
	float err;
	float last_err;
	float kp,ki,kd;
	float err_integral;
	float v;
}agl,ds;

float x1=0,y1=0,x2=0,y2=0;    //两点求一条直线
float angle_PID(float setangle)
{
	agl.kp=17;
	agl.ki=0;
	agl.set=setangle;
	agl.actual=Angle;
	agl.err=agl.set-agl.actual;
	if(agl.err<-180)
	{
		agl.err+=360;
	}
	if(agl.err>180)
	{
		agl.err=agl.err-360;
	}
	agl.err_integral+=agl.err;
	agl.v=agl.kp*agl.err+agl.ki*agl.err_integral;
	agl.last_err=agl.err;
	return agl.v;
}
float k,b;
float det_s;
float temp_ds;
float set_angle;
float Get_dS(void)
{
	set_angle=(atan2(y2-y1,x2-x1)*180/Pi)-90;
	if(x1==x2)
	{
		if(set_angle==0)
		{
			det_s=x1-posX;
		}
		else
		{
			det_s=posX-x1;
		}
	}
	else if(y1==y2)
	{ 
		if(set_angle==-90)
		{
			det_s=posY-y1;
		}
		else
		{
			det_s=y1-posY;
		}
	}
	else
	{
		k=(y2-y1)/(x2-x1);
		b=y1-k*x1;
		temp_ds=(k*posX+b-posY)/sqrt(1+k*k);//求点到直线的距离
		if(set_angle<0)
		{
			det_s=-temp_ds;
		}
		if(set_angle>0)
		{
			det_s=temp_ds;
		}	
	}
	return det_s;
} 
float ds_PID(void)
{
	ds.kp=1.1;
	ds.ki=0;
	ds.set=0;
	ds.actual=Get_dS();
	ds.err=ds.set-ds.actual;
	ds.err_integral+=ds.err;
	ds.v=ds.kp*ds.err+ds.ki*ds.err_integral;
	ds.last_err=ds.err;
	return ds.v;
}
uint8_t line;
void Get_line(void)
{
	if(posX<650&&posY<=1300)
	{
		line=1;
	}
	if(posX<=1350&&posY>1300)
	{
		line=2;
	}
	if(posX>1350&&posY>700)
	{
		line=3;
	}
	if(posX>=650&&posY<700)
	{
		line=4;
	}
}
float DeltV=0,DeltV1=0,DeltV2=0;
float V1,V2;
void walk_line(float vel)
{
	DeltV1=angle_PID(set_angle);
	DeltV2=ds_PID();
	DeltV=DeltV1+DeltV2;
		
	V1=vel+DeltV;
	V2=-vel+DeltV;
	VelCrl(CAN2,1,V1*Pulse2mm );
	VelCrl(CAN2,2,V2*Pulse2mm ); 
}

void WalkTask(void)
{ 
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem,0, &os_err);
	int X,Y,angle;
	int angle_erro;
	int m=0,n=0,a1=0,a2=0,a3=0,v1=0,v2=0;//这些变量名只是为了让串口发出数据，所以命名很随意
	while (1)
	{
		OSSemPend(PeriodSem, 0,&os_err);
		X=(int)(posX);
		Y=(int)(posY);
		angle=(int)(Angle);
		angle_erro=(int)(agl.err);
		m=(int)(set_angle);
		n=(int)Get_dS();
		a1=(int)DeltV1;
		a2=(int)DeltV2;
		a3=(int)DeltV;
		v1=(int)V1;
		v2=(int)V2;
		USART_OUT(UART4,(uint8_t*)"%d %d %d %d\r\n",line,X,Y,angle);
//		USART_OUT(UART4,(uint8_t*)"Setangle=%d angle_erro=%d DV1=%d \r\n",m,angle_erro,a1);
//		USART_OUT(UART4,(uint8_t*)"ds=%d DV2=%d\r\n",n,a2);
//		USART_OUT(UART4,(uint8_t*)"DV=%d V1=%d V2=%d\r\n\r\n",a3,v1,v2);
		Get_line();
		switch (line)
		{
			case 1:
				x1=0;
				y1=0;
				x2=0;
				y2=2000;
				break;
			case 2:
				x1=0;
				y1=2000;
				x2=2000;
				y2=2000;
				break;
			case 3:
				x1=2000;
				y1=2000;
				x2=2000;
				y2=0;
				break;
			case 4:
				x1=2000;
				y1=0;
				x2=0;
				y2=0;
				break;
		}
		walk_line(1000.0);  //按照速度行驶
	}
}


	
	
