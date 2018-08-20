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
#include "math.h"

#define PI   3.1415926

/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
extern char pposokflag;

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

void  Walk_Straight(float speed1,float speed2)
{
	VelCrl(CAN2,1,speed1*10.87f);
	VelCrl(CAN2,2,-speed2*10.87f);
}

void  Walk_Virer(float right_v,float left_v)
{
	VelCrl(CAN2,1,right_v);
	VelCrl(CAN2,2,-left_v);
}

float Angle_Pid(float err)
{
	static float Iterm=0;
	float Dterm=0,errlast=0,Uk=0;
	if(err>=180)
		err-=360;
	else if(err<=-180)
		err+=360;
	Iterm +=Ki*err;
	Dterm=Kd*(err-errlast);
	errlast=err;
	Uk=Kp*err+Iterm+Dterm;
//	if(Uk>=400)
//		Uk=400;
//	else if(Uk<=-400)
//		Uk=-400;
	return Uk;
}
float Distance_Pid(float err)
{
	static float Iterm=0;
	float Dterm=0,errlast=0,Uk=0;
	Iterm +=Distance_Ki*err;
	Dterm=Distance_Kd*(err-errlast);
	errlast=err;
	Uk=Distance_Kp*err+Iterm+Dterm;
	return Uk;
}

float PID_OUT(float x,float y,float a,float b,float c,int quadrant)
{
	static float d=0,output=0,ang=0,err=0;
	d=fabs(a*GetXpos()+b*GetYpos()+c)/sqrtf(a*a+b*b);
	err=-Distance_Pid(0-d);
	if(err>=90)
		err=90;
	switch(quadrant)
	{
		case 1:
			ang=atan(-a/b)*180/PI-90;
			if((-(b*GetYpos()+c)/a)>GetXpos())
			{
				output=Angle_Pid(err-(ang-GetAngle()));
			}
			else if((-(b*GetYpos()+c)/a)<GetXpos())
			{
				output=Angle_Pid(-err-(ang-GetAngle()));
			}
			break;
		case 2:
			ang=atan(-a/b)*180/PI+90;
			if((-(b*GetYpos()+c)/a)>GetXpos())
			{
				output=Angle_Pid(err-(ang-GetAngle()));
			}
			else if((-(b*GetYpos()+c)/a)<GetXpos())
			{
				output=Angle_Pid(-err-(ang-GetAngle()));
			}
			break;
		case 3:
			ang=atan(-a/b)*180/PI+90;
			if((-(b*GetYpos()+c)/a)>GetXpos())
			{
				output=Angle_Pid(-err-(ang-GetAngle()));
			}
			else if((-(b*GetYpos()+c)/a)<GetXpos())
			{
				output=Angle_Pid(err-(ang-GetAngle()));
			}
			break;
		case 4:
			ang=atan(-a/b)*180/PI-90;
			if((-(b*GetYpos()+c)/a)>GetXpos())
			{
				output=Angle_Pid(-err-(ang-GetAngle()));
			}
			else if((-(b*GetYpos()+c)/a)<GetXpos())
			{
				output=Angle_Pid(err-(ang-GetAngle()));
			}
			break;
		case 5:
			ang=-90;
			if(GetYpos()>=-c)
			{
					output=Angle_Pid(err-(ang-GetAngle()));
			}
			else if(GetYpos()<-c)
			{
					output=Angle_Pid(-err-(ang-GetAngle())); 	
			}
			break;
			case 6:
			ang=90;
			if(GetYpos()<=-c)
			{
				output=Angle_Pid(err-(ang-GetAngle()));
			}
			else if(GetYpos()>-c)
			{
				output=Angle_Pid(-err-(ang-GetAngle()));
			}
			break;
		case 7:
			ang=0;
			if(GetXpos()<=-c)
			{
				output=Angle_Pid(err-(ang-GetAngle()));
			}
			else if(GetXpos()>-c)
			{
				output=Angle_Pid(-err-(ang-GetAngle()));
			}
			break;
		case 8:
			ang=180;
			if(GetXpos()>=-c)
			{
					output=Angle_Pid(err-(ang-GetAngle()));
			}
			else if(GetXpos()<-c)
			{
					output=Angle_Pid(-err-(ang-GetAngle()));
			}
			break;
	}
	return output;
}


float Angle_change(float a1,float a2,int c)
{
	static float a3=0;
	if(c==0)
		a3=a1-a2;
	else if(c==1)
		a3=a1+a2;
	if(a3>=180)
		a3-=360;
	else if(a3<=-180)
		a3+=360;
	return a3;		
}

void Virer_PID_Out(float x0,float y0,float radius,float speed,int Direction)
{
	static float d=0,err=0,output=0,ang1=0,ang2=0,angle1=0,angle2=0,speed_f=0,speed_s=0;
	d=sqrtf((GetXpos()-x0)*(GetXpos()-x0)+(GetYpos()-y0)*(GetYpos()-y0));
	err=Distance_Pid(radius-d);
	if(err<=-90)
		err=-90;
	else if(err>=90)
		err=90;
	ang2=360/(2*PI*radius/speed*100);
	angle1=atan2f(y0-GetYpos(),x0-GetXpos())*180/PI;
	speed_f=(radius+WHEEL_TREAD/2)/radius*(speed*4096/(120*PI));
	speed_s=(radius-WHEEL_TREAD/2)/radius*(speed*4096/(120*PI));
	switch(Direction)
	{
		case 0:
			angle2=Angle_change(angle1,90,0);
			ang1=Angle_change(angle2,90,0);
			if(d>radius)
			{
				output=Angle_Pid(-err+ang2+(ang1-GetAngle()));
			  Walk_Virer(speed_f+output,speed_s-output);
			}
			else if(d<radius)
			{
				output=Angle_Pid(-err+ang2+(ang1-GetAngle()));
				Walk_Virer(speed_f+output,speed_s-output);
			}
			break;
		case 1:
			angle2=Angle_change(angle1,90,1);
			ang1=Angle_change(angle2,90,0);
			if(d>radius)
			{
				output=Angle_Pid(err-ang2+(ang1-GetAngle()));
				Walk_Virer(speed_s+output,speed_f-output);
			}
			else if(d<radius)
			{
				output=Angle_Pid(err-ang2+(ang1-GetAngle()));
				Walk_Virer(speed_s+output,speed_f-output);
			}
			break;
	}
	USART_OUT(UART4,(uint8_t*) "%d\t",(int)angle1);
	USART_OUT(UART4,(uint8_t*) "%d\t",(int)angle2);
	USART_OUT(UART4,(uint8_t*) "%d\t",(int)ang1);
	USART_OUT(UART4,(uint8_t*) "%d\t",(int)err);
	USART_OUT(UART4,(uint8_t*) "%d\t",(int)output);
	USART_OUT(UART4,(uint8_t*) "%d\t",(int)GetXpos());
	USART_OUT(UART4,(uint8_t*) "%d\r\n",(int)GetYpos());
}	
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	USART3_Init(115200);
	UART4_Init(921600);
	TIM_Init(TIM2,999,83,1,0);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,1,0,0);
	VelLoopCfg(CAN2,2,0,0);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	delay_s(2);
	#if CARNUM == 1
	driveGyro();
	while(!pposokflag);
	#elif CARNUM == 4
	delay_s(12);
	#endif
	OSTaskSuspend(OS_PRIO_SELF);
}

void WalkTask(void)
{
	int cnt=0,i=0;
	float pid_out;
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		
		OSSemPend(PeriodSem, 0, &os_err);
			//USART_OUT(UART4,(uint8_t*) "%d\t",(int)GetAngle());
//			USART_OUT(UART4,(uint8_t*) "%d\t",(int)GetXpos());
//			USART_OUT(UART4,(uint8_t*) "%d\r\n",(int)GetYpos());
			//USART_OUT(UART4,(uint8_t*) "%d\t",(int)r2);
		  //USART_OUT(UART4,(uint8_t*) "%d\t",(int)Angle_Pid(GetAngle()));
			//USART_OUT(UART4,(uint8_t*) "%d\r\n",(int)err2);
		
		Virer_PID_Out(-1000,0,500,1000,0);
			
//		switch(cnt)
//		{
//			case 0:
//				if(GetYpos()<(1750-LV*500))
//				{
//					pid_out=PID_OUT(GetXpos(),GetYpos(),1,0,0,7);
//					Walk_Straight(v-pid_out,v+pid_out);
//				}
//				else
//				{
//					cnt++;
//				}
//				break;
//			case 1:
//				if(GetXpos()>-(1750-LV*500))
//				{
//					pid_out=PID_OUT(GetXpos(),GetYpos(),0,1,-2000,6);
//					Walk_Straight(v-pid_out,v+pid_out);
//				}
//				else
//				{
//					cnt++;
//				}
//				break;
//			case 2:
//				if(GetYpos()>(250+LV*500))
//				{
//					pid_out=PID_OUT(GetXpos(),GetYpos(),1,0,2000,8);
//					Walk_Straight(v-pid_out,v+pid_out);
//				}
//				else
//				{
//					cnt++;
//				}
//				break;	
//			case 3:
//				if(GetXpos()<-(250+LV*500))
//				{
//					pid_out=PID_OUT(GetXpos(),GetYpos(),0,1,0,5);
//					Walk_Straight(v-pid_out,v+pid_out);
//				}
//				else
//				{
//					cnt=0;
//				}
//				break;	
//		}
	}
}
