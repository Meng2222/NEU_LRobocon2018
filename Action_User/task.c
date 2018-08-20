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
#include "stm32f4xx_adc.h"

#define Pulse2mm COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi
#define pai 3.1415926
#define aa 1
#define bb 0
#define cc 0
#define FB 2
#define xx -2200
#define yy 0
#define r 1000
//顺1逆2
#define Speed 1
/*
一个脉冲是4096/(120*Pi)
定义输入速度mm/s和半径mm
*/
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
int startflag=0;
extern int posokflag;
int left,right;
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//定时器10ms pwm CAN通信 右1左2 轮子CAN2  引脚 串口3/4
	TIM_Init(TIM2,999,83,0x00,0x00);
	Adc_Init();
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
	USART_OUT(UART4,(uint8_t*)"ss\r\n");
	while(!startflag)
	{
	  left=Get_Adc_Average(15,5);
	  right=Get_Adc_Average(14,5);
	  if(left<100)
		  startflag=1;
	  else if(right<100)
		  startflag=2;
	  else startflag=0;
		USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\r\n",left,right,startflag);
	}
	USART_OUT(UART4,(uint8_t*)"ok\r\n");
	OSTaskSuspend(OS_PRIO_SELF);
}

int mark=0;
float Out_Agl=0;
int vel1,vel2,vel,turnflag=0;
int X,Y,angle;
int Out_Pulse;
int flag=0,Flag=0;
float R_cover=2000;
extern struct Pos_t position;
void Round(float speed,float R);
void PID(int Agl_Flag);
void PID_Agl(float Set_Angle);
void PID_Awy(float D_err);
void Strght_Walk (float a,float b,float c,int F_B);
void Square_Walk(void);
void SET_openRound(float x,float y,float R,float clock,float speed);
void SET_closeRound(float x,float y,float R,float clock,float speed);
void Round_Cover(void);
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
		USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t%d\t%d\r\n",X,Y,angle,Flag,R_cover);
		Round_Cover();
    //SET_closeRound(xx,yy,r,Set_Clock,Speed);
		//Square_Walk();
		//Strght_Walk (aa,bb,cc,FB);
		//Round(0.1,0.2);
		//我的1号车走一个正方形
		/**switch(flag)
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
		}**/
	}
}
//1m/s走任0意固定直线PID：通过距离PID的输出值输入到角度PID，让距离越来越近的同时角度越来越接近设定角度值
/**void Strght_Walk (float a,float b,float c,int F_B)
{
	float k,Agl,Dis;;
	Dis=fabs(a*position.X+b*position.Y+c)/sqrt(pow(a,2)+pow(b,2));
	if(b==0)
  {
	  if(position.X>-c/a) 
		{
			if(F_B==1)
			{
				PID_Awy(Dis);
				PID_Agl(90-Out_Agl);
			}
			else if(F_B==2)
			{
				PID_Awy(Dis);
				PID_Agl(-90+Out_Agl);
			}
		}
		else if(position.X<-c/a)
		{
			if(F_B==1)
			{
				PID_Awy(Dis);
				PID_Agl(90+Out_Agl);
			}
			else if(F_B==2)
			{
				PID_Awy(Dis);
				PID_Agl(-90-Out_Agl);
			}
		}
		else
		{
			if(F_B==1)
				PID_Agl(90);
			else if(F_B==2)
				PID_Agl(-90);
		}
  }
	else if(a==0)
	{
	  if(position.Y>-c/b) 
		{
			if(F_B==1)
			{
				PID_Awy(Dis);
				PID_Agl(180+Out_Agl);
			}
			else if(F_B==2)
			{
				PID_Awy(Dis);
				PID_Agl(0-Out_Agl);
			}
		}
		else if(position.Y<-c/b)
		{
			if(F_B==1)
			{
				PID_Awy(Dis);
				PID_Agl(180-Out_Agl);
			}
			else if(F_B==2)
			{
				PID_Awy(Dis);
				PID_Agl(0+Out_Agl);
			}
		}
		else
		{
			if(F_B==1)
				PID_Agl(180);
			else if(F_B==2)
				PID_Agl(0);
		}
	}
  else
  {
		k=-a/b;
		Agl=(atan(-a/b))*180/pai;
	  if(k>0)
	  {
	    if(position.Y<k*X-c/b)
			{
				if(F_B==1)
				{
					PID_Awy(Dis);
					PID_Agl(180-Agl-Out_Agl);
				}
				else if(F_B==2)
				{
					PID_Awy(Dis);
				  PID_Agl(-Agl+Out_Agl);
				}
			}
		  else if(position.Y>k*X-c/b)
			{
				if(F_B==1)
				{
					PID_Awy(Dis);
			    PID_Agl(180-Agl+Out_Agl);
				}
				else if(F_B==2)
				{
					PID_Awy(Dis);
				  PID_Agl(-Agl-Out_Agl);
				}
			}
		  else 
			{
				if(F_B==1)
			    PID_Agl(180-Agl);
				else if(F_B==2)
				  PID_Agl(-Agl);
			}
	  }
	  else if(k<0)
	  {
		  if(position.Y>k*X-c/b)
			{
			  if(F_B==1)
				{
					PID_Awy(Dis);
			    PID_Agl(-Agl-Out_Agl);
				}
				else if(F_B==2)
				{
					PID_Awy(Dis);
				  PID_Agl(-Agl-180+Out_Agl);
				}
			}
		  else if(position.Y<k*X-c/b)
			{
			  if(F_B==1)
				{
					PID_Awy(Dis);
			    PID_Agl(-Agl+Out_Agl);
				}
				else if(F_B==2)
				{
					PID_Awy(Dis);
				  PID_Agl(-Agl-180-Out_Agl);
				}
			}
		  else 
			{
			  if(F_B==1)
			    PID_Agl(-Agl);
				else if(F_B==2)
				  PID_Agl(-Agl-180);
			}
	  }
  }
	VelCrl(CAN2,1,10865);
  VelCrl(CAN2,2,-10865-Out_Pulse);
}**/

//一轮向前一轮向后原地转-正方形
/**void Square_Walk(void)

{
	float Dis;
	switch(flag)
	{
		case 0:
			Dis=-position.Y;
			PID_Awy(Dis);
		  PID_Agl(0+Out_Agl);
		  if(position.X<-1500)
				flag=1;
			break;
		case 1:
		  Dis=-(position.X+2000);
		  PID_Awy(Dis);
		  PID_Agl(90+Out_Agl);
		  if(position.Y>1500)
				flag=2;
			break;
		case 2:
			Dis=position.Y-2000;
		  PID_Awy(Dis);
		  PID_Agl(180+Out_Agl);
		  if(position.X>-500)
			  flag=3;
			break;
		case 3:
			Dis=position.X;
		  PID_Awy(Dis);
		  PID_Agl(-90+Out_Agl);
		  if(position.Y<500)
				flag=0;
			break;
	}
		VelCrl(CAN2,1,8000-Out_Pulse);
		VelCrl(CAN2,2,-8000-Out_Pulse);
}**/

//走任意固定圆
void SET_closeRound(float x,float y,float R,float clock,float speed)
{
	float Distance=0;
	float k;
	float Agl;
	Distance=sqrt(pow(position.X-x,2)+pow(position.Y-y,2))-R;
	k=(position.X-x)/(y-position.Y);
	//顺1逆2
	if(clock==1)
	{
		if(position.Y<y)
		  Agl=-atan(k)*180/pai;
	  else if(position.X<=x&&position.Y>y)
		  Agl=180-atan(k)*180/pai;
	  else if(position.X>x&&position.Y>y)
		  Agl=-180-atan(k)*180/pai;
	  else if(position.Y==y&&position.X<=x)
		  Agl=90;
	  else if(position.Y==y&&position.X>x)
		  Agl=-90;
	  PID_Awy(Distance);
		PID_Agl(Agl+Out_Agl);
	}
	else if(clock==2)
	{
		if(position.Y>y)
		  Agl=-atan(k)*180/pai;
	  else if(position.X<x&&position.Y<y)
		  Agl=-180-atan(k)*180/pai;
	  else if(position.X>=x&&position.Y<y)
		  Agl=180-atan(k)*180/pai;
	  else if(position.Y==y&&position.X<=x)
		  Agl=-90;
	  else if(position.Y==y&&position.X>x)
		  Agl=90;
	  PID_Awy(Distance);
		PID_Agl(Agl-Out_Agl);
	}
	vel=(int)(speed*10865);
	VelCrl(CAN2,1,vel-Out_Pulse);
	VelCrl(CAN2,2,-vel-Out_Pulse);
}

void Round_Cover(void)
{
	static int count=0,lastcount=0,set=0,number=0;
	float last_x,last_y;
	SET_closeRound(xx,yy,R_cover,startflag,Speed);
	if(startflag==1)
	{
		if(position.angle>-87&&position.angle<-85)
		  count=1;
	  else 
		  count=0;
	  if(position.angle>0&&position.angle<90)
		  set=1;
  }
	else if(startflag==2)
	{
		if(position.angle>85&&position.angle<87)
		  count=1;
	  else 
	   	count=0;
		if(position.angle>-90&&position.angle<0)
		  set=1;
  }
	if(R_cover>1800)
		Flag=1;
	else if(R_cover<600)
		Flag=0;
	if(count==1&&lastcount==0&&set)
	{
		if(Flag)
			R_cover=R_cover-400;
		else
			R_cover=R_cover+400;
		set=0;
	}
	if(position.X>last_x-5&&position.X<last_x+5&&position.Y>last_y-5&&position.Y<last_y+5)
	  number++;
	else number=0;
	if(number>=5)
	{
		number=0;
		//倒退
		//VelCrl(CAN2,1,-8000);
	  //VelCrl(CAN2,2,8000);
		if(R_cover>400)
			R_cover=R_cover-400;
		else 
			R_cover=R_cover+400;
	}
	lastcount=count;
	last_x=position.X;
	last_y=position.Y;
}
void PID_Agl(float Set_Angle)
{
	float A_err;
	static float Last_Aerr,Sum_Aerr;
	A_err=Set_Angle-position.angle;
	if(A_err>180)
		A_err=A_err-360;
	if(A_err<-180)
		A_err=A_err+360;
	Sum_Aerr=Sum_Aerr+A_err;
	Out_Pulse=(int)(300*A_err);
		//+5*Sum_Aerr+5*(A_err-Last_Aerr));
	Last_Aerr=A_err;
}
void PID_Awy(float D_err)
{
	static float Last_Derr,Sum_Derr;
	Sum_Derr=Sum_Derr+D_err;
	Out_Agl=D_err/10;
	if(Out_Agl>=90)
    Out_Agl=90;
	//+5*Sum_Derr+5*(D_err-Last_Derr)); 
  Last_Derr=D_err;
}

/**void PID(int Agl_Flag)
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
}**/
/**void Round(float speed,float R)
{
	vel1=(int)(10865*speed*(R-0.217)/R);
  vel2=-(int)(10865*speed*(R+0.217)/R);
}
**/
