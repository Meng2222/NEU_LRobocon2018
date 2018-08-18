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
float last_error;
float new_error;
pos_t xya;
static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
static float set_angle=0;
int iSOKFlag=0;
static int n=0;
static int up_down;
int t=0;
int kpa=50;
int kpd=6;
int kdd=2;
int aord;
static float d;
static float Aout=0;
static float Dout=0;
int light_number=1;
float car_v=0.5;
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
extern uint8_t opsFlag;
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM_Init(TIM2,1000-1,84-1,0x00,0x00);
	USART3_Init(115200);
	UART4_Init(921600);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,1,50000,50000);
	VelLoopCfg(CAN2,2,50000,50000);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	delay_s(2);
	
	#if car==1
		
     driveGyro();
     //USART_OUT(UART4,(uint8_t*)"OKOPSOPS");
	 while(!opsFlag);
	#elif car == 4
     delay_s(10);	
	  delay_s(5);
	 #endif
	
	OSTaskSuspend(OS_PRIO_SELF);
}
static int Right_cr1;
static int Left_cr2;
void WalkTask(void)
{   void Anglepid();
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
		USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\r\n",x,y,angle);		
		
		go(car_v);
		
	}
}
void go(float v)
{ 
	
     
	 //void Light(float,float,float,int);
     void Round(float ,float ,float ,float ,float );
   	 float V=v*1000;	 
	 int right;
	 int left;
//	if(light_number==1)
//	{ Light(1,0,0,1);
//	}else if(light_number==2)
//	{ Light(0,1,-2000,1);
//	}else if(light_number==3)
//	{ Light(1,0,-2000,-1);
//	}else if(light_number==4)
//	{ Light(0,1,0,-1);
//	}

	 Round (0,1000,1000,V,-1);
	if(!aord)
	{
	 VelCrl(CAN2,1,Right_cr1+Dout);
	 VelCrl(CAN2,2,Left_cr2+Dout);
	}else 
	{
		 VelCrl(CAN2,1,V/377*4096+Aout);
	     VelCrl(CAN2,2,-V/377*4096+Aout);
	}
	 right=Right_cr1+Dout;
	 left=Left_cr2+Dout;	
	 Aout=0;
	 Dout=0;
	 USART_OUT(UART4,(uint8_t*)"Right=%d\t",right);
	 USART_OUT(UART4,(uint8_t*)"Left=%d\t",left);
}

void pid_angle(float angle,int s)
{
    float nowerror_angle;
	int set;
	if(s==1)
	{if(xya.angle>-180&&xya.angle<angle-180)		
		nowerror_angle=angle-xya.angle-360;
	  else nowerror_angle=angle-xya.angle;
	}
	else if(s==-1)
	{if(xya.angle<180&&xya.angle>angle+180)		
		nowerror_angle=angle-xya.angle+360;

		else nowerror_angle=angle-xya.angle;
	}
	if(t)
	{
		 if(xya.angle<0)
			angle=-180;
		 else angle=180;
		 
		   nowerror_angle=angle-xya.angle;
	}
	
		 
	Aout=kpa*nowerror_angle;	
	int n=Aout;
	set=angle;
	//USART_OUT(UART4,(uint8_t*)"s=%d\t",s);
	//USART_OUT(UART4,(uint8_t*)"Aout=%d\t",n);
	USART_OUT(UART4,(uint8_t*)"set_angle=%d\t",set);
}
void pid_xy(float D,int n,int r)
{
  
//	if(f==1)
//	{if(up_down==1)
//	  {
//		  nowerror_d=-D;			  
//	  }else nowerror_d=D;
//	}else if(f==-1)
//    {
//		if(up_down==1)
//	  {
//		  nowerror_d=D;			  
//	  }else nowerror_d=-D; 
//	}
	last_error=new_error;
	new_error=D-r;
	

	
	Dout=kpd*D*n+kdd*(new_error-last_error);	
	
 // USART_OUT(UART4,(uint8_t*)"Dout=%d\t\r\n",n);
//	USART_OUT(UART4,(uint8_t*)"f=%d\t\r\n",f);
}


void Light(float a,float b,int n)
{  
	void pid_angle(float,int);
	
	
	//float light=a*xya.x+xya.y*b+c;
	
	int di;
	int l;
	int s=n;
	int f;
		
    //d=fabs(light)/sqrt((a*a)+(b*b));
	
	
	if(b)
	{ if(n==1)
		{ if(-a/b>0)
		  {   f=1;
			  set_angle=atan2(-a/b,1)/3.14159*180;
		  }
		  else if(-a/b<0)
		  {    f=-1;
			  set_angle=atan2(a/b,-1)/3.14159*180;
		  }
		}else
		{
			if(-a/b>0)
			{
		     set_angle=atan2(a/b,-1)/3.14159*180;
				f=-1;
			}
		  else if(-a/b<0)
		  {   f=1;
			  set_angle=atan2(-a/b,1)/3.14159*180;
		  }
		}
	}
	else if(b==0)
	{
		if(n==1)
		{   f=-1;
		   	set_angle=90;
		}
        else 
		{   f=1;
			set_angle=-90;          			
		}
	} 
     
	if(a==0)
	{
		if(n==1)
		{   f=1;
			set_angle=0;
		
		}
        else 
		{   f=-1;
			set_angle=-180;
            t=1;			
		}
		
	}
    di=d;	
	
	
//	USART_OUT(UART4,(uint8_t*)"d=%d\t\r\n",di);
//	USART_OUT(UART4,(uint8_t*)"l=%d\t\r\n",l);
	


	pid_angle(set_angle,s);
	t=0;
}
void Round(float x,float y,float r,float v,float n)
{ 
	int s;
	int D;
	d= sqrt(pow((xya.x-x),2)+pow((xya.y-y),2));
	Right_cr1=(v/r*(r+n*217))/377*4096;
	Left_cr2=-(v/r*(r-n*217))/377*4096;
	D=d;
	USART_OUT(UART4,(uint8_t*)"d=%d\t\r\n",D);
//	if((d-r)>=500)
//	{ 
//	  Light(xya.y-y,-xya.x+x,-(xya.y-y)/fabs((xya.y-y)));
//      	aord=1;
//		
//	}else 	
	     aord=0;
		 pid_xy(d,n,r);
	 
}
	