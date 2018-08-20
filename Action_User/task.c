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
#include "stm32f4xx_adc.h"
#include "pps.h"
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
static int if_go=0;
int t=0;
int last_angle=90;
int new_angle;
int kpa=200;
int kpd=15;
int kdd=2;
int aord;
int if_add=1;
int if_compare=1;
static float d=1900;
static float Aout=0;
static float Dout=0;
float Left_d;
float Right_d;

float tangent_angle;
float add_or_dec=-1;
float R=1900;
float right_cril;
float left_cril;
int time_number=0;
int leftorright=1;
float car_v=1000;
int compare_number=100;
int if_back=0;
int last_back=0;
int i=0;
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
	Adc_Init();
	USART3_Init(115200);
	/*一直等待定位系统初始化完成*/
	delay_ms(2000);
	WaitOpsPrepare();
	
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
        Right_d=Get_Adc_Average(14,10);
		Left_d=Get_Adc_Average(15,10);
		if(leftorright)
		{if(Right_d<100)
			{ 
				leftorright=0;
				if_go=1;
			}
		if(Left_d<100)
		    { 
				leftorright=0;
				if_go=-1;
		    }
	    }
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
   	 float V=v;	 
	 int right;
	 int left;
	  

   
    if(if_go!=0)	
	 Round(0,1900,R,V,if_go);
    if(time_number<compare_number)
    {		
		if_back=0;
		//if(compare_number==5)
		//compare_number=10;
		if_add=1;
		right_cril=Right_cr1+Dout+Aout;
        left_cril=Left_cr2+Dout+Aout;	
	}
	else if(time_number>=compare_number)
	{	
		
		//compare_number=10;
		 if_back=1;		 
		 if_add=0;
		for(i=0;i<=1000;i++)
		{   
			VelCrl(CAN2,1,-10000);
			VelCrl(CAN2,2,10000);
			time_number=0;
		}
		right_cril=0;
        left_cril=0;	
	}
    
	if(if_back>last_back)
	{  
		if_go=-if_go;
		
	}
	last_back=if_back;
	   
	   
      
	 VelCrl(CAN2,1,right_cril);
	 VelCrl(CAN2,2,left_cril);
	 right=right_cril;
	 left=left_cril;	
	 Aout=0;
	 Dout=0;
	 USART_OUT(UART4,(uint8_t*)"Right=%d\t",right);	   
	 USART_OUT(UART4,(uint8_t*)"Left=%d\t",left);
	 USART_OUT(UART4,(uint8_t*)"time_number=%d\t",time_number);
}

void pid_angle(float angle,int second_driection)
{
    float nowerror_angle;
	int set;
	if(second_driection==1)
	{if(xya.angle>-180&&xya.angle<angle-180)		
		nowerror_angle=angle-xya.angle-360;
	  else nowerror_angle=angle-xya.angle;
	}
	else if(second_driection==-1)
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
	
	
	Dout=kpd*new_error*n;//+kdd*(new_error-last_error);	
	
 // USART_OUT(UART4,(uint8_t*)"Dout=%d\t\r\n",n);
//	USART_OUT(UART4,(uint8_t*)"f=%d\t\r\n",f);
}

int cril_flag=0;
int lastcril_flag=0;
int if_in=0;
void Light(float a,float b,int n,int round)
{  
	void pid_angle(float,int);
	

	//float light=a*xya.x+xya.y*b+c;
	

	int set_R;
	int second_driection;
	
		
    //d=fabs(light)/sqrt((a*a)+(b*b));
	
	
	
	if(b!=0)
	{ if(n==1)
		{ if(-a/b>0)
		  {   
			  set_angle=atan2(-a/b,1)/3.14159*180;
		  }
		  else if(-a/b<0)
		  {    
			  set_angle=atan2(a/b,-1)/3.14159*180;
		  }
		}else
		{
			if(-a/b>0)
			{
		     set_angle=atan2(a/b,-1)/3.14159*180;
				
			}
		  else if(-a/b<0)
		  {   
			  set_angle=atan2(-a/b,1)/3.14159*180;
		  }
		}
	}
	else if(b==0)
	{
		if(round==-1)
		{   if(n==1)
		   	{
				second_driection=1;
				set_angle=90;
			}
			else 
			{
				set_angle=-90;
		        second_driection=-1;
				t=1;
			}
		}
        else 
		{   
			if(n==1)
		   {
				second_driection=-1;
				set_angle=90;
			    t=1;
			}
			
			else 
			{
				set_angle=-90;
		        second_driection=1;
			}       			
		}
	} 
     
	if(a==0)
	{
		if(n==1)
		{  
			set_angle=0;
		
		}
        else 
		{   
			set_angle=-180;
            			
		}
		
	}
     new_angle=set_angle;
    //比较//
     if(new_angle==last_angle)
	{
		time_number++;
		
	}else time_number=0;
	last_angle=new_angle;
	
	if(round==-1)
	{ 
		tangent_angle=set_angle-90;
		if(tangent_angle<-180)
			tangent_angle+=360;
	}else
	{
		tangent_angle=set_angle+90;
		if(tangent_angle>180)
			tangent_angle-=360;
	}
	
	
	
//	USART_OUT(UART4,(uint8_t*)"d=%d\t\r\n",di);
//	USART_OUT(UART4,(uint8_t*)"l=%d\t\r\n",l);
	
    if(round==-1)
	{
		if(b<0)
			second_driection=-1;
		else second_driection=1;
	}else
	{
		if(b<0)
			second_driection=1;
		else second_driection=-1;
		
	}
  if(if_add)	
  {
	if(set_angle<=-80&&set_angle>=-82)
	{
		cril_flag=1;
		if_in=1;
	}else cril_flag=0;
	
	if(cril_flag>lastcril_flag)
	{
		R=add_or_dec*249+R;
		
		if(R>=1900)
		add_or_dec=-1;
        if(R<=800)	
		add_or_dec=1;
	}
	
	lastcril_flag=cril_flag;
  }
    set_R=R;
	USART_OUT(UART4,(uint8_t*)"set_R=%d\t",set_R);
	USART_OUT(UART4,(uint8_t*)"if_in=%d\t",if_in);
	pid_angle(tangent_angle,second_driection);
	t=0;
	if_in=0;
    if_add=1;
}
void Round(float x,float y,float r,float v,float round)
{ 
	
	int D;
	d= sqrt(pow((xya.x-x),2)+pow((xya.y-y),2));
	Right_cr1=(v/r*(r+round*217))/377*4096;
	Left_cr2=-(v/r*(r-round*217))/377*4096;
	D=d;
	USART_OUT(UART4,(uint8_t*)"d=%d\t\r\n",D);
	if(xya.y-y)
	Light(xya.y-y,-xya.x+x,(xya.y-y)/fabs((xya.y-y)),round);
	else
	{	    if((xya.x-x)>0)
			Light(xya.y-y,-xya.x+x,1,round);
			else 	Light(xya.y-y,-xya.x+x,-1,round);		
	}
    pid_xy(d,round,r);
	
	 
}


