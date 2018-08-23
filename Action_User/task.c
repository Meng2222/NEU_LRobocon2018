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
#include "fort.h"
#include "moveBase.h"
/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
// 宏定义棍子收球电机ID
#define COLLECT_BALL_ID (8)
// 宏定义推球电机ID
#define PUSH_BALL_ID (6)
// 宏定义送弹机构送弹时电机应该到达位置：单位位脉冲
#define PUSH_POSITION (4500)
// 宏定义送弹机构收回时电机位置
#define PUSH_RESET_POSITION (5)


float add_angle=90;
int push_balltime=0;
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
int kpa1=10000;
int kpd1=800;
int kpa=150;
int kpd=6;
int kdd=2;
int aord;
int if_add=1;
int if_compare=1;
static float d=1900;
static float Aout=0;
static float Dout=0;
float Left_d;
float Right_d;
float Add_V=0;
float tangent_angle;
float add_or_dec=-1;
float R=2000;
float right_cril;
float left_cril;
int time_number=0;
int leftorright=1;
float car_v=1000;
int compare_number=80;
int if_back=0;
int last_back=0;
int i=0;
int q=0;
float LIGHT_D;
int up_down;
float turn_cril;
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
	TIM_Init(TIM2,1000-1,84-1,0x00,0x00);
	USART1_Init(921600);
	USART3_Init(115200);
	UART4_Init(921600);
	UART5_Init(921600);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	
	ElmoInit(CAN2);
	
	#if car ==1
	ElmoInit(CAN1);
	VelLoopCfg(CAN2,1,50000,50000);
	VelLoopCfg(CAN2,2,50000,50000);
	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	Adc_Init();
	
	//棍子收球电机//
	// 配置速度环
    VelLoopCfg(CAN1, 8, 50000, 50000);
    		
	
	
	// 推球装置配置位置环
    PosLoopCfg(CAN1, PUSH_BALL_ID, 50000,50000,20000);
   	
    MotorOn(CAN1,6);
	
	MotorOn(CAN1,8);
	#else
	VelLoopCfg(CAN2,5,10000000,10000000);
	VelLoopCfg(CAN2,6,10000000,10000000);
	MotorOn(CAN2,5);
	MotorOn(CAN2,6);
	#endif
		
	/*一直等待定位系统初始化完成*/
	delay_ms(2000);
	BEEP_ON;
	WaitOpsPrepare();
	
	OSTaskSuspend(OS_PRIO_SELF);
}
static int Right_cr1;
static int Left_cr2;
extern FortType fort;
void WalkTask(void)
{   void Anglepid();
	void SendUint8();
    void go(float);
	void YawAngleCtr(float);
	CPU_INT08U os_err;
	os_err = os_err;
    int x;
	int y;
	int angle;
	int RIGHT_ADC;
	int LEFT_ADC;
	int xv;
	int yv;
	int head_cril=-turn_cril;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{		
		OSSemPend(PeriodSem,  0, &os_err);
		
		x=(int)xya.x;
		y=(int)xya.y;
		xv=xya.x_v;
		yv=xya.y_v;
		angle=(int)xya.angle;
	 #if car==1	
		push_balltime++;
		 //控制电机的转速，脉冲//
        VelCrl(CAN1,8,60*4096); 
		//控制发射枪电机转速//
	    ShooterVelCtrl(50);
		
		//航向电机//	
	    YawPosCtrl(add_angle);
		
		if(push_balltime==200)
		{ // 推球
			add_angle+=50;
	      PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);
		}else if(push_balltime==400)
		{// 复位//
	        add_angle-=50;
			PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);
		}
		push_balltime%=400;
        Right_d=Get_Adc_Average(14,10);
		Left_d=Get_Adc_Average(15,10);
		if(leftorright)
		{if(Right_d<50)
			{ 
				leftorright=0;
				//1为逆时针//
				if_go=1;
			}
		if(Left_d<50)
		    {   
				leftorright=0;
				//-1为顺时针//
				if_go=-1;
		    }
	    }
		USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",x,y,angle,(int)fort.yawPosReceive,(int)fort.shooterVelReceive,(int)fort.laserAValueReceive,(int)fort.laserBValueReceive);
		Add_V=sqrt(pow(xya.x_v,2)+pow(xya.y_v,2));
   #elif car!=1
    	USART_OUT(USART1,(uint8_t*)"%d\t%d\t%d\t%d\t%d\t%d\r\n",x,y,angle,(int)Dout,(int)Aout,(int)turn_cril);
   #endif		
		
		
	
				
		//USART_OUT(USART1,(uint8_t*)"x_v=%d",xv);
        //USART_OUT(USART1,(uint8_t*)"y_v=%d\r\n",yv);			
		go(car_v);
		
	}
}
void go(float v)
{ 
	
     
	 void Light(float,float,float,int);
     void Round(float ,float ,float ,float ,float );
   	 float V=v;	 
	 int right;
	 int left;
	  

   #if car==1
	if(if_go!=0)	
	{
		Round(0,2000,R,V,if_go);
		q++;
	}
	if(q>=100)
	{ if(Add_V>150)
	 {		
		if_back=0;
		//if(compare_number==5)
		//compare_number=10;
		if_add=1;
		
	 }
	 else 
	 {	
		
		//compare_number=10;
		 if_back=1;		 
		 if_add=0;
		for(i=0;i<=1200;i++)
		{   
			VelCrl(CAN2,1,-10000);
			VelCrl(CAN2,2,10000);
			time_number=0;
		}
		
		if(if_go==1)
		{for(i=0;i<=200;i++)
		{   
			VelCrl(CAN2,1,-5000);
			VelCrl(CAN2,2,-5000);
			time_number=0;
		}
		}
		if(if_go==-1)
		{for(i=0;i<=200;i++)
		{   
			VelCrl(CAN2,1,5000);
			VelCrl(CAN2,2,5000);
			time_number=0;
		}
		}
		right_cril=0;
		left_cril=0;	
		q=0;
	 }
	 
	}
	
    q%=100;
	if(if_back>last_back)
	{  
		if_go=-if_go;
		
		if(R>=1900)
		add_or_dec=-1;
		if(R<=800)	
		add_or_dec=1;
		R=add_or_dec*249+R;
		
	}
	last_back=if_back;
	   
	 right_cril=Right_cr1+Dout+Aout;
	 left_cril=Left_cr2+Dout+Aout;
	 VelCrl(CAN2,1,right_cril);
	 VelCrl(CAN2,2, left_cril);
	  right=right_cril;
	 left=left_cril;	USART_OUT(USART1,(uint8_t*)"Right=%d\t",right);	   
	 USART_OUT(USART1,(uint8_t*)"Left=%d\t",left);
	 USART_OUT(USART1,(uint8_t*)"time_number=%d\t",time_number);
	 USART_OUT(USART1,(uint8_t*)"if_go=%d\t",if_go);
	 Aout=0;
	 Dout=0;
	#else 
	 Light(1,0,500,1);
	 turn_cril=Dout+Aout;
	 VelCrl(CAN2,5,200000);
	 VelCrl(CAN2,6,-turn_cril);
	 #endif
	

	 

}

void pid_angle(float angle,int second_driection)
{
    float nowerror_angle;
	int set;
	int set_kp;
	#if car==1
	set_kp=kpa;
	#else 
	set_kp=kpa1;
	#endif
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
	
	
		 
	Aout=set_kp*nowerror_angle;	
	int n=Aout;
	set=angle;
	//USART_OUT(UART4,(uint8_t*)"s=%d\t",s);
	//USART_OUT(UART4,(uint8_t*)"Aout=%d\t",n);
	USART_OUT(UART4,(uint8_t*)"set_angle=%d\t",set);
}
void pid_xy(float D,int n,int r)
{
	
	
//  #if car!=1
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
//	#endif
	last_error=new_error;
	new_error=D-r;
	
	
	Dout=kpd*new_error*n;//+kdd*(new_error-last_error);	
	
 // USART_OUT(UART4,(uint8_t*)"Dout=%d\t\r\n",n);
//	USART_OUT(UART4,(uint8_t*)"f=%d\t\r\n",f);
}

int cril_flag=0;
int lastcril_flag=0;
int if_in=0;
void get_angle(float a,float b,int n,int round)
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
  { if(if_go==1)
	{if(set_angle<=-80&&set_angle>=-82)
	{
		cril_flag=1;
		if_in=1;
	}else cril_flag=0;
    
    }
	 if(if_go==-1)
	{if(set_angle>=-100&&set_angle<=-98)
	{
		cril_flag=1;
		if_in=1;
	}else cril_flag=0;
    
    }
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
	get_angle(xya.y-y,-xya.x+x,(xya.y-y)/fabs((xya.y-y)),round);
	else
	{	    if((xya.x-x)>0)
			get_angle(xya.y-y,-xya.x+x,1,round);
			else 	get_angle(xya.y-y,-xya.x+x,-1,round);		
	}
    pid_xy(d,round,r);
	
	 
}




void Light(float a,float b,float c,int n)
{  
	void pid_angle(float,int);
	void pid_xy2(float ,int );
    LIGHT_D=fabs(a*xya.x+b*xya.y+c)/(sqrt(pow(a,2)+pow(b,2)));
	//float light=a*xya.x+xya.y*b+c;
	
    
	int set_R;
	int second_driection;
	int f;
		
    //d=fabs(light)/sqrt((a*a)+(b*b));
	if((a*xya.x+xya.y*b+c)>=0)
	    up_down=1;
	else if((a*xya.x+xya.y*b+c)<0)
		up_down=-1;
	
	
	if(b!=0)
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
			{ f=-1;
		     set_angle=atan2(a/b,-1)/3.14159*180;
				
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
		   	
			{	
				f=-1;
				set_angle=90;
			}
			else 
			{   f=1;
				set_angle=-90;
		        
				t=1;
			}
		
    
		
	 }
     
	if(a==0)
	{
		if(n==1)
		{  f=1;
			set_angle=0;
		
		}
        else 
		{   f=-1;
			set_angle=-180;
            			
		}
		
	}
    
  
    
	
	
	
//	USART_OUT(UART4,(uint8_t*)"d=%d\t\r\n",di);
//	USART_OUT(UART4,(uint8_t*)"l=%d\t\r\n",l);
	
   
	

  
    set_R=R;
	USART_OUT(UART4,(uint8_t*)"set_R=%d\t",set_R);
	USART_OUT(UART4,(uint8_t*)"if_in=%d\t",if_in);
	pid_angle(set_angle,n);
	pid_xy2(LIGHT_D,f);
	t=0;
	if_in=0;
    if_add=1;
}
void pid_xy2(float D,int f)
{
	int  nowerror_d;
	
  #if car!=1
	if(f==1)
	{if(up_down==1)
	  {
		  nowerror_d=-D;			  
	  }else nowerror_d=D;
	}else if(f==-1)
    {
		if(up_down==1)
	  {
		  nowerror_d=D;			  
	  }else nowerror_d=-D; 
	}
	#endif

	
		
	Dout=kpd1*nowerror_d;//+kdd*(new_error-last_error);	
	
 // USART_OUT(UART4,(uint8_t*)"Dout=%d\t\r\n",n);
//	USART_OUT(UART4,(uint8_t*)"f=%d\t\r\n",f);
}
