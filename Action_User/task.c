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
int if_push=0;
int diagonal;
float body_angle;
float ADC_A;
float ADC_B;
int if_turnto90;
float if_addorreduce_angle=1;
float Back_maichong;
float Head_maichong;
float add_angle=0;
int push_balltime=0;
float last_error;
float new_error;
float s;
pos_t xya;
static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
static float set_angle=0;
int iSOKFlag=0;
static int if_go=0;//-1为顺时针//1为逆时针//
int t=0;
int last_angle=90;
int new_angle;
int kpa1=10000;
int kpd1=400;//
int kpa=200;//旧车1m/s圆弧闭环为150//新车/15000
int kpd=6;//旧车1m/s圆弧闭环为6//新车400
int kdd=2;
int aord;
int if_add=1;
int if_compare=1;
static float d;
static float Aout=0;
static float Dout=0;
float Left_d;
float Right_d;
float Add_V=0;
float tangent_angle;
float add_or_dec=-1;
float R=2000;//投球为1500//
float right_cril;
float left_cril;
int time_number=0;
int leftorright=1;
float car_v=1700;
int compare_number=80;
int if_back=0;
int last_back=0;
int i=0;
int q=0;
float LIGHT_D;
int up_down;
float turn_cril;
int change_compere_angle;
float get_differ_angle(float angle,int round);
float get_addorreduce_angle(float differ_angle,float x);
 void get_sendangle(void);
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
	YawPosCtrl(170);
	//棍子收球电机//
	// 配置速度环
    VelLoopCfg(CAN1, 8, 50000, 50000);
   	
	// 推球装置配置位置环
    PosLoopCfg(CAN1, PUSH_BALL_ID, 2000000,2000000,3000000);
   	
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
{   void pull_ball();
	void Anglepid();
	void SendUint8();
    void go(float);
	void YawAngleCtr(float);
	float get_roll_v();
	CPU_INT08U os_err;
	os_err = os_err;
    int x;
	int y;
	int angle;
	int RIGHT_ADC;
	int LEFT_ADC;
	int xv;
	int yv;
	float get_d=(ADC_A+ADC_B)/2;
	int head_cril=-turn_cril;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{		
		OSSemPend(PeriodSem,  0, &os_err);
	
		ADC_A=fort.laserAValueReceive*2.5112+38.72;
		ADC_B=fort.laserBValueReceive*2.4267+358.54;
		x=(int)xya.x;
		y=(int)xya.y;
		xv=xya.x_v;
		yv=xya.y_v;
		angle=(int)xya.angle;
	    #if car==1	
		pull_ball();
			
		 //控制电机的转速，脉冲//
        VelCrl(CAN1,8,300*4096); 
		//控制发射枪电机转速//	
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
		    {  				leftorright=0;
				//-1为顺时针//
				if_go=-1;
		    }
	    }
		//USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",x,y,angle,(int)xya.angle_v,(int)fort.yawPosReceive,(int)fort.shooterVelReceive,(int)ADC_A,(int)ADC_B,(int)get_roll_v());
		Add_V=sqrt(pow(xya.x_v,2)+pow(xya.y_v,2));
   #elif car!=1
    	USART_OUT(USART1,(uint8_t*)"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",x,y,angle,(int)Dout,(int)Aout,(int)turn_cril,(int)LIGHT_D,(int)new_error);
		Add_V=sqrt(pow(xya.x_v,2)+pow(xya.y_v,2));
   #endif		
		
		//USART_OUT(USART1,(uint8_t*)"x_v=%d",xv);
        //USART_OUT(USART1,(uint8_t*)"y_v=%d\r\n",yv);			
		go(car_v);
		
		
	}
}
void go(float v)
{   
	 float get_roll_v();
	 void pull_ball(void);
	
	 float Turn_v_to_headmaichong(float,float);
     float Turn_v_to_backmaichong(float);
	 void Light(float,float,float,int);
     void Round(float ,float ,float ,float ,float );
   	 float V=v;	 
	 int right;
	 int left;
	 int car_angle=set_angle;
     int get_paotai_angle=fort.yawPosReceive;
	 float send_angle;
	if(if_go!=0)	
	{ 
		if(if_push==1)
		{
	    ShooterVelCtrl(get_roll_v());		
		//航向电机//
		get_sendangle();	
		}			
		Round(0,2350,R,V,if_go);
		q++;       
   }
	
	if(q>=100)
	{ if(Add_V>150)
	 {		
		if_back=0;		
		if_add=1;
		
	 }
	 else 
	 {	
		 if_back=1;		 
		 if_add=0;
		for(i=0;i<=1500;i++)
		{   
			#if car==1
			VelCrl(CAN2,1,-10000);
			VelCrl(CAN2,2,10000);
			#else
			VelCrl(CAN2,6,0);
			VelCrl(CAN2,5,-300000);			
			#endif
			time_number=0;
		}
		
		if(if_go==1)
		{for(i=0;i<=500;i++)
		 {  
			#if car==1
			VelCrl(CAN2,1,-5000);
			VelCrl(CAN2,2,-5000);
			#else
			VelCrl(CAN2,5,-100000);
			VelCrl(CAN2,6,100000);			
			#endif
			time_number=0;
		 }
		}
		if(if_go==-1)
		{for(i=0;i<=500;i++)
		{   
			#if car==1
			VelCrl(CAN2,1,5000);
			VelCrl(CAN2,2,5000);
			#else
			VelCrl(CAN2,5,-100000);
			VelCrl(CAN2,6,-100000);			
			#endif
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
	
		    if(R>=2100)
			{   R=2100;
				add_or_dec=-1;
			}
			if(R<=600)
			{	
				add_or_dec=1;
				R=1500;
				if_push=1;
				car_v=2000;
			}
			R=add_or_dec*300+R;
	   
	
			
		
	}
	 last_back=if_back;
	 #if car==1
	 right_cril=Right_cr1+Dout+Aout;
	 left_cril=Left_cr2+Dout+Aout;
     VelCrl(CAN2,1,right_cril);
	 VelCrl(CAN2,2, left_cril);
	 right=right_cril;
	 left=left_cril;
//   USART_OUT(USART1,(uint8_t*)"Right=%d\t",right);	   
//	 USART_OUT(USART1,(uint8_t*)"Left=%d\t",left);
//	 USART_OUT(USART1,(uint8_t*)"time_number=%d\t",time_number);
//	 USART_OUT(USART1,(uint8_t*)"if_go=%d\t",if_go);
	 Aout=0;
	 Dout=0;
	 #elif  car==2 	 
	 turn_cril=-Dout-Aout;
	 VelCrl(CAN2,5,Turn_v_to_backmaichong(v));
	 VelCrl(CAN2,6,Turn_v_to_headmaichong(v,R)+turn_cril);
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
  if(if_add&&!if_push)	
  { if(if_go==1)
	{if(set_angle>=-100&&set_angle<=-98)
	{
		cril_flag=1;
		if_in=1;
	}else cril_flag=0;
    
    }
	 if(if_go==-1)
	{if(set_angle>=-82&&set_angle<=-80)
	{
		cril_flag=1;
		if_in=1;
	}else cril_flag=0;
    
    }
	if(cril_flag>lastcril_flag)
	{
		R=add_or_dec*500+R;
		if(R>=2100)		
		{
			add_or_dec=-1;
			R=2100;
		}
		if(R<=600)
		{   
            add_or_dec=1;
			R=1500;
			car_v=1370;
			if_add=0;
			if_push=1;
		}
		
	}
	
	lastcril_flag=cril_flag;
  }
    change_compere_angle=(int)set_angle;
    set_R=R;
	//USART_OUT(UART4,(uint8_t*)"set_R=%d\t",set_R);
	//USART_OUT(UART4,(uint8_t*)"if_in=%d\t",if_in);
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
				f=1;
				set_angle=90;
			}
			else 
			{   f=-1;
				set_angle=-90;
		        
				t=1;
			}
		
    
		
	 }
     
	if(a==0)
	{
		if(n==1)
		{  f=-1;
			set_angle=0;
		
		}
        else 
		{   f=1;
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
	

	
		
	Dout=kpd1*nowerror_d;//+kdd*(new_error-last_error);	
	
 // USART_OUT(UART4,(uint8_t*)"Dout=%d\t\r\n",n);
//	USART_OUT(UART4,(uint8_t*)"f=%d\t\r\n",f);
}
//将速度转化为后轮的脉冲数//
float Turn_v_to_backmaichong(float v)
{
	return(v/(Pi*WHEEL_DIAMETER)*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO);
}
//将圆弧闭环时将速度转化为前轮的脉冲数//
float Turn_v_to_headmaichong(float v,float r)
{
	return(v/r*TURN_AROUND_WHEEL_TO_BACK_WHEEL/(Pi*TURN_AROUND_WHEEL_DIAMETER)*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO);
}	
//得到炮台转轮速度//
float get_roll_v(void)
{   
	float roll_v;
	//四号车的炮台速度比较小需要加大//
	//一号车在顺时针 -1 4   1 3.5//	四号炮台+0.2  ///4.5   4
	float	get_d(float,float);
	if(s<=4000)
	{
	if(if_go==1&&Add_V>=150)
	{
		if( diagonal==1)
		roll_v=((sqrt(19600*pow(s,2)/((sqrt(3))*s-400)))/377*3.55);
		else
		roll_v=((sqrt(19600*pow(s,2)/((sqrt(3))*s-400)))/377*3.55);
	}else
	{   if( diagonal==-1)
		roll_v=((sqrt(19600*pow(s,2)/((sqrt(3))*s-400)))/377*3.55);
		else
		roll_v=((sqrt(19600*pow(s,2)/((sqrt(3))*s-400)))/377*3.55);
	}
    }else roll_v=(50);
	
	USART_OUT(UART4,(uint8_t*)"roll_v=%d\t",(int)roll_v);
	if(roll_v<=100)
		return roll_v;
	else return 90;
	
}

//使航向电机对准目标//
void get_sendangle(void)
{   
	float get_d(float,float);
	void pull_ball();
	float get_angle2(float a,float b,int n,int round);

	
	float point_x;
	float point_y;
    float sendsend_angle;
	float getget_angle;	
    float add_angle;
	float cartopoint_angle;
	float x;
	if(if_go==-1)
	{	if(change_compere_angle<=-45&&change_compere_angle>=-135)
		{
			point_x=-2400;
			point_y=200;
			diagonal=1;
			x=get_d(point_x,point_y);
			body_angle=get_angle2(xya.y-point_y,-xya.x+point_x,(xya.y-point_y)/fabs((xya.y-point_y)),if_go);
			cartopoint_angle=body_angle+180;
			if(cartopoint_angle>180)
			   cartopoint_angle-=360;
			add_angle=get_addorreduce_angle(get_differ_angle(cartopoint_angle,if_go),x);
			body_angle=body_angle+add_angle*0.8;
			getget_angle=90+body_angle;
			if(xya.compare_angle<180&&xya.compare_angle>getget_angle-170)
			YawPosCtrl(80-body_angle+xya.compare_angle);
			else if(xya.compare_angle<getget_angle-198&&xya.compare_angle>=-180 )
			YawPosCtrl(440-body_angle+xya.compare_angle);
			if(change_compere_angle<=-50&&change_compere_angle>=-125)
				pull_ball();
			else push_balltime=0;
		}else if((change_compere_angle<=180&&change_compere_angle>=135)||(change_compere_angle>=-180&&change_compere_angle<=-135))
		{
			point_x=-2400;
			point_y=4900;
		    diagonal=-1;
			x=get_d(point_x,point_y);
			body_angle=get_angle2(xya.y-point_y,-xya.x+point_x,(xya.y-point_y)/fabs((xya.y-point_y)),if_go);
			cartopoint_angle=body_angle+180;
			if(cartopoint_angle>180)
			   cartopoint_angle-=360;
			add_angle=get_addorreduce_angle(get_differ_angle(cartopoint_angle,if_go),x);
			body_angle=body_angle+add_angle*0.8;
			getget_angle=90+body_angle;
			if(xya.compare_angle<180&&xya.compare_angle>getget_angle-170)
			YawPosCtrl(80-body_angle+xya.compare_angle);
			else if(xya.compare_angle<getget_angle-198&&xya.compare_angle>=-180 )
			YawPosCtrl(440-body_angle+xya.compare_angle);
			if((change_compere_angle<=-140)||(change_compere_angle<=180&&change_compere_angle>=145))
			 pull_ball();
			else push_balltime=0;
		}else if(change_compere_angle<=135&&change_compere_angle>=45)
		{
			point_x=2400;
			point_y=4700;
           	diagonal=1;
			x=get_d(point_x,point_y);
			body_angle=get_angle2(xya.y-point_y,-xya.x+point_x,(xya.y-point_y)/fabs((xya.y-point_y)),if_go);
			cartopoint_angle=body_angle+180;
			if(cartopoint_angle>180)
			   cartopoint_angle-=360;
			add_angle=get_addorreduce_angle(get_differ_angle(cartopoint_angle,if_go),x);
			body_angle=body_angle+add_angle*0.8;
			getget_angle=90+body_angle;
			if(xya.compare_angle<getget_angle+172&&xya.compare_angle>=-180)
			YawPosCtrl(80-body_angle+xya.compare_angle);			
			else if(xya.compare_angle<=180&&xya.compare_angle>=getget_angle+198 )
				YawPosCtrl(280-body_angle+xya.compare_angle);
			if(change_compere_angle<=125&&change_compere_angle>=55)
				pull_ball();
			else push_balltime=0;
		}else if(change_compere_angle<=45&&change_compere_angle>=-45)
		{
			point_x=2280;
			point_y=-90;
			diagonal=-1;
			x=get_d(point_x,point_y);
			body_angle=get_angle2(xya.y-point_y,-xya.x+point_x,(xya.y-point_y)/fabs((xya.y-point_y)),if_go);
			cartopoint_angle=body_angle+180;
			if(cartopoint_angle>180)
			   cartopoint_angle-=360;
			add_angle=get_addorreduce_angle(get_differ_angle(cartopoint_angle,if_go),x);
			body_angle=body_angle+add_angle*0.8;
			getget_angle=-270+body_angle;
			if(xya.compare_angle<getget_angle+172&&xya.compare_angle>=-180)
			YawPosCtrl(440-body_angle+xya.compare_angle);			
			else if(xya.compare_angle<=180&&xya.compare_angle>=getget_angle+198 )
				YawPosCtrl(80-body_angle+xya.compare_angle);
			if(change_compere_angle<=40&&change_compere_angle>=-35)
				pull_ball();
			else push_balltime=0;
		}
	  s=get_d(point_x,point_y);
	}else 
	{
		if((change_compere_angle<=-135&&change_compere_angle>=-180)||(change_compere_angle>=135&&change_compere_angle<=180))
		{ 
			diagonal=1;
			point_x=-2400;
			point_y=0;		    
			body_angle=get_angle2(xya.y-point_y,-xya.x+point_x,(xya.y-point_y)/fabs((xya.y-point_y)),if_go);
			x=get_d(point_x,point_y);
			cartopoint_angle=body_angle-180;
			if(cartopoint_angle<-180)
			   cartopoint_angle+=360;
			add_angle=get_addorreduce_angle(get_differ_angle(cartopoint_angle,if_go),x);
			body_angle=body_angle-add_angle*0.9;
			getget_angle=90+body_angle;
			if(xya.compare_angle<180&&xya.compare_angle>getget_angle-170)
			YawPosCtrl(80-body_angle+xya.compare_angle+add_angle);
			else if(xya.compare_angle<getget_angle-198&&xya.compare_angle>=-180 )
			YawPosCtrl(440-body_angle+xya.compare_angle);
			if((change_compere_angle>=140)||(change_compere_angle>=-180&&change_compere_angle<=-145))
			 pull_ball();
			else push_balltime=0;
		}else if(change_compere_angle<=135&&change_compere_angle>=45)
		{   diagonal=-1;
			point_x=-2400;
			point_y=4700;
			body_angle=get_angle2(xya.y-point_y,-xya.x+point_x,(xya.y-point_y)/fabs((xya.y-point_y)),if_go);
			x=get_d(point_x,point_y);
			cartopoint_angle=body_angle-180;
			if(cartopoint_angle<-180)
			   cartopoint_angle+=360;
			add_angle=get_addorreduce_angle(get_differ_angle(cartopoint_angle,if_go),x);
			body_angle=body_angle-add_angle*0.9;
			getget_angle=90+body_angle;
			if(xya.compare_angle<180&&xya.compare_angle>getget_angle-170)
			YawPosCtrl(80-body_angle+xya.compare_angle);
			else if(xya.compare_angle<getget_angle-198&&xya.compare_angle>=-180 )
			YawPosCtrl(440-body_angle+xya.compare_angle+add_angle);
			if(change_compere_angle>=50&&change_compere_angle<=125)
				pull_ball();
			else push_balltime=0;
		}else if(change_compere_angle<=45&&change_compere_angle>=-45)
		{   diagonal=1;
			point_x=2400;
			point_y=4700;			
			body_angle=get_angle2(xya.y-point_y,-xya.x+point_x,(xya.y-point_y)/fabs((xya.y-point_y)),if_go);
			x=get_d(point_x,point_y);
			cartopoint_angle=body_angle+180;
			if(cartopoint_angle>180)
			   cartopoint_angle-=360;
			add_angle=get_addorreduce_angle(get_differ_angle(cartopoint_angle,if_go),x);
			body_angle=body_angle-add_angle*0.9;
			getget_angle=90+body_angle;
			if(xya.compare_angle<getget_angle+172&&xya.compare_angle>=-180)
			YawPosCtrl(80-body_angle+xya.compare_angle+add_angle);			
			else if(xya.compare_angle<=180&&xya.compare_angle>=getget_angle+198 )
				YawPosCtrl(280-body_angle+xya.compare_angle);
			if(change_compere_angle>=-40&&change_compere_angle<=35)
				pull_ball();
			else push_balltime=0;
		}else if(change_compere_angle<=-45&&change_compere_angle>=-135)
		{    diagonal=-1;
			point_x=2400;
			point_y=0;
			body_angle=get_angle2(xya.y-point_y,-xya.x+point_x,(xya.y-point_y)/fabs((xya.y-point_y)),if_go);
			x=get_d(point_x,point_y);
			cartopoint_angle=body_angle+180;
			if(cartopoint_angle>180)
			   cartopoint_angle-=360;
			add_angle=get_addorreduce_angle(get_differ_angle(cartopoint_angle,if_go),x);
			body_angle=body_angle-add_angle*0.9;
			getget_angle=-270+body_angle;
			if(xya.compare_angle<getget_angle+172&&xya.compare_angle>=-180)
			YawPosCtrl(440-body_angle+xya.compare_angle+add_angle);			
			else if(xya.compare_angle<=180&&xya.compare_angle>=getget_angle+198)
				YawPosCtrl(80-body_angle+xya.compare_angle);
			if(change_compere_angle>=-130&&change_compere_angle<=-55)
				pull_ball();
			else push_balltime=0;
		}
		s=get_d(point_x,point_y);
	}
	USART_OUT(UART4,(uint8_t*)"add_angle=%d\t",(int)add_angle);
	//USART_OUT(UART4,(uint8_t*)"push_balltime=%d\t",(int)push_balltime);
}
float get_angle2(float a,float b,int n,int round)
{ 	
	if(b!=0)
	{ if(n==1)
		{ if(-a/b>0)
		  {   
			  return(atan2(-a/b,1)/3.14159*180);
		  }
		  else if(-a/b<0)
		  {    
			  return(atan2(a/b,-1)/3.14159*180);
		  }
		}else
		{
			if(-a/b>0)
			{
		    return(atan2(a/b,-1)/3.14159*180);
				
			}
		  else if(-a/b<0)
		  {   
			 return(atan2(-a/b,1)/3.14159*180);
		  }
		}
	}
	else if(b==0)
	{
		if(round==-1)//顺指针
		{   if(n==1)
		   	{
				
				return(90);
			}
			else 
			{
				return(-90);
		       
				
			}
		}
        else 
		{   
			if(n==1)
		   {
			
				return(90);
			  
			}
			
			else 
			{
				return(-90);
		      
			}       			
		}
	} 
     
	if(a==0)
	{
		if(n==1)
		{  
			return(0);
		
		}
        else 
		{   
			return(-180);
            			
		}
		
	}	
}

//推球//
void pull_ball(void)
{
	push_balltime++;
	if(push_balltime==800||push_balltime==100)
	{ // 推球	
	  PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);
	}else if(push_balltime==200||push_balltime==320)
	{// 复位//	   
	  PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);
	}
	if(push_balltime>=220)
	  push_balltime=221;
	
}
//得到车与四个点的距离//
float get_d(float x,float y)
{
	return(sqrt(pow (xya.x-x,2)+pow(xya.y-y,2)));
}
//得到车与四个点的角度与 车的速度角度的插值//
float get_differ_angle(float angle,int round)
{
	float nowdiffer_angle;

	if(round==1)
	{if(xya.angle>-180&&xya.angle<angle-180)		
		nowdiffer_angle=angle-xya.angle-360;
	  else nowdiffer_angle=angle-xya.angle;
	}
	else if(round==-1)
	{if(xya.angle<180&&xya.angle>angle+180)		
		nowdiffer_angle=angle-xya.angle+360;

		else nowdiffer_angle=angle-xya.angle;
	}
	return(nowdiffer_angle);
}
//得到炮台相对于算出的角度应该加或者减的角//
float get_addorreduce_angle(float differ_angle,float x)
{
	float t=sqrt(2*(sqrt(3)*x-800)/9800);
	float x_v=x/t;
	float need_v=sqrt(pow(Add_V,2)+pow(x_v,2)-2*x_v*Add_V*cos(fabs(differ_angle)/180*Pi));
	float cos_x=(pow(need_v,2)+pow(x_v,2)-pow(Add_V,2))/(2*x_v*need_v);
	return(acos(cos_x)/Pi*180);
}