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
#include "pps.h"
#include "stm32f4xx_adc.h"
#include "fort.h"

#define Pulse2mm COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi
#define aa 1
#define bb 0
#define cc 0
//y大于0的FB都等1
#define FB 2
#define xx 0
#define yy -2300    //18:-2330  17:-2300
#define r 1760
//顺1逆2
#define Set_Clock 1
#define Speed 1000
#define COLLECT_BALL_ID (8)
#define PUSH_BALL_ID (6)
#define PUSH_POSITION (4500)
#define PUSH_RESET_POSITION (5)
#define GUN_YAW_ID (7)
#define COUNT_PER_ROUND (4096.0f)
#define COUNT_PER_DEGREE  (COUNT_PER_ROUND/360.0f)
#define YAW_REDUCTION_RATIO (4.0f)
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
int startflag=1;
float angle,fixang,Rps,d;
extern int posokflag;
int left,right;
int front_vel,behind_vel;
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM_Init(TIM2,999,83,0x00,0x00);
	#if CAR_CONTRAL==17
  //Adc_Init();
	UART4_Init(921600);
	USART3_Init(115200);
  UART5_Init(921600);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN1);
	ElmoInit(CAN2);
	//轮子 左2右1
	VelLoopCfg(CAN2,1,1000,0);
	VelLoopCfg(CAN2,2,1000,0);
	VelLoopCfg(CAN1, 8, 50000, 50000);
  PosLoopCfg(CAN1, 6, 500000,500000,200000);
	MotorOn(CAN2,1);
  MotorOn(CAN2,2);
  MotorOn(CAN1,8);
  MotorOn(CAN1,6);
	delay_s(2);
	WaitOpsPrepare();
//	while(!startflag)
//	{
//	  left=Get_Adc_Average(15,5);
//	  right=Get_Adc_Average(14,5);
//	  if(left<100)
//		  startflag=1;
//	  else if(right<100)
//		  startflag=2;
//	  else startflag=0;
//		USART_OUT(USART1,(uint8_t*)"%d\t%d\t%d\r\n",left,right,startflag);
//	}
	#elif CAR_CONTRAL==18
	USART1_Init(921600);
	USART3_Init(115200);
  UART5_Init(921600);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	//轮子后5前6
	VelLoopCfg(CAN2,6,10000000,10000000);
  VelLoopCfg(CAN2,5,10000000,10000000);
	MotorOn(CAN2,5);
  MotorOn(CAN2,6);
	delay_s(2);
	WaitOpsPrepare();
	#endif
	OSTaskSuspend(OS_PRIO_SELF);
}

int mark=0;
float Out_Agl=0;
int vel1,vel2,vel,turnflag=0;
float X,Y;
int Out_Pulse;
int flag=0,Flag=0,T_Flag=0,min=0;
float R_cover=560;
extern Pos_t position;

void PID(int Agl_Flag);
void PID_Agl(float Set_Angle);
void PID_Awy(float D_err);
void Open_Round(float speed,float R);
void N_Open_Round(float speed1,float round);
void Squart_Walk(void);
void Set_Square(void);
void Strght_Walk (float a,float b,float c,int F_B);
void N_Strght_Walk (float a,float b,float c,int F_B);
void SET_closeRound(float x,float y,float R,float clock,float speed);
void N_SET_closeRound(float x,float y,float R,float clock,float speed);
void Round_Cover(void);
void N_Round_Cover(void);
void GPS_Aim_Pitch(void);

int timCnt=0,flag1=0,flag2=0,sign=0,pushflag=0,pushCnt1=0,pushCnt2=0,lastpushflag;
int laserflag=0,courseflag=0;
float lastangle,diffangle,Angle;
extern FortType fort;
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		#if CAR_CONTRAL==18
		//18非差速转换坐标原点
		X=position.ppsX-OPS_TO_BACK_WHEEL*sin(position.ppsAngle*Pi/180);
		Y=position.ppsY-OPS_TO_BACK_WHEEL+OPS_TO_BACK_WHEEL*cos(position.ppsAngle*Pi/180); 
		#endif
		
    GPS_Aim_Pitch();
		//Round_Cover();
		//N_Round_Cover();
    SET_closeRound(xx,yy,r,startflag,Speed);
		//N_SET_closeRound(xx,yy,r,Set_Clock,Speed);
		//N_Open_Round(500,500);
		//Square_Walk();
		//Set_Square();
		//Strght_Walk (aa,bb,cc,FB);
		//N_Strght_Walk (aa,bb,cc,FB);
		//Open_Round(0.1,0.2);
	}
}

void LASER_Aim_Pitch(void)
{
	if(!laserflag)
	{
		if(Angle<=0) courseflag=1;
		else if(Angle>=342) courseflag=0;
		if(courseflag) Angle=Angle+1;
		else Angle=Angle-1;
	}
	if((fort.laserAValueReceive>1350)&&(fort.laserBValueReceive>400&&fort.laserBValueReceive<1300))
    laserflag=1;
	if(laserflag)
	{
		courseflag=1;
		//一直推球+意境瞄准给墙转速投球
		
	}
}
void GPS_Aim_Pitch(void)
{
	VelCrl(CAN1,COLLECT_BALL_ID,60*4096);
	if(startflag==1)
	{
		if(position.ppsY>position.ppsX-2300&&position.ppsY>=-position.ppsX-2300)
		{
			fixang=atan((100-position.ppsY)/(2400-position.ppsX))*180/Pi+90;
			d=sqrt(pow(position.ppsX-2200,2)+pow(position.ppsY+100,2));
		}
		else if(position.ppsY<=position.ppsX-2300&&position.ppsY>-position.ppsX-2300)
		{
			fixang=atan((-4700-position.ppsY)/(2400-position.ppsX))*180/Pi+90;
			d=sqrt(pow(position.ppsX-2200,2)+pow(position.ppsY+4500,2));
		}
		else if(position.ppsY<position.ppsX-2300&&position.ppsY<=-position.ppsX-2300)
		{
			fixang=atan((-4700-position.ppsY)/(-2400-position.ppsX))*180/Pi-90;
			d=sqrt(pow(position.ppsX+2200,2)+pow(position.ppsY+4500,2));
		}
		else if(position.ppsY>=position.ppsX-2300&&position.ppsY<-position.ppsX-2300)
		{
			fixang=atan((100-position.ppsY)/(-2400-position.ppsX))*180/Pi-90;
			d=sqrt(pow(position.ppsX+2200,2)+pow(position.ppsY+100,2));
		}
		if((position.ppsY>-2*position.ppsX-2300&&position.ppsY>position.ppsX-2300)
		||(position.ppsY<0.5*position.ppsX-2300&&position.ppsY>-position.ppsX-2300)
		||(position.ppsY<-2*position.ppsX-2300&&position.ppsY<position.ppsX-2300)
		||(position.ppsY>0.5*position.ppsX-2300&&position.ppsY<-position.ppsX-2300))
		  pushflag=1;
		else pushflag=0;
	}
	else if(startflag==2)
	{
    if(position.ppsY>=position.ppsX-2300&&position.ppsY>-position.ppsX-2300)
		{
			fixang=atan((100-position.ppsY)/(-2400-position.ppsX))*180/Pi-90;
			d=sqrt(pow(position.ppsX+2200,2)+pow(position.ppsY+100,2));
		}
	  else if(position.ppsY>position.ppsX-2300&&position.ppsY<=-position.ppsX-2300)
	  {
		  fixang=atan((-4700-position.ppsY)/(-2400-position.ppsX))*180/Pi-90;
		  d=sqrt(pow(position.ppsX+2200,2)+pow(position.ppsY+4500,2));
	  }
		else if(position.ppsY<=position.ppsX-2300&&position.ppsY<-position.ppsX-2300)
		{
			fixang=atan((-4700-position.ppsY)/(2400-position.ppsX))*180/Pi+90;
			d=sqrt(pow(position.ppsX-2200,2)+pow(position.ppsY+4500,2));
		}
		else if(position.ppsY<position.ppsX-2300&&position.ppsY>=-position.ppsX-2300)
		{
			fixang=atan((100-position.ppsY)/(2400-position.ppsX))*180/Pi+90;
      d=sqrt(pow(position.ppsX-2200,2)+pow(position.ppsY+100,2));
		}
		if((position.ppsY>2*position.ppsX-2300&&position.ppsY>-position.ppsX-2300)
		||(position.ppsY<-0.5*position.ppsX-2300&&position.ppsY>position.ppsX-2300)
		||(position.ppsY<2*position.ppsX-2300&&position.ppsY<-position.ppsX-2300)
		||(position.ppsY>-0.5*position.ppsX-2300&&position.ppsY<position.ppsX-2300))
		  pushflag=1;
		else pushflag=0;
	}
	diffangle=fixang-position.ppsAngle;
	if(diffangle>180)
		diffangle=diffangle-360;
	else if(diffangle<-180)
		diffangle=diffangle+360;
	angle=170-diffangle;
	if(angle<0) angle=0;
	else if(angle>342) angle=342;
  YawPosCtrl(angle);
	if(lastpushflag==0&&pushflag==1)
		pushCnt1=0;
	lastpushflag=pushflag;
	if(pushflag)
	{
		pushCnt1++;
		pushCnt1=pushCnt1%1000;
		if(pushCnt1%50==0)
		{
		 	pushCnt2++;
		  pushCnt2=pushCnt2%100;
		}
		if(pushCnt2%2)
		  PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);
		else
			PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);
	}
	Rps=40.0/3102.505*d+35.7-9;
	ShooterVelCtrl(Rps);
}
float Agl;
//17差速覆盖大部分 倒退的临界速度要改  转换半径改成了400
void Round_Cover(void)
{
	static int count=0,lastcount=0,set=0,number=0,figure=0,Cnt=0,stopflag=0;
	Cnt++;
	if(Cnt>=100) Cnt=100;//限幅
	if(Cnt>50)//防止一开局的停止进入这里
	{
	  if(sqrt(pow(position.ppsSpeedX,2)+pow(position.ppsSpeedY,2))<200)
		  number++;
	  else number=0;
	  if(number>50)//长时间停下 倒退标志位 置1
	  {
		  number=0;
		  stopflag=1;
	  }
  }
	if(stopflag)//倒退 改变半径
	{
		figure++;
		VelCrl(CAN2,1,-10865);
	  VelCrl(CAN2,2,10865);
		if(figure>100)//后退0.5秒后出去以新半径转圈
		{
			figure=0;
			stopflag=0;
			if(R_cover>700)
				R_cover=R_cover-400;
			else
			{
				min++;
				min=min%10;
				if(min%2)
					R_cover=R_cover+400;
				else
					R_cover=R_cover+800;
			}
		}
	}
	else//转圈
	{
	  SET_closeRound(xx,yy,R_cover,startflag,Speed);
	  if(startflag==1)
	  {
		  if(Agl>130&&Agl<132)
		    count=1;
	    else 
		    count=0;
	    if(Agl>-90&&Agl<0)
		    set=1;
    }
  	else if(startflag==2)
	  {
		  if(Agl>-132&&Agl<-130)
		    count=1;
	    else 
	     	count=0;
		  if(Agl>0&&Agl<90)
		    set=1;
    }
	  if(R_cover>2000)
		  Flag=1;
	  else if(R_cover<700)
		  Flag=0;
	  if(count==1&&lastcount==0&&set)
	  {
		  if(Flag)
			  R_cover=R_cover-400;
		  else
		  	R_cover=R_cover+400;
		  set=0;
	  }
	  lastcount=count;
  }

}

//18非差速覆盖大部分
void N_Round_Cover(void)
{
	static int count=0,lastcount=0,set=0,number=0,figure=0,Cnt=0,stopflag=0;
	Cnt++;
	if(Cnt>=100) Cnt=100;//限幅
	if(Cnt>50)//防止一开局的停止进入这里
	{
	  if(sqrt(pow(position.ppsSpeedX,2)+pow(position.ppsSpeedY,2))<200)
		  number++;
	  else number=0;
	  if(number>50)//长时间停下 倒退标志位 置1
	  {
		  number=0;
		  stopflag=1;
	  }
  }
	USART_OUT(USART1,(uint8_t*)"%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)(X),(int)(Y),Cnt,number,stopflag,(int)(R_cover));
	if(stopflag)//倒退 改变半径
	{
		figure++;
		VelCrl(CAN2,5,-12730*REDUCTION_RATIO);
	  VelCrl(CAN2,6,0);
		if(figure>100)//后退0.5秒后出去以新半径转圈
		{
			figure=0;
			stopflag=0;
			if(R_cover>700)
				R_cover=R_cover-300;
			else
			{
				min++;
				min=min%10;
				if(min%2)
					R_cover=R_cover+600;
				else
					R_cover=R_cover+300;
			}
		}
	}
	else//转圈
	{
	  N_SET_closeRound(xx,yy,R_cover,startflag,Speed);
	  if(startflag==1)
	  {
		  if(Agl>93&&Agl<95)
		    count=1;
	    else 
		    count=0;
	    if(Agl>-90&&Agl<0)
		    set=1;
    }
  	else if(startflag==2)
	  {
		  if(Agl>-95&&Agl<-93)
		    count=1;
	    else 
	     	count=0;
		  if(Agl>0&&Agl<90)
		    set=1;
    }
	  if(R_cover>2000)
		  Flag=1;
	  else if(R_cover<700)
		  Flag=0;
	  if(count==1&&lastcount==0&&set)
	  {
		  if(Flag)
			  R_cover=R_cover-300;
		  else
		  	R_cover=R_cover+300;
		  set=0;
	  }
	  lastcount=count;
  }
}

//17差速走任意固定圆   要改KP:400
void SET_closeRound(float x,float y,float R,float clock,float speed)
{
	float Distance=0;
	float k;
	Distance=sqrt(pow(position.ppsX-x,2)+pow(position.ppsY-y,2))-R;
	k=(position.ppsX-x)/(y-position.ppsY);
	//顺1逆2
	if(clock==1)
	{
		if(position.ppsY>y)
		  Agl=90+atan(k)*180/Pi;
	  else if(position.ppsY<y)
		  Agl=-90+atan(k)*180/Pi;
	  else if(position.ppsY==y&&position.ppsX>=x)
		  Agl=0;
	  else if(position.ppsY==y&&position.ppsX<x)
		  Agl=180;
	  PID_Awy(Distance);
		PID_Agl(Agl-Out_Agl);
	}
	else if(clock==2)
	{
		if(position.ppsY>y)
		  Agl=-90+atan(k)*180/Pi;
	  else if(position.ppsY<y)
		  Agl=90+atan(k)*180/Pi;
	  else if(position.ppsY==y&&position.ppsX>=x)
		  Agl=180;
	  else if(position.ppsY==y&&position.ppsX<x)
		  Agl=0;
	  PID_Awy(Distance);
		PID_Agl(Agl+Out_Agl);
	}
	vel=(int)(speed*4096/(Pi*WHEEL_DIAMETER));
	VelCrl(CAN2,1,vel+Out_Pulse);
	VelCrl(CAN2,2,-vel+Out_Pulse);
}

//18非差速任意固定圆   要改KP:5000
void N_SET_closeRound(float x,float y,float R,float clock,float speed)
{
	float Distance=0;
	float k;
	Distance=sqrt(pow(X-x,2)+pow(Y-y,2))-R;
	k=(X-x)/(y-Y);
	//顺1逆2
	if(clock==1)
	{
		if(Y>y)
		  Agl=90+atan(k)*180/Pi;
	  else if(Y<y)
		  Agl=-90+atan(k)*180/Pi;
	  else if(Y==y&&X>=x)
		  Agl=0;
	  else if(Y==y&&X<x)
		  Agl=180;
	  PID_Awy(Distance);
		PID_Agl(Agl-Out_Agl);
	}
	else if(clock==2)
	{
		if(Y>y)
		  Agl=-90+atan(k)*180/Pi;
	  else if(Y<y)
		  Agl=90+atan(k)*180/Pi;
	  else if(Y==y&&X>=x)
		  Agl=180;
	  else if(Y==y&&X<x)
		  Agl=0;
	  PID_Awy(Distance);
		PID_Agl(Agl+Out_Agl);
	}
	vel=(int)(speed*8196*REDUCTION_RATIO/(Pi*WHEEL_DIAMETER));
	VelCrl(CAN2,5,vel);
	VelCrl(CAN2,6,-Out_Pulse*REDUCTION_RATIO);
}


//原1号车-17差速1m/s走任0意固定直线PID：通过距离PID的输出值输入到角度PID，让距离越来越近的同时角度越来越接近设定角度值
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

//18非差速-任意固定直线
void N_Strght_Walk (float a,float b,float c,int F_B)
{
	float k,Agl,Dis;
	Dis=fabs(a*X+b*Y+c)/sqrt(pow(a,2)+pow(b,2));
	if(b==0)
  {
	  if(X>-c/a) 
		{
			if(F_B==1)
			{
				PID_Awy(Dis);
				PID_Agl(180+Out_Agl);
			}
			else if(F_B==2)
			{
				PID_Awy(Dis);
				PID_Agl(-Out_Agl);
			}
		}
		else if(X<-c/a)
		{
			if(F_B==1)
			{
				PID_Awy(Dis);
				PID_Agl(180-Out_Agl);
			}
			else if(F_B==2)
			{
				PID_Awy(Dis);
				PID_Agl(Out_Agl);
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
	else if(a==0)
	{
	  if(Y>-c/b) 
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
		else if(Y<-c/b)
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
  else
  {
		k=-a/b;
		Agl=(atan(-a/b))*180/Pi;
	  if(k>0)
	  {
	    if(Y<k*X-c/b)
			{
				if(F_B==1)
				{
					PID_Awy(Dis);
					PID_Agl(Agl+90+Out_Agl);
				}
				else if(F_B==2)
				{
					PID_Awy(Dis);
				  PID_Agl(Agl-90-Out_Agl);
				}
			}
		  else if(Y>k*X-c/b)
			{
				if(F_B==1)
				{
					PID_Awy(Dis);
			    PID_Agl(Agl+90-Out_Agl);
				}
				else if(F_B==2)
				{
					PID_Awy(Dis);
				  PID_Agl(Agl-90+Out_Agl);
				}
			}
		  else 
			{
				if(F_B==1)
			    PID_Agl(Agl+90);
				else if(F_B==2)
				  PID_Agl(Agl-90);
			}
	  }
	  else if(k<0)
	  {
		  if(Y>k*X-c/b)
			{
			  if(F_B==1)
				{
					PID_Awy(Dis);
			    PID_Agl(Agl-90+Out_Agl);
				}
				else if(F_B==2)
				{
					PID_Awy(Dis);
				  PID_Agl(Agl+90-Out_Agl);
				}
			}
		  else if(Y<k*X-c/b)
			{
			  if(F_B==1)
				{
					PID_Awy(Dis);
			    PID_Agl(Agl-90-Out_Agl);
				}
				else if(F_B==2)
				{
					PID_Awy(Dis);
				  PID_Agl(Agl+90+Out_Agl);
				}
			}
		  else 
			{
			  if(F_B==1)
			    PID_Agl(Agl-90);
				else if(F_B==2)
				  PID_Agl(Agl+90);
			}
	  }
  }
	VelCrl(CAN2,5,Speed*REDUCTION_RATIO*8196/(Pi*WHEEL_DIAMETER));
  VelCrl(CAN2,6,-Out_Pulse*REDUCTION_RATIO);
}


//原1号车-17差速一轮向前一轮向后原地转-固定某一正方形原1号车
/**void Set_Square(void)

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

//我的原1号车走一个正方形-仅角度闭环
/**void Squart_Walk(void)
{
	switch(flag)
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

//17差速开环圆
void Open_Round(float speed,float R)
{
	vel1=(int)(10865*speed*(R-0.217)/R);
  vel2=-(int)(10865*speed*(R+0.217)/R);
}
//18非差速开环圆
void N_Open_Round(float speed1,float round) 
{
	float speed2;
	speed2=speed1*TURN_AROUND_WHEEL_TO_BACK_WHEEL/round;
	behind_vel=(int)(speed1*REDUCTION_RATIO*8192/(Pi*WHEEL_DIAMETER));
	front_vel=(int)(speed2*REDUCTION_RATIO*8192/(Pi*TURN_AROUND_WHEEL_DIAMETER));
	//USART_OUT(USART1,(uint8_t*)"%d\t%d\t%d\r\n",behind_vel,front_vel,speed2);
	VelCrl(CAN2,5,behind_vel);
	VelCrl(CAN2,6,front_vel);
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

void PID_Agl(float Set_Angle)
{
	float A_err;
	static float Last_Aerr,Sum_Aerr;
	A_err=Set_Angle-position.ppsAngle;
	if(A_err>180)
		A_err=A_err-360;
	if(A_err<-180)
		A_err=A_err+360;
	Sum_Aerr=Sum_Aerr+A_err;
	Out_Pulse=(int)(400*A_err);
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
