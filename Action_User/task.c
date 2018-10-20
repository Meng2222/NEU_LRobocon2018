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
#include "pps.h"
#include "fort.h"
#include "Walk.h" 
#include "PushBall.h" 
#include "Point.h" 
#include "PushBall.h"
#include "DebugData.h"
void boomAccident();
uint8_t Ballcolor[5];
int beforePushPos=0;
int lastPushPos;

//顺时针和逆时针
int clockFlg=1;
extern float goRealAng;
extern union push_p push_position;
extern FortType fort;
extern int normalPush;
extern int pushBallFlag;
extern int AreaChange;
extern int pushBallpos;
float point_s[3][4]={2082,2469,2270,2422,0,0,0,0,2367,2349,2398,2352};
extern int point_number;
int get_cycle_num=0;
float recover_buff=0;

//走形设定的半径和速度
float run_R=0;
float run_r=0;
float run_V=0;
int ifNeedshoot=0;
extern float carToDointD;
extern int deadZone;
extern int roundCnt;
extern int pointErrDeal;
extern int shoot_over;
extern int go_and_shoot;
int PressFlag=1;
extern int runAgain;
extern int nothingCanDo;
uint8_t acceleration_ready=0;

//推球的变量//
int no_BallTime=0,Out_AreaTime=0,Out_Area_noballTime=0,No_StableTime=0,No_Stable_noballTime=0;

//n指向,round顺逆时针，ax+by+c=0；
float CountAngle(float a,float b,int n,int round);
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

	//can1 can2初始化
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	USART3_Init(115200);
	UART4_Init(921600);
	UART5_Init(921600);

	 //中断位*10  主周期10ms
	TIM_Init(TIM2,1000-1,83,0x01,0x03); 
	Adc_Init();

	//驱动初始化
	ElmoInit(CAN2);

	//驱动初始化
	ElmoInit(CAN1);

	//收球电机初始化
	MotorOn(CAN2,5);             

	//收球电机初始化
	MotorOn(CAN2,6);             

	 //初始化按键
	GPIO_Init_Pins(GPIOB,4,GPIO_Mode_IN);

	//初始化按键
	GPIO_Init_Pins(GPIOE,1,GPIO_Mode_IN); 

	//TIM_Init(TIM2, 99, 839, 1, 0);
	BEEP_ON;
	
	// 配置位置环
	while(PressFlag==1)
	{
		PressFlag=LetBallOut();
	
		//反转
		VelCrl(CAN2,6,80*32768);//80
		VelCrl(CAN2,5,-80*32768);//-80
	}
		VelCrl(CAN2,6,0*32768);
		VelCrl(CAN2,5,0*32768);

	/*一直等待定位系统初始化完成*/
	delay_s(4);
	WaitOpsPrepare();
	VelLoopCfg(CAN1,1,32768*10,32768*100);
	VelLoopCfg(CAN1,2,32768*10,32768*100);
	PosLoopCfg(CAN2, PUSH_BALL_ID, 20000000,20000000,20000000);
	
	//两轮电机初始化
	MotorOn(CAN1,1);             
	MotorOn(CAN1,2);             

	//	MotorOn(CAN2,PUSH_BALL_ID);  //推球电机初始化
	VelCrl(CAN2,6,-80*32768);//-60
	VelCrl(CAN2,5,80*32768);//60
	AdcFlag();
	start_mode();
	YawPosCtrl(0);
	OSTaskSuspend(OS_PRIO_SELF);
}

void WalkTask(void)
{
	extern int stableFlg;
	CPU_INT08U os_err;
	os_err = os_err;
	int T=0;
 
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		systerm_change();	

		ReadActualPos(CAN2, 7);
		run_r=Radius();

		if(stableFlg==0)
		{
			//执行走形
		     Walkline(0,2350,run_r,run_V,clockFlg);   
			if(!pointErrDeal&&normalPush)
			{
				Out_AreaTime=0;		//射球圈  非射球区域的计时
				T=0;				//射球圈  射球区域的计时
				No_StableTime++;	//非射球圈的计时 周期为112
				no_BallTime=0;		//射球圈  没有球的计时

				//在非射球圈识别到白球 把非射球圈的没有球的计时置0 并且准备着
				if(Ballcolor[2]==Need_ball_collor)		
				{
					No_Stable_noballTime=0;	//非射球圈 没有球的计时

				}

				//黑球 而且非射球圈的计时为50时
				if(Ballcolor[2]==No_need_ball_collor&&No_StableTime==50)						{
					pushBallpos-=32768;
					No_Stable_noballTime=0;

				}

				//没有球 持续130ms 反推
				else if(Ballcolor[2]==0)	
				{
					No_Stable_noballTime++;
					if(No_Stable_noballTime>=130)
					{
						pushBallpos-=32768/2;
						No_Stable_noballTime=0;

					}
				}
				if(No_StableTime>=112)	
				{
					No_StableTime=0;
				}
			}
		}
		
		else if(stableFlg==1&&!runAgain)	
		 {
			 Walkline(0,2300,run_r,run_V,clockFlg);
             if(!pointErrDeal&&normalPush)
			 {		

				 //进入射球圈 把非射球圈的计时全部置0
				No_StableTime=0;		
				No_Stable_noballTime=0;
				if(pushBallFlag==1)		//如果进入射球区域
				{
					T++;				//射球圈计时开始
					Out_AreaTime=0;	
					if(Ballcolor[2]==Need_ball_collor&&fabs(carToDointD-3000)<20)		//白球 在t=90时射出
					{
						ifNeedshoot=1;
						pushBallpos+=32768;
						no_BallTime=0;
					}
					else if(Ballcolor[2]==No_need_ball_collor&&T==10)	//黑球 在t=10时反推
					{
						pushBallpos-=32768;
						no_BallTime
						=0;

					}
					else if(Ballcolor[2]==0)
					{
						no_BallTime++;
						if(no_BallTime>=150)
						{
							pushBallpos-=32768/2;
							no_BallTime=0;
	
						}
					
					}
					if(T>=112)
					{
						T=0;
					}
				}
				else if(pushBallFlag==0)		//射球圈非射球区域
				{
					Out_AreaTime++;				//射球圈非射球区域计时
					T=0;
					if(Ballcolor[2]==Need_ball_collor)
					{
						Out_Area_noballTime=0;

					}				
					if(Ballcolor[2]==No_need_ball_collor&&Out_AreaTime==50)
					{
						pushBallpos-=32768;
						Out_Area_noballTime=0;

					}
					else if(Ballcolor[2]==0)
					{
						Out_Area_noballTime++;
						if(Out_Area_noballTime>=150)
						{
							pushBallpos-=32768/2;
							Out_Area_noballTime=0;
	
						}						
					}
					if(Out_AreaTime>=112)
					{
						Out_AreaTime=0;
					}
				}
			}			 
		 }
		 PushBallErrorDeal();
	   PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushBallpos);
	
		if(pointErrDeal==0&&!runAgain)
	{		
       GetSendAngle();                                                          
	}
	
	    if(abs(lastPushPos-pushBallpos)>16380)
		{
			beforePushPos=lastPushPos;
			lastPushPos=pushBallpos;
		}		 
		if(shoot_over==1)
		{
			if(nothingCanDo==1)
			{
				roundCnt=1;
				nothingCanDo=0;
			}else 
			{
				pointErrDeal=0;
				shoot_over=0;
			}
		}

		//故障处理
		errdeal();		

		//蓝牙发数
		debugdata();

   
	}
}
