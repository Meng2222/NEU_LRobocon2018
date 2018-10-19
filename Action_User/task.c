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
int BeforePushposition=0;
int LastpushBallposition;

//顺时针和逆时针
int clockFlg=1;
extern float go_real_send_angle;
extern union push_p push_position;
extern FortType fort;
extern int normal_push;
extern int PushballFlag;
extern int AreaChange;
extern int PushBallPosition;
float point_s[3][4]={2082,2469,2270,2422,0,0,0,0,2367,2349,2398,2352};
extern float s;
extern int point_number;
int get_cycle_num=0;
float recover_buff=0;

//走形设定的半径和速度
float run_R=0;
float run_r=0;
float run_V=0;
int if_white_shoot=0;

extern int deadZone;
extern int roundCnt;
extern int pointErrDeal;
extern int shoot_over;
extern int go_and_shoot;
int PressFlag=1;
extern int runAgain;
extern int nothingCanDo;
uint8_t acceleration_ready=0;
int no_BallTime=0,Out_AreaTime=0,Out_Area_noballTime=0,No_StableTime=0,No_Stable_noballTime=0;//推球的变量//
float get_angle2(float a,float b,int n,int round);//n指向,round顺逆时针，ax+by+c=0；
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
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);//can1初始化
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	USART3_Init(115200);
	UART4_Init(921600);
	UART5_Init(921600);
	TIM_Init(TIM2,1000-1,83,0x01,0x03);  /////中断位*10  主周期10ms
	Adc_Init();
	ElmoInit(CAN2);//驱动初始化
	ElmoInit(CAN1);//驱动初始化
	MotorOn(CAN2,5);             //收球电机初始化
	MotorOn(CAN2,6);             //收球电机初始化
	GPIO_Init_Pins(GPIOB,4,GPIO_Mode_IN); //初始化按键
	GPIO_Init_Pins(GPIOE,1,GPIO_Mode_IN); //初始化按键

	
	
	//TIM_Init(TIM2, 99, 839, 1, 0);
	BEEP_ON;
	// 配置位置环
	while(PressFlag==1)
	{
		PressFlag=Let_BallOut();
		//反转
		VelCrl(CAN2,6,80*32768);//80
		VelCrl(CAN2,5,-80*32768);//-80
	}
		VelCrl(CAN2,6,0*32768);
		VelCrl(CAN2,5,0*32768);

	
	/*一直等待定位系统初始化完成*/
	delay_s(4);
	WaitOpsPrepare();
	VelLoopCfg(CAN1,1,32768*10,32768*100);//速度环初始化
	VelLoopCfg(CAN1,2,32768*10,32768*100);
	PosLoopCfg(CAN2, PUSH_BALL_ID, 20000000,20000000,20000000);
	MotorOn(CAN1,1);             //两轮电机初始化
	MotorOn(CAN1,2);             //两轮电机初始化
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
	extern int stable_flag;
	CPU_INT08U os_err;
	os_err = os_err;
	int T=0;
 
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
//		MotorOff(CAN2,7);
   //   USART_OUT( UART4, (uint8_t*)"Ballcolor=%d ", Ballcolor[2]);
		systerm_change();	
	//	get_cycle_num=round_counter();
		ReadActualPos(CAN2, 7);
		run_r=Radius();
//		normal_push=1;
		if(stable_flag==0)
		{
		     Walkline(0,2350,run_r,run_V,clockFlg);   ////圆心X坐标  圆心Y坐标 半径  方向  速
			if(!pointErrDeal&&normal_push)//&&run_again
			{
				Out_AreaTime=0;		//射球圈  非射球区域的计时
				T=0;				//射球圈  射球区域的计时
				No_StableTime++;	//非射球圈的计时 周期为112
				no_BallTime=0;		//射球圈  没有球的计时
				if(Ballcolor[2]==Need_ball_collor)		//在非射球圈识别到白球 把非射球圈的没有球的计时置0 并且准备着
				{
					No_Stable_noballTime=0;	//非射球圈 没有球的计时
	//				USART_OUT( UART4, (uint8_t*)"PrepareOK%d\r\n ", Ballcolor[2]);
				}
				if(Ballcolor[2]==No_need_ball_collor&&No_StableTime==50)		//黑球 而且非射球圈的计时为50时
				{
					PushBallPosition-=32768;
					No_Stable_noballTime=0;
	//				USART_OUT( UART4, (uint8_t*)"%d\r\n ", Ballcolor[2]);
				}
				else if(Ballcolor[2]==0)	//没有球 持续130ms 反推
				{
					No_Stable_noballTime++;
					if(No_Stable_noballTime>=130)
					{
						PushBallPosition-=32768/2;
						No_Stable_noballTime=0;
	//					USART_OUT( UART4, (uint8_t*)"No_Stable_Change\r\n ");
					}
				}
				if(No_StableTime>=112)	
				{
					No_StableTime=0;
				}
			}
		}
		
		else if(stable_flag==1&&!runAgain)	
		 {
			 Walkline(0,2300,run_r,run_V,clockFlg);
             if(!pointErrDeal&&normal_push)//&&run_again/*&&normal_push*/
			 {		
				No_StableTime=0;		//进入射球圈 把非射球圈的计时全部置0
				No_Stable_noballTime=0;
				if(PushballFlag==1)		//如果进入射球区域
				{
					T++;				//射球圈计时开始
					Out_AreaTime=0;	
					if(Ballcolor[2]==Need_ball_collor&&fabs(s-3000)<20)		//白球 在t=90时射出
					{
						if_white_shoot=1;
						PushBallPosition+=32768;
						no_BallTime=0;
	//					USART_OUT( UART4, (uint8_t*)"tong__car s=%d\t ",(int) s);
	//					USART_OUT(UART4,(uint8_t*)" tong_number %d\t\r\n",(int)point_number);
					}
					else if(Ballcolor[2]==No_need_ball_collor&&T==10)	//黑球 在t=10时反推
					{
						PushBallPosition-=32768;
						no_BallTime
						=0;
	//					USART_OUT( UART4, (uint8_t*)"%d\r\n ", Ballcolor[2]);
					}
					else if(Ballcolor[2]==0)
					{
						no_BallTime++;
						if(no_BallTime>=150)
						{
							PushBallPosition-=32768/2;
							no_BallTime=0;
	//						USART_OUT( UART4, (uint8_t*)"Change\r\n ");	
						}
	//					USART_OUT( UART4, (uint8_t*)"%d\r\n ", Ballcolor[2]);					
					}
					if(T>=112)
					{
						T=0;
					}
				}
				else if(PushballFlag==0)		//射球圈非射球区域
				{
					Out_AreaTime++;				//射球圈非射球区域计时
					T=0;
					if(Ballcolor[2]==Need_ball_collor)
					{
						Out_Area_noballTime=0;
	//					USART_OUT( UART4, (uint8_t*)"PrepareOK%d\r\n ", Ballcolor[2]);
					}				
					if(Ballcolor[2]==No_need_ball_collor&&Out_AreaTime==50)
					{
						PushBallPosition-=32768;
						Out_Area_noballTime=0;
	//					USART_OUT( UART4, (uint8_t*)"%d\r\n ", Ballcolor[2]);
					}
					else if(Ballcolor[2]==0)
					{
						Out_Area_noballTime++;
						if(Out_Area_noballTime>=150)
						{
							PushBallPosition-=32768/2;
							Out_Area_noballTime=0;
	//						USART_OUT( UART4, (uint8_t*)"OutArea_Change\r\n ");	
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
	   PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,PushBallPosition);
	
		if(pointErrDeal==0&&!runAgain)//&&run_again
	{		
       get_sendangle();                                                 //谢尚锦
      // push_ball();                                                     //谢尚锦
         
	}
	
	    if(abs(LastpushBallposition-PushBallPosition)>16380)
		{
			BeforePushposition=LastpushBallposition;
			LastpushBallposition=PushBallPosition;
		}		 
		if(shoot_over==1)
		{
			if(nothingCanDo==1)//被卡死打定点，打完之后
			{
				roundCnt=1;//回到第一圈重新开始
				nothingCanDo=0;
			}else 
			{
				pointErrDeal=0;
				shoot_over=0;
			}
		}
		errdeal();////故障处理		

		debugdata();
//		USART_OUT( UART4, (uint8_t*)"round: %d",roundCnt);
//		USART_OUT( UART4, (uint8_t*)"\r\n");
	//	USART_OUT( UART4, (uint8_t*)"%d \r\n", (int)point_errdeal);




////////////////发数测试////////////////////////////
//	USART_OUT(UART4,(uint8_t*) "%d	%d	%d	%d\r\n",(int)(GetX()),(int)(GetY()),Radius(),(int)GetAngle());
//	USART_OUT(UART4,(uint8_t*) "%d	%d\r\n",(int)ReadLaserAValue(),(int)ReadLaserBValue());
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)GetX());
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)GetY());
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)run_r);
//USART_OUT( UART4, (uint8_t*)"%d ", (int)get_angle2(GetY()-2400,-GetX()+0,(GetY()-2400)/fabs((GetY()-2400)),Sn));
//  ShooterVelCtrl(80);
 // USART_OUT( UART4, (uint8_t*)"\r\n");

   
	}
}
