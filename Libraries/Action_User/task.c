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
#include "void.h"
#include "fort.h"
#define NEED_BALL_COLLOR (1)
#define NO_NEED_BALL_COLLOR (2)
uint8_t Ballcolor[5];
int Sn=1;//顺时针和逆时针
extern FortType fort;
extern int dec_flag;
extern int PushballFlag;
extern int AreaChange;
extern uint8_t start_ready;
float run_R=0;
float run_r=0;
float  buff_cnt=1;//控制半径buff
float  Buff_cnt=1;//控制速度buff
float run_V=0;
int trouble_flag=0;
uint8_t acceleration_ready=0;
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
	
	
	//TIM_Init(TIM2, 99, 839, 1, 0);
	BEEP_ON;
	// 配置位置环
	
	
	
	/*一直等待定位系统初始化完成*/
	delay_s(2);
	WaitOpsPrepare();
	VelLoopCfg(CAN1,1,32768*50,32768*80);//速度环初始化
	VelLoopCfg(CAN1,2,32768*50,32768*80);
	PosLoopCfg(CAN2, PUSH_BALL_ID, 10000000,10000000,10000000);
	MotorOn(CAN1,1);             //两轮电机初始化
	MotorOn(CAN1,2);             //两轮电机初始化
	MotorOn(CAN2,5);             //收球电机初始化
	MotorOn(CAN2,6);             //收球电机初始化
	MotorOn(CAN2,PUSH_BALL_ID);  //推球电机初始化
	
	VelCrl(CAN2,6,-80*32768);
	VelCrl(CAN2,5,80*32768);
//	while(AdcFlag()==0)
//		if(AdcFlag()>0)
		Sn=1;
//		if(AdcFlag()<0)
//		Sn=-1;	
//		start_mode();
	OSTaskSuspend(OS_PRIO_SELF);
}

void WalkTask(void)
{
	extern int stable_flag;
	CPU_INT08U os_err;
	os_err = os_err;
	int t=0;
	int PushBallPosition=32768/2;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		
		OSSemPend(PeriodSem, 0, &os_err);
		systerm_change();
		//推球方式:接收指令才能推球和复位,并且自动准备好进行下一次推球和复位//
//		if(PushballFlag==1)
//		{
//			t++;
//			if(t==20)
//			{
//				PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);
//			}
//			else if(t==120)
//			{
//				PushballFlag=0;
//			}
//			else if(t==141)
//			{
//				PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);	
//			}
//			else if(t==240)
//			{
//				t=0;
//				PushballFlag=0;
//			}
//			if(AreaChange==1)
//			{
//				t=0;
//			}
//		}
		//走行//	
//		if(start_ready==0)
//		run_r=startRadius();
//        if(start_ready==1)
//		run_r=Radius();
//		if(stable_flag==0)		
//		{
/*
			if(trouble_flag==0)//遇到障碍变轨是不要刷新标准半径
		{
		   buff_cnt+=2;
	       if(buff_cnt>=300)
	       {
			   buff_cnt=300;
			   acceleration_ready=1;
		   }
		   if(dec_flag==1&&((int)run_R!=900))//半径减小
		   run_r=run_R+300-buff_cnt;
		   
		   if(dec_flag==-1)//半径增大
		   run_r=run_R-300+buff_cnt;  
		   
		   if(dec_flag==0) 
		   run_r=400;//400启动半径
		   
		   if(dec_flag==1&&(int)run_R==900)
		   {
			 run_V=V_buff(1,5,0.0025,1.5);
			 run_r=R_buff(-1,6,4,900);
		   } 
		   if(!(dec_flag==1&&((int)run_R!=900)))//半径减小
		   run_V=run_r/600;
     	}
*/
/*
			if(trouble_flag==1&&run_r>=1.5)
			{
				run_V=run_r/600-1.2;
				run_V+=0.005;
				if(run_V>=(run_r/600))
				{
					run_V=run_r/600;
				}
			}
*/
//		Walkline(0,2350,run_r,run_V,Sn);   ////圆心X坐标  圆心Y坐标 半径  方向  速度
//		}
//		if(stable_flag==1)	
//		{
//		 
//		  Walkline(0,2300,1500,1.5,Sn);
          PushballFlag=1;
//		  			if(PushballFlag==1)
//			{
//				t++;
//				if(Ballcolor[2]==NEED_BALL_COLLOR&&t==20)
//				{
//					PushBallPosition+=32768;
//					USART_OUT( UART4, (uint8_t*)"%d\r\n ", Ballcolor[2]);
//				}
//				else if(Ballcolor[2]==NO_NEED_BALL_COLLOR&&t==1)
//				{
//					PushBallPosition-=32768;
//					USART_OUT( UART4, (uint8_t*)"%d\r\n ", Ballcolor[2]);
//				}
//				else if(Ballcolor[2]==0&&t==1)
//				{					
//					//PushBallPosition-=32768/2;	
//					USART_OUT( UART4, (uint8_t*)"%d\r\n ", Ballcolor[2]);					
//				}
//				if(t==100)
//				{
//					PushballFlag=0;
//					t=0;
//				}
//			}
//		  PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,PushBallPosition);
//			void move_gun(float,float);
//           move_gun(2400,0);
       push_ball();
	   ShooterVelCtrl(/*get_roll_v()*/25);		
			
		//航向电机//
//		  get_sendangle();	                         	////射球模块
	
//		}			
//		errdeal();////故障处理	
//     USART_OUT( UART4, (uint8_t*)"%d ", (int)run_R);
//     USART_OUT( UART4, (uint8_t*)"%d ", (int)run_r);
//      USART_OUT( UART4, (uint8_t*)"%d ", (int)(run_V*10));			
//         
//     USART_OUT( UART4, (uint8_t*)"%d ", (int)ReadLaserBValue());
//     USART_OUT( UART4, (uint8_t*)"%d ", (int)ReadLaserAValue());
//	 USART_OUT( UART4, (uint8_t*)"\r\n");

////////////////发数测试////////////////////////////
//	USART_OUT(UART4,(uint8_t*) "%d	%d	%d	%d\r\n",(int)(GetX()),(int)(GetY()),Radius(),(int)GetAngle());
//	USART_OUT(UART4,(uint8_t*) "%d	%d\r\n",(int)ReadLaserAValue(),(int)ReadLaserBValue());
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)GetX());
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)GetY());
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)run_r);
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)run_R);
//  USART_OUT( UART4, (uint8_t*)"\r\n");

   
	}
}
