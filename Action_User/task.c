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
#include "pid.h"
#include "adc.h"
#include "pps.h"
#include "gun.h"
#include "fort.h"
/*
===============================================================
						信号量定义
===============================================================
*/
extern char updownFlag;
extern char pposokflag;
int Sign = 1;
int cnt1 = 0;//记录倒车时间
char cnt2 = 0;
int cnt3 = 0;
char changeR = 0;//记录如何改变半径
char taskFlag = 0;//切换运动状态标志
float distanceL = 0,distanceR = 0;//定义ADC左右距离
char dirFlag = 1;//直线运动方向
char changeFlag = 0;//切换直线标志
extern char updownFlag;//在线左右标志
char inAngleFlag = 0;//进入规定角度范围
extern float setAngle;//直线设置角度
extern float setAngle_R;//圆形设置角度
extern float errorRadius;//圆心距离;
extern float errorAngle,errorDis;//等于位置环输出角度-实际角度，位置偏差
extern float phase1,phase2,uOutAngle,uOutDis;//角度PID输出，位置PID输出
extern float Uout;//PID计算的输出值
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
PId_t Angle_PId = {
	.Kp = 14,
	.Ki = 0,
	.Kd = 1.2,
	.sumErr = 0,
	.lastErr = 0	
};
PId_t Dis_PId = {
	.Kp = 0.2 * 400 / SPEED,
	.Ki = 0.28 * 0.01 * 800 / SPEED,
	.Kd = 0,
	.sumErr = 0,
	.lastErr = 0
};
LastPos_t LastPos = {
	.x = 100,
	.y = 100,
	.angle = 0
};
//初始化圆形结构体
Round_t Rnd_PID ={
	.x = 0,
	.y = 2400,
	.radius = 1800,
	.vel = SPEED,
	.side = 1 //-1为顺,1为逆
};
char posFlag = 0;//需要存位置标志
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	UART4_Init(921600);
	USART1_Init(115200);
	UART5_Init(921600);
	USART3_Init(115200);
	Adc_Init();
	
	TIM_Init(TIM2, 99, 839, 1, 0);
	TIM_Init(TIM2, 1000-1, 84-1, 0x01, 0x03); //定时器初始化1ms
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	ElmoInit(CAN1);
	VelLoopCfg(CAN2, 1, 500000, 500000);
	VelLoopCfg(CAN2, 2, 500000, 500000);
	//棍子收球电机 配置速度环
	VelLoopCfg(CAN1,COLLECT_BALL_ID, 50000, 50000);
	
	//推球电机 配置位置环
	PosLoopCfg(CAN1, PUSH_BALL_ID, 50000,50000,20000);
	
	//航向电机配置位置环
	PosLoopCfg(CAN1, GUN_YAW_ID, 50000,50000,20000);
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	//棍子收球电机
	MotorOn(CAN1, COLLECT_BALL_ID);
	//推球电机
	MotorOn(CAN1, PUSH_BALL_ID);
	//航向电机
	MotorOn(CAN1, GUN_YAW_ID);
	
	delay_ms(2000);

	/*一直等待定位系统初始化完成*/
	BEEP_ON;
	//WaitOpsPrepare();
	OSTaskSuspend(OS_PRIO_SELF);
}
extern FortType fort;
extern num_t u_Num;
//void WalkTask(void)
//{	
//	CPU_INT08U os_err;
//	os_err = os_err;
//	OSSemSet(PeriodSem, 0, &os_err);
//	while (1)
//	{
//		OSSemPend(PeriodSem, 0, &os_err);
//		//USART_OUT(UART4,(uint8_t*)"%d\t",(int)inAngleFlag);
//		//USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)inAngleFlag,(int)taskFlag);
//		//USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)Rnd_PID.radius,(int)Sign);
//	//	USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)distanceL,(int)distanceR);
//		//USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)taskFlag,(int)Rnd_PID.radius);
////		USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)errorRadius,(int)setAngle_R);
////		USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)uOutAngle,(int)uOutDis);
////		USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)phase1,(int)phase2);
//		//USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t\r\n",(int)GetAngle(),(int)GetX(),(int)GetY());
//		switch(taskFlag)
//		{
//			case 0:
//			{
//				distanceR = (4400.f/4096.f)*(float)Get_Adc_Average(14,10);
//				distanceL = (4400.f/4096.f)*(float)Get_Adc_Average(15,10);
//				if(distanceR<=50)
//				{
//					Rnd_PID.side = 1;
//					taskFlag = 1;
//				}
//				else if(distanceL<=50)
//				{
//					Rnd_PID.side = -1;
//					taskFlag = 1;
//				}
//			}break;
//			case 1:
//			{
//				//检测是否接触到物体
//				if(sqrtf(powf((LastPos.x-GetX()),2)+powf((LastPos.y-GetY()),2))<=5) 
//				{
//					cnt2++;
//					if(cnt2>100)
//					{
//						taskFlag = 2;
//						cnt2 = 0;
//					}
//				}
//				//判断由于接触到物体而更改了执行半径，改回原有半径
//				if(changeR !=0)
//				{
//					cnt3++;
//					if(cnt3>=200)
//					{
//						if(changeR == 1)
//							Rnd_PID.radius += 400;
//						else if(changeR == 2)
//							Rnd_PID.radius -= 400;	
//						cnt3 = 0;
//						changeR = 0;
//					}
//				}
//				//正常情况的更改半径
//				if(Rnd_PID.side == 1)
//				{
//					if(GetAngle()<-130&&GetAngle()>-170)
//					{
//						inAngleFlag = 1;
//					}
//					if(fabsf(4*GetX() +GetY()-2400)<=100 && inAngleFlag == 1 && GetAngle()<0)
//					{
//						if((Rnd_PID.radius>=1800||Rnd_PID.radius<=600))
//							Sign = -Sign;
//						Rnd_PID.radius += Sign * 400;
//						if(Rnd_PID.radius>1800)
//							Rnd_PID.radius=1800;
//						if(Rnd_PID.radius<=600)
//							Rnd_PID.radius=600;
//						inAngleFlag = 0;
//					}
//				}
//				else if(Rnd_PID.side == -1)
//				{
//					if(GetAngle()>130&&GetAngle()<170)
//					{
//						inAngleFlag = 1;
//					}	
//					if(fabsf(4*GetX() -GetY()+2400)<=100 && inAngleFlag == 1 && GetAngle()>0)
//					{
//						if((Rnd_PID.radius>=1800||Rnd_PID.radius<=600))
//							Sign = -Sign;
//						Rnd_PID.radius += Sign * 400;
//						if(Rnd_PID.radius>1800)
//							Rnd_PID.radius=1800;
//						if(Rnd_PID.radius<=600)
//							Rnd_PID.radius=600;
//						inAngleFlag = 0;
//					}
//				}
//				WalkRoundPID(&Rnd_PID);
//				LastPos.x = GetX();
//				LastPos.y = GetY();
//				LastPos.angle = GetAngle();
//			}break;
//			case 2:
//			{
//				if(Rnd_PID.radius>600)
//				{
//					Rnd_PID.radius -= 400;
//					changeR = 1;//减半径了
//				}
//				else if(Rnd_PID.radius<=600)
//				{
//					Rnd_PID.radius += 400;
//					changeR = 2;//加半径了
//				}
//				taskFlag = 3;
//			}break;
//			case 3:
//			{
//				//有一定差速倒车，保证垂直于目标圆
//				cnt1 ++;
//				if((Rnd_PID.side == 1 && changeR == 1)||(Rnd_PID.side == -1 && changeR == 2))	
//				{
//					VelCrl(CAN2,1,-9000);//右轮
//					VelCrl(CAN2,2,12000);
//				}
//				else if((Rnd_PID.side == 1 && changeR == 2)||(Rnd_PID.side == -1 && changeR == 1))	
//				{
//					VelCrl(CAN2,1,-12000);//左轮
//					VelCrl(CAN2,2,9000);
//				}
//				if(cnt1>=150)
//				{
//					cnt1 = 0;
//					taskFlag = 1;
//				}
//			}break;
//		}
//	}
//}

void WalkTask(void)
{	
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		//控制发射枪电机转速
		ShooterVelCtrl(10);
		
		//// 控制电机的转速，脉冲。OK
		VelCrl(CAN1,COLLECT_BALL_ID,0*4096); 
		
		//推球电机,配置位置环 OK
		// 推球
//		PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);
//		// 复位
//		PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);
		
		//控制航向电机
		YawPosCtrl(90);
		delay_s(2);
		YawPosCtrl(270);
		delay_s(2);
		
	}
}
