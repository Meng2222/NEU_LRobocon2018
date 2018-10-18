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
#include "fort.h"
#include "math.h"
#include "key.h"

#define One_Meter_Per_Second (10865.0)            //车轮一米每秒的设定值   4096*(1000/120π)
#define BaseVelocity (0.5 * One_Meter_Per_Second) //基础速度               0.5m/s                         //四号车编号             4
#define Side_Length (2000)                        //方形边长               2m
#define Angle_Error_Range (3)					  //角度误差范围           3 


// 宏定义棍子收球电机1ID
#define COLLECT_BALL_1_ID (5)
// 宏定义棍子收球电机2ID
#define COLLECT_BALL_2_ID (6)
// 宏定义推球电机ID
#define PUSH_BALL_ID (7)


/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;



extern char buffer[20];
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
	OSTaskSuspend(OS_PRIO_SELF);
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
	Init();
	OSTaskSuspend(OS_PRIO_SELF);

}


//炮台发回的值
uint8_t adcFlag=0;
usartValue uv4;
extern FortType fort;
extern int32_t pushPos;
extern int32_t pushPulse;
extern uint8_t keyFlag;
extern struct comend Cmd;
uint8_t modeFlag=0;
uint8_t rightBall=0;
uint8_t wrongBall=0;
uint8_t keyFlag=0;
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	//PID参数
	Angle_PidPara((800*KP_A),0,KD_A);
	Distance_PidPara(KP_D,0,25); 
	
	
	
	//开关控制要的球的颜色
	if(KEY_STATUS)
	{
		rightBall=2;
		wrongBall=1;
	}
	else
	{
		rightBall=1;
		wrongBall=2;
	}

	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		
		//正常运行
		if(!keyFlag)
		{ 
			
			ShooterVelCtrl(60);
			YawPosCtrl(0);
			BallColorRecognition();
			//走形
//			Walk(&adcFlag);
//			
//			//发球
//			Shoot(adcFlag); 
			
			USART_OUT(UART4, " %d\t", (int)adcFlag);
			USART_OUT(UART4, " %d\t", (int)uv4.flgOne);
			USART_OUT(UART4, " %d\t", (int)uv4.ball);
			USART_OUT(UART4, " %d\t", (int)uv4.errflg);
			USART_OUT(UART4, " %d\t", (int)uv4.judgeSp);
			USART_OUT(UART4, " %d\t", (int)uv4.shootFlg);
			USART_OUT(UART4, " %d\t", (int)uv4.shootangle);
			USART_OUT(UART4, " %d\t", (int)fort.yawPosReceive);
			USART_OUT(UART4, " %d\t", (int)uv4.distance);
			USART_OUT(UART4, " %d\t", (int)uv4.shootSp);
			USART_OUT(UART4, " %d\t", (int)uv4.shootSp2);
			USART_OUT(UART4, " %d\t", (int)Cmd.shoot);
			USART_OUT(UART4, " %d\t", (int)fort.shooterVelReceive);
			USART_OUT(UART4, " %d\t", (int)fort.laserAValueReceive);
			USART_OUT(UART4, " %d\t", (int)fort.laserBValueReceive);
			USART_OUT(UART4, " %d\t", (int)uv4.ready[0]);
			USART_OUT(UART4, " %d\t", (int)uv4.ready[1]);
			USART_OUT(UART4, " %d\t", (int)uv4.ready[2]);
			USART_OUT(UART4, " %d\t", (int)uv4.ready[3]);
			USART_OUT(UART4, " %d\t", (int)pushPos);
			USART_OUT(UART4, " %d\t", (int)pushPulse);
			USART_OUT(UART4, " %d\t", (int)GetGyro());
			USART_OUT(UART4, " %d\t", (int)GetAngle());
			USART_OUT(UART4, " %d\t", (int)GetSpeeedX());
			USART_OUT(UART4, " %d\t", (int)GetSpeeedY());
			USART_OUT(UART4, " %d\t", (int)GetPosX());
			USART_OUT(UART4, " %d\r\n", (int)GetPosY());
		}
		
		//倒球
		else
		{
			MotorOff(CAN1, BACK_WHEEL_ID);
			MotorOff(CAN1, TURN_AROUND_WHEEL_ID);
			MotorOff(CAN2, PUSH_BALL_ID);
			
			ShooterVelCtrl(0);
			
			VelCrl(CAN2,COLLECT_BALL_1_ID,-80*OTHER_COUNTS_PER_ROUND); 

			//收球电机2
			VelCrl(CAN2,COLLECT_BALL_2_ID,80*OTHER_COUNTS_PER_ROUND); 
		}			
	}
}
 
//初始化函数
void Init(void)
{
	delay_ms(5000);
	TIM_Init(TIM2, 999, 84, 0x01, 0x03);
	USART3_Init(115200);
	UART4_Init(921600);
	UART5_Init(921600);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	KeyOne();
	KeyTwo();
	ElmoInit(CAN1);
	ElmoInit(CAN2);
	
	//后轮电机初始化
	VelLoopCfg(CAN1, BACK_WHEEL_ID, 40000000, 40000000);
	
	//前轮电机初始化
	VelLoopCfg(CAN1, TURN_AROUND_WHEEL_ID, 40000000, 40000000);
	
	//收球电机初始化
	VelLoopCfg(CAN2, COLLECT_BALL_1_ID, 500000, 500000);
	
	//收球电机初始化
	VelLoopCfg(CAN2, COLLECT_BALL_2_ID, 500000, 500000);

	//推球电机初始化
	PosLoopCfg(CAN2, PUSH_BALL_ID, 10000000,10000000,5000000);
	
	MotorOn(CAN1, BACK_WHEEL_ID);
	MotorOn(CAN1, TURN_AROUND_WHEEL_ID);
	MotorOn(CAN2, COLLECT_BALL_1_ID);
	MotorOn(CAN2, COLLECT_BALL_2_ID);

	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1) == 1)	
		keyFlag=1;
	else
		keyFlag=0;
	if(keyFlag == 0)
	{
		
		delay_ms(5000);
		
		//初始化定位系统
		PosConfig();
		
		//开启收球电机1,2
		VelCrl(CAN2,COLLECT_BALL_1_ID,60*OTHER_COUNTS_PER_ROUND); 	
		VelCrl(CAN2,COLLECT_BALL_2_ID,-60*OTHER_COUNTS_PER_ROUND); 
		
		//等待激光触发
		GetDirection(&adcFlag);
	}
	else;

	
}

/**
* @brief 定位系统初始化
* @param none
* @retval none
* @attention 
*/
//
extern uint8_t sendFlag;
uint8_t isOKFlag;

void PosConfig(void)
{
	while(!isOKFlag)
	{
		delay_ms(1);
		USART_SendData(USART3,'A');
		USART_SendData(USART3,'T');
		USART_SendData(USART3,'\r');
		USART_SendData(USART3,'\n');

	}
	isOKFlag=0;
	while(!sendFlag);
	sendFlag=0;
}

/**
* @brief 激光触发
* @param getFlag：触发模式标志位
* @retval none
* @attention 
*/


void GetDirection(uint8_t *getAdcFlag)
{
	static uint8_t cnt=0;
	uint16_t laserA=0;
	uint16_t laserB=0;
	static float minLaserA=1500,minLaserB=1500;
	static int reduceFlag=0,overFlag=0;
	while(1)
	{
		delay_ms(2);
		YawPosCtrl(90);
		
		USART_OUT(UART4, "A	 %d\t", (int)fort.laserAValueReceive);
		USART_OUT(UART4, "B	 %d\r\n", (int)fort.laserBValueReceive);
		
		laserA=fort.laserAValueReceive*2.48+24.8;
		laserB=fort.laserBValueReceive*2.48+24.8;
		if((laserA<1500||laserB<1500)  && laserA > 50 && laserB > 50)
		{
			cnt++;
			if(cnt > 40)
				reduceFlag=1;
		}
		else
		{
			cnt=0;
		}
		if(reduceFlag&&(!overFlag))
		{
			if(laserA<minLaserA)
				minLaserA=laserA;
			if(laserB<minLaserB)
				minLaserB=laserB;
			if(laserA>1500&&laserB>1500)
				overFlag=1;
		}
		if(overFlag)
		{
			if(minLaserB < 80)
				{
				  (*getAdcFlag)=0;
				break;
				}
			else if(minLaserA < 80)
				{
				  (*getAdcFlag)=3;
				  break;
				}
			else if(minLaserB > 100 && minLaserB < 200)
				{
				  (*getAdcFlag)=1;
				  break;
				}
			else if(minLaserA > 100 && minLaserA < 200)
				{
				  (*getAdcFlag)=4;
				  break;
				}
				 else if(minLaserB > 250 && minLaserB < 500)
				{
				  (*getAdcFlag)=2;
				  break;
				}
			else if(minLaserA > 250 && minLaserA < 500)
				{
				  (*getAdcFlag)=5;
				  break;
				}
			else
				{
					overFlag=0;
					minLaserA=1500;
					minLaserB=1500;
				}
		}
	}
	cnt=0;
}

