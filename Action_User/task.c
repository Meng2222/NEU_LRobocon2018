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

//用于串口返回值
usartValue uv4;
extern FortType fort;
extern int32_t pushPos;
extern int32_t pushPulse;
extern uint8_t keyFlag;
extern struct comend Cmd;

//要的球的颜色 ，为1 白色，为2 黑色
uint8_t rightBall=0;

//不要的球的颜色 ，为1 白色，为2 黑色
uint8_t wrongBall=0;

//倒球开关的值，为0 正常运行，为1 倒球
uint8_t keyFlag=0;
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	//PID参数
	Angle_PidPara((800*KP_A),0,KD_A);
	Distance_PidPara(KP_D,0,25); 
	//开关控制要的球的颜色
	if(KEY_STATUS_ONE)
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
			//走形
			Walk(&adcFlag);
			
			//发球
			Shoot(adcFlag); 
			
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)adcFlag);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)uv4.flgOne);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)uv4.ball);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)uv4.errflg);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)uv4.judgeSp);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)uv4.shootFlg);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)uv4.shootangle);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)fort.yawPosReceive);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)uv4.distance);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)uv4.shootSp);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)fort.shooterVelReceive);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)fort.laserAValueReceive);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)fort.laserBValueReceive);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)uv4.ready[0]);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)uv4.ready[1]);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)uv4.ready[2]);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)uv4.ready[3]);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)pushPos);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)pushPulse);
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)GetGyro());
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)GetAngle());
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)GetSpeeedX());
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)GetSpeeedY());
			USART_OUT(UART4, (uint8_t *)" %d\t", (int)GetPosX());
			USART_OUT(UART4, (uint8_t *)" %d\r\n", (int)GetPosY());
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

	if(KEY_STATUS_TWO)	
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
* @brief 激光触发，手挡住放开之后触发
* @param getFlag：触发模式标志位
* @retval none
* @attention 
*/


void GetDirection(uint8_t *getFlag)
{
	uint8_t cnt=0;
	uint16_t Laser_A=0;
	uint16_t Laser_B=0;
	float MinLaser_A=1500,MinLaser_B=1500;
	static int reduceflag=0,overflag=0;
	while(1)
	{
		delay_ms(2);
		YawPosCtrl(90);
		
		Laser_A=fort.laserAValueReceive*2.48f+24.8f;
		Laser_B=fort.laserBValueReceive*2.48f+24.8f;
		
		//激光A或者激光B测得距离小于1500大于50，并且大于40次
		if((Laser_A<1500||Laser_B<1500)  && Laser_A > 50 && Laser_B > 50)
		{
			cnt++;
			if(cnt > 40)
				reduceflag=1;
		}
		else
		{
			cnt=0;
		}
		if(reduceflag&&(!overflag))
		{
			//记录激光最小值
			if(Laser_A<MinLaser_A)
				MinLaser_A=Laser_A;
			if(Laser_B<MinLaser_B)
				MinLaser_B=Laser_B;
			
			//手放开之后启动
			if(Laser_A>1500&&Laser_B>1500)
				overflag=1;
		}
		if(overflag)
		{
			//记录激光B小于80
			if(MinLaser_B < 80)
				{
				  (*getFlag)=0;
				break;
				}
			//记录激光A小于80
			else if(MinLaser_A < 80)
				{
				  (*getFlag)=3;
				  break;
				}
			else if(MinLaser_B > 100 && MinLaser_B < 200)
				{
				  (*getFlag)=1;
				  break;
				}
			else if(MinLaser_A > 100 && MinLaser_A < 200)
				{
				  (*getFlag)=4;
				  break;
				}
				 else if(MinLaser_B > 250 && MinLaser_B < 500)
				{
				  (*getFlag)=2;
				  break;
				}
			else if(MinLaser_A > 250 && MinLaser_A < 500)
				{
				  (*getFlag)=5;
				  break;
				}
			else
				{
					overflag=0;
					MinLaser_A=1500;
					MinLaser_B=1500;
				}
		}
	}
	cnt=0;
}

