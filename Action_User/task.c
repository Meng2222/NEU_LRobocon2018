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
/*
===============================================================
						信号量定义
===============================================================
*/
#define KP 18.0f
#define KI 0.0f
#define KD 1.2f

extern char pposokflag;
extern Pos_t Pos;
float errorAngle = 0;
float phase1 = 0,phase2 = 0,Uout = 0;
char flag = 0;
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
void WalkLine(float speed);
void WalkLine2PID(float vel,float setAngle);
void WalkRound(float vel,float radius,char side);
float PID_Compentate(float err);
static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
extern char isOKFlag;
void driveGyro(void){
	while(!isOKFlag)
	{
		delay_ms(5);
		USART_SendData(USART3,'A');
		USART_SendData(USART3,'T');
		USART_SendData(USART3,'\r');
		USART_SendData(USART3,'\n');
	}
	isOKFlag = 0;
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
typedef struct {
	float x;
	float y;
}PosNow_t;//定义转换位置时存当时位置
PosNow_t PosNow;
char posFlag = 0;//需要存位置标志
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	USART3_Init(115200);
	UART4_Init(921600);
	TIM_Init(TIM2, 1000-1, 84-1, 0x01, 0x03); //定时器初始化1ms
	//CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2, 1, 500000, 500000);
	VelLoopCfg(CAN2, 2, 500000, 500000);
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	delay_s(2);
	#if CARNUM == 1
	driveGyro();
	while(!pposokflag);
	#elif CARNUM == 4
	delay_s(10);
	#endif
	OSTaskSuspend(OS_PRIO_SELF);
}

void WalkTask(void)
{
	
	CPU_INT08U os_err;
	os_err = os_err;
	int cnt = 0;
	PosNow_t PosNow = {
		.x = 0,
		.y = 0
	};
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		cnt++;
		if(cnt>=10)
		{
			USART_OUT(UART4,(uint8_t*)"%d\t",(int)flag);
			USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t",(int)Pos.angle,(int)Pos.x,(int)Pos.y);
//			USART_OUT(UART4,(uint8_t*)"%d\t%d\r\n",(int)phase1,(int)phase2);
			USART_OUT(UART4,(uint8_t*)"%d\t%d\r\n",(int)PosNow.x,(int)PosNow.y);
			cnt = 0;
		}
		if(posFlag == 1)
		{
			PosNow.x = Pos.x;
			PosNow.y = Pos.y;
			posFlag = 0;
		}
		switch(flag)
		{
			case 0:
			{
				WalkLine2PID(400,0);
				if(Pos.y >= PosNow.y + 1340)
				{
					flag = 1;
					posFlag = 1;
				}
			}break;
			case 1:
			{
				WalkLine2PID(400,-90);
				if(Pos.x >= PosNow.x + 1340)
				{
					flag = 2;
					posFlag = 1;
				}
			}break;
			case 2:
			{
				WalkLine2PID(400,-180);
				if(Pos.y <= PosNow.y - 1340)
				{
					flag = 3;
					posFlag = 1;
				}
			}break;
			case 3:
			{
				WalkLine2PID(400,90);
				if(Pos.x <= PosNow.x - 1340)
				{
					flag = 0;
					posFlag = 1;
				}
			}break;
			default: 
				break;	
		
		}
	}
}

/**
* @brief  直线函数
* @param  vel: 速度，单位：mm每秒，范围：最小速度限制到最大速度限制
*/
void WalkLine(float vel)
{
	float phase = 0;
	phase = 4096.f * vel / (WHEEL_DIAMETER * PI);
	VelCrl(CAN2,1,phase);
	VelCrl(CAN2,2,-phase);
}

/**
* @brief  直线函数用PID调节
* @param  vel: 速度，单位：mm每秒，范围：最小速度限制到最大速度限制
*/
void WalkLine2PID(float vel,float setAngle)
{
	errorAngle = setAngle - Pos.angle;
	if(errorAngle > 180.0f)
		errorAngle = errorAngle - 360.0f;
	else if(errorAngle < -180.0f)
		errorAngle = 360.0f + errorAngle;		
	Uout = PID_Compentate(errorAngle);
	phase1 = 4096.f * (vel + Uout)/ (WHEEL_DIAMETER * PI);
	phase2 = 4096.f * (vel - Uout)/ (WHEEL_DIAMETER * PI);
	VelCrl(CAN2,1,phase1);
	VelCrl(CAN2,2,-phase2);
}
/**
* @brief  圆形函数
* @param  vel: 速度，单位：mm每秒，范围：最小速度限制到最大速度限制
* @param  radius: 半径，单位：mm，范围：最小限制到最大限制
* @param  side: 半径，1圆心在右，0 圆心正左
*/
void WalkRound(float vel,float radius,char side)
{
	float phase1 = 0,phase2 = 0;
	float v1 = 0,v2 = 0;
	v1 = (radius - WHEEL_TREAD / 2) / radius * vel;
	v2 = (radius + WHEEL_TREAD / 2) / radius * vel;
	phase1 = 4096.f *v1 / (WHEEL_DIAMETER * PI);
	phase2 = 4096.f *v2 / (WHEEL_DIAMETER * PI);
	if(side == 1)
	{
		VelCrl(CAN2,1,phase1);
		VelCrl(CAN2,2,-phase2);
	}
	else if(side == 0)
	{
		VelCrl(CAN2,1,phase2);
		VelCrl(CAN2,2,-phase1);
	}
}

float PID_Compentate(float err)
{
	float Uout = 0;
	static float lastErr = 0;
	static float sumErr = 0;
	sumErr += err;
	Uout = KP * err + KI * sumErr +KD *(err - lastErr);
	lastErr = err;
	return Uout;
}
