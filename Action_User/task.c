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

#define One_Meter_Per_Second (10865.0)            //车轮一米每秒的设定值   4096*(1000/120π)
#define BaseVelocity (0.5 * One_Meter_Per_Second) //基础速度               0.5m/s                         //四号车编号             4
#define Side_Length (2000)                        //方形边长               2m
#define Angle_Error_Range (3)                     //角度误差范围           3 

#define Pulse2mm COUNTS_PER_ROUND/(WHEEL_DIAMETER*PI)

// 宏定义棍子收球电机ID
#define COLLECT_BALL_ID (8)
// 宏定义推球电机ID
#define PUSH_BALL_ID (6)
//宏定义左轮电机ID
#define LEFT_ID (1)
//宏定义右轮电机ID
#define RIGHT_ID (2)
// 电机旋转一周的脉冲数
#define COUNT_PER_ROUND (4096.0f)
// 宏定义每度对应脉冲数
#define COUNT_PER_DEGREE  (COUNT_PER_ROUND/360.0f)
// 宏定义航向角减速比
#define YAW_REDUCTION_RATIO (4.0f)




uint8_t adcFlag=0;
struct usartValue_{
	uint32_t cnt;//用于检测是否数据丢失
	float xValue;//串口输出x坐标
	float yValue;//串口输出y坐标
	float angleValue;//串口输出角度值
	float pidValueOut;//PID输出
	float d;
	float turnAngleValue;//
	uint8_t flagValue;
	float shootangle;
	float shootSp;
}usartValue;


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
extern FortType fort;
extern uint8_t notShoot[2];
extern float posXAdd;
extern float posYAdd;

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	
	uint16_t Cnt=0;
	float DD=0;
	float SS=0;
	//PID参数
	Angle_PidPara(20,0,1500);
	Distance_PidPara(0.09,0,0); 
	Speed_PidPara(2,0,0);
	
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		
		//走位
		Walk(adcFlag);
		//发球
		Shoot(adcFlag,250); 
//		DD=sqrt(((GetPosY()-105)*(GetPosY()-105))+((GetPosX()+2200)*(GetPosX()+2200)));
//		SS=DD*0.012+35.7;
//		Cnt++;
//		ShooterVelCtrl(SS);
//		if(Cnt == 150)
//		{
//			PosCrl(CAN1, 0x06,ABSOLUTE_MODE,4500);
//		}
//		else if(Cnt == 300)
//		{
//			PosCrl(CAN1, 0x06,ABSOLUTE_MODE,5);
//			Cnt=0;
//		}
		
		USART_OUT(UART4, " %d\t", (int)posXAdd);
		USART_OUT(UART4, " %d\r\n", (int)posYAdd);
		//USART_OUT(UART4, " %d\r\n", (int)GetSpeeedY());		
	}
}
 
//初始化函数
void Init(void)
{
	TIM_Init(TIM2, 999, 84, 0x01, 0x03);
	USART3_Init(115200);
	UART4_Init(921600);
	UART5_Init(921600);
	Adc_Init();
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	
//	//右轮电机初始化
	VelLoopCfg(CAN2, LEFT_ID, 20000000, 20000000);
	
	//左轮电机初始化
	VelLoopCfg(CAN2, RIGHT_ID, 20000000, 20000000);
	
	//收球电机初始化
	VelLoopCfg(CAN1, COLLECT_BALL_ID, 500000, 500000);
//	
	//推球电机初始化
	PosLoopCfg(CAN1, PUSH_BALL_ID, 5000000,5000000,200000);
	
	MotorOn(CAN2, LEFT_ID);
	MotorOn(CAN2, RIGHT_ID);
	MotorOn(CAN1, COLLECT_BALL_ID);
	MotorOn(CAN1, PUSH_BALL_ID);
	delay_ms(5000);
	PosConfig();
	
	GetDirection(&adcFlag);
	
	//收球电机
	VelCrl(CAN1,COLLECT_BALL_ID,5*60*4096); 
	
	
}


//车1定位系统初始化
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



