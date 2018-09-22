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
#include "moveBase.h"

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
	float X;
	float Y;
	float V;
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

extern Msg_t frontspeedBuffer;
extern Msg_t backspeedBuffer;

extern uint8_t ballColor;
extern int32_t pushPos;
extern int32_t pushPulse;

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	uint16_t Cnt=0;


	//PID参数
	

	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		ReadActualVel(CAN1,BACK_WHEEL_ID);
		ReadActualVel(CAN1,TURN_AROUND_WHEEL_ID);	
	  ReadActualPos(CAN2, PUSH_BALL_ID);		
		
		//VelCrl(CAN1,2,1200*REDUCTION_RATIO*NEW_CAR_COUNTS_PER_ROUND/(PI*TURN_AROUND_WHEEL_DIAMETER));//前轮
		//Tangencystraightline();
		//N_Back_Strght_Walk(1,0,-700,2,800);
		//The_Collect_Round();
		USART_OUT(UART4, " %d\t%d\t%d\t%d\r\n", (int)(GetPosX()),(int)(GetPosY()),(int)(((frontspeedBuffer.data32[1]/8192.0f)/REDUCTION_RATIO)*PI*TURN_AROUND_WHEEL_DIAMETER),(int)(((backspeedBuffer.data32[1]/8192.0f)/REDUCTION_RATIO)*PI*WHEEL_DIAMETER));		
			
		//BallColorRecognition();

    //收球电机1
   VelCrl(CAN2,COLLECT_BALL_1_ID,60*OTHER_COUNTS_PER_ROUND); 
	
		//收球电机1
   VelCrl(CAN2,COLLECT_BALL_2_ID,-60*OTHER_COUNTS_PER_ROUND); 
		//走位
    //Walk(adcFlag);
		//发球
		//Shoot(adcFlag,200);


//		USART_OUT(UART4, " %d\t", pushPos);
//		USART_OUT(UART4, " %d\t", pushPulse);
//		
//		USART_OUT(UART4, " %d\r\n", (int)ballColor);
	}
}
 
//初始化函数
void Init(void)
{

	TIM_Init(TIM2, 999, 84, 0x01, 0x03);
	USART3_Init(115200);
	UART4_Init(921600);
	UART5_Init(921600);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN1);
	ElmoInit(CAN2);
	delay_s(2);
	

	//后轮电机初始化
	VelLoopCfg(CAN1, BACK_WHEEL_ID, 40000000, 40000000);
	
	//前轮电机初始化
	VelLoopCfg(CAN1, TURN_AROUND_WHEEL_ID, 40000000, 40000000);
	
	//收球电机初始化
	VelLoopCfg(CAN2, COLLECT_BALL_1_ID, 500000, 500000);
	
	//收球电机初始化
	VelLoopCfg(CAN2, COLLECT_BALL_2_ID, 500000, 500000);

	//推球电机初始化
	PosLoopCfg(CAN2, PUSH_BALL_ID, 5000000,5000000,200000);
	
	MotorOn(CAN1, BACK_WHEEL_ID);
	MotorOn(CAN1, TURN_AROUND_WHEEL_ID);
	MotorOn(CAN2, COLLECT_BALL_1_ID);
	MotorOn(CAN2, COLLECT_BALL_2_ID);
//	MotorOn(CAN2, PUSH_BALL_ID);
	delay_ms(5000);
	PosConfig();

	GetDirection(&adcFlag);
	

	
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


void GetDirection(uint8_t *getFlag)
{
	uint16_t Laser_A=0;
	uint16_t Laser_B=0;
	
	while(1)
	{
		Laser_A=fort.laserAValueReceive;
		Laser_B=fort.laserBValueReceive;
		if(Laser_A < 70)
		{
			(*getFlag)=0;
			break;
		}
		else if(Laser_B < 70)
		{
			(*getFlag)=1;
			break;
		}
		else;
		USART_OUT(UART4, "Laser Ready\r\n");
	}
}

