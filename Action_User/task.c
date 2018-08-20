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
#include "math.h"
#define num five
#define one 1
#define two 2
#define three 3
#define four 4
#define five 5
/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

Pos_t pos;
extern Pos_t posTmp;
extern PID_Value PID_Angle, PID_Line, PID_Round;
extern Line_Value Line;
extern Round_Value Round;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];

#if CarNum == CarOne
int isOKFlag = 0;    //定位系统初始化完毕标志位
uint8_t opsFlag = 0; //收到定位系统数据标志位
//一直以5ms间隔发“AT/r/n”给定位系统，等待定位系统回数 OK定位系统初始化结束
int IsSendOK(void)
{
	return isOKFlag;
}
void SetOKFlagZero(void)
{
	isOKFlag = 0;
}
void driveGyro(void)
{
	while(!IsSendOK())
	{
		delay_ms(5);
		USART_SendData(USART3, 'A');
		USART_SendData(USART3, 'T');
		USART_SendData(USART3, '\r');
		USART_SendData(USART3, '\n');
	}
	SetOKFlagZero();
}
#endif


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
	
	TIM_Init(TIM2, 999, 83, 0, 0);
	USART3_Init(115200);
	UART4_Init(921600);
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);
	
	ElmoInit(CAN2);
	VelLoopCfg(CAN2, 1, 53333333, 53333333);
	VelLoopCfg(CAN2, 2, 53333333, 53333333);
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	
	//定位系统初始化延时
    #if CarNum == CarOne        
		delay_s(2);
		driveGyro();
		while(!opsFlag);
    #elif CarNum == CarFour
		delay_s(10);
		delay_s(5);
    #endif
	OSTaskSuspend(OS_PRIO_SELF);
}


void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	int cntSendTime = 0;
	float BaseVelocity = 0, adjustVelocityTurn = 0, adjustVelocityAngle = 0, adjustVelocityLine = 0, adjustVelocityRound = 0;

	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		
		//将四号车的坐标系转化为一号车的标准坐标系
		#if CarNum == CarOne
		    pos.x = posTmp.y;
		    pos.y = posTmp.x * -1;
		    pos.angle = posTmp.angle * -1;
		#elif CarNum == CarFour
		    pos.x = posTmp.x;
		    pos.y = posTmp.y;
		    pos.angle = posTmp.angle;
        #endif
		
		#if num == 1
		    BaseVelocity = Go(0.5);
		    VelCrl(CAN2, 1, (int)(BaseVelocity));//右轮
		    VelCrl(CAN2, 2, (int)((BaseVelocity) * -1.0));//左轮
		#elif num == 2
		    BaseVelocity = Go(0.5);
		    adjustVelocityTurn = Turn(0.5, 1.0, 2);
			VelCrl(CAN2, 1, (int)(BaseVelocity + adjustVelocityTurn));//右轮
		    VelCrl(CAN2, 2, (int)((BaseVelocity - adjustVelocityTurn) * -1.0));//左轮
		#elif num == 3
		    BaseVelocity = Go(0.5);
		    adjustVelocityAngle = PID_Angle_Operation(90, pos.angle, &PID_Angle);
			VelCrl(CAN2, 1, (int)(BaseVelocity +  adjustVelocityAngle));//右轮
		    VelCrl(CAN2, 2, (int)((BaseVelocity -  adjustVelocityAngle) * -1.0));//左轮
		#elif num == 4
		    BaseVelocity = Go(0.5);
		    Line.Line_A = -1.0;
		    Line.Line_B = -1.0;
		    Line.Line_C = -1.0;
            Line.Line_Mode = 1;			
		    Line_Operation(&Line);
		    adjustVelocityLine = PID_Line_Operation(pos.angle, &Line, &PID_Line);
			VelCrl(CAN2, 1, (int)(BaseVelocity + adjustVelocityLine));//右轮
		    VelCrl(CAN2, 2, (int)((BaseVelocity - adjustVelocityLine) * -1.0));//左轮
		#elif num == 5
		    BaseVelocity = Go(0.5);		 
            Round.Round_Center_x = -1.5;             //圆心横坐标x          -1.5m
            Round.Round_Center_y  = 1.5;             //圆心纵坐标y          1.5m
            Round.Round_Radius = 1.0;                //圆半径               1.0m
		    Round_Operation(&Round);
		    adjustVelocityRound = PID_Round_Operation(pos.angle, &Round, &PID_Round);
		    VelCrl(CAN2, 1, (int)(BaseVelocity + adjustVelocityRound));//右轮
		    VelCrl(CAN2, 2, (int)((BaseVelocity - adjustVelocityRound) * -1.0));//左轮			
		#endif

		
		//以20 * 10ms为间隔发送数据
		cntSendTime++;
		cntSendTime = cntSendTime % 20;
		if(cntSendTime == 1)
		{
            USART_OUT(UART4, (uint8_t*)"x=%d,y=%d,ang=%d,BV=%d,TV=%d,AV=%d,LV=%d,RV=%d\r\n", \
			(int)pos.x, (int)pos.y, (int)pos.angle, (int)BaseVelocity, (int)adjustVelocityTurn, (int)adjustVelocityAngle, (int)adjustVelocityLine, (int)adjustVelocityRound);
		}
		OSSemSet(PeriodSem, 0, &os_err);
	}
}
