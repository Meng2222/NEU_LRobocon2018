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
/*
===============================================================
						信号量定义
===============================================================
*/

extern char updownFlag;
extern char pposokflag;
extern Pos_t Pos;
extern float errorAngle,errorDis;//等于位置环输出角度-实际角度，位置偏差
extern float phase1,phase2,uOutAngle,uOutDis;//角度PID输出，位置PID输出
extern float Uout;//PID计算的输出值
extern char flag;
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
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
PId_t Angle_PId = {
	.Kp = 9,
	.Ki = 0,
	.Kd = 1.0
};
PId_t Dis_PId = {
	.Kp = 0.3*400/(SPEED),
	.Ki = 0,
	.Kd = 0
};//PosNow_t PosNow;
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
	delay_s(5);
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
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		cnt++;
		if(cnt>=15)
		{
			//USART_OUT(UART4,(uint8_t*)"%d\t",(int)updownFlag);//标志
			//USART_OUT(UART4,(uint8_t*)"%d\t",(int)Pos.x);//标志
			//USART_OUT(UART4,(uint8_t*)"%d\t",(int)errorDis);//距离差
			//USART_OUT(UART4,(uint8_t*)"%d\t",(int)setAngle);//距离差
			//USART_OUT(UART4,(uint8_t*)"%d\t",(int)uOutAngle);//脉冲
			//USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)uOutDis,(int)errorAngle);//设定角度，实际角度，偏差
			USART_OUT(UART4,(uint8_t*)"%d\t%d\t",(int)phase1,(int)phase2);
			USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t\r\n",(int)Pos.angle,(int)Pos.x,(int)Pos.y);
//			USART_OUT(UART4,(uint8_t*)"%d\t%d\r\n",(int)PosNow.x,(int)PosNow.y);
			cnt = 0;
		}
		KownedLinePID(1,1,500,2);
	}
}

