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
#include "PID.H"
#include "environmental.h"


/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

typedef struct
{
    float posX;
    float posY;
    float angle;
}Pos;

Pos Pos_t;
Pos lastpos;
PIDCtrlStructure PidA;
_Bool directionFlag = 0;

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
    TIM_Init(TIM2, 999, 83, 0x00, 0x00);
    CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
    CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);
    VelLoopCfg(CAN2, 1, 80000000, 80000000);
    VelLoopCfg(CAN2, 2, 80000000, 80000000);
	ElmoInit(CAN2);
    USART3_Init(115200);
    UART4_Init(921600);
#ifdef CAR1
    while(!GetOkFlag())
    {
        delay_ms(5);
        USART_OUT(USART3, (uint8_t *)"AT\r\n");
    };
#endif
#ifdef CAR4
    delay_s(10);
#endif
	OSTaskSuspend(OS_PRIO_SELF);
}

float GetX(void)
{
    return (int32_t)Pos_t.posX;
}

float GetY(void)
{
    return (int32_t)Pos_t.posY;
}
float GetA(void)
{
    return (int32_t)Pos_t.angle;
}

void CtrlTool(float Uout)
{
    TurnAround(Uout, 500);
}

//void straight(int32_t Num)
//{
//    GoStraight((float)Num);
//}

void ExSet(void)
{
        if(PidA.ExOut >= 180)
        {
            PidA.ExOut -= 360;
        }
        PidA.ExOut += 90;
        lastpos.posX = GetX();
        lastpos.posY = GetY();
        PIDCtrlInit();
        directionFlag = !directionFlag;
}

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
    MotorOn(CAN2, 1);
    MotorOn(CAN2, 2);
	OSSemSet(PeriodSem, 0, &os_err);
    PidA.KP = 20;
    PidA.KI = 0.01;
    PidA.KD = 20;
    PidA.GetVar = GetA;
    PidA.ExOut = 0;
    PidA.Ctrl = CtrlTool;
    lastpos.posX = 0;
    lastpos.posY = 0;
    lastpos.angle = 0;
    PIDCtrlInit();
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
        if(__fabs(GetX() - lastpos.posX) >= 2000 && directionFlag)
        {
            ExSet();
        }
        if(__fabs(GetY() - lastpos.posY) >= 2000 && !directionFlag)
        {
            ExSet();
        }
        PIDCtrl(&PidA);
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) Pos_t.posX);
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) Pos_t.posY);
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) Pos_t.angle);
        USART_OUT(UART4, (uint8_t *)"%d   \n", (int32_t) PidA.ExOut);
	}
}
