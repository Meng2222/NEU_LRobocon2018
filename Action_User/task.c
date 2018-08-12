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

#define CAR4


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
Pos ExPos_t;

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
    while(GetOkFlag())
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

int32_t GetX(void)
{
    return (int32_t)Pos_t.posX;
}

int32_t GetY(void)
{
    return (int32_t)Pos_t.posY;
}
int32_t GetA(void)
{
    return (int32_t)Pos_t.angle;
}

void circle(int32_t Num)
{
    USART_OUT(UART4, (uint8_t *)"%d\n", Num);
    if(Num != 0)
    {
        Num /= 2000;
        if(Num > 0)
        {
            MakeCircle(Num, 500 , LEFT);
        }
        else
        {
            MakeCircle(-1 * Num, -500, RIGHT);
        }
    }
}

//void straight(int32_t Num)
//{
//    GoStraight((float)Num);
//}

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
    uint16_t counter = 0;
    MotorOn(CAN2, 1);
    MotorOn(CAN2, 2);
	OSSemSet(PeriodSem, 0, &os_err);
//    PIDCtrlStructure PidX;
//    PIDCtrlStructure PidY;
    PIDCtrlStructure PidA;
    ExPos_t.angle = 0;
//    ExPos_t.posX = 0;
//    ExPos_t.posY = 2000;
//    PidX.KP = 100;
//    PidX.KI = 1;
//    PidX.KD = 10000;
//    PidX.GetVar = GetX;
//    PidX.ExOut = ExPos_t.posX;
//    PidX.Ctrl = circle;
//    PidY.KP = 100;
//    PidY.KI = 1;
//    PidY.KD = 10000;
//    PidY.GetVar = GetY;
//    PidY.ExOut = ExPos_t.posY;
//    PidY.Ctrl = straight;
    PidA.KP = 100;
    PidA.KI = 20;
    PidA.KD = 200;
    PidA.GetVar = GetA;
    PidA.ExOut = ExPos_t.angle;
    PidA.Ctrl = circle;
    PIDCtrlInit();
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
        counter++;
        if(counter >= 400)
        {
            USART_OUT(UART4, (uint8_t *)"%d    ", GetX());
            USART_OUT(UART4, (uint8_t *)"%d    ", GetY());
            USART_OUT(UART4, (uint8_t *)"%d    \n", GetA());
            if(ExPos_t.angle <= -180)
            {
                ExPos_t.angle = 180;
            }
            ExPos_t.angle -= 90;
            PidA.ExOut = ExPos_t.angle;
            PIDCtrlInit();
            counter = 0;
        }
        if(ExPos_t.angle - GetA() == 0)
        {
            //GoStraight(500);
        }
        else
        {
            PIDCtrl(&PidA);
        }
	}
}
