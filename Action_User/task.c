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
#include "Pos.h"

#define DIRFORLINE backward


/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

PIDCtrlStructure PidA;
PIDCtrlStructure PidB;

point nowPoint;

line Target =
{
    .a = 1,
    .b = 0,
    .c = 1000,
};
    
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
    delay_s(10);
    delay_s(5);
	OSTaskSuspend(OS_PRIO_SELF);
}

float GetDistance(void)
{
    if(RelDir2Line(&Target, DIRFORLINE) == right)
    {
        return Point2Line(&Target);
    }
    else if(RelDir2Line(&Target, DIRFORLINE) == left)
    {
        return -1 * Point2Line(&Target);
    }
    else
    {
        return 0;
    }
}

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
    MotorOn(CAN2, 1);
    MotorOn(CAN2, 2);
	OSSemSet(PeriodSem, 0, &os_err);
    PidA.KP = 5;
    PidA.KI = 0.005;
    PidA.KD = 2.5;
    PidA.GetVar = GetA;
    PidA.ExOut = LineDir(&Target, DIRFORLINE);
    PidB.KP = 5;
    PidB.KI = 0;
    PidB.KD = 2.5;
    PidB.GetVar = GetDistance;
    PidB.ExOut = 0;
    PIDCtrlInit1();
    PIDCtrlInit2();
    float uout1 = 0, uout2 = 0; 
    float k1 = 0, k2 = 0; 
    float anglediff = 0;
    reldir reldirNow2Line;
    USART_OUT(UART4, (uint8_t *)"u1   u2   x   y   a   \r\n");
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
        GetNowPoint(&nowPoint);
        uout1 = PIDCtrl1(&PidA);
        uout2 = __fabs(PIDCtrl2(&PidB));
        reldirNow2Line = RelDir2Line(&Target, DIRFORLINE);
        anglediff = __fabs(GetA() - PidA.ExOut);
        if(anglediff > 180)
        {
            anglediff = __fabs(anglediff - 360);
        }
        if(reldirNow2Line == right)
        {
            if(anglediff <= 90)
            {
                k1 = 0.14 * 3;
                k2 = 0.06 * 3;
            }
            else
            {
                k1 = 0.06 * 3;
                k2 = 0.14 * 3;
            }
        }
        else if(reldirNow2Line == left)
        {
            if(anglediff <= 90)
            {
                k1 = 0.06 * 3;
                k2 = 0.14 * 3;
            }
            else
            {
                k1 = 0.14 * 3;
                k2 = 0.06 * 3;
            }
        }
        if((int32_t) GetDistance() == 0)
        {
            k1 = 0;
            k2 = 0;
            PIDCtrlInit2();
            if((int32_t)GetA() == (int32_t)PidA.ExOut)
            {
                PIDCtrlInit1();
            }
        }
//        if(__fabs(500 + k1 * uout2 + uout1) > 1500 || __fabs(500 + k1 * uout2 - uout1) > 1500)
//        {
//            uout1 = 0;
//            uout2 = 0;
//            PIDCtrlInit1();
//            PIDCtrlInit2();
//            while(1)
//            {
//                WheelSpeed(0, 1);//right
//                WheelSpeed(0, 2);//left
//                USART_OUT(UART4, (uint8_t *)"overspeed!   \r\n");
//            }
//        }
        WheelSpeed(500 + k1 * uout2 + uout1, 1);//right
//        USART_OUT(UART4, (uint8_t *)"%d   \r\n", (int32_t) 500 + k1 * uout2 + uout1);
        WheelSpeed(500 + k2 * uout2 - uout1, 2);//left
//        USART_OUT(UART4, (uint8_t *)"%d   \r\n", (int32_t) 500 + k2 * uout2 - uout1);
//        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) k1);
//        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) k2);
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) uout1);
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) uout2);
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) nowPoint.x);
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) nowPoint.y);
        USART_OUT(UART4, (uint8_t *)"%d   \r\n", (int32_t) GetA());
	}
}
