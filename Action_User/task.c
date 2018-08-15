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
    .b = 1,
    .c = -1000,
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
    return Point2Line(&Target);
}

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
    MotorOn(CAN2, 1);
    MotorOn(CAN2, 2);
	OSSemSet(PeriodSem, 0, &os_err);
    PidA.KP = 5;
    PidA.KI = 0.025;
    PidA.KD = 5;
    PidA.GetVar = GetA;
    PidA.ExOut = LineDir(&Target, forward);
    PidB.KP = 1;
    PidB.KI = 0.0005;
    PidB.KD = 10;
    PidB.GetVar = GetDistance;
    PidB.ExOut = 0;
    PIDCtrlInit1();
    PIDCtrlInit2();
    float uout1 = 0, uout2 = 0; 
    float k1 = 0, k2 = 0; 
    reldir reldirNow2Line;
    USART_OUT(UART4, (uint8_t *)"u1   u2   x   y   a   \r\n");
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
        GetNowPoint(&nowPoint);
        uout1 = PIDCtrl1(&PidA);
        uout2 = PIDCtrl2(&PidB);
        reldirNow2Line = RelDir2Line(&Target, forward);
        if(reldirNow2Line == right)
        {
            k1 = 0.7;
            k2 = 1.3;
        }
        else if(reldirNow2Line == left)
        {
            k1 = 1.3;
            k2 = 0.7;
        }
        else
        {
            k1 = 0;
            k2 = 0;
            if((int32_t)GetA() == (int32_t)PidA.ExOut)
            {
                PIDCtrlInit1();
            }
        }
//        if(__fabs(500 + k1 * uout2 + uout1) > 1000 || __fabs(500 + k1 * uout2 - uout1) > 1000)
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
