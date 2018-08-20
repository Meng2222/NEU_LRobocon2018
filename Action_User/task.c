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

//#define DT 500
#define safe 1
#define X1 50
#define X2 100
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

//static const linewithdir line1 = {0, 1, 0, forward};
//static const linewithdir line2 = {1, 0, 0, forward};
//static const linewithdir line3 = {0, 1, -2000, backward};
//static const linewithdir line4 = {1, 0, -2000, backward};

//static const linewithdir corss1 = {1000-DT, 1000, -1 * (1000*(2000 - DT)), forward};
//static const linewithdir corss2 = {1000, DT - 1000, -1 * DT * 1000, forward};

#define CCX 1000
#define CCY 1000
static point circlecentre;
static const reldir circledir = right;
static const float circleradius = 1000;

//static linewithdir Target =
//{
//    .a = 1,
//    .b = 0,
//    .c = 0,
//    .linedir = forward,
//};

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

float LengthProcessing(float x)
{
    if(x <= X1)
    {
        return 0;
    }
    else if(x >= X2)
    {
        return 1;
    }
    else
    {
        return (atan((x - X1) / (X2 - x)) / 1.57);
    }
}

float LengthProcessing2(float x)
{
    return (x/X1)/((x/X1)+1);
}

float LengthProcessing3(float x)
{
    return (1 - pow(100, (-1 * x / X1)));
}

float GetP(void)
{
    float tempNum = 0;
    tempNum =Point2Point(circlecentre, nowPoint) - circleradius;
    if(tempNum > 0)
    {
        return LengthProcessing(__fabs(tempNum));
    }
    else if(tempNum < 0)
    {
        return -1 * LengthProcessing(__fabs(tempNum));
    }
    else
    {
        return 0;
    }
}

//void ChangeRoad(void)
//{
//    reldir dir2corss1,dir2corss2;
//    dir2corss1 = RelDir2Line(corss1, nowPoint);
//    dir2corss2 = RelDir2Line(corss2, nowPoint);
//    if(dir2corss1 == left)
//    {
//        if(dir2corss2 == right)
//        {
//            Target = line1;
//        }
//        else if(dir2corss2 == left)
//        {
//            Target = line2;
//        }
//    }
//    else if(dir2corss1 == right)
//    {
//        if(dir2corss2 == left)
//        {
//            Target = line3;
//        }
//        else if(dir2corss2 == right)
//        {
//            Target = line4;
//        }
//    }
//}

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
    float uout1 = 0, uout2 = 0; 
    linewithdir line2centre;
	OSSemSet(PeriodSem, 0, &os_err);
    PidA.KP = 20; //20
    PidA.KI = 0.01; //0.01
    PidA.KD = 10; //20
    PidA.GetVar = GetA;
    PidA.ExOut = VDirForLine(line2centre, circledir);
    PidB.KP = 5; //10
    PidB.KI = 0.0001; //0.0001
    PidB.KD = 10; //20
    PidB.GetVar = GetA;
    PidB.ExOut = PidA.ExOut + 90 * GetP();
    circlecentre = setPointXY(CCX, CCY);
    MotorOn(CAN2, 1);
    MotorOn(CAN2, 2);
    PIDCtrlInit1();
    PIDCtrlInit2();
//    float anglediff = 0;
//    reldir reldirNow2Line;
    USART_OUT(UART4, (uint8_t *)"u1   u2   x   y   a   ea2   ea   jl   e1   e2\r\n");
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
        nowPoint = GetNowPoint();
//        reldirNow2Line = RelDir2Line(&Target, linedir);
//        anglediff = __fabs(GetA() - PidB.ExOut);
//        ChangeRoad();
        line2centre = DirlinePoint2Point(circlecentre, nowPoint);
        PidA.ExOut = VDirForLine(line2centre, circledir);
        PidB.ExOut = PidA.ExOut - 90 * GetP();
        if(PidB.ExOut >= 180)
        {
            PidB.ExOut -= 360;
        }
        if(PidB.ExOut <= -180)
        {
            PidB.ExOut += 360;
        }
////        if(anglediff > 180)
////        {
////            anglediff = __fabs(anglediff - 360);
////        }
////        if(anglediff > 90)
////        {
////            uout2 = -1 * uout2;
////        }
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
        uout1 = PIDCtrl1(PidA);
        uout2 = PIDCtrl2(PidB);
        WheelSpeed(1000 + uout2 / safe + uout1 / safe, 1);//right
        WheelSpeed(1000 - uout2 / safe - uout1 / safe, 2);//left
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) uout1);
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) uout2);
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) nowPoint.x);
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) nowPoint.y);
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) GetA());
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) PidB.ExOut);
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) PidA.ExOut);
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) Point2Point(circlecentre, nowPoint));
        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) PidA.ExOut - GetA());
        USART_OUT(UART4, (uint8_t *)"%d   \r\n", (int32_t) PidB.ExOut - GetA());
	}
}
