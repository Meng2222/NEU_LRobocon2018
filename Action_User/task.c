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
#include "Pos.h"
#include "pps.h"
#include "othermoto.h"
#include "RoboWalk.h"

/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
OS_EVENT *PeriodSem2;
OS_EVENT *PeriodSem3;
OS_EVENT *ErrSem;
OS_EVENT *LaunchSem;

PIDCtrlStructure PidA;
PIDCtrlStructure PidB;   

point nowPoint;

static struct
{
    float left;
    float right;
}wheelspeed={0, 0};

static _Bool errCheckOn_Flag;
    
point storagePos;

extern enum {clockwise, anticlockwise} Dir2TurnAround;

void App_Task()
{
	CPU_INT08U os_err;
	os_err = os_err; /*防止警告...*/

	/*创建信号量*/
	PeriodSem = OSSemCreate(0);
	PeriodSem2 = OSSemCreate(0);
	PeriodSem3 = OSSemCreate(0);
	ErrSem = OSSemCreate(0);
    LaunchSem = OSSemCreate(0);
	/*创建任务*/
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, /*初始化任务*/
						  (void *)0,
						  (OS_STK *)&App_ConfigStk[Config_TASK_START_STK_SIZE - 1],
						  (INT8U)Config_TASK_START_PRIO);

	os_err = OSTaskCreate((void (*)(void *))WalkTask,
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  (INT8U)Walk_TASK_PRIO);
                          
	os_err = OSTaskCreate((void (*)(void *))ErrCheck,
						  (void *)0,
						  (OS_STK *)&ErrCheckStk[ERR_CHECK_STK_SIZE - 1],
						  (INT8U)ERR_CHECK_PRIO);
                          
	os_err = OSTaskCreate((void (*)(void *))ErrSolve,
						  (void *)0,
						  (OS_STK *)&ErrSolveStk[ERR_SOLVE_STK_SIZE - 1],
						  (INT8U)ERR_SOLVE_PRIO);
                          
    os_err = OSTaskCreate((void (*)(void *))LaunchTheBall,
                          (void *)0,
                          (OS_STK *)&LaunchTheBallStk[LAUNCH_THE_BALL_STK_SIZE - 1],
                          (INT8U)LAUNCH_THE_BALL_PRIO);
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
    UART4_Init(921600);
    ppsInit();
    OtherMotoInit();
    RoboWalkInit();
    OSTaskSuspend(ERR_CHECK_PRIO);
    errCheckOn_Flag = 0;
	OSTaskSuspend(OS_PRIO_SELF);
}

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
    point relPoint, centre;
    float uout1 = 0, uout2 = 0;
    float delta = 0;
    extern float circleradius, length;
//    USART_OUT(UART4, (uint8_t *)"x   y   a\r\n");
    Wait4ADC();
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
        nowPoint = GetNowPoint();
        centre = setPointXY(0, 2400);
        relPoint = RelPos(centre, nowPoint);
//        if(Dir2TurnAround == clockwise)
//        {
//            delta = (int32_t)(relPoint.a + (45 * ((circleradius - 800) / 1200))) % 90;
//        }
//        else
//        {
//            delta = (int32_t)(relPoint.a - (45 * ((circleradius - 800) / 1200))) % 90;
//        }
        if(Dir2TurnAround == clockwise)
        {
            delta = (int32_t)(relPoint.a + 50) % 90;
        }
        else
        {
            delta = (int32_t)(relPoint.a + 40) % 90;
        }
        if(delta == 0)
        {
            OSSemPost(LaunchSem);
        }
        storagePos = GetPosToLauncher(SuitableStoragePos());
        LauncherYAWCtrl(storagePos.a);
//        LauncherYAWCtrl(0);
//        LauncherWheelSpeedCtrl(60);
//        SendBall2Launcher();
        ChangeRoad();
        uout1 = PIDCtrl1(PidA);
        uout2 = PIDCtrl2(PidB);
        wheelspeed.right = SPEED_OF_CAR + uout1 + uout2;
        wheelspeed.left = SPEED_OF_CAR - uout1 - uout2;
        WheelSpeed(wheelspeed.right, 1);
        WheelSpeed(wheelspeed.left, 2);
        if(!errCheckOn_Flag)
        {
            errCheckOn_Flag = 1;
            OSTaskResume(ERR_CHECK_PRIO);
        }
//        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) nowPoint.x);
//        USART_OUT(UART4, (uint8_t *)"%d   ", (int32_t) nowPoint.y);
//        USART_OUT(UART4, (uint8_t *)"%d   \r\n", (int32_t) GetA());
	}
}

void ErrCheck(void)
{
    CPU_INT08U os_err;
    os_err = os_err;
    point lastpoint;
    lastpoint = setPointXY(0, 0);
    uint8_t counter = 0;
    OSSemSet(PeriodSem2, 0, &os_err);
    while (1)
    {
        OSSemPend(PeriodSem2, 0, &os_err);
        if(Point2Point(lastpoint, nowPoint) >= 10)
        {
            counter = 0;
        }
        if(Point2Point(lastpoint, nowPoint) < 10)
        {
            counter++;
        }
        if(counter >= 10)
        {
            counter = 0;
            OSSemPost(ErrSem);
        }
        lastpoint = nowPoint;
    }
}

void ErrSolve(void)
{
    CPU_INT08U os_err;
    os_err = os_err;
    OSSemSet(ErrSem, 0, &os_err);
    while(1)
    {
        OSSemPend(ErrSem, 0, &os_err);
        OSTaskSuspend(Walk_TASK_PRIO);
        OSTaskSuspend(ERR_CHECK_PRIO);
        PIDCtrlInit1();
        PIDCtrlInit2();
        uint8_t counter = 100;
        OSSemSet(PeriodSem, 0, &os_err);
        while(counter--)
        {
            OSSemPend(PeriodSem, 0, &os_err);
            WheelSpeed(-1 * wheelspeed.right, 1);
            WheelSpeed(-1 * wheelspeed.left, 2);
        }   
        Change_Dir_2_Turn_Around();
        OSSemSet(PeriodSem, 0, &os_err);
        OSSemSet(PeriodSem2, 0, &os_err);
        OSTaskResume(Walk_TASK_PRIO);
        errCheckOn_Flag = 0;
    }
}

void LaunchTheBall(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
    uint8_t counter = 50;
    OSSemSet(LaunchSem, 0, &os_err);
	while (1)
	{
        OSSemPend(LaunchSem, 0, &os_err);
        OSTaskSuspend(Walk_TASK_PRIO);
        OSTaskSuspend(ERR_CHECK_PRIO);
        WheelSpeed(0, 1);
        WheelSpeed(0, 2);
        nowPoint = GetNowPoint();
        storagePos = GetPosToLauncher(SuitableStoragePos()); 
//        counter = 75 ;
//        OSSemSet(PeriodSem3, 0, &os_err);
//        while(counter--)
//        {
//            OSSemPend(PeriodSem3, 0, &os_err);
//            nowPoint = GetNowPoint();
//            WheelSpeed(0, 1);
//            WheelSpeed(0, 2); 
//            storagePos = GetPosToLauncher(SuitableStoragePos());
//            LauncherYAWCtrl(storagePos.a);
//            LauncherWheelSpeedCtrl(SuitableSpeed2Launch(storagePos.r));
//        }
        while(!(LaunchFlag(storagePos)))
        {
            WheelSpeed(0, 1);
            WheelSpeed(0, 2); 
            nowPoint = GetNowPoint();
            storagePos = GetPosToLauncher(SuitableStoragePos());
            LauncherYAWCtrl(storagePos.a);
            LauncherWheelSpeedCtrl(SuitableSpeed2Launch(storagePos.r));
        }
        counter = 50;
        OSSemSet(PeriodSem3, 0, &os_err);
        while(counter--)
        {
            OSSemPend(PeriodSem3, 0, &os_err);
            nowPoint = GetNowPoint();
            WheelSpeed(0, 1);
            WheelSpeed(0, 2);
            if(counter == 50)
            {
                SendBall2Launcher();
            }
            if(counter == 25)
            {
                SendBall2Launcher();
            }
////            storagePos = GetPosToLauncher(SuitableStoragePos());
////            LauncherYAWCtrl(storagePos.a);
//            LauncherYAWCtrl(0);
//            if(!(Delta4Laser()))
//            {
//                for(uint8_t i = 0; i < 50; i++)
//                {
//                    if(Delta4Laser())
//                    {
//                        break;
//                    }
//                    storagePos.a += 0.1f;
//                }
//                for(uint8_t i = 0; i < 100; i++)
//                {
//                    if(Delta4Laser())
//                    {
//                        break;
//                    }
//                    storagePos.a -= 0.1f;
//                }
//            }
////            LauncherWheelSpeedCtrl(SuitableSpeed2Launch(storagePos.r));
////            SendBall2Launcher();
        }
        OSSemSet(PeriodSem, 0, &os_err);
        OSSemSet(PeriodSem2, 0, &os_err);
        OSTaskResume(Walk_TASK_PRIO);
        errCheckOn_Flag = 0;
	}
}
