/****************************************Copyright (c)****************************************************
**
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               app_cfg.h
** Descriptions:            ucosii configuration
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-11-9
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Descriptions:
**
*********************************************************************************************************/

#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__
#include  <os_cpu.h>
/*
*********************************************************************************************************
*                                       MODULE ENABLE / DISABLE
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              TASKS NAMES
*********************************************************************************************************
*/
extern  void  App_Task(void);

static  void  App_TaskStart(void);
static 	void  ConfigTask(void);
static 	void  WalkTask(void);
static 	void  ErrCheck(void);
static  void  ErrSolve(void);
static  void  LaunchTheBall(void);

/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/

#define  APP_TASK_START_PRIO						10u
#define  Config_TASK_START_PRIO						11u
#define  Walk_TASK_PRIO								15u
#define  ERR_CHECK_PRIO								14u
#define  ERR_SOLVE_PRIO								13u
#define  LAUNCH_THE_BALL_PRIO						12u




/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*                             Size of the task stacks (# of OS_STK entries)
*********************************************************************************************************
*/
#define  APP_TASK_START_STK_SIZE					256u
#define  Config_TASK_START_STK_SIZE					256u
#define  Walk_TASK_STK_SIZE							512u
#define  ERR_CHECK_STK_SIZE							256u
#define  ERR_SOLVE_STK_SIZE							256u
#define  LAUNCH_THE_BALL_STK_SIZE					256u

/*
*********************************************************************************************************
*                                            TASK STACK
*
*********************************************************************************************************
*/

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
static OS_STK ErrCheckStk[ERR_CHECK_STK_SIZE];
static OS_STK ErrSolveStk[ERR_SOLVE_STK_SIZE];
static OS_STK LaunchTheBallStk[LAUNCH_THE_BALL_STK_SIZE];

/*
*********************************************************************************************************
*                                                  LIB
*********************************************************************************************************
*/



#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

