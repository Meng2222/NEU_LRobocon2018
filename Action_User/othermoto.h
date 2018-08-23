#ifndef OTHERMOTO_H
#define OTHERMOTO_H

#include "includes.h"
/**
 * 该文件中有关航向轮和发射电机的相关函数包装自fort.h
 */

// 宏定义推球电机ID,收球电机ID,航向电机ID
#define PUSH_BALL_ID (6)
#define GUN_YAW_ID (7)
#define COLLECT_BALL_ID (8)

// 宏定义送弹机构送弹时电机应该到达位置：单位位脉冲
#define PUSH_POSITION (4500)
// 宏定义送弹机构收回时电机位置
#define PUSH_RESET_POSITION (5)

#define LAUNCHER_SPEED_DEFINE (0)
#define COLLECTER_SPEED_DEFINE (0)

//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
// 宏定义发射机构航向电机ID
// 电机旋转一周的脉冲数
#define COUNT_PER_ROUND (4096.0f)
// 宏定义每度对应脉冲数
#define COUNT_PER_DEGREE  (COUNT_PER_ROUND/360.0f)
// 宏定义航向角减速比
#define YAW_REDUCTION_RATIO (4.0f)

/**
 * [OthermotoInit 除移动系统外其他电机的初始化]
 */
void OtherMotoInit(void);

/**
 * [LauncherWheelSpeedCtrl用来控制发射器的电机转速]
 * @param speed [该参数是发射器电机转速(转/秒)范围0~100]
 */
void LauncherWheelSpeedCtrl(int32_t speed);

/**
 * [CollecterWheelSpeedCtrl用来控制收球机构的电机转速]
 * @param speed [该参数是收球机构电机转速(转/秒)范围0~60]
 */
void CollecterWheelSpeedCtrl(int32_t speed);

/**
 * [SendBall2Launcher用来推球]
 * [函数将会在推球状态运行200次,之后再复位运行200次(可修改,具体看函数实现).]
 */
void SendBall2Launcher(void);

/**
 * [YawAngleCtr 发射航向角控制函数]
 * @param yawAngle [转台航向角度，范围为0~360度.]
 */
void LauncherYAWCtrl(float yawAngle);

/**
 * [返回发射系统的各种信息]
 */
float GetLauncherSpeed(void);
float GetLauncherAngle(void);
float GetLaserA(void);
float GetLaserB(void);
#endif
