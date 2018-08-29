#ifndef OTHERMOTO_H
#define OTHERMOTO_H

#include "includes.h"
#include "Pos.h"
/**
 * 该文件中有关航向轮和发射电机的相关函数包装自fort.h
 */

// 宏定义推球电机ID,收球电机ID,航向电机ID
#define PUSH_BALL_ID (6)
#define GUN_YAW_ID (7)
#define COLLECT_BALL_ID (8)

//储藏区中心到边上的距离
#define LENGTH_TO_THE_SIDE (200)

#define LENGTH_BETWEEN_LASER (95)

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
void CollecterWheelSpeedCtrl(float speed);

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

/**
 * [GetPosToLauncher 返回对应储藏区相对于发射器的相对位置]
 * @param  storageInAirPoint [对应储藏区,其值可以是storageInAirPointlb, storageInAirPointlf, storageInAirPointrb, storageInAirPointrf]
 * @return                   [相对位置]
 */
point GetPosToLauncher(const point storageInAirPoint);

/**
 * [SuitableStoragePos 返回最近的储藏区]
 * @return  [如题]
 */
point SuitableStoragePos(void);

/**
 * [SuitableSpeed2Launch 返回适合投球的发射轮转速]
 * @param  distance [与储藏区的距离]
 * @return          [适合发射的发射轮转速]
 */
float SuitableSpeed2Launch(float distance);

/**
 * [Delta4Laser 判断激光瞄准是否在误差范围内]
 * @return  [在范围内返回1,反之返回0]
 */
_Bool Delta4Laser(void);

/**
 * [LaunchFlag 确认是否达到发射条件]
 * @param  storagePos [要瞄准的位置]
 * @return            [准备好返回1,否则返回0]
 */
_Bool LaunchFlag(point storagePos);
    
#endif
