#ifndef ROBOWALK_H
#define ROBOWALK_H

#include "Pos.h"
#include "PID.h"
#include "adc.h"
#include "moveBase.h"

#define SPEED_OF_CAR 2000

/**
 * [RoboWalkInit 机器人行走初始化]
 */
void RoboWalkInit(void);

/**
 * [SetDir2TurnAround 根据Dir2TurnAround的值改变相关线路方向]
 */
void SetDir2TurnAround(void);

/**
 * [ChangeRoad 改变线路]
 */
void ChangeRoad(void);

/**
 * [Wait4ADC 等待激光被遮挡]
 */
void Wait4ADC(void);

/**
 * [Change_Dir_2_Turn_Around 路径反向]
 */
void Change_Dir_2_Turn_Around(void);

#endif
