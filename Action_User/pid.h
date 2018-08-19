#ifndef	__PID_H
#define __PID_H
#include "stm32f4xx_it.h"
#include "moveBase.h"
#include "can.h"
#include "elmo.h"
#include "math.h"
typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float lastErr;
	float sumErr;
}PId_t;

typedef struct {
	float x;
	float y;
	float radius;
	float vel;
	float side;
}Round_t;
#define SPEED 800
void KownedLinePID(float a,float b,float c,char dir);
void WalkLine(float vel);
void WalkLine2PID(float vel,float setAngle);
void WalkRound(float vel,float radius,char side);
float PID_Compentate(float err,PId_t* PId_tTYPE);
void Angle_PID(float vel,float value);
float Dis_PID(float error);
void WalkRoundPID(Round_t* PID_RndTYPE);
//typedef struct {
//	float x;
//	float y;
//}PosNow_t;//定义转换位置时存当时位置
#endif
