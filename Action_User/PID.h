#ifndef _PID_H
#define _PID_H
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
#include "stm32f4xx.h"
#include <ucos_ii.h>
#include <math.h>

#define ACW 0
#define CW 1
#define manual 1
#define Auto 0
void WalkStraight(int v);
void WalkRound(u8 direction, int v,int r);
void driveGyro(void);
void SetTunings(float p,float i,float d);
void Init_PID(float angle);
void PID_Angle(u8 status,float Angle_Set,float Angle,int v);
void PID_Coordinate(float x0,float y0,int v);
void PID_Line(float x1,float y1,float x2,float y2,int v);
void PID_Sauare(float v);
void PID_Round(float x0,float y0,float r,float v,u8 direction);
void PID_Coordinate_following(float v);

#endif
