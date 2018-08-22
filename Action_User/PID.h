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
#include "adc.h"
#include "includes.h"
#include "usart.h"
#include "movebase.h"
/*返回定位系统的角度*/
float GetAngle(void);
/*返回定位系统的X值*/
float GetX(void);
/*返回定位系统的Y值*/
float GetY(void);
/*返回定位系统的X轴的速度*/
float GetSpeedX(void);
/*返回定位系统的角度*/
float GetSpeedY(void);
/*返回定位系统的Z轴角速度值*/
float GetWZ(void);
float ABS(float thing);
float Compare(float a1,float b1);
float constrain(float amt, float high, float low);

#define manual 1
#define Arc 2
#define Line 3
#define Coordinate 4
#define ACW 5
#define CW 6
#define Left 7
#define Right 8
typedef struct line
{
	float x1;
	float y1;
	float x2;
	float y2;
	float line_A;
	float line_B;
	float line_C;
	float line_kp;
	float line_Angle;
	float line_Error;	
}Line_Value;

typedef struct arc
{
	float x0;
	float y0;
	float r0;
	float arc_kp;
	float arc_Direction;
	float arc_Angle;
	float arc_Error;
}Arc_Value;

typedef struct coordinate
{
	float x3;
	float y3;
	float coordinate_Angle;
}Coordinate_Value;

typedef struct PID
{
	int Mode;
	int Mode_Last;
	float Angle_Set;
	float kp;
	float ki;
	float kd;
	float Angle;
	float Angle_Last;
	float Error;
	float ITerm;
	float DTerm;
	float X;
	float Y;
	float X_Speed;
	float Y_Speed;
	float V;
	float V_Set;
	int Line_Num;
	int Arc_Num;
	int Coordinate_Num;
	Line_Value *l;
	Arc_Value *r;
	Coordinate_Value *c;
	float vel;
}PID_Value;

void PID_Init(PID_Value *PID_a);
void PID_Line_Init(void);
void PID_Arc_Init(void);
void PID_Coordinate_Init(void);
void Init_PID(PID_Value *pid_init);
void PID_Control(PID_Value *p);
void GO(PID_Value *p_GO);
void PID_Control_Competition(PID_Value *pid);
void PID_Pre(PID_Value *p);

#endif
