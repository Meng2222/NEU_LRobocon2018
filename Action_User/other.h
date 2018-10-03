#ifndef OTHER_H
#define OTHER_H
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
#include "PID.h"
#include "fort.h"
#include "pps.h"
#include "math.h"
#include "stm32f4xx_it.h"

//#include <cruntime.h>
#include <stdlib.h>
//#include <search.h>
//#include <internal.h>

#define Compensation_Angle (0.f)//ÓÒÕý

typedef struct S_Record
{
	float now;
	float p10;
	float p20;
	float p30;
	float p40;
	float p50;
	float p60;
	float p70;
	float p80;
	float p90;
	float p100;
}Record;
void GetFloat (float Num10, int places);
void GetLaserData(void);
void GetPositionValue(PID_Value *pid_out);
void SetFortAngle(PID_Value *pos,float set_angle);
void RadarCorrection(PID_Value *pos);
void Entertainment(void);
void Power_On_Self_Test(PID_Value *pos);
#endif
