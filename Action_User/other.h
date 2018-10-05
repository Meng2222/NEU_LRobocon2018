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

#define Compensation_Angle (0.f)		//右正

#define SET_ROLLER_SPEED (70)			//收球电机转速
#define LEFT_ROLLER_THRESHOLD (4) /*3*/	//收球电机转速阈值
#define RIGHT_ROLLER_THRESHOLD (5) /*6*///收球电机转速阈值
/*1*/
#define LEFT_FIRST_THRESHOLD ()			//>=4或最小值<=65(+)   <=8(+) 最小值>54(-2)   积分[ , ]
#define RIGHT_FIRST_THRESHOLD ()		//>=3或最小值<=62(+)   <=10(+)最小值>55(-2)   积分[45(-) , 97(+)]
/*2*/
#define LEFT_SECOND_THRESHOLD ()		//>=10(-)或最小值<=53(+)   <=12(+)最小值>47(-)   积分[146(-) , 233(+)]
#define RIGHT_SECOND_THRESHOLD ()		//>=15(-)或最小值<=53(+)   <=18(+)最小值>47(-)   积分[130(-) , 242(+)]
/*3*/
#define LEFT_THIRED_THRESHOLD ()		//>=14(-)或最小值<=41(+)   <=18(+)最小值>36(-)   积分[230(-) , 457(+)]
#define RIGHT_THIRED_THRESHOLD ()		//>=18(-)或最小值<=52(+)   <=23(+)最小值>43(-)   积分[223(-) , 359(+)]
/*4*/
#define LEFT_FOURTH_THRESHOLD ()		//有效值点数[ , ]   最小值范围[ , ]   积分区间[ , ]
#define RIGHT_FOURTH_THRESHOLD ()		//有效值点数[ , ]   最小值范围[ , ]   积分区间[ , ]
/*5*/
#define LEFT_FIFTH_THRESHOLD ()			//有效值点数[ , ]   最小值范围[ , ]   积分区间[ , ]
#define RIGHT_FIFTH_THRESHOLD ()		//有效值点数[ , ]   最小值范围[ , ]   积分区间[ , ]
/*More*/
#define LEFT_FIFTH_THRESHOLD ()			//有效值点数[ , ]   最小值范围[ , ]   积分区间[ , ]
#define RIGHT_FIFTH_THRESHOLD ()		//有效值点数[ , ]   最小值范围[ , ]   积分区间[ , ]

void GetFloat (float Num10, int places);
void GetLaserData(void);
void GetPositionValue(PID_Value *pid_out);
void SetFortAngle(PID_Value *pos,float set_angle);
void RadarCorrection(PID_Value *pos);
void Entertainment(void);
void Power_On_Self_Test(PID_Value *pos);
void CntGolf(void);
#endif
