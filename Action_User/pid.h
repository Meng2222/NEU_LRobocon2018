#ifndef __PID_H
#define __PID_H
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
#define SPEED 1200
#define R 2400
void Compute1(float a,float b,float r,float v,int dir);
void Compute2(float a,float b,float r,float v,int dir);
void straightPID(float r,float speed,int dir);
void SetTunings1(double Kp1,double Ki1,double Kd1);
void SetTunings2(double Kp2,double Ki2,double Kd2);
void Double_closed_loop(float a,float b,float r,float v,int dir);


#endif
