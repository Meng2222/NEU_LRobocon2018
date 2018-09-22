#ifndef __PID_H
#define __PID_H

#include<stdint.h>
#include "usart.h"
#define ON  1
#define OFF 0


float AnglePid(float valueSet,float valueNow);
float DisplacementPid(float valueSet,float valueNow);
float DistancePid(float valueSet,float valueNow);
float GyroPid(float valueSet,float valueNow);
uint8_t PidSwitch(uint8_t sw);
void Angle_PidPara(float fKp,float fKi,float fKd);
void Distance_PidPara(float fKp,float fKi,float fKd);
void Gyro_PidPara(float fKp,float fKi,float fKd);

#endif


