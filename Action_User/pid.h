#ifndef __PID_H
#define __PID_H

#include<stdint.h>
#include "usart.h"
#define ON  1
#define OFF 0


float Pid(float valueSet,float valueNow);
uint8_t PidSwitch(uint8_t sw);
void PidPara(float fKp,float fKi,float fKd);

#endif


