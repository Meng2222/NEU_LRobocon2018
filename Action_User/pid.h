#ifndef __PID_H
#define __PID_H

#include<stdint.h>

#define ON  1
#define OFF 0


float Pid(uint16_t valueNow,uint16_t valueSet,uint8_t swt);
uint8_t PidSwitch(uint8_t sw);
void PidPara(float fKp,float fKi,float fKd);

#endif


