#ifndef PID_H
#define PID_H

#include "stdint.h"

typedef struct
{
    float KP;
    float KI;
    float KD;
    float ExOut;
    float (*GetVar)(void);
}PIDCtrlStructure;

float PIDCtrl1(const PIDCtrlStructure);
void PIDCtrlInit1(void);
float PIDCtrl2(const PIDCtrlStructure);
void PIDCtrlInit2(void);

#endif
