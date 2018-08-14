#ifndef PID_H
#define PID_H

#include "stdint.h"

static float UOut = 0;
static float err = 0;
static float iSum = 0;
static float lasterr = 0;
static float dDiff = 0;

typedef struct
{
    float KP;
    float KI;
    float KD;
    float ExOut;
    float (*GetVar)(void);
    void (*Ctrl)(float);
}PIDCtrlStructure;

void PIDCtrl(PIDCtrlStructure *);
void PIDCtrlInit(void);

#endif
