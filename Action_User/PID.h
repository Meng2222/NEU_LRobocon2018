#ifndef PID_H
#define PID_H

#include "stdint.h"

static float UOut1 = 0;
static float err1 = 0;
static float iSum1 = 0;
static float lasterr1 = 0;
static float dDiff1 = 0;

static float UOut2 = 0;
static float err2 = 0;
static float iSum2 = 0;
static float lasterr2 = 0;
static float dDiff2 = 0;

typedef struct
{
    float KP;
    float KI;
    float KD;
    float ExOut;
    float (*GetVar)(void);
}PIDCtrlStructure;

float PIDCtrl1(PIDCtrlStructure *);
void PIDCtrlInit1(void);
float PIDCtrl2(PIDCtrlStructure *);
void PIDCtrlInit2(void);

#endif
