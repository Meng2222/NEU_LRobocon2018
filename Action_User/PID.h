#ifndef PID_H
#define PID_H

#include "stdint.h"

typedef struct
{
    float err;
    float iSum;
    float lasterr;
    float dDiff;
}PIDCtrlTemp;

static PIDCtrlTemp Temp1 = {0, 0, 0, 0};
static PIDCtrlTemp Temp2 = {0, 0, 0, 0};

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

float PIDCtrl1(const PIDCtrlStructure);
void PIDCtrlInit1(void);
float PIDCtrl2(const PIDCtrlStructure);
void PIDCtrlInit2(void);

#endif
