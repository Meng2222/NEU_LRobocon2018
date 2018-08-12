#ifndef PID_H
#define PID_H

#include "stdint.h"

static int32_t UOut = 0;
static int32_t err = 0;
static int32_t iSum = 0;
static int32_t lasterr = 0;
static int32_t dDiff = 0;
static int32_t varGetted = 0;

typedef struct
{
    int32_t KP;
    int32_t KI;
    int32_t KD;
    int32_t ExOut;
    int32_t (*GetVar)(void);
    void (*Ctrl)(int32_t);
}PIDCtrlStructure;

void PIDCtrl(PIDCtrlStructure *);
void PIDCtrlInit(void);
int32_t ExRep(int32_t ExAngle, int32_t GetA);

#endif
