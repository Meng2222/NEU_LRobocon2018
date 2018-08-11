#ifndef __STATUS_H__
#define __STATUS_H__


void straight(float v);
void circular(float v,float r,char direction);
int exchange(float v);

void SetAngle(float val);
void SetXpos(float val);
void SetYpos(float val);
float GetXpos(void);
float GetYpos(void);
float GetAngle(void);
float AnglePID(float Angle,float SetAngle);

struct PID
{
	float p;
	float i;
	float d;
};


//³µ1×¨ÓÃ
int isSendOK(void);
void SetOKFlagZero(void);
void driveGyro(void);
#define Left '1'
#define Right '2'
#endif
