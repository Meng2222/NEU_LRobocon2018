#ifndef __STATUS_H__
#define __STATUS_H__
#define Car 1
#define pi 3.1415

void straight(float v);
void circular(float v,float r,char direction);
int exchange(float v);

void SetAngle(float val);
void SetXpos(float val);
void SetYpos(float val);
float GetXpos(void);
float GetYpos(void);
float GetAngle(void);
//int AngleChange(void);
float AnglePID(float Angle,float SetAngle);
float DirectionPID( float distance );
void line( float a ,float b ,float c , char direction , float v );

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
