#ifndef __STATUS_H__
#define __STATUS_H__
#define Car 4
#define pi 3.1415

void straight(float v);
void circular(float v,float r,int direction);
int exchange(float v);
float AnglePID(float Angle,float SetAngle);
int AngleChange(void);
float DirectionPID( float distance );
void line( float a ,float b ,float c , int direction , float v );
float GetRoundSetAngle(float X,float Y,int DIRECTION);
void surround(float X,float Y,float R,float V,int DIRECTION);

void BubbleSort(float *a,int number);
void SetAngle(float val);
void SetXpos(float val);
void SetYpos(float val);
float GetXpos(void);
float GetYpos(void);
float GetAngle(void);


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
#define Clockwise 1
#define Anti_clockwise 2
#endif
