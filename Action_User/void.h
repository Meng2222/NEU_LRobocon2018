#ifndef __STATUS_H__
#define __STATUS_H__
#define Car 4
#define pi 3.14159
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
float DirectionPID( float distance,float setdistance );
void Walkline( int setx,int sety,int r, int direction , float v );

struct PID
{
	float p;
	float i;
	float d;
};
void Walkback(float v);
void Walkaway(float v);
void Walkahead(float v);
int AdcFlag(void);
int Radius(void);
//³µ1×¨ÓÃ
int isSendOK(void);
void SetOKFlagZero(void);
void driveGyro(void);
void errdeal(void);
void PushBall(int T);
void ShootBall(void);
int  Distopow(float distance);
#define Left '1'
#define Right '2'
#endif
