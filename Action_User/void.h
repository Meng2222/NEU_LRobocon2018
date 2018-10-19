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
void Walkline( float corex,float corey,float Radium,float V_loop,int SN );//正常走行,输入圆心坐标，半径，方向
void pull_ball(void);
void  aim_fire();
struct PID
{
	float p;
	float i;
	float d;
};
void push_ball(void);
void systerm_change();//坐标系转换
void Walkback(float v);
void Walk_left_away(float v,int time);
void Walk_right_away(float v,int time);
void Walkahead(float v);
int AdcFlag(void);
void start_mode();
float startRadius();//启动走形模式
float R_buff(int buff_type,int buff_degree,int per_val,float standard_R);
float V_buff(float Buff_type,float Buff_degree,float Per_val,float standard_V);
int Radius(void);
int isSendOK(void);
void SetOKFlagZero(void);
void driveGyro(void);
void errdeal(void);
void PushBall(int T);
float get_body_angle(float a,float b,int n,int round);
void ShootBall(void);
float  Distopow(float distance);
void PushBall2(int T);
void PushBall3(int T);
int Adcangle(void);
int Adcangle2(void);
int Limitxy(void);
float get_roll_v(void);
void get_sendangle(void);
float get_body_angle(float a,float b,int n,int round);
#define Left '1'
#define Right '2'
#endif
