#ifndef __PUSHBALL_H
#define __PUSHBALL_H

float GetRollV(void);
void GetSendAngle(void);
float GetDis(float x,float y);
float GetDiffer_angle(float angle);
float GetCompensateAng(float differ_angle,float x);
float make_angle_in_wide(float angle,float point_angle);
void MoveGun(float point_x,float point_y);
int abs(int);
void choose_point(void);
void PushBallErrorDeal(void);
void push_ball(void);
#define Need_ball_collor 2
#define No_need_ball_collor 1			//1|2为白，2|1为黑

#endif
