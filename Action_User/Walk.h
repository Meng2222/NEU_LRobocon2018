#ifndef __WALK_H
#define __WALK_H

void systerm_change(void);
int AdcFlag(void);//启动ADC，第一次挡设置顺逆时针，第二次挡设置启动半径
void start_mode(void);
int exchange(float v);//
float err(float distance,float ANGLE);//距离偏差转化辅助角度函数
void rrun_to(float BETA,float Vm);//由当前角度pao向BETA
void line(float piontx,float pionty,int zone_opt);
void run_to(float BETA,float Vm);//由当前角度pao向BETA
float a_gen(float a,float b,int n);//输出给定直线的方向角  ax+by+c=0,y=(-a/b)*x+c0;k=-a/b=(action.y-y)/(action.x-x)----a=action.y-y,  ,b=-(action.x-x)
void Walkline( float corex,float corey,float Radium,float V_loop,int clockFlg );//正常走行,输入圆心坐标，半径，速度，方向		
void Walkback(float time);//故障处理
void Walk_left_away(int time);//故障处理
void Walk_right_away(int time);//故障处理
void Walkahead(float v);//故障处理
float R_buff(int buff_type,int buff_degree,int per_val,float standard_R);
float V_buff(float Buff_type,float Buff_degree,float Per_val,float standard_V);
float round_setting(int radiuss);
float startRadius(void);//启动 走形模式
int Radius(void);//常规走形模式
float CountAngle(float a,float b,int n,int round);
void errdeal(void);//故障处理主控制
int round_counter(void);
void errdealtest(void);
int boomAccident(void);
 typedef struct{

	float x;
	float y;
	float angle;
    float x_speed;
	float y_speed;	   
	}pos_t;
#endif
