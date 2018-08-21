#include "math.h"
#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "moveBase.h"
#include "pid.h"
#include "pps.h"
float a,b,r,v;
int dir,speed;

double Input1,Output1;
double errSum1=0,lastErr1=0;
double kp1,ki1,kd1,error1;
int SampleTime1=1000;

double Input2,Output2;
double errSum2=0,lastErr2=0;
double kp2,ki2,kd2,error2;
int SampleTime2=1000;



void Double_closed_loop(float a,float b,float r,float v,int dir)
{


		Compute1(a,b,r,v,dir);
		Compute2(a,b,r,v,dir);
		straightPID(r,v,dir);

}
void straightPID(float r,float speed,int dir)
{
	float ratio1=(r-WHEEL_TREAD/2)/r;
	float ratio2=(r+WHEEL_TREAD/2)/r;
	if(dir<0)
	{	
//		VelCrl(CAN2,1,ratio2*(COUNTS_PER_ROUND*speed)/(WHEEL_DIAMETER*3.1415)-Output1);
//		VelCrl(CAN2,2,-ratio1*(COUNTS_PER_ROUND*speed)/(WHEEL_DIAMETER*3.1415)-Output1);
		VelCrl(CAN2,1,(COUNTS_PER_ROUND*speed)/(WHEEL_DIAMETER*3.1415)-Output1);
		VelCrl(CAN2,2,-(COUNTS_PER_ROUND*speed)/(WHEEL_DIAMETER*3.1415)-Output1);
	}
	if(dir>0)
	{	
//		VelCrl(CAN2,1,ratio1*(COUNTS_PER_ROUND*speed)/(WHEEL_DIAMETER*3.1415)-Output1);
//		VelCrl(CAN2,2,-ratio2*(COUNTS_PER_ROUND*speed)/(WHEEL_DIAMETER*3.1415)-Output1);
		VelCrl(CAN2,1,(COUNTS_PER_ROUND*speed)/(WHEEL_DIAMETER*3.1415)-Output1);
		VelCrl(CAN2,2,-(COUNTS_PER_ROUND*speed)/(WHEEL_DIAMETER*3.1415)-Output1);
	}
	
	
}



float Angle_pointcircle;
float Angle_qie;
float Angle_w;
void Compute1(float a,float b,float r,float v,int dir)//角度PID
{	
	
	Angle_pointcircle=atan2(b-GetY(),a-GetX())*180/3.1415;//点到圆心角度
	
	if(dir>0)
	{
		Angle_qie=Angle_pointcircle+90;//顺时针切线角度
	}
	if(dir<0)
	{
		Angle_qie=Angle_pointcircle-90;//逆时针切线角度
	}
	
	if(Angle_qie>=180.0f)
	{
		Angle_qie-=360.0f;
	}
	
	if(Angle_qie<=-180.0f)
	{
		Angle_qie+=360.0f;
	}
		
	Angle_w=3.6*r*2*3.1415/v;//每10ms角度应偏转值
		
	Input1=GetAngle()+90.0f;//定位系统转换到直角坐标系下的当前角度
	
	if(Input1>=180.0f)
	{
		Input1-=360.0f;
	}
	
	if(Input1<=-180.0f)
	{
		Input1+=360.0f;
	}
	
	error1=Input1-Angle_qie+Angle_w+Output2;
		
	if(error1>=180.0f)
	{
		error1-=360.0f;
	}
	
	if(error1<=-180.0f)
	{
		error1+=360.0f;
	}	
		
	errSum1+=error1;
	double dErr1=error1-lastErr1;
	Output1=kp1*error1+ki1*errSum1+kd1*dErr1;
	lastErr1=error1;
}


void SetTunings1(double Kp1,double Ki1,double Kd1)
{
	double SampleTimeInSec1=((double)SampleTime1)/1000;
	kp1=Kp1;
	ki1=Ki1*SampleTimeInSec1; 
	kd1=Kd1*SampleTimeInSec1;

}

float distance;
void Compute2(float a,float b,float r,float v,int dir)//位置PID
{	
	
	distance= sqrt(pow((GetX()-a),2)+pow((GetY()-b),2));//点到圆心距离
	error2=dir*(distance-r);
	errSum2+=error2;
	double dErr2=error2-lastErr2;
	if(ki2*errSum2>15)
		Output2=kp2*error2+15+kd2*dErr2;
	else if(ki2*errSum2<-15)
		Output2=kp2*error2-15+kd2*dErr2;
	else
		Output2=kp2*error2+ki2*errSum2+kd2*dErr2;
	lastErr2=error2;
}

void SetTunings2(double Kp2,double Ki2,double Kd2)
{
	double SampleTimeInSec2=((double)SampleTime2)/1000;
	kp2=Kp2;
	ki2=Ki2*SampleTimeInSec2;
	kd2=Kd2*SampleTimeInSec2;
}
