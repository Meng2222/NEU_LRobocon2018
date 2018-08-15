#include "status.h"
#include "elmo.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "timer.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "math.h"

struct position
{
	float x;
	float y;
	float angle;
}pos_t;

/**
* @brief  速度转对应脉冲
* @param  v；设定速度
* @author ACTION
* @note 单位 m/s
*/
int exchange(float v)
{
	int pulse=0;
	pulse=(int)(v*4096/0.335352);
	return pulse;
}

/**
* @brief  直走
* @param  v；设定速度
* @author ACTION
* @note 单位 m/s
*/
void straight(float v)
{
	int pulse=0;
	pulse=exchange(v);
	VelCrl(CAN2,0x01,pulse);
	VelCrl(CAN2,0x02,-pulse);
}

/**
* @brief  转圈
* @param  v；设定速度
* @param  r；设定半径
* @param  direction：左右转向（Left左转，Right右转）
* @author ACTION
* @note 单位 v:m/s，r：m 半径为车轴中心到圆心距离，r>=l/2,l=0.434m
*/
void circular(float v,float r,char direction)
{
	float v_big=0,v_small=0,pulse_big=0,pulse_small=0;
	v_big=v+0.434*v/(2*r);
	v_small=v-0.434*v/(2*r);
	pulse_big=exchange(v_big);
	pulse_small=exchange(v_small);
	switch(direction)
	{
		case Left:	VelCrl(CAN2,0x01,pulse_big);
								VelCrl(CAN2,0x02,-pulse_small);
								break;
		case Right: VelCrl(CAN2,0x02,pulse_big);
								VelCrl(CAN2,0x01,-pulse_small);
								break;
	}
}

/**
* @brief  角度环pid
* @param  angle:实际角度
* @param  setangle；设定角度
* @author ACTION
* @note
*/
float LastAngleErr=0,AngI=0;
int ChangeFlag=0;
extern int T;
float AnglePID(float Angle,float SetAngle)
{
	struct PID Ang;
	Ang.p=320;
	Ang.i=0;
	Ang.d=0;
	float err=0,u=0,err1=0,err2=0;
	err1=SetAngle-Angle;
	if(err1>=0)
		err2=err1-360;
	else
		err2=360+err1;
	if(err1>180||err1<-180)
	{
		err=err2;
	}
	else
	{
		err=err1;
	}
	AngI+=err;
	u=Ang.p*err+Ang.i*AngI+Ang.d*(err-LastAngleErr);
	LastAngleErr=err;
	return u;
}
//extern float setangle;

///**
//* @brief  走方形状态切换
//* @author ACTION
//* @note
//*/
//int AngleChange(void)
//{
//	int flag=0;
//	flag=1;
//	if(pos_t.y>=2000&&ChangeFlag==0&&setangle==0)
//		ChangeFlag=1;
//	else if(pos_t.x<=-2000&&ChangeFlag==1&&setangle==90)
//		ChangeFlag=2;
//	else if(pos_t.y<=0&&ChangeFlag==2&&(setangle==180||setangle==-180))
//		ChangeFlag=3;
//	else if(pos_t.x>=0&&ChangeFlag==3&&setangle==-90)
//		ChangeFlag=0;
//	else
//		flag=0;
//	return flag;
//}

/**
* @brief  位置环pid
* @param  distance:实际据直线最短距离
* @author ACTION
* @note 直线右侧为正
*/
float LastDirErr=0,DirI=0;
float DirectionPID( float distance )
{
	struct PID Dir;
	if(distance<300&&distance>-300)
		Dir.p=0.1;
	else if(distance<500&&distance>-500)
		Dir.p=0.06+12;
	else
		Dir.p=0.03+30;
	Dir.i=0;
	Dir.d=0;
	float u=0;
	DirI+=distance;
	u=Dir.p*distance+Dir.i*DirI+Dir.d*(distance-LastDirErr);
	LastDirErr=distance;
	return u;
}
/**
* @brief  走设定直线
* @param  a,b,c:直线方程ax+by+c=0
* @param  direction:直线方向
* @param  v:设定速度
* @author ACTION
* @note direction:1正方向,2负方向			v:单位m/s
*/
	int v1=0,v2=0;
float AngPID=0,DirPID=0;
void line( float a ,float b ,float c , char direction , float v )
{
	char ca='0';
	float SetAngle=0,distance=0,k=0,Ang=0,diff=0;
	float x=0,y=0;

	#if Car==4
	Ang=GetAngle();
	x=GetXpos();
	y=GetYpos();
	#elif Car==1
	Ang=-GetAngle();
	y=-GetXpos();
	x=GetYpos();
	#endif
	if(a!=0&&b!=0)
	{
		k=a/b;
		if(k>0)
		{
			SetAngle=90-atan(k)*180/pi;
		}
		else
		{
			SetAngle=-90-atan(k)*180/pi;
		}
		distance=(a*x+b*y+c)/sqrt(a*a+b*b);
		if((a<0&&b<0)||(b>0&&a<0))
		{
			distance=-distance;
		}
		if(direction==2)
		{
			distance=-distance;
			SetAngle=-SetAngle;
		}
	}
	else if(a==0)
	{
		distance=c/b-y;
		SetAngle=90;
				if(direction==2)
		{
			distance=-distance;
			SetAngle=-SetAngle;
		}
	}
		else if(b==0)
	{
		distance=x-c/a;
		SetAngle=0;
				if(direction==2)
		{
			distance=-distance;
			SetAngle=180;
		}
	}
	DirPID=DirectionPID(distance);//测试
	if(DirPID>70)
		DirPID=70;
	else if(DirPID<-70)
		DirPID=-70;
	SetAngle+=DirPID;
		AngPID=AnglePID(Ang,SetAngle);
		diff=AngPID;
	v1=(int)(exchange(v)+diff);
	v2=(int)(-exchange(v)+diff);
		VelCrl(CAN2,0x01,v1);
		VelCrl(CAN2,0x02,v2);
		//USART_OUT(UART4, "SetAng:%d	Ang:%d	Dir:%d	",(int)(SetAngle),(int)(Ang),(int)(distance));
		//USART_OUT(UART4, "AngPID:%d	DirPID:%d	",(int)(AngPID),(int)(DirPID));
	//	USART_OUT(UART4, "01:%d	02:%d	",v1,v2);
}





//读取函数
void SetAngle(float val)
{
	pos_t.angle=val;
}

void SetXpos(float val)
{
	pos_t.x=val;
}

void SetYpos(float val)
{
	pos_t.y=val;
}

float GetXpos(void)
{
	return pos_t.x;
}

float GetYpos(void)
{
	return pos_t.y;
}

float GetAngle(void)
{
	return pos_t.angle;
}


	





//以下车1专用
int isOKFlag=0;
int isSendOK(void)
{
	return isOKFlag;
}

void SetOKFlagZero(void)
{
	isOKFlag=0;
}

void driveGyro(void)
{
	while(!isOKFlag)
	{
		delay_ms(5);
		USART_SendData(USART3,'A');
		USART_SendData(USART3,'T');
		USART_SendData(USART3,'\r');
		USART_SendData(USART3,'\n');
	}
	SetOKFlagZero();
}
