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
#include "moveBase.h"

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
* @param  direction：顺逆转向（Anti_clockwise逆,Clockwise顺）
* @author ACTION
* @note 单位 v:m/s，r：m 半径为车轴中心到圆心距离，r>=l/2,l=0.434m
*/
void circular(float v,float r,int direction)
{
	float v_big=0,v_small=0,pulse_big=0,pulse_small=0;
	v_big=v+0.434*v/(2*r);
	v_small=v-0.434*v/(2*r);
	pulse_big=exchange(v_big);
	pulse_small=exchange(v_small);
	switch(direction)
	{
		case Anti_clockwise:	VelCrl(CAN2,0x01,pulse_big);
								VelCrl(CAN2,0x02,-pulse_small);
								break;
		case Clockwise: VelCrl(CAN2,0x02,pulse_big);
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
extern int T;
float AnglePID(float Angle,float SetAngle)
{
	struct PID Ang;
	Ang.p=160;
	Ang.i=0;
	Ang.d=10;
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

int ChangeFlag=0;
/**
* @brief  走方形状态切换
* @author ACTION
* @note
*/
int AngleChange(void)
{
	int x=0,y=0;
	x=GetXpos();
	y=GetYpos();
	if(y>=1300&&ChangeFlag==0)
		ChangeFlag=1;
	else if(x>=1300&&ChangeFlag==1)
		ChangeFlag=2;
	else if(y<=700&&ChangeFlag==2)
		ChangeFlag=3;
	else if(x<=700&&ChangeFlag==3)
		ChangeFlag=0;
//	USART_OUT(UART4, "%d	",ChangeFlag);
	return ChangeFlag;
}

/**
* @brief  位置环pid
* @param  distance:实际据直线最短距离
* @author ACTION
* @note 直线右侧为正
*/
float LastDirErr=0,DirI=0;
float DirectionPID( float distance )
{
	float u=0;
	DirI+=distance;
	if(DirI>10000)
		DirI=10000;
	struct PID Dir;
	Dir.i=0;
	Dir.d=0.001;
	if(distance<100&&distance>-100)
	{
		Dir.p=0.12;
		u=Dir.p*distance+Dir.i*DirI+Dir.d*(distance-LastDirErr);
	}
	else if(distance<400&&distance>-400)
	{
		Dir.p=0.06;
		u=Dir.p*distance+Dir.i*DirI+Dir.d*(distance-LastDirErr)+6;
	}
	else
	{
		Dir.p=0.03;
		u=Dir.p*distance+Dir.i*DirI+Dir.d*(distance-LastDirErr)+18;
	}
	LastDirErr=distance;
	return u;
}
/**
* @brief  走设定直线
* @param  a,b,c:直线方程ax+by+c=0
* @param  direction:直线方向
* @param  v:设定速度
* @author ACTION
* @note direction:1正方向,-1负方向			v:单位m/s
*/
	int v1=0,v2=0;
float AngPID=0,DirPID=0;
void line( float a ,float b ,float c , int direction , float v )
{
	float SetAngle=0,distance=0,k=0,Ang=0,diff=0;
	float x=0,y=0;
	Ang=GetAngle();
	x=GetXpos();
	y=GetYpos();

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
		if(direction==-1)
		{
			distance=-distance;
			SetAngle=-SetAngle;
		}
	}
	else if(a==0)
	{
		distance=-c/b-y;
		SetAngle=-90;
				if(direction==-1)
		{
			distance=-distance;
			SetAngle=-SetAngle;
		}
	}
		else if(b==0)
	{
		distance=x+c/a;
		SetAngle=0;
				if(direction==-1)
		{
			distance=-distance;
			SetAngle=180;
		}
	}
	DirPID=DirectionPID(distance);
	if(DirPID>70)
		DirPID=70;
	else if(DirPID<-70)
		DirPID=-70;
//	USART_OUT(UART4, "%d	",(int)(SetAngle));
	SetAngle+=DirPID;
	AngPID=AnglePID(Ang,SetAngle);
	diff=AngPID;
	v1=(int)(exchange(v)+diff);
	v2=(int)(-exchange(v)+diff);
	VelCrl(CAN2,0x01,v1);
	VelCrl(CAN2,0x02,v2);
//	USART_OUT(UART4, "x,y,SA,Aerr,Derr,Apid,Dpid:		");
//	USART_OUT(UART4, "%d	%d	",(int)(x),(int)(y));
//	USART_OUT(UART4, "%d	%d	%d	",(int)(SetAngle),(int)(SetAngle-Ang),(int)(distance));
//	USART_OUT(UART4, "%d	%d\r\n",(int)(AngPID),(int)(DirPID));
	//	USART_OUT(UART4, "01:%d	02:%d	",v1,v2);
}


/**
* @brief  求画设定圆时当前坐标设定角(入圆后）
* @param  x,y 设定圆的圆心
* @param  DIRECTION: 顺逆时针
* @author ACTION
* @note direction:1顺,-1逆		
*/
float GetRoundSetAngle(float X,float Y,int DIRECTION)
{
	float x=0,y=0,SurroundSetAngle=0;
	x=GetXpos();
	y=GetYpos();
		if(Y-y==0)
	{
		if(X>=x)
		{
			if(DIRECTION==1)
				SurroundSetAngle=0;
			else
				SurroundSetAngle=180;
		}
		else
		{
			if(DIRECTION==-1)
				SurroundSetAngle=0;
			else
				SurroundSetAngle=180;
		}
	}
	else if(X-x==0)
	{
		if(Y>=y)
			SurroundSetAngle=90*DIRECTION;
		else
			SurroundSetAngle=-90*DIRECTION;
	}
	else if(X>x&&Y>y)
	{
		if(DIRECTION==1)
			SurroundSetAngle=atan((Y-y)/(X-x))*180/pi;
		else
			SurroundSetAngle=atan((Y-y)/(X-x))*180/pi-180;
	}
	else if(X<x&&Y>y)
	{
		if(DIRECTION==1)
			SurroundSetAngle=90+atan((x-X)/(Y-y))*180/pi;
		else
			SurroundSetAngle=-90+atan((x-X)/(Y-y))*180/pi;
	}
	else if(X<x&&Y<y)
	{
		if(DIRECTION==1)
			SurroundSetAngle=-90-atan((x-X)/(y-Y))*180/pi;
		else
			SurroundSetAngle=-90-atan((x-X)/(y-Y))*180/pi+180;
	}
	else if(X>x&&Y<y)
	{
		if(DIRECTION==1)
			SurroundSetAngle=-atan((y-Y)/(X-x))*180/pi;
		else
			SurroundSetAngle=-atan((y-Y)/(X-x))*180/pi+180;
	}
	return SurroundSetAngle;
}

float GetSpotAngle(float X,float Y)
{
	float SpotAngle=0,x=0,y=0;
	x=GetXpos();
	y=GetYpos();
	if(X>x)
		SpotAngle=atan((Y-y)/(X-x))*180/pi;
	else
		SpotAngle=atan((Y-y)/(X-x))*180/pi+180;
	return SpotAngle;
}
/**
* @brief  画设定圆
* @param  X,Y 设定圆坐标
* @param  R: 设定半径
* @param  V:设定速度
* @param  DIRECTION :顺逆时针
* @author ACTION
* @note direction:1顺,-1逆			V:单位m/s
*/
#define InternalDeviation 5
void surround(float X,float Y,float R,float V,int DIRECTION)
{
	float Ang=0,x=0,y=0,distance=0,SurroundAngle=0,SurroundK=0,d=0;
	float Diff=0,SurroundSetAngle=0,DirPID=0;
	Ang=GetAngle();
	x=GetXpos();
	y=GetYpos();
	R=0.9*R;
	if(X-x<=0.01&X-x>=0)
		x-=0.001;
	if(X-x>=-0.01&X-x<=0)
		x+=0.001;
	d=sqrt(pow(X-x,2)+pow(Y-y,2));
	distance=d-R;
	if(distance<=200&&distance>=-200)
	{
		DirPID=DirectionPID(distance);
		if(DirPID>70)
			DirPID=70;
		else if(DirPID<-70)
			DirPID=-70;
		if(DIRECTION==1)
			SurroundSetAngle=GetRoundSetAngle(X,Y,DIRECTION)-DirPID-InternalDeviation;
		else
			SurroundSetAngle=GetRoundSetAngle(X,Y,DIRECTION)+DirPID+InternalDeviation;
	
		Diff=AnglePID(Ang,SurroundSetAngle);//+WHEEL_TREAD*V/(2*R)
		if(DIRECTION==1)
		{
			VelCrl(CAN2,0x01,exchange(V)+Diff);
			VelCrl(CAN2,0x02,-exchange(V)+Diff);
		}
		else
		{
			VelCrl(CAN2,0x01,exchange(V)+Diff);
			VelCrl(CAN2,0x02,-exchange(V)+Diff);
		}
	}
	else if(distance>200)
	{
			SurroundAngle=GetSpotAngle(X,Y)+DIRECTION*(asin(R/d)*180/pi-InternalDeviation);
			SurroundK=tan(SurroundAngle*pi/180);
			if(Y-y>=0)
				line(-SurroundK,1,SurroundK*x-y,1,V);
			else
				line(-SurroundK,1,SurroundK*x-y,-1,V);

	}
	else
	{
		if(y+MOVEBASE_LENGTH>=Y)
			
			line(1,0,-x,DIRECTION,V);
		else if(x>=X)
			line(0,1,-y-MOVEBASE_LENGTH,1,V);
		else
			line(0,1,-y-MOVEBASE_LENGTH,-1,V);
	}
	USART_OUT(UART4, "%d	%d	%d	",(int)(x),(int)(y),(int)(GetSpotAngle(X,Y)));
	USART_OUT(UART4, "%d	%d	%d	%d\r\n",(int)(SurroundSetAngle),(int)(SurroundK),(int)(Ang),(int)(distance));
}




//冒泡排序
void BubbleSort(float *a,int number)
{
	int i=0,j=0,t=0;
	for(i=0;i<number-1;i++)
	{
		for(j=0;j<number-i-1;j++)
    {
      if(*(a+j+1)>*(a+j))
      {
        t=*(a+j+1);
        *(a+j+1)=*(a+j);
        *(a+j)=t;
      }
     }
	}
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
	#if Car==4
	return pos_t.x;
	#elif Car==1
	return pos_t.y;
	#endif
	
}

float GetYpos(void)
{
	#if Car==4
	return pos_t.y;
	#elif Car==1
	return -pos_t.x;
	#endif
	
}

float GetAngle(void)
{
	#if Car==4
	return pos_t.angle;
	#elif Car==1
	return -pos_t.angle;
	#endif
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
