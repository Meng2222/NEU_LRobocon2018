#include "status.h"
#include "elmo.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "timer.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
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


struct position
{
	float x;
	float y;
	float angle;
}pos_t;
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

float LastAngleErr=0,i=0;
int ChangeFlag=0;
extern int T;
float AnglePID(float Angle,float SetAngle)
{
	struct PID Ang;
	Ang.p=180;
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
	i+=err;
	u=Ang.p*err+Ang.i*i+Ang.d*(err-LastAngleErr);
	if(T%10==0)
		USART_OUT(UART4, "err:%d	case:%d\r\n",(int)err,ChangeFlag);
	LastAngleErr=err;
	return u;
}
extern float setangle;

int AngleChange(void)
{
	int flag=0;
	flag=1;
	if(pos_t.y>=2000&&ChangeFlag==0&&setangle==0)
		ChangeFlag=1;
	else if(pos_t.x<=-2000&&ChangeFlag==1&&setangle==90)
		ChangeFlag=2;
	else if(pos_t.y<=0&&ChangeFlag==2&&(setangle==180||setangle==-180))
		ChangeFlag=3;
	else if(pos_t.x>=0&&ChangeFlag==3&&setangle==-90)
		ChangeFlag=0;
	else
		flag=0;
	return flag;
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
		TIM_Delayms(TIM4,5);
		USART_SendData(USART3,'A');
		USART_SendData(USART3,'T');
		USART_SendData(USART3,'\r');
		USART_SendData(USART3,'\n');
	}
	SetOKFlagZero();
}
