#include "void.h"
#include "elmo.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "timer.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "math.h"
#include "movebase.h"
#include "pps.h"
#include "fort.h"
#define mode 3
#define Angchange 1
#define recolong 800
#define Pulse2mm COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi)
struct position
{
	float x;
	float y;
	float angle;
}pos_t;
/////////////////////////脉冲转速度////////////////////////////////
int exchange(float v)
{
	int pulse=0;
	pulse=(int)(v*4096/0.335352);
	return pulse;
}
//////////////////AngPID//////////////////////
float AnglePID(float Angle,float SetAngle)
{
	struct PID Ang;
	Ang.p=100;   				//////////300
	Ang.i=0;
	Ang.d=0;
	float err=0,u=0,err1=0,err2=0;
	err1=SetAngle-Angle;
	//////////////////劣弧调节//////////////////
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
	u=Ang.p*err;
	return u;
}

//////////////////////////方向PID///////////////////////////
float DirectionPID( float distance,float setdistance )
{
	struct PID Dir;
	float u=0,errd=0;
	errd=setdistance-distance;
	if(distance<300&&distance>-300)
	{
		Dir.p=0.06;
		u=Dir.p*errd;
	}
	else if(distance<500&&distance>-500)
	{
		Dir.p=0.06;
		u=Dir.p*errd;
	}
	else
	{
		Dir.p=0.06;
		u=Dir.p*errd;
	}
	return u;
}
//////////////绕圈///////////////
void Walkline( int setx,int sety, int r,int direction , float v )
{
	int vright=0,vleft=0;
	float AngPID=0,DirPID=0;
	float SetAngle=0,distance=0,k=0,Ang=0,diff=0,anglek=0;
	float x=0,y=0;
	Ang=GetAngle();
	x=GetX();
	y=GetY();
////////读取角度
	distance=sqrt(((x-setx)*(x-setx)+(y-sety)*(y-sety)));   ///////distance取值
	if((sety-y)!=0&&(setx-x)!=0)
	{
		k=(sety-y)/(setx-x);
		anglek=atan(k)*180/pi;
		if(direction==1)
		{
			if(setx>x&&sety>y)
			{
				SetAngle=anglek-180;
			}
			else if(setx>x&&sety<y)
			{
				SetAngle=180+anglek;
			}
			else if(setx<x&&sety<y)
			{
				SetAngle=anglek;
			}
			else if(setx<x&&sety>y)
			{
				SetAngle=anglek;
			}
		}
		if(direction==2)
		{
			if(setx>x&&sety>y)
			{
				SetAngle=anglek;
			}
			else if(setx>x&&sety<y)
			{
				SetAngle=anglek;
			}
			else if(setx<x&&sety<y)
			{
				SetAngle=anglek-180;
			}
			else if(setx<x&&sety>y)
			{
				SetAngle=anglek+180;
			}		
		}
	}
	else if(sety==y)
	{
		if(direction==1)
		{
			if(setx<x)
			{
				SetAngle=-180;
			}
			else if(setx>x)
			{
				SetAngle=0;
			}
		}
		if(direction==2)
		{
			if(setx<x)
			{
				SetAngle=0;
			}
			else if(setx>x)
			{
				SetAngle=-180;
			}
		}
	}
		else if(setx==x)
	{
		if(distance==1)
		{
			if(sety<y)
			{
				SetAngle=-90;
			}
			if(sety>y)
			{
				SetAngle=90;
			}
		}
		if(direction==2)
		{
			if(sety<y)
			{
				SetAngle=90;
			}
			if(sety>y)
			{
				SetAngle=-90;
			}			
		}
	}
	DirPID=DirectionPID(distance,r);
	if(DirPID>70)
		DirPID=70;
	else if(DirPID<-70)
		DirPID=-70;
///////////////////////角度限制///////////////////////
	if(direction==1)
	{
		SetAngle-=DirPID;
	}
	else if(direction==2)
	{
		SetAngle+=DirPID;
	}
	AngPID=AnglePID(Ang,SetAngle);
	diff=AngPID;
//USART_OUT(UART4,(uint8_t*) "diff:%d	distance:%d	%d	%d\r\n",(int)(diff),(int)(distance),(int)(GetXpos()),(int)(GetYpos()));   ///////////////////////////////////test//////////////////////////
	vright=(int)(exchange(v)+diff);
	vleft=(int)(-exchange(v)+diff);
	if(direction==0)
	{
		vright=0;
		vleft=0;
	}
		VelCrl(CAN2,0x01,vright);
		VelCrl(CAN2,0x02,vleft);
}

void Walkback(float v)
{
	VelCrl(CAN2,0x01,-exchange(v));
	VelCrl(CAN2,0x02,exchange(v));
}
void Walkaway(float v)
{
	VelCrl(CAN2,0x01,exchange(v+0.4));
	VelCrl(CAN2,0x02,-exchange(v-0.3));	
}
void Walkahead(float v)
{
	VelCrl(CAN2,0x01,exchange(v));
	VelCrl(CAN2,0x02,-exchange(v));
}
int AdcFlag(void)
{
		static int adc_num1,adc_num2,AdcFLAG=0;
		adc_num1=Get_Adc_Average(15,30);	///左轮
		adc_num2=Get_Adc_Average(14,30);  ///右轮
		if(adc_num1>20&&adc_num1<600&&AdcFLAG==0)
		{
			AdcFLAG=2;  ////顺时针
		}
		else if(adc_num2>20&&adc_num2<600&&AdcFLAG==0)
		{
			AdcFLAG=1;  ////逆时针
		}
	//	USART_OUT(UART4,(uint8_t*)"%d	%d	%d\n",adc_num1,adc_num2,AdcFLAG);	 /////////ADC  test///////////
		return AdcFLAG;
}
int Radius(void)
{
		static int LastX=0;
		static int r=1400;   ////初始半径
		static int Rflag=0;
		if(r>=1500)
		{
			Rflag=0;
		}
		else if (r<=200)
		{
			Rflag=1;
		}
		if((int)GetX()>0&&LastX<0)
		{
			if(Rflag==0)
			{
				r-=400;
			}
			else if(Rflag==1)
			{
				r+=400;
			}
		}
		LastX=(int)GetX();
		return(r);
}
void errdeal(void)
{
		static int Lastx=0,Lasty=0,errtime=0;
		if((Lastx==(int)GetX())&&(Lasty==(int)GetY())&&AdcFlag()!=0)
		{
			errtime++;
		}
		else if(Lastx!=(int)GetX||Lasty!=(int)GetY)
		{
			errtime=0;
		}
			if(errtime>10)   
			{
				for(int i=0;i<3000;i++)
				{
					Walkback(1.0);
					i++;
				}
				for(int i=0;i<2000;i++)
				{
					Walkaway(1.0);
					i++;
				}
//				for(int i=0;i<2000;i++)
//				{
//					Walkahead(0.7);
//					i++;
//				}				
				errtime=0;
			}			
		Lastx=(int)GetX();
		Lasty=(int)GetY();
//			USART_OUT(UART4,(uint8_t*)"%d %d %d\n",Lastx,Lasty,errtime);          /////////errtime  test////////////
}

float  Distopow(float distance)
{
	float power;
//  power=0.02*distance+41;
//	power=0.0153*distance+34.046;
		power=(40.0/3102.505)*distance+35.7;
	return power;
}

void PushBall(int T)
{
	static int t=0;
	t++;
	if(t==T)
	{
		// 推球
		YawPosCtrl(Adcangle2()+20);    /////航向电机
		PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);		
	}
	if(t==2*T)
	{
		// 复位
		PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);		
		t=0;
	}
//	USART_OUT(UART4,(uint8_t*) "%d\r\n",t);	
}

void PushBall2(int T)
{
	static int t=0;
	t++;
	if(t==T)
	{
		// 推球
		PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);		
	}
	if(t==2*T)
	{
		// 复位
		PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);		
		t=0;
	}
	//	USART_OUT(UART4,(uint8_t*) "%d\r\n",t);	
}
void PushBall3(int T)
{
	static int t=0;
	t++;
	if(t==1)
	{
		// 推球
		PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);		
	}
	if(t==T)
	{
		// 复位
		PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);		
	}
	if(t==2*T)
	{
		t=0;
	}
	//	USART_OUT(UART4,(uint8_t*) "%d\r\n",t);	
}
void ShootBall(void)
{
		float power,distance;
//	distance=ReadLaserAValue();
		distance=ReadLaserBValue()*2.4973-160;        ////////
		power=Distopow(distance);
		if(power>=95)
		{
			power=95;
		}
		if(ReadLaserBValue()>800)											///////
		{
			power=Distopow(800*2.4973-160);
		}
		ShooterVelCtrl(power);     /////发射枪转速
	#if mode==1
		YawPosCtrl(Adcangle());    /////航向电机
		if((int)ReadLaserAValue()<=recolong&&(int)ReadLaserBValue()>=recolong)
		{
			PushBall3(50);
		}
	#elif mode==2
		YawPosCtrl(Adcangle2());    /////航向电机
		if((int)ReadLaserAValue()<=recolong&&(int)ReadLaserBValue()<=recolong)
		{
			YawPosCtrl(Adcangle2()+20);    /////航向电机补充偏移角
			PushBall3(50);
		}
	#elif mode==3
		static int angle=0;
		YawPosCtrl(210+angle);    /////航向电机
		if(((int)ReadLaserBValue()<=recolong||(int)ReadLaserAValue()<=recolong))     /////
		{
			angle++;
			PushBall3(50);
		}
		else if((int)ReadLaserBValue()>recolong&&(int)ReadLaserAValue()>recolong)  /////
		{
			YawPosCtrl(210);    /////航向电机
			angle=0;  
		}
	#endif
		USART_OUT(UART4,(uint8_t*) "%d	%d	%d\r\n",(int)ReadLaserAValue(),(int)ReadLaserBValue(),power);
}
int Adcangle(void)
{
	static int angle=220,direction=1;
	if((int)ReadLaserBValue()<=recolong&&(int)ReadLaserAValue()>=recolong)      /////
	{
		//PushBall2(100);
	}
	if((int)ReadLaserBValue()>=recolong&&(int)ReadLaserAValue()>=recolong)      /////
	{
		if(angle>=340)
		{
			direction=1;
		}
		if(angle<=160)
		{
			direction=2;
		}	
		if(direction==1)
		{
		angle-=Angchange;
		}
		if(direction==2)
		{
		angle+=Angchange;
		}
	}
	if((int)ReadLaserAValue()<=recolong&&(int)ReadLaserBValue()<=recolong)
	{
		angle+=Angchange;
	}
	if((int)ReadLaserAValue()>=recolong&&(int)ReadLaserBValue()<=recolong)
	{
		angle+=Angchange;
	}
	return(angle);
}

int Adcangle2(void)
{
	static int angle=220,direction=1;
	if((int)ReadLaserAValue()<=recolong&&(int)ReadLaserBValue()<=recolong)
	{
		angle+=Angchange;
	}
	if((int)ReadLaserAValue()>=recolong&&(int)ReadLaserBValue()>=recolong)
	{
		if(angle>=340)
		{
			direction=1;
		}
		if(angle<=160)
		{
			direction=2;
		}	
		if(direction==1)
		{
		angle-=Angchange;
		}
		if(direction==2)
		{
		angle+=Angchange;
		}		
	}
//	if((int)ReadLaserAValue()<=recolong&&(int)ReadLaserBValue()>=recolong)
//	{
//		angle-=Angchange;
//	}
	if((int)ReadLaserAValue()>=recolong&&(int)ReadLaserBValue()<=recolong)
	{
		angle+=Angchange;
	}	
	return angle;
}

//int ShootFlag(void)
//{
//	int Shootflag;
//	if(GetX()>1500&&GetY()<600)
//	{
//		Shootflag=1;
//	}
//	if(GetX()>1500&&GetY()<4800&&GetY()>3800)
//	{
//		Shootflag=1;
//	}
//	if(GetX()<)
//}	
