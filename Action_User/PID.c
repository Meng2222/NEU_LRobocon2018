#include "PID.h"


/*
=========================================================================
							直行
					入口参数：速度,单位mm/s
=========================================================================
*/
void WalkStraight(int v)                             
{
	VelCrl(CAN2,1,(4096/378)*v);
	VelCrl(CAN2,2,-((4096/378)*v));
}
/*
=========================================================================
						    画圈
入口参数;               前进方向  ACW  or  CW;
                       轴中间的速度 v 单位mm/s
                        圈的半径  r   单位mm
=========================================================================
*/
void WalkRound(u8 direction, int v,int r)           //画圈，入口参数
{
	int w = v/r;
	if(direction == CW)
	{
		VelCrl(CAN2,1,(4096/378)*(w*(r-217)));
		VelCrl(CAN2,2,-((4096/378)*(w*(r+217))));
	}
	else
	{
		VelCrl(CAN2,1,(4096/378)*(w*(r+217)));
		VelCrl(CAN2,2,-((4096/378)*(w*(r-217))));
	}
}

/*
=========================================================================
                       PID控制——底盘
误差分析：	1.车轮打滑（1m/s刹车有100mm误差）；
			2.定位系统陀螺仪漂移（两圈1度左右）；
			3.车速导致的转弯误差（1m/s转弯有250mm）；
			4.PID角度超调（0.05左右的超调）；
			5.PID坐标函数应用在直线PID上时不好控制误差；
			6.车轮有最大加速度（经不完全计算大概）
			7.以后如果需要提高精度，可以考虑把PID函数控制精度改为0.1mm/s
========================================================================
*/

int angle_set = 0;
extern u8 isOKFlag;
extern u8 issendOK;
float kp = 20;
float ki = 0.01;
float kd = 40;
float lastAngle = 0;
float velocity = 0;
float ITerm = 0;
float velocityMax1 = 3600;
float velocityMax2 = -3600;
float velocityMin1 = 100;
float velocityMin2 = -100;
extern float angle;
u8 coordinateCnt = 0;
void driveGyro(void)                                                 //1车定位系统使能
{
	while(!isOKFlag)
	{
		TIM_Delayms(TIM4,5);
		USART_SendData(USART3,'A');
		USART_SendData(USART3,'T');
		USART_SendData(USART3,'\r');
		USART_SendData(USART3,'\n');
	}
	isOKFlag = 0;
	while(!issendOK);
	issendOK = 0;
}

void SetTunings(float p,float i,float d)                             //kp，ki，kd设置函数
{
	kp = p;
	ki = i;
	kd = d;
}

void Init_PID(float angle)                                           //PID参数初始化（重开pid时需要用到）
{
	lastAngle = angle;
	ITerm = velocity;
    if(ITerm > velocityMax1) ITerm= velocityMax1;
    else if(ITerm < velocityMax2) ITerm= velocityMax2;
}

void PID_Angle(u8 status,float Angle_Set,float Angle,int v)          //PID角度控制，使车头锁定一个固定方向
{
	static u8 lastStatus = 0;
	if(status == manual)
	{
		lastStatus = status;
		return;
	}
	if(lastStatus == manual && status == Auto) Init_PID(Angle);	
	if(Angle_Set>180) Angle_Set = Angle_Set - 360;
	if(Angle_Set<-180) Angle_Set = Angle_Set + 360;	
	if(Angle_Set>=0)
	{
		if(Angle>=Angle_Set-180) Angle = Angle-Angle_Set;
		else Angle = Angle + 360 - Angle_Set;
	}
	if(Angle_Set<0)
	{
		if(Angle<=Angle_Set+180) Angle = Angle-Angle_Set;
		else Angle = Angle - 360 - Angle_Set;
	}
//	if(Angle_Set - lastAngle>10 || Angle_Set - lastAngle<-10)
	float error = 0 - Angle;
	if(error>10 || error<-10) ki = 0.05;
	else ki = 0.2;
	ITerm += ki*error;
	if(ITerm > velocityMax1) ITerm= velocityMax1;
    if(ITerm < velocityMax2) ITerm= velocityMax2;
//	if(ITerm > 0 && ITerm < velocityMin1) ITerm = velocityMin1;
//	if(ITerm < 0 && ITerm > velocityMin2) ITerm = velocityMin2;
	float DTerm = lastAngle - Angle;
	velocity = kp*error + ITerm + kd*DTerm;
	if(velocity > velocityMax1) velocity = velocityMax1;
    if(velocity < velocityMax2) velocity = velocityMax2;
//	if(velocity > 0 && velocity < velocityMin1) velocity = velocityMin1;
//  if(velocity < 0 && velocity > velocityMin2) velocity = velocityMin2;
	lastAngle = Angle;
	lastStatus = status;
	VelCrl(CAN2,1,(((4096/378)*velocity)+(4096/378)*v));
	VelCrl(CAN2,2,(((4096/378)*velocity)-(4096/378)*v));
}

extern union u8andfloat                                              //引用定位系统数据
{   
	uint8_t data[24];
	float ActVal[6];
}posture;



void PID_Coordinate(float x0,float y0,int v)                         //PID坐标控制，引导车子驶向一个固定坐标
{
	if(posture.ActVal[3]>x0 && posture.ActVal[4]>y0) PID_Angle(Auto,(90+((atan((posture.ActVal[4]-y0)/(posture.ActVal[3]-x0)))*(180/3.141592))),posture.ActVal[0],v);
	if(posture.ActVal[3]>x0 && posture.ActVal[4]<y0) PID_Angle(Auto,(90-((atan((y0-posture.ActVal[4])/(posture.ActVal[3]-x0)))*(180/3.141592))),posture.ActVal[0],v);
	if(posture.ActVal[3]<x0 && posture.ActVal[4]>y0) PID_Angle(Auto,(-90-((atan((posture.ActVal[4]-y0)/(x0-posture.ActVal[3])))*(180/3.141592))),posture.ActVal[0],v);
	if(posture.ActVal[3]<x0 && posture.ActVal[4]<y0) PID_Angle(Auto,(-90+((atan((y0-posture.ActVal[4])/(x0-posture.ActVal[3])))*(180/3.141592))),posture.ActVal[0],v);
	if(posture.ActVal[3]==x0 && posture.ActVal[4]<y0) PID_Angle(Auto,0,posture.ActVal[0],v);
	if(posture.ActVal[3]==x0 && posture.ActVal[4]>=y0) PID_Angle(Auto,180,posture.ActVal[0],v);
//	if((x0-50)<posture.ActVal[3] && posture.ActVal[3]<(x0+50) && (y0-50)<posture.ActVal[4] && posture.ActVal[4]<(y0+50)) coordinateCnt++;
}

void PID_Line(float x1,float y1,float x2,float y2,int v)             //PID直线，保证车子在一条直线上前进
{
	float Line_A = y1-y2;
	float Line_B = x2-x1;
	float Line_C = x1*y2-x2*y1;
	float error = (Line_A*posture.ActVal[3]+Line_B*posture.ActVal[4]+Line_C)/sqrt(Line_A*Line_A+Line_B*Line_B);
	if(error>900) error = 900;
	if(error<-900) error = -900;
	if(error>-100 && error<100) SetTunings(20,0,20);
	else SetTunings(30,0,40);
	if(x1>x2 && y1>=y2) PID_Angle(Auto,(90+((atan((y1-y2)/(x1-x2)))*(180/3.141592)))-error/20,posture.ActVal[0],v);
	if(x1>x2 && y1<y2) PID_Angle(Auto,(90-((atan((y2-y1)/(x1-x2)))*(180/3.141592)))-error/20,posture.ActVal[0],v);
	if(x1<x2 && y1>=y2) PID_Angle(Auto,(-90-((atan((y1-y2)/(x2-x1)))*(180/3.141592)))-error/20,posture.ActVal[0],v);
	if(x1<x2 && y1<y2) PID_Angle(Auto,(-90+((atan((y2-y1)/(x2-x1)))*(180/3.141592)))-error/20,posture.ActVal[0],v);
	if(x1==x2 && y2>=y1) 
	{
		if((posture.ActVal[3]-x1)<=450 && (posture.ActVal[3]-x1)>=-450) error = posture.ActVal[3]-x1;
		else if(error>900) error = 900;
		else if(error<-900) error = -900;
		if(error>-100 && error<100) SetTunings(20,0,20);
		else SetTunings(30,0,40);
		PID_Angle(Auto,0+error/20,posture.ActVal[0],v);
	}
	if(x1==x2 && y2<y1) 
	{
		if((posture.ActVal[3]-x1)<=450 && (posture.ActVal[3]-x1)>=-450) error = posture.ActVal[3]-x1;
		else if(error>900) error = 900;
		else if(error<-900) error = -900;
		if(error>-100 && error<100) SetTunings(20,0,20);
		else SetTunings(30,0,40);
		PID_Angle(Auto,180-(posture.ActVal[3]-x1)/20,posture.ActVal[0],v);
	}
}

void PID_Sauare(float v)                                             //PID正方形，两米见方，速度1m的参数
{
	static u8 lineCnt = 0;
	switch(lineCnt)
	{
		case 0:
			PID_Line(0,0,0,2000,v);
			if(posture.ActVal[4]>1340) lineCnt++;
			break;
		case 1:
			PID_Line(0,2000,-2000,2000,v);
			if(posture.ActVal[3]<-1430) lineCnt++;
			break;
		case 2:
			PID_Line(-2000,2000,-2000,0,v);
			if(posture.ActVal[4]<650) lineCnt++;
			break;
		case 3:
			PID_Line(-2000,0,0,0,v);
			if(posture.ActVal[3]>-650) lineCnt++;
			break;
		default:
			lineCnt = 0;
			break;
	}	
}

void PID_Round(float x0,float y0,float r,float v,u8 direction)       //PID圆形，可输入圆心，半径，速度，方向
{
	float error = sqrt((posture.ActVal[3]-x0)*(posture.ActVal[3]-x0)+(posture.ActVal[4]-y0)*(posture.ActVal[4]-y0))-r;
	if(error>900) error = 900;
	if(error<-900) error = -900;
	if(error>-100 && error<100) SetTunings(20,0,20);
	else SetTunings(30,0,40);
	if(direction == ACW)
	{
		if(posture.ActVal[3]>x0 && posture.ActVal[4]>y0) PID_Angle(Auto,(+((atan((posture.ActVal[4]-y0)/(posture.ActVal[3]-x0)))*(180/3.141592))+error/10),posture.ActVal[0],v);
		if(posture.ActVal[3]>x0 && posture.ActVal[4]<y0) PID_Angle(Auto,(-((atan((y0-posture.ActVal[4])/(posture.ActVal[3]-x0)))*(180/3.141592))+error/10),posture.ActVal[0],v);
		if(posture.ActVal[3]<x0 && posture.ActVal[4]>y0) PID_Angle(Auto,(-180-((atan((posture.ActVal[4]-y0)/(x0-posture.ActVal[3])))*(180/3.141592))+error/10),posture.ActVal[0],v);
		if(posture.ActVal[3]<x0 && posture.ActVal[4]<y0) PID_Angle(Auto,(-180+((atan((y0-posture.ActVal[4])/(x0-posture.ActVal[3])))*(180/3.141592))+error/10),posture.ActVal[0],v);
		if(posture.ActVal[3]==x0 && posture.ActVal[4]<y0) PID_Angle(Auto,(-90+error/10),posture.ActVal[0],v);
		if(posture.ActVal[3]==x0 && posture.ActVal[4]>=y0) PID_Angle(Auto,(90+error/10),posture.ActVal[0],v);
	}
	if(direction == CW)
	{
		if(posture.ActVal[3]>x0 && posture.ActVal[4]>y0) PID_Angle(Auto,(180+((atan((posture.ActVal[4]-y0)/(posture.ActVal[3]-x0)))*(180/3.141592))-error/10),posture.ActVal[0],v);
		if(posture.ActVal[3]>x0 && posture.ActVal[4]<y0) PID_Angle(Auto,(180-((atan((y0-posture.ActVal[4])/(posture.ActVal[3]-x0)))*(180/3.141592))-error/10),posture.ActVal[0],v);
		if(posture.ActVal[3]<x0 && posture.ActVal[4]>y0) PID_Angle(Auto,(-((atan((posture.ActVal[4]-y0)/(x0-posture.ActVal[3])))*(180/3.141592))-error/10),posture.ActVal[0],v);
		if(posture.ActVal[3]<x0 && posture.ActVal[4]<y0) PID_Angle(Auto,(+((atan((y0-posture.ActVal[4])/(x0-posture.ActVal[3])))*(180/3.141592))-error/10),posture.ActVal[0],v);
		if(posture.ActVal[3]==x0 && posture.ActVal[4]<y0) PID_Angle(Auto,(90-error/10),posture.ActVal[0],v);
		if(posture.ActVal[3]==x0 && posture.ActVal[4]>=y0) PID_Angle(Auto,(-90-error/10),posture.ActVal[0],v);
	}
}

void PID_Coordinate_following(float v)                               //PID坐标跟随，y=sinx，未完成(开始时的加速段会有较大振荡)
{	
	float x = posture.ActVal[3]+50;
	float y = 500*sin(x*3.141592f/1000);
	PID_Coordinate(x,y,v);
}
/*
//	int x_last = 0;
//	int y_last = 0;
//	int v = 0;
//			USART_OUT(UART4,(uint8_t*)"angle = %d  ", (int)posture.ActVal[0]);
//			USART_OUT(UART4,(uint8_t*)"x = %d  ", (int)posture.ActVal[3]);
//			USART_OUT(UART4,(uint8_t*)"y = %d  ", (int)posture.ActVal[4]);
//			USART_OUT(UART4,(uint8_t*)"v = %d  ", (int)velocity);
//float angle_change(float angle_1)
//{
//	if(angle_set == 0) return angle_1;
//	if(angle_set == 90)
//	{
//		if(angle_1 < -90) return (360+angle_1);
//		else return angle_1;
//	}
//	if(angle_set == 180)
//	{
//		if(angle_1 < 0) return (360+angle_1);
//		else return angle_1;
//	}
//	if(angle_set == 270)
//	{
//		if(angle_1 < 90) return (360+angle_1);
//		else return angle_1;
//	}
//}
//void PID_Angle(u8 status,float Angle_Set,float Angle,int v)
//{
//	static u8 lastStatus = 0;
//	if(status == manual)
//	{
//		lastStatus = status;
//		return;
//	}
//	if(lastStatus == manual && status == Auto) Init_PID(Angle);
//	float error = Angle_Set - Angle;
//	ITerm += ki*error;
//	if(ITerm > velocityMax1) ITerm= velocityMax1;
//    if(ITerm < velocityMax2) ITerm= velocityMax2;
////	if(ITerm > 0 && ITerm < velocityMin1) ITerm = velocityMin1;
////	if(ITerm < 0 && ITerm > velocityMin2) ITerm = velocityMin2;
//	float DTerm = lastAngle - Angle;
//	velocity = kp*error + ITerm + kd*DTerm;
////	velocity = kp*error + kd*DTerm;
//	if(velocity > velocityMax1) velocity = velocityMax1;
//    if(velocity < velocityMax2) velocity = velocityMax2;
////	if(velocity > 0 && velocity < velocityMin1) velocity = velocityMin1;
////  if(velocity < 0 && velocity > velocityMin2) velocity = velocityMin2;
//	lastAngle = Angle;
//	lastStatus = status;
//	VelCrl(CAN2,1,(((4096/378)*(velocity))+(4096/378)*v));
//	VelCrl(CAN2,2,(((4096/378)*(velocity))-(4096/378)*v));
//}

//void PID_Line(float x1,float y1,float x2,float y2,int v)
//{
//	float y = posture.ActVal[4];
//	float x = posture.ActVal[3];
//	static u8 delay_error = 0;
//	static u8 delay_line = 0;
//	static float k = 0;
//	static float y3 = 0;
//	static float x3 = 0;
//	static float y4 = 0;
//	static float error = 0;
//	if(x1==x2)
//	{
//		error = sqrt((x-x1)*(x-x1));
//		if(error < 100) 
//		{
//			delay_error = 0;
//			delay_line++;
//			if(delay_line>0)
//			{
//				PID_Coordinate(x2,y2,v);
//				delay_line = 100;
//			}
//		}
//		else
//		{
//			delay_line = 0;
////			if(delay_error == 0)
////			{
//				y4 = y;
////			}
//			delay_error++;
//			if(delay_error>49)
//			{
//				if((x-x1)>200 || (x-x1)<-200) PID_Coordinate(x2,y4,v);
//				else PID_Coordinate(x2,y4,200);
//				delay_error = 150;
//			}
//			else PID_Coordinate(x2,y2,v);			
//		}
//	}
//	else
//	{
//		k = (y2-y1)/(x2-x1);
//		y3 = k*(x-x1+k*(y-y1))/((k*k+1)+k*k)+y1;
//		x3 = (k*k*x1+x+k*y-k*y1)/((k*k+1)+k*k);
//		error = sqrt((x-x3)*(x-x3)+(y-y3)*(y-y3));
//		if(error<100)
//		{
//			delay_error = 0;
//			delay_line++;
//			if(delay_line>0)
//			{
//				PID_Coordinate(x2,y2,v);
//				delay_line = 100;
//			}
//		}
//		else
//		{
//			delay_line = 0;
////			if(delay_error == 0)
////			{				
//				
////			}
//			delay_error++;
//			if(delay_error>49)
//			{
//				if(error<200) PID_Coordinate(x3,y3,200);
//				else PID_Coordinate(x3,y3,v);
//				delay_error = 150;
//			}
//			else PID_Coordinate(x2,y2,v);
//		}
//	}
//}

//			pid方向
//		if(posture.ActVal[3]-x_last > 1800 || posture.ActVal[3]-x_last < -1800 || posture.ActVal[4]-y_last > 1800 || posture.ActVal[4]-y_last < -1800)
//		{
//			coordinateCnt++;
//			x_last = posture.ActVal[3];
//			y_last = posture.ActVal[4];
//		}
//		
//		switch (coordinateCnt)
//		{
//			case 0:
//				angle_set = 0;
//				v = 0;
//				if(posture.ActVal[0]>-5 && posture.ActVal[0]<5) v=500;
//				PID_Angle(Auto,angle_set,posture.ActVal[0],v);
//				break;
//			case 1:
//				angle_set = 90;
//				v = 0;
//				if(posture.ActVal[0]>85 && posture.ActVal[0]<95) v=500;
//				PID_Angle(Auto,angle_set,posture.ActVal[0],v);
//				break;
//			case 2:
//				angle_set = 180;
//				v = 0;
//				if(posture.ActVal[0]>175 || posture.ActVal[0]<-175) v=500;
//				PID_Angle(Auto,angle_set,posture.ActVal[0],v);
//				break;
//			case 3:
//				angle_set = -90;
//				v = 0;
//				if(posture.ActVal[0]>-95 && posture.ActVal[0]<-85) v=500;
//				PID_Angle(Auto,angle_set,posture.ActVal[0],v);
//				break;
//		}
//		if(coordinateCnt == 4) coordinateCnt=0;

//		
////pid坐标

//		switch (coordinateCnt)
//		{
//			case 0:
//				PID_Coordinate(0,0,500);
//				break;
//			case 1:
//				PID_Coordinate(1500,1500,500);
//				break;
//			case 2:
//				PID_Coordinate(1000,2000,1000);
//				break;
//			case 3:
//				PID_Coordinate(0,1500,1000);
//				break;
//			case 4:
//				PID_Coordinate(-1000,2000,1000);
//				break;
//			case 5:
//				PID_Coordinate(-1500,1500,1000);
//				break;
//		}
//		if(coordinateCnt == 6) coordinateCnt=0;
*/
