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
#include "movebase.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "math.h"
#include "adc.h"
#include "pps.h"
#include "fort.h"

float Input,error,Setpoint;
float errSum=0,lasterror=0;
double Output,Dis_Output;
float Kp,Ki,Kd,Kp2,Ki2,Kd2;
float ratio1,ratio2;
int flag=1,anti_flag=1;
int adcflag=0;

void Distance_PID(float a,float b,float c,int dir);
void driveGyro(void);
void Angle_PID(float a,float b,float c,int dir);
void SetTuning(float kp,float ki,float kd);		//设定PID值
void Dis_Pidtuning(float Dis_kp,float Dis_ki,float Dis_kd);

//设置，转圆圈，距离PID参数，以及角度PID
void Circle_Dis_PID(float x,float y,float r,float vel,float orient);
void Circle_Angle_PID(float x,float y,float r,float vel,float orient);


/*
1mm是4096/(120*Pi)
定义输入速度mm/s和半径mm
*/

//void vel_radious(float vel,float radious)		//差速轮，转圈，没加PID调节
//{												//
//	ratio1=(radious-WHEEL_DIAMETER/2)/radious;
//	ratio2=(radious+WHEEL_DIAMETER/2)/radious;
//	VelCrl(CAN2,1,ratio1*vel*Pulse2mm);			//右轮
//	VelCrl(CAN2,2,-ratio2*vel*Pulse2mm);		//左轮
//}


void walk_circle(float vel)
{
//		ratio1=(r+WHEEL_TREAD/2)/r;
//		ratio2=(r-WHEEL_TREAD/2)/r;
		VelCrl(CAN2,1,vel*Pulse2mm-Output/2);		//右轮
		VelCrl(CAN2,2,-vel*Pulse2mm-Output/2);		//左轮
}


void walk_stragiht(float speed)					//走直线，给定速度
{
	VelCrl(CAN2,1,speed*COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi)+Output/2);						
	VelCrl(CAN2,2,-speed*COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi)+Output/2);	//mm/s
}


//void PID_Set(float a,float b,float c,float dir)
//{
//	Distance_PID(a,b,c,dir);	
//	Angle_PID(a,b,c,dir);	
//}	

void Bias(void)
{
	if(error>=180)
	{error-=360;}
	else if(error<=-180)
	{error+=360;}
}
void Circle_PID_set(float x,float y,float r,float vel,float orient)
{
	
	Circle_Dis_PID(x,y,r,vel,orient);
	Circle_Angle_PID(x,y,r,vel,orient);
}

// 


/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];

void App_Task()
{
	CPU_INT08U os_err;
	os_err = os_err; /*防止警告...*/

	/*创建信号量*/
	PeriodSem = OSSemCreate(0);

	/*创建任务*/
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, /*初始化任务*/
						  (void *)0,
						  (OS_STK *)&App_ConfigStk[Config_TASK_START_STK_SIZE - 1],
						  (INT8U)Config_TASK_START_PRIO);

	os_err = OSTaskCreate((void (*)(void *))WalkTask,
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  (INT8U)Walk_TASK_PRIO);
}

/*
   ===============================================================
   初始化任务
   ===============================================================
   */
void ConfigTask(void)
	
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Adc_Init();
//	TIM_Init(TIM2,1000-1,84-1,1,3);	//产生10ms中断，抢占优先级为1，响应优先级为3
	TIM_Init(TIM2, 99, 839, 1, 0);
	USART3_Init(115200);
	UART4_Init(921600);
	USART1_Init(921600);			//USART1，蓝牙收发数据
	UART5_Init(921600);
	
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);


	/*一直等待定位系统初始化完成*/
	BEEP_ON;
	ElmoInit(CAN1);										//电机使能（通电）
	ElmoInit(CAN2);		//电机使能（通电）
	
	
	VelLoopCfg(CAN1,5,10000000,10000000);				//驱动后轮
	VelLoopCfg(CAN1,6,10000000,10000000);				// 控制新底盘转向电机
	VelLoopCfg(CAN1,8,50000,50000);						//棍子收球电机	
	PosLoopCfg(CAN1, PUSH_BALL_ID, 50000,50000,20000);	//配置位置环，推球电机PUSH_BALL_ID是6
	VelLoopCfg(CAN2,1, 500000, 500000);					//驱动器速度环初始化
	VelLoopCfg(CAN2,2, 500000, 500000);					//驱动器初始化


	MotorOn(CAN1,6);
	MotorOn(CAN1,8);	
	MotorOn(CAN2,1);								
	MotorOn(CAN2,2);

	delay_s(2);
	WaitOpsPrepare();
	OSTaskSuspend(OS_PRIO_SELF);

}
extern FortType fort;
void WalkTask(void)
{

	CPU_INT08U os_err;
	os_err = os_err;
	int state=0,stateflag=1,direction=1;
	float leftlaser=0,rightlaser=0,LastGetx=0,LastGety=0;
	int push_ball_delay=0;

	int cnt=0;
	float dec_value=0;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		push_ball_delay++;
		USART_OUT(USART1,(uint8_t*)"%d\t",(int)GetAngle());
		USART_OUT(USART1,(uint8_t*)"%d\t",(int)GetX());
		USART_OUT(USART1,(uint8_t*)"%d\t",(int)GetY());
		USART_OUT(USART1,(uint8_t*)"cnt:%d\t adcflag:%d\t",(int)cnt,(int)adcflag);
		USART_OUT(USART1,(uint8_t*)"Out:%d\tDis_Out:%d\r\n",(int)Output,(int)Dis_Output);

		SetTuning(380,0,0);									
		Dis_Pidtuning(0.14,0.2*0.01,0);							
		Circle_Dis_PID(0,2400,1500,500,1);					//闭环圆形，0.07，与速度成反比	
		Circle_Angle_PID(0,2400,1500,500,1);				//对应0.07的是380
		walk_circle(500);
		VelCrl(CAN1,COLLECT_BALL_ID,60*4096); 							// 棍子收球
		PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);			// 推球
		ShooterVelCtrl(0);				//发射电机转速控制
		YawPosCtrl(0);					//炮台航向控制
		if(push_ball_delay>100)
		{
			PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);	// 复位
			push_ball_delay=0;
		}
	}
}





void Angle_PID(float a,float b,float c,int dir)				//PID控制角度偏差【方形】
{
	if((dir == 1&& a*b<0)||(dir == -1&& a*b>0))
	{ 
		Input=GetAngle()+90.0f;
		if(Input>=180.0f)
		{
			Input-=360.0f;
		}
		Setpoint=atan(-a/b)*180/Pi;	
		error = Setpoint - Input + Dis_Output;		
	}
	else if((dir == 1&& a*b>0)||(dir == -1&& a*b<0))
	{
		Input=GetAngle()+90.0f-360.0f;
		if(Input<=-180.0f)
		{
			Input+=360.0f;
		}
		Setpoint=atan(-a/b)*180/Pi-180;
		error = Setpoint - Input + Dis_Output;		
	}
	
	if(dir == 1)
	{
		if(b == 0&&a != 0)
		{
			Input=GetAngle()+90.0f;
			if(Input>180)
			{
				Input-=360.0f;
			}
			Setpoint=90;
			error = Setpoint - Input + Dis_Output;			
		}
		else if(b != 0&& a == 0)
		{
			Input=GetAngle()+90.0f;
			if(Input>=180)
			{
				Input-=360.0f;
			}
			Setpoint=0;	
			error = Setpoint - Input - Dis_Output;			
		}
	}
	else if(dir == -1)
	{
		if(b == 0&&a != 0)
		{
			Input=GetAngle()+90;
			if(Input>=180)
			{
				Input-=360.0f;
			}
			Setpoint=-90;
			error = Setpoint - Input + Dis_Output;			
		}
		else if(b != 0&& a == 0)
		{
			Input=GetAngle()+90.0f-360.0f;
			if(Input<=-180.0f)
			{
				Input+=360.0f;
			}
			Setpoint=-180.0f;
			error = Setpoint - Input - Dis_Output;			
		}
	}
	Bias();
	errSum+=error;
	float dErr=error-lasterror;
	Output=Kp*error+Ki*errSum+Kd*dErr;
	lasterror=error;
}

void Distance_PID(float a,float b,float c,int dir)				//PID控制角度偏差
{
	static float Dis_lasterr,Dis_errSum=0,Dis_error=0;
	float dDis_Err;
	if(dir == 1)
	{
		Dis_error = (a*GetX()+b*GetY()+c)/(sqrt(a*a+b*b));
	}
	else if(dir == -1)
	{
		Dis_error = -(a*GetX()+b*GetY()+c)/(sqrt(a*a+b*b));
	}
	Dis_errSum+=Dis_error;
	dDis_Err=Dis_error-Dis_lasterr;
	Dis_Output=Kp2*Dis_error+Ki2*Dis_errSum+Kd2*dDis_Err;
	if(Dis_Output>90)
		Dis_Output=90;
	else if (Dis_Output<-90)
		Dis_Output=-90;
	Dis_lasterr=Dis_error;
}



void Circle_Dis_PID(float x,float y,float r,float vel,float orient)			//转圈距离PID
{
	static float Dis_lasterr,Dis_errSum=0,Dis_error=0;
	float dDis_Err;
	float d;				//d表示点到圆心的距离
	d=sqrt((GetX()-x)*(GetX()-x)+(GetY()-y)*(GetY()-y));
	Dis_error=d-r;
	if(orient == 1)
	{
		Dis_error = Dis_error;
	}
	else if(orient == -1)
	{
		Dis_error = -Dis_error;
	}
	Dis_errSum+=Dis_error;
	dDis_Err=Dis_error-Dis_lasterr;
	if(Ki2*Dis_errSum>15)
		Dis_Output=Kp2*Dis_error+15+Kd2*dDis_Err;
	else if(Ki2*Dis_errSum<-15)
		Dis_Output=Kp2*Dis_error-15+Kd2*dDis_Err;
	else
		Dis_Output=Kp2*Dis_error+Ki2*Dis_errSum+Kd2*dDis_Err;
	Dis_lasterr=Dis_error;
}

void Circle_Angle_PID(float x,float y,float r,float vel,float orient)		//转圈角度PID
{
	Input=GetAngle()+90;		//转化为直角坐标系
	if(Input>=180)
	{
		Input-=360;
	}
	else if(Input<=-180)
	{
		Input+=360;
	}
	
	if(orient == 1)
	{
		Setpoint=atan2(y-GetY(),x-GetX())*180/Pi + 90 ;
	}
	else if(orient == -1)	
	{
		Setpoint=atan2(y-GetY(),x-GetX())*180/Pi - 90 + 360 ;
	}
	
	if(Setpoint>=180)
	{
		Setpoint-=360;
	}
	else if(Setpoint<=-180)
	{
		Setpoint+=360;
	}
	
	error = Input - Setpoint + Dis_Output;
	Bias();
	errSum+=error;
	float dErr=error-lasterror;
	Output=Kp*error+Ki*errSum+Kd*dErr;
	lasterror=error;	
}

void Dis_Pidtuning(float Dis_kp,float Dis_ki,float Dis_kd)
{
	Kp2=Dis_kp;
	Ki2=Dis_ki;
	Kd2=Dis_kd;
}

void SetTuning(float kp,float ki,float kd)
{
	Kp=kp;
	Ki=ki;
	Kd=kd;
}
/*旧小车蓝牙串口UART4，及给定坐标，半径画圆
USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetAngle());
USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetX());
USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetY());
USART_OUT(UART4,(uint8_t*)"cnt:%d\t adcflag:%d\t",(int)cnt,(int)adcflag);
USART_OUT(UART4,(uint8_t*)"Out:%d\tDis_Out:%d\r\n",(int)Output,(int)Dis_Output);
SetTuning(380,0,0);									
Dis_Pidtuning(0.07,0.2*0.01,0);							
Circle_Dis_PID(0,2400,1500,1000,1);					//闭环圆形，0.07，与速度成反比	
Circle_Angle_PID(0,2400,1500,1000,1);				//对应0.07的是380
walk_circle(1000);


*/


/*
平稳走方形，误差小于5%
SetTuning(380,0,0);					//PID参数
Dis_Pidtuning(0.07,0,0);								
switch(state)
{
	case 1:
		PID_Set(1.0f,0.0f,0.0f,1);		//x=0,Y轴正方向，速度1m/s
		walk_stragiht(Whirl_Vel);
		if(GetY()>=1430)
		{
			state++;
		}	break;
	case 2:
		PID_Set(0.0f,1.0f,-2000.0f,1);		//y=+2000,X轴正方向，速度1m/s
		walk_stragiht(Whirl_Vel);
		if(GetX()>=1430)
		{
			state++;
		}	break;
	case 3:
		PID_Set(1.0f,0.0f,-2000.0f,-1);		//x=+2000,Y轴负方向，速度1m/s
		walk_stragiht(Whirl_Vel);
		if(GetY()<=580)
		{
			state++;
		}	break;
	case 4:
		PID_Set(0.0f,1.0f,0.0f,-1);		//y=0,X轴负方向，速度1m/s
		walk_stragiht(Whirl_Vel);
		if(GetX()<=610)
		{
			state=1;
		}	break;
	default:break;
		
}
*/




/*
1.能够走遍省赛场地的大部分区域，角落可以暂时不考虑
2.从出发区出发
3.学会将ADC采集到的数值转换为激光传感器测得的距离，车初始化完成以后不马上出发，等待挡住左侧激光测距或者右侧激光测距后才出发。
4.如果触发左侧激光，则从左侧出发，顺时针绕场；如果触发右侧激光，则从右侧触发，逆时针绕场
5.注意处理走行过程中的特殊情况，比如撞到场地上、撞到对方机器人上、在某个位置卡住等

	SetTuning(380,0,0);									
	Dis_Pidtuning(0.07,0.2*0.01,0);	
	if(adcflag == 0)
	{
		rightlaser= (float)Get_Adc_Average(14,10)*4400.f/4096.f;
		leftlaser=(float)Get_Adc_Average(15,10)*4400.f/4096.f;
		if(leftlaser<=200)
		{
			adcflag=1;
			state=1;
			direction=1;
		}
		else if(rightlaser<=200)	
		{
			adcflag=1;
			state=1;
			direction=-1;			
		}
	}
	
	if(adcflag == 1)
	{
		cnt++;
		if(cnt >= 100&&cnt <=250)
		{
			if(fabs(LastGetx-GetX())<=100||fabs(LastGety-GetY())<=100)
			{
				VelCrl(CAN2,1,-500*COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi));		//倒退					
				VelCrl(CAN2,2,500*COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi));
				state=0;
			}
		}
		if(cnt > 250)
		{
			cnt=0;
			LastGetx=GetX();
			LastGety=GetY();
			state=1;
		}

	}
	
	switch(state)
	{
		case 1 :
			Circle_PID_set(0,2400,2000-dec_value,Whirl_Vel,direction);
			walk_circle(Whirl_Vel);
			if(direction==1)
			{
				if(GetAngle() >=90 &&GetAngle() <=96)
				{
					if(flag == 1)
					{
						state=1;
						dec_value+=400;
						if(dec_value>=2000)
						{
							dec_value=2000;
							flag=0;
						}
					}
					if(flag == 0)
					{
						state=1;
						dec_value-=400;
						if(dec_value<=0)
						{
							dec_value=0;
							flag=1;
						}
					}
				}
			}
			else if(direction == -1)
			{
				if(GetAngle() >=-96 &&GetAngle() <=-90)
				{
					if(flag == 1)
					{
						state=1;
						dec_value+=400;
						if(dec_value>=2000)
						{
							dec_value=2000;
							flag=0;
						}
					}
					else if(flag == 0)
					{
						state=1;
						dec_value-=400;
						if(dec_value<=0)
						{
							dec_value=0;
							flag=1;
						}
					}
				}
			}
			break;
		case 2:
			break;
		default:
			break;
		
	}
*/







/******************************************************************

while(1)内
角度闭环实现方形行走
*******************************************************************/


//		switch(state)
//		{
//			case 1:		
//				Setpoint=0;
//				if(GetY()>=1800&&GetY()<=2000)
//				{
//					state++;
//				}
//				if(GetAngle()>=-1&&GetAngle()<=1)
//				{
//					VelCrl(CAN2,1,Whirl_Vel);
//					VelCrl(CAN2,2,-Whirl_Vel);
//				}
//				else
//				{
//					VelCrl(CAN2,1,Whirl_Vel+Output/2);		
//					VelCrl(CAN2,2,-Whirl_Vel+Output/2);
//				}
//				break;
//			case 2:		
//				Setpoint=-90;
//				if(GetX()>=1800&&GetX()<=2000)
//				{
//					state++;
//				}
//				if(GetAngle()>=-91&&GetAngle()<=-89)
//				{
//					VelCrl(CAN2,1,Whirl_Vel);
//					VelCrl(CAN2,2,-Whirl_Vel);
//				}
//				else
//				{
//					VelCrl(CAN2,1,Whirl_Vel+Output/2);
//					VelCrl(CAN2,2,-Whirl_Vel+Output/2);
//				}
//				break;
//			case 3:		
//				Setpoint=-180;
//				if(GetY()>=0&&GetY()<=200)
//				{
//					state++;
//				}
//				if(GetAngle()>=-180&&GetAngle()<=-178)
//				{
//					VelCrl(CAN2,1,Whirl_Vel);
//					VelCrl(CAN2,2,-Whirl_Vel);
//				}
//				else
//				{
//					VelCrl(CAN2,1,Whirl_Vel+Output/2);
//					VelCrl(CAN2,2,-Whirl_Vel+Output/2);
//				}
//				break;
//			case 4:			
//				Setpoint=90;
//				if(GetX()>=0&&GetX()<=200)
//				{
//					state=1;
//				}
//				if(GetAngle()>=89&&GetAngle()<=91)
//				{
//					VelCrl(CAN2,1,Whirl_Vel);
//					VelCrl(CAN2,2,-Whirl_Vel);
//				}
//				else
//				{
//					VelCrl(CAN2,1,Whirl_Vel+Output/2);
//					VelCrl(CAN2,2,-Whirl_Vel+Output/2);
//				}
//				break;
//			default:break;
//		}

