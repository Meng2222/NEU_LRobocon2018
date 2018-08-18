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


extern int isOKFlag;
extern char pposokflag;
float Input,error,Setpoint;
float errSum=0,lasterror=0;
double Output,Dis_Output;
float Kp,Ki,Kd,Kp2,Ki2,Kd2;
float ratio1,ratio2;

//float Pos_PID(float Pos_Kp,float Pos_Ki,float Pos_Kd,float Set_Pos,float Tar_Pos);

void Distance_PID(float a,float b,float c,int dir);
void driveGyro(void);
void Angle_PID(float a,float b,float c,int dir);
void SetTuning(float kp,float ki,float kd);		//设定PID值
void Dis_Pidtuning(float Dis_kp,float Dis_ki,float Dis_kd);
/*
1mm是4096/(120*Pi)
定义输入速度mm/s和半径mm
*/

//void vel_radious(float vel,float radious)		//转圈
//{
//	ratio1=(radious-Whirl_Vel/2)/radious;
//	ratio2=(radious+Whirl_Vel/2)/radious;
//	VelCrl(CAN2,1,ratio1*vel*Pulse2mm);			//右轮
//	VelCrl(CAN2,2,-ratio2*vel*Pulse2mm);		//左轮
//}



void walk_stragiht(float speed)
{
	VelCrl(CAN2,1,speed*COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi)+Output/2);						
	VelCrl(CAN2,2,-speed*COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi)+Output/2);	//mm/s
}

void Bias(void)
{
	if(error>=180)
	{error-=360;}
	else if(error<=-180)
	{error+=360;}
}

void PID_Set(float a,float b,float c,int dir)
{
	Distance_PID(a,b,c,dir);
	Angle_PID(a,b,c,dir);	
}


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

	TIM_Init(TIM2,1000-1,84-1,1,3);	//产生10ms中断，抢占优先级为1，响应优先级为3

	USART3_Init(115200);
	UART4_Init(921600);
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	
	ElmoInit(CAN2);							//电机使能（通电）
	VelLoopCfg(CAN2,1, 500000, 500000);				//驱动器速度环初始化
	VelLoopCfg(CAN2,2, 500000, 500000);				//驱动器初始化
	MotorOn(CAN2,1);								
	MotorOn(CAN2,2);


//	PosLoopCfg(CAN2, 1, 5000, 5000,RightWhirlVel);		//驱动器位置环初始化,右轮
//	PosLoopCfg(CAN2, 2, 5000, 5000,LeftWhirlVel);		//左轮

	delay_s(2);
	#if CARNUM == 1
	driveGyro();
	while(!pposokflag);
	#elif CARNUM == 4
	delay_s(10);	
	#endif					//等待10s挂起
	OSTaskSuspend(OS_PRIO_SELF);

}

void WalkTask(void)
{

	CPU_INT08U os_err;
	os_err = os_err;
	int state=1;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetAngle());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetXpos());
		USART_OUT(UART4,(uint8_t*)"%d\t",(int)GetYpos());
		USART_OUT(UART4,(uint8_t*)"Input:%dOut:%d\tDis_Out:%d\r\n",(int)Input,(int)Output,(int)Dis_Output);
		SetTuning(380,0,0);					//PID参数
		Dis_Pidtuning(0.07,0,0);			
//		PID_Set(1.0f,0.0f,-1000.0f,-1);		//x=+1000,Y轴负方向，速度1m/s
//		walk_stragiht(1000);						
		switch(state)
		{
			case 1:
				PID_Set(1.0f,0.0f,0.0f,1);		//x=0,Y轴正方向，速度1m/s
				walk_stragiht(Whirl_Vel);
				if(GetYpos()>=1430)
				{
					state++;
				}	break;
			case 2:
				PID_Set(0.0f,1.0f,-2000.0f,1);		//y=+2000,X轴正方向，速度1m/s
				walk_stragiht(Whirl_Vel);
				if(GetXpos()>=1430)
				{
					state++;
				}	break;
			case 3:
				PID_Set(1.0f,0.0f,-2000.0f,-1);		//x=+2000,Y轴负方向，速度1m/s
				walk_stragiht(Whirl_Vel);
				if(GetYpos()<=580)
				{
					state++;
				}	break;
			case 4:
				PID_Set(0.0f,1.0f,0.0f,-1);		//y=0,X轴负方向，速度1m/s
				walk_stragiht(Whirl_Vel);
				if(GetXpos()<=610)
				{
					state=1;
				}	break;
			default:break;
				
		}

	}
}


void Angle_PID(float a,float b,float c,int dir)				//PID控制角度偏差
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
		Dis_error = (a*GetXpos()+b*GetYpos()+c)/(sqrt(a*a+b*b));
	}
	else if(dir == -1)
	{
		Dis_error = -(a*GetXpos()+b*GetYpos()+c)/(sqrt(a*a+b*b));
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
	isOKFlag = 0;
}

//float Pos_PID(float Pos_Kp,float Pos_Ki,float Pos_Kd,float Set_Pos,float Tar_Pos)
//{
//	 static float Bias,Pos_Output,Pos_errSum=0,Last_Bias=0;
//	 if(Tar_Pos-Set_Pos<200) 
//	 Last_Bias=0;
//	 Bias=Set_Pos-Tar_Pos;                                 
//	 Pos_errSum+=Bias;	 
//	 float dPos_err=Bias-Last_Bias;	
//	 Pos_Output=Pos_Kp*Bias+Pos_Ki*Pos_errSum+Pos_Kd*dPos_err;       
//	 Last_Bias=Bias;                                      
//	 return Pos_Output;   	
//}


/******************************************************************

while(1)内
角度闭环实现方形行走
*******************************************************************/


//		switch(state)
//		{
//			case 1:		
//				Setpoint=0;
//				if(GetYpos()>=1800&&GetYpos()<=2000)
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
//				if(GetXpos()>=1800&&GetXpos()<=2000)
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
//				if(GetYpos()>=0&&GetYpos()<=200)
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
//				if(GetXpos()>=0&&GetXpos()<=200)
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

