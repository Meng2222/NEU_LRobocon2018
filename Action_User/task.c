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
#include "math.h"
#include "pps.h"

#define PI   3.1415926
// 宏定义棍子收球电机ID
#define COLLECT_BALL_ID (8)
// 宏定义推球电机ID
#define PUSH_BALL_ID (6)
// 宏定义送弹机构送弹时电机应该到达位置：单位位脉冲
#define PUSH_POSITION (4500)
// 宏定义送弹机构收回时电机位置
#define PUSH_RESET_POSITION (5)
//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
// 宏定义发射机构航向电机ID
#define GUN_YAW_ID (7)
// 电机旋转一周的脉冲数
#define COUNT_PER_ROUND (4096.0f)
// 宏定义每度对应脉冲数
#define COUNT_PER_DEGREE  (COUNT_PER_ROUND/360.0f)
// 宏定义航向角减速比
#define YAW_REDUCTION_RATIO (4.0f)



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

void  Walk_Straight(float speed1,float speed2)
{
	VelCrl(CAN2,1,speed1*10.87f);
	VelCrl(CAN2,2,-speed2*10.87f);
}

void  Walk_Virer(float right_v,float left_v)
{
	VelCrl(CAN2,1,right_v);
	VelCrl(CAN2,2,-left_v);
}

float Angle_Pid(float err)
{
	static float Iterm=0;
	float Dterm=0,errlast=0,Uk=0;
	if(err>=180)
		err-=360;
	else if(err<=-180)
		err+=360;
	Iterm +=Ki*err;
	Dterm=Kd*(err-errlast);
	errlast=err;
	Uk=Kp*err+Iterm+Dterm;
//	if(Uk>=6000)
//		Uk=6000;
//	else if(Uk<=-6000)
//		Uk=-6000;

//	USART_OUT(UART4,(uint8_t*) "%d\t",(int)chenhao);
//		USART_OUT(UART4,(uint8_t*) "%d\r\n",(int)GetAngle());
	return Uk;
}
float Distance_Pid(float err)
{
	static float Iterm=0;
	float Dterm=0,errlast=0,Uk=0;
	Iterm +=Distance_Ki*err;
	if(Iterm>=10)
		Iterm = 10;
	else if(Iterm<=-10)
		Iterm = -10;
	Dterm=Distance_Kd*(err-errlast);
	errlast=err;
	Uk=Distance_Kp*err+Iterm+Dterm;
	return Uk;
}

float PID_OUT(float x,float y,float a,float b,float c,int quadrant)
{
	static float d=0,output=0,ang=0,err=0;
	d=fabs(a*GetX()+b*GetY()+c)/sqrtf(a*a+b*b);
	err=-Distance_Pid(0-d);
	if(err>=90)
		err=90;
	switch(quadrant)
	{
		case 1:
			ang=atan(-a/b)*180/PI-90;
			if((-(b*GetY()+c)/a)>GetX())
			{
				output=Angle_Pid(err-(ang-GetAngle()));
			}
			else if((-(b*GetY()+c)/a)<GetX())
			{
				output=Angle_Pid(-err-(ang-GetAngle()));
			}
			break;
		case 2:
			ang=atan(-a/b)*180/PI+90;
			if((-(b*GetY()+c)/a)>GetX())
			{
				output=Angle_Pid(err-(ang-GetAngle()));
			}
			else if((-(b*GetY()+c)/a)<GetX())
			{
				output=Angle_Pid(-err-(ang-GetAngle()));
			}
			break;
		case 3:
			ang=atan(-a/b)*180/PI+90;
			if((-(b*GetY()+c)/a)>GetX())
			{
				output=Angle_Pid(-err-(ang-GetAngle()));
			}
			else if((-(b*GetY()+c)/a)<GetX())
			{
				output=Angle_Pid(err-(ang-GetAngle()));
			}
			break;
		case 4:
			ang=atan(-a/b)*180/PI-90;
			if((-(b*GetY()+c)/a)>GetX())
			{
				output=Angle_Pid(-err-(ang-GetAngle()));
			}
			else if((-(b*GetY()+c)/a)<GetX())
			{
				output=Angle_Pid(err-(ang-GetAngle()));
			}
			break;
		case 5:
			ang=-90;
			if(GetY()>=-c)
			{
					output=Angle_Pid(err-(ang-GetAngle()));
			}
			else if(GetY()<-c)
			{
					output=Angle_Pid(-err-(ang-GetAngle())); 	
			}
			break;
			case 6:
			ang=90;
			if(GetY()<=-c)
			{
				output=Angle_Pid(err-(ang-GetAngle()));
			}
			else if(GetY()>-c)
			{
				output=Angle_Pid(-err-(ang-GetAngle()));
			}
			break;
		case 7:
			ang=0;
			if(GetX()<=-c)
			{
				output=Angle_Pid(err-(ang-GetAngle()));
			}
			else if(GetX()>-c)
			{
				output=Angle_Pid(-err-(ang-GetAngle()));
			}
			break;
		case 8:
			ang=180;
			if(GetX()>=-c)
			{
					output=Angle_Pid(err-(ang-GetAngle()));
			}
			else if(GetX()<-c)
			{
					output=Angle_Pid(-err-(ang-GetAngle()));
			}
			break;
	}
	return output;
}


float Angle_change(float a1,float a2,int c)
{
	static float a3=0;
	if(c==0)
		a3=a1-a2;
	else if(c==1)
		a3=a1+a2;
	if(a3>=180)
		a3-=360;
	else if(a3<=-180)
		a3+=360;
	return a3;		
}

void Virer_PID_Out(float x0,float y0,float radius,float speed,int Direction)	
{
	static float d=0,err=0,output=0,ang1=0,ang2=0,angle1=0,angle2=0,speed_f=0,speed_s=0;
	d=sqrtf((GetX()-x0)*(GetX()-x0)+(GetY()-y0)*(GetY()-y0));
	err=Distance_Pid(radius-d);
	if(err<=-90)
		err=-90;
	else if(err>=90)
		err=90;
	ang2=360/(2*PI*radius/speed*100);
	angle1=atan2f(y0-GetY(),x0-GetX())*180/PI;
	speed_f=(speed*4096/(120*PI));
	speed_s=(speed*4096/(120*PI));
	switch(Direction)
	{
		case 0:
			angle2=Angle_change(angle1,90,0);
			ang1=Angle_change(angle2,90,0);
			if(d>radius)
			{
			
				output=Angle_Pid(-err+ang2+(ang1-GetAngle()));
			  Walk_Virer(speed_f+output,speed_s-output);
			}
			else if(d<radius)
			{
			
				output=Angle_Pid(-err+ang2+(ang1-GetAngle()));
				Walk_Virer(speed_f+output,speed_s-output);
			}
			break;
		case 1:
			angle2=Angle_change(angle1,90,1);
			ang1=Angle_change(angle2,90,0);
			if(d>radius)
			{
				output=Angle_Pid(err-ang2+(ang1-GetAngle()));
				Walk_Virer(speed_s+output,speed_f-output);
			}
			else if(d<radius)
			{
				output=Angle_Pid(err-ang2+(ang1-GetAngle()));
				Walk_Virer(speed_s+output,speed_f-output);
			}
			break;
	}
//	USART_OUT(UART4,(uint8_t*) "%d\t",(int)angle1);
//	//USART_OUT(UART4,(uint8_t*) "%d\t",(int)angle2);
//	USART_OUT(UART4,(uint8_t*) "%d\t",(int)ang1);
//	USART_OUT(UART4,(uint8_t*) "%d\t",(int)err);
//		USART_OUT(UART4,(uint8_t*) "%d\t",(int)ang2);
//	
//	USART_OUT(UART4,(uint8_t*) "%d\t",(int)output);
//	USART_OUT(UART4,(uint8_t*) "%d\t",(int)GetAngle());
//	USART_OUT(UART4,(uint8_t*) "%d\t",(int)GetX());
//	USART_OUT(UART4,(uint8_t*) "%d\r\n",(int)GetY());
}	

typedef union
{
    //这个32位整型数是给电机发送的速度（脉冲/s）
    int32_t Int32 ;
    //通过串口发送数据每次只能发8位
    uint8_t Uint8[4];

}num_t;

//定义联合体
num_t u_Num;

void SendUint8(void)
{
    u_Num.Int32 = 1000;

    //起始位
    USART_SendData(USART1, 'A');
    //通过串口1发数
    USART_SendData(USART1, u_Num.Uint8[0]);
    USART_SendData(USART1, u_Num.Uint8[1]);
    USART_SendData(USART1, u_Num.Uint8[2]);
    USART_SendData(USART1, u_Num.Uint8[3]);
    //终止位
    USART_SendData(USART1, 'J');
}
//// 发射航向角转换函数 由度转换为脉冲
//// yawAngle为角度，范围180到-180之间，初始位置为0度。

//// 将角度转换为脉冲
//float YawTransform(float yawAngle)
//{
//	return (yawAngle * YAW_REDUCTION_RATIO * COUNT_PER_DEGREE);
//}

////发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
//void YawAngleCtr(float yawAngle)
//{
//	PosCrl(CAN1, GUN_YAW_ID, POS_ABS, YawTransform(yawAngle));
//}
//// 同样要配置位置环
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	USART3_Init(115200);
	UART4_Init(921600);
	TIM_Init(TIM2,999,83,1,0);
	Adc_Init();
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	VelLoopCfg(CAN2,1,0,0);
	VelLoopCfg(CAN2,2,0,0);
	// 配置速度环
	VelLoopCfg(CAN1, 8, 50000, 50000);
	// 配置位置环
	PosLoopCfg(CAN1, PUSH_BALL_ID, 50000,50000,20000);

	MotorOn(CAN2,1);
	MotorOn(CAN2,2);
	delay_s(2);


	/*一直等待定位系统初始化完成*/
	WaitOpsPrepare();
	OSTaskSuspend(OS_PRIO_SELF);
}

void WalkTask(void)
{
	int cnt=0,LV=0,Arm=0;
	int dir=0,counter=0,flg=0,flag=0,jg_flg=0,count=0,tr_cnt=0,dir_v=0;
	float pid_out,dis_R,dis_L,X_last=10000,Y_last=10000;
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		
		OSSemPend(PeriodSem, 0, &os_err);
//		dis_R=(4400.f/4096.f)*(float)Get_Adc_Average(14,10);
//		dis_L=(4400.f/4096.f)*(float)Get_Adc_Average(15,10);
//		count++;
//		if(count>=100&&dir!=0)
//		{
//			count=0;
//			if(fabs(X_last-GetX())<=100&&fabs(Y_last-GetY())<=100)
//			{
//				dir_v=dir;
//				dir=3;
//			}
//			
//			X_last=GetX();
//			Y_last=GetY();
//			
//		}
//		USART_OUT(UART4,(uint8_t*) "%d\t",(int)dis_R);
//		USART_OUT(UART4,(uint8_t*) "%d\r\n",(int)dis_L);
//		if(jg_flg==0)
//		{
//			
//			if(dis_R<=50)
//			{
//				dir=1;
//				jg_flg=1;
//			}
//			else if(dis_L<=50)
//			{
//				dir=2;
//				jg_flg=1;
//			}
//		}
//		
//		switch(dir)
//		{
//			case 1:
//				Virer_PID_Out(0,2200,2000-counter*400,v,0);
//				if(GetAngle()<=-150&&GetAngle()>=-170)
//				{
//					flag=1;
//				}
//				if(GetAngle()<=-80&&GetAngle()>=-110&&GetX()>=-50&&GetX()<=50&&flag==1)
//						{
//							flag=0;
//							if(counter>=3)
//								flg=1;
//							else if(counter==0)
//								flg=0;
//							if(flg==0)
//								counter++;
//							else if(flg==1)
//								counter--;		
//						}	
//				break;
//			case 2:
//				Virer_PID_Out(0,2200,2000-counter*400,v,1);
//				if(GetAngle()>=150&&GetAngle()<=170)
//				{
//					flag=1;
//				}
//				if(GetAngle()>=80&&GetAngle()<=110&&GetX()>=-50&&GetX()<=50&&flag==1)
//						{
//							flag=0;
//							if(counter>=3)
//								flg=1;
//							else if(counter==0)
//								flg=0;
//							if(flg==0)
//								counter++;
//							else if(flg==1)
//								counter--;		
//						}	
//				break;	
//			case 3:
//				tr_cnt++;
//				if(tr_cnt<=100)
//				{
//					Walk_Straight(-1000,-1000);
//				}
//				else
//				{
//					tr_cnt=0;
//					if(counter==3)
//						counter--;
//					else
//						counter++;
//					dir=dir_v;
//				}
//				break;		
//		}

SendUint8();
// 控制电机的转速，脉冲。
VelCrl(CAN1,COLLECT_BALL_ID,60*4096);
// 推球
PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);
// 复位
PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);



//		Virer_PID_Out(0,2200,2000-counter*400,v,0);
//		if(GetAngle()<=-150&&GetAngle()>=-170)
//		{
//			flag=1;
//		}
//		if(GetAngle()<=-80&&GetAngle()>=-110&&GetX()>=-50&&GetX()<=50&&flag==1)
//				{
//					flag=0;
//					if(counter>=3)
//						flg=1;
//					else if(counter==0)
//						flg=0;
//					if(flg==0)
//						counter++;
//					else if(flg==1)
//						counter--;		
//				}	
		 
			
//		switch(cnt)
//		{
//			case 0:
//				if(GetX()>-1200)
//				{
//					pid_out=PID_OUT(GetX(),GetY(),0,1,0,6);
//					Walk_Straight(v-pid_out,v+pid_out);
//				}
//				else 
//				{
//					cnt++;
//				}
//			case 1:
//				if(GetY()<(3450-LV*300))
//				{
//					pid_out=PID_OUT(GetX(),GetY(),1,0,1750-Arm*300,7);
//					Walk_Straight(v-pid_out,v+pid_out);
//				}
//				else
//				{
//					cnt++;
//				}
//				break;
//			case 2:
//				if(GetX()<(1200-LV*300))
//				{
//					pid_out=PID_OUT(GetX(),GetY(),0,1,-(4000-Arm*300),5);
//					Walk_Straight(v-pid_out,v+pid_out);
//				}
//				else
//				{
//					cnt++;
//				}
//				break;
//			case 3:
//				if(GetY()>(1300+LV*300))
//				{
//					pid_out=PID_OUT(GetX(),GetY(),1,0,-(1750-Arm*300),8);
//					Walk_Straight(v-pid_out,v+pid_out);
//				}
//				else
//				{
//					cnt++;
//				}
//				break;	
//			case 4:
//				if(GetX()>-(700-LV*300))
//				{
//					pid_out=PID_OUT(GetX(),GetY(),0,1,-(750+Arm*300),6);
//					Walk_Straight(v-pid_out,v+pid_out);
//				}
//				else
//				{
//					cnt=1;
//					LV++;
//					Arm++;
//				}
//				break;	
//		}
	}
}
