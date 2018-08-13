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

#define Veh 1      //车号选择 【1】 || 【4】

extern int t;
extern float X,Y,Angl;  //XY坐标（浮点） 校正角度（浮点）
int j=0;
//void delay10(void)
//{
//if(t<10000){i=1;}
//if(t>=10000){i=0;}
//}

void delay1(void)
{if(t<100){j=0;}
if(t>=100){j=1;}}

void Move(int V1,int V2)
{
VelCrl(CAN2,1,V1);//右轮
VelCrl(CAN2,2,V2);//左轮
}

int isOKFlag=0;////////////////////////////////////////////////////////////
int IsSendOK(void)
{
return isOKFlag;
}
void SetOKFlagZero(void)
{
isOKFlag=0;
}
void driveGyro(void)
{
while(!IsSendOK())
{
delay_ms(5);
USART_SendData(USART3,'A');
USART_SendData(USART3,'T');
USART_SendData(USART3,'\r');
USART_SendData(USART3,'\n');
}
SetOKFlagZero();
}////////////////////////////////////////////////////////////////////////////

//#define Pulse2mm COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi)


///*
//一个脉冲是4096/(120*Pi)
//定义输入速度mm/s和半径mm
//*/
//float ratio1,ratio2;
//void vel_radious(float vel,float radious)
//{
//	ratio1=(radious+WHEEL_TREAD/2)/radious;
//	ratio2=(radious-WHEEL_TREAD/2)/radious;
//	VelCrl(CAN2,1,ratio1*vel*Pulse2mm);
//	VelCrl(CAN2,2,-ratio2*vel*Pulse2mm);
//}


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

	os_err = OSTaskCreate((void (*)(void *))WalkTask,    /*运动指令*/
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  (INT8U)Walk_TASK_PRIO);
    OSTaskSuspend(OS_PRIO_SELF);
}

/*
   ===============================================================
   初始化任务
   ===============================================================
   */
void ConfigTask(void)        //初始化
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM_Init(TIM2,1000-1,84-1,1,3);	//产生10ms中断，抢占优先级为1，响应优先级为3

	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	
	VelLoopCfg(CAN2,1, 5000, 5000);				//驱动器速度环初始化
	VelLoopCfg(CAN2,2, 5000, 5000);
	
	ElmoInit(CAN2);								//驱动器初始化
	MotorOn(CAN2,1);							//电机使能（通电）
	MotorOn(CAN2,2);
	
	//TIM4_Pwm_Init (9999,83);//pwm初始化（10ms）
	TIM_Init(TIM2, 999, 83, 0X01, 0X03);//TIM2 1ms中断
	
	USART3_Init(115200);
	UART4_Init(921600);
	
//	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);//CAN1通信
	
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);//CAN2通信
	
	//Elmo驱动器初始化
	ElmoInit(CAN2);
	
	//驱动器速度环初始化
	VelLoopCfg(CAN2,1,2048000,2048000);
	VelLoopCfg(CAN2,2,2048000,2048000);

//	//驱动器位置环初始化
//	PosLoopCfg(CAN2,1,2048000,2048000,1024);
//	PosLoopCfg(CAN2,2,2048000,2048000,1024);
	
	//电机使能（通电）
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	
	if(Veh==1)//////////////////////////////////////////////////////////////
	{
	driveGyro();////////////////////////////////////////////////////////////
	}///////////////////////////////////////////////////////////////////////
	else
	{
	delay_s(10);
	delay_s(5);
	}
//	delay_s(5);
	
	OSTaskSuspend(OS_PRIO_SELF);
	
}
/*====================================================================================
                                  参数选择
======================================================================================*/
#define v 0.5      //车身速度 
#define r 2        //半径 || 边长
                   //Mode1：车身旋转半径（填入0则直行）（r>0逆时针运动；r<0顺时针运动）
                   //Mode2：正方形边长（r=0为直行）
#define direction 0//方向（0为前进，1为后退————Mode1适用
#define Mode 2     //模式选择：
				   //0调试状态（目前设置为静止）
				   //1直行（r=0）||圆周运动 前进/后退; 
                   //2直行（r=0）||多边形运动（此时r为多边形边长）（带自动校正）
#define angle 90   //Mode1：（无实际意义）
                   //Mode2：多边形邻边角度
int v1,v2;         //两轮速度（2左 || 1右）

int x,y,angl;      //XY坐标（整数） 校正角度（整数）

/*====================================================================================
                          此处定义车1、车4 x、y、angl
======================================================================================*/

int ex;
void Teh_Choose(void)
{
x=(int)X;
y=(int)Y;
angl=(int)Angl;
if(Veh==1)
{
ex=y;
y=-x;
x=ex;
angl=-angl;
}
else if(Veh==4)
{
x=x;
y=y;
angl=angl;
}
else if(j==1)
{
USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%s%s%s%s\r\n","V","e","h","_","E","r","r","o","r");//Veh_Error
j=0;t=0;
}
}

/*====================================================================================
                                角度锁定（方案）
======================================================================================*/
//#define para 1/180

//void Angle_Lock(int ang)//锁定角度方案一
//{
//	if(ang>0&&ang<=90)
//	{
//		if(angl<=ang+2&&angl>ang-2)
//		{Move(v1,-v1);}
//		if(angl<=-135)
//		{Move((-0.005*v1*(360+angl-ang)),-v1);}//
//		if(angl>ang+2)
//		{Move(-0.005*v1*(angl-ang),-v1);}//
//		if(angl<=ang-2&&angl>-135)
//		{Move(v1,0.005*v1*(ang-angl));}//
//	}
//	if(ang>90&&ang<=180)
//	{
//		if(angl<=ang+2&&angl>ang-2)
//		{Move(v1,-v1);}
//		if(angl<=-45)
//		{Move(-0.005*v1*(360+angl-ang),-v1);}
//		if(angl>ang+2)
//		{Move(-0.005*v1*(angl-ang),-v1);}
//		if(angl<=ang-2&&angl>-45)
//		{Move(v1,0.005*v1*(ang-angl));}
//	}
//	if(ang>-180&&ang<=-90)
//	{
//		if(angl<=ang+2&&angl>ang-2)
//		{Move(v1,-v1);}
//		if(angl<=45&&angl>ang+2)
//		{Move(-0.005*v1*(angl-ang),-v1);}
//		if(angl<=ang-2)
//		{Move(v1,0.005*v1*(ang-angl));}
//		if(angl>45)
//		{Move(v1,0.005*v1*(360-angl+ang));}
//	}
//	
//	if(ang>-90&&ang<=0)
//	{
//		if(angl<=ang+2&&angl>ang-2)
//		{Move(v1,-v1);}
//		if(angl<=135&&angl>ang+2)
//		{Move(-0.005*v1*(angl-ang),-v1);}
//		if(angl<=ang-2)
//		{Move(v1,0.005*v1*(ang-angl));}
//		if(angl>135)
//		{Move(v1,0.005*v1*(360-angl+ang));}
//	}
////if(angl<=A+2&&angl>=A-2)
////{Move(v1,-v1);}
////if(angl>A+2)
////{Move(0,-v1);}
////if(angl<A-2)
////{Move(v1,0);}
//}

//void Angle_Lock2(int ang)//锁定角度方案二
//{
//	if(ang>0&&ang<=90)
//	{
//	if(angl<=ang+2&&angl>ang-2)
//	{Move(v1,-v1);}
//	if(angl<=-135)
//	{Move(-(0.005*(360+angl-ang))*(0.005*(360+angl-ang))*(0.005*(360+angl-ang))*(0.005*(360+angl-ang))*(0.005*(360+angl-ang))*v1,-v1);}//
//	if(angl>ang+2)
//	{Move(-0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*v1,-v1);}//
//	if(angl<=ang-2&&angl>-135)
//	{Move(v1,0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*v1);}//
//	}
//	if(ang>90&&ang<=180)
//	{
//	if(angl<=ang+2&&angl>ang-2)
//	{Move(v1,-v1);}
//	if(angl<=-45)
//	{Move(-0.005*(360+angl-ang)*0.005*(360+angl-ang)*0.005*(360+angl-ang)*0.005*(360+angl-ang)*0.005*(360+angl-ang)*v1,-v1);}
//	if(angl>ang+2)
//	{Move(-0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*v1,-v1);}
//	if(angl<=ang-2&&angl>-45)
//	{Move(v1,0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*v1);}
//	}
//	if(ang>-180&&ang<=-90)
//	{
//	if(angl<=ang+2&&angl>ang-2)
//	{Move(v1,-v1);}
//	if(angl<=45&&angl>ang+2)
//	{Move(-0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*v1,-v1);}
//	if(angl<=ang-2)
//	{Move(v1,0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*v1);}
//	if(angl>45)
//	{Move(v1,0.005*(360-angl+ang)*0.005*(360-angl+ang)*0.005*(360-angl+ang)*0.005*(360-angl+ang)*0.005*(360-angl+ang)*v1);}
//	}
//	
//	if(ang>-90&&ang<=0)
//	{
//	if(angl<=ang+2&&angl>ang-2)
//	{Move(v1,-v1);}
//	if(angl<=135&&angl>ang+2)
//	{Move(-0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*v1,-v1);}
//	if(angl<=ang-2)
//	{Move(v1,0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*v1);}
//	if(angl>135)
//	{Move(v1,0.005*(360-angl+ang)*0.005*(360-angl+ang)*0.005*(360-angl+ang)*0.005*(360-angl+ang)*0.005*(360-angl+ang)*v1);}
//	}
//}

void Angle_Lock3(int ang)//锁定角度方案3
{
	if(ang>0&&ang<=90)
	{
		if(angl==ang)
		{Move(v1,-v1);}
		if(angl<=-135)
		{Move(v1,-v1-600*(360+angl-ang));}//
		if(angl>ang)
		{Move(v1,-v1-600*(angl-ang));}//
		if(angl<=ang&&angl>-135)
		{Move(v1+600*(ang-angl),-v1);}//
	}
	if(ang>90&&ang<=180)
	{
		if(angl==ang)
		{Move(v1,-v1);}
		if(angl<=-45)
		{Move(v1,-v1-600*(360+angl-ang));}
		if(angl>ang)
		{Move(v1,-v1-600*(angl-ang));}
		if(angl<=ang&&angl>-45)
		{Move(v1+600*(ang-angl),-v1);}
	}
	if(ang>-180&&ang<=-90)
	{
		if(angl==ang)
		{Move(v1,-v1);}
		if(angl<=45&&angl>ang)
		{Move(v1,-v1-600*(angl-ang));}
		if(angl<=ang)
		{Move(v1+600*(ang-angl),-v1);}
		if(angl>45)
		{Move(v1+600*(360-angl+ang),-v1);}
	}
	
	if(ang>-90&&ang<=0)
	{
		if(angl==ang)
		{Move(v1,-v1);}
		if(angl<=135&&angl>ang)
		{Move(v1,-v1-600*(angl-ang));}
		if(angl<=ang)
		{Move(v1+600*(ang-angl),-v1);}
		if(angl>135)
		{Move(v1+600*(360-angl+ang),-v1);}
	}
}

int ANG;
void Angle_Lock4(int ang)//锁定角度方案4
{ANG=angl-ang;
if(ANG>180){ANG=ANG-360;}
if(ANG<-180){ANG=ANG+360;}
if(ANG==0){Move(v1,-v1);}
if(ANG>0){Move(v1,-v1-600*ANG);}
if(ANG<0){Move(v1-600*(ANG),-v1);}
}

int t1=0;
void Move_Mode2 (int d1,int a1)//多边形——————【失败】
{
	t=0;
	while(1)
	{
	if(y<1000*r&&t1==0){Move(v1,-v1);}
	if(y>=1000*r){t1=t;  Move(0,-v1);  t=0;}
	if(angl+angle>=-180&&angl+angle<=180){a1=angl+angle;}
	if(angl+angle>=180){a1=angl+angle-360;}
	if(angl<angl+angle&&a1>=-180&&a1<=180){}//角度转换边长问题【未解决】（8.11）。。。凉了凉了（8.12）
	}
}










/*====================================================================================
                                    程序 mode1 2 3
======================================================================================*/
int x1=0,y1=0;
int mo=1;

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem,0,&os_err);
	OSSemPend(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		
		Teh_Choose();/////////////////////////////////////////////////////
		
//		x=(int)X;
//		y=(int)Y;
//		angl=(int)Angl;
		
		delay1();
		if(j==1)
		{
			delay1();
			USART_OUT(UART4,(uint8_t*)"%s%s","X",":");
			USART_OUT(UART4,(uint8_t*)"%d  ",x);
			USART_OUT(UART4,(uint8_t*)"%s%s","Y",":");
			USART_OUT(UART4,(uint8_t*)"%d  ",y);
			USART_OUT(UART4,(uint8_t*)"%s%s","A",":");
			USART_OUT(UART4,(uint8_t*)"%d\r\n",angl);
		}
		
		if(Mode==0)//测试状态
	    {
			VelCrl(CAN2,1,256);//右轮
			VelCrl(CAN2,2,256);//左轮
			while(j)
			{
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%d\r\n","M","o","d","e",0);//mode0
			t=0;j=0;
			}
		}
		if(Mode==1)                       //Mode1 直行（r=0）或圆周运动 前进/后退
		{
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%d\r\n","M","o","d","e",1);//mode1
			if(r!=0)                            //圆周运动
			{
				v1=(int)((10865*v)+(217*v*10.8/r));
				v2=(int)((10865*v)-(217*v*10.8/r));
				if(direction==0)                      //圆周运动（前进）
				{Move(v1,-v2);}
				else if(direction==1)                 //圆周运动（后退）
				{Move(-v1,v2);}
			}
			else                                //直行
			{
				v1=(int)10865*v;
				if(direction==0)                      //直行（前进）
				{Move(v1,-v1);}
				if(direction==1)                      //直行（后退）
				{Move(-v1,v1);}
			}
			while(j)
			{
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%d\r\n","M","o","d","e",1);
			t=0;j=0;
			}
	    }
		if(Mode==2)                       //Mode2 直行（r=0）||多边形运动（带自动校正）（r为多边形边长；angle为多边形邻边角度） 
		{
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%d\r\n","M","o","d","e",2);
			v1=(int)10865*v;
			if(r==0) //直行or正方形（附自动校正）
			{
//				{//【方案一】
//					if(angl>=-2&&angl<=1)
//					{Move(v1,-v1);}
//					if(angl>1&&angl<=180)
//					//{Move(v1,0);}//1左转
//					{Move(0,-v1);}//4右转
//					if(angl>=-180&&angl<-2)
//					//{Move(0,-v1);}//1右转
//					{Move(v1,0);}//4左转
//				}
//				{//【方案2】（失败）——摇头丸【失败】
//				Angle_Lock2(90);
//				}
//				{//【方案3】（已改）——摇头丸【失败】
//				Angle_Lock2(179);
//				}
//				{//【方案4】（尝试）————可行---
//				Angle_Lock3(90);
//				}
				{//【方案5】（尝试）————可行---【最终方案】
				Angle_Lock4(179);
				}
			}
			if(r!=0)//正方形//4车
			{
				if(y<=(y1+1000*r)&&mo==1){Angle_Lock4(0);}
				if(y>(y1+1000*r)&&mo==1){x1=x;y1=y;Angle_Lock4(-90);mo=2;}
				
				if(x<=(x1+1000*r)&&mo==2){Angle_Lock4(-90);}
				if(x>(x1+1000*r)&&mo==2){x1=x;y1=y;Angle_Lock4(179);mo=3;}
				
				if(y>(y1-1000*r)&&mo==3){Angle_Lock4(179);}
				if(y<=(y1-1000*r)&&mo==3){x1=x;y1=y;Angle_Lock4(90);mo=4;}
				
				if(x>(x1-1000*r)&&mo==4){Angle_Lock4(90);}
				if(x<(x1-1000*r)&&mo==4){x1=x;y1=y;Angle_Lock4(0);mo=1;}
			}
//			使用PID控制！！！
//			时钟方案【不可行】
//			if(t<=1000*r/v)                              //直行r(m)
//			{Move(v1,-v1);}
//			if(t>(1000*r/v)&&t<=1000*r/v+3.8*angle/v)    //旋转angle度
//			{Move(v1,v1);}
//			if(t>(1000*r/v+3.8*angle/v))                 //t归0
//			{t=0;}
			while(j)
			{
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%d\r\n","M","o","d","e",2);//mode2
			t=0;
			j=0;
			}
			
		}
		if(Mode==3)//蛇皮走位
		{
		
		}
		OSSemPend(PeriodSem, 0, &os_err);
//		vel_radious(500.0,500.0);			//半径为0.5m，速度为0.5m/s
	}
}
