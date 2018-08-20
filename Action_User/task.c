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
#include "adc.h"

//#define Veh 1         //车号选择 【1】 || 【4】
extern int t;           //时间校正【1ms】   
extern float X,Y,Angl;  //【系统反馈数据】XY坐标（浮点） 校正角度（浮点）
int j=0;

void delay1(void)       //计时函数【100ms】
{if(t<100){j=0;}
if(t>=100){j=1;}}

void Move(int V1,int V2)//运动函数
{
VelCrl(CAN2,1,V1);//右轮
VelCrl(CAN2,2,V2);//左轮
}

int isOKFlag=0;////////////////////////////////////////////////////////////|
int IsSendOK(void)                                                       //|
{                                                                        //|  
return isOKFlag;                                                         //|
}                                                                        //|
void SetOKFlagZero(void)                                                 //|
{                                                                        //|
isOKFlag=0;                                                              //|
}                                                                        //|
void driveGyro(void)                                                     //|
{                                                                        //|————————确认蓝牙连接AT\OK
while(!IsSendOK())                                                       //|
{                                                                        //|
delay_ms(5);                                                             //|
USART_SendData(USART3,'A');                                              //|
USART_SendData(USART3,'T');                                              //|
USART_SendData(USART3,'\r');                                             //|
USART_SendData(USART3,'\n');                                             //|
}                                                                        //|
SetOKFlagZero();                                                         //|
}//////////////////////////////////////////////////////////////////////////|

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

extern int asd;

//===================================================================================================================================================
//===================================================================================================================================================
// 
//                           信号量定义
//
//===================================================================================================================================================
//===================================================================================================================================================
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


//===================================================================================================================================================
//===================================================================================================================================================
// 
//                             初始化任务
//
//===================================================================================================================================================
//===================================================================================================================================================

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
	VelLoopCfg(CAN2,1,40960000,40960000);
	VelLoopCfg(CAN2,2,40960000,40960000);

//	//驱动器位置环初始化
//	PosLoopCfg(CAN2,1,2048000,2048000,1024);
//	PosLoopCfg(CAN2,2,2048000,2048000,1024);
	
	//电机使能（通电）
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	
	Adc_Init();
	
	if(Veh==1)//
	{
	delay_s(2);
	driveGyro();
	while(asd);
	}//
	if(Veh==4)
	{
	delay_s(10);
	delay_s(5);
	}
	
//	delay_s(5);
	
	OSTaskSuspend(OS_PRIO_SELF);
}
//====================================================================================   
// _   _   _   _   _   _   _   _   _                _   _   _   _   _   _   _   _   _ 
//| |_| |_| |_| |_| |_| |_| |_| |_| |_【参数选择】_| |_| |_| |_| |_| |_| |_| |_| |_| |
//
//====================================================================================
#define v          1     //【Mode1】【Mode2】【Mode3】【Mode4】【Mode5】车身速度（m/s）
#define r          1.f     //【Mode1】【Mode2】         【Mode4】【Mode5】        半径 || 边长（m）
                           //【Mode1】：车身旋转半径（填入0则直行）（r>0逆时针运动；r<0顺时针运动）
                           //【Mode2】：正方形边长（r=0为直行）
                           //【Mode4】：正方形半边长（标志位到4边长度）
                           //【Mode5】：定圆半径
#define x_C        1000.f  //【Mode5】：圆心坐标x
#define y_C        2000.f  //【Mode5】：圆心坐标y
#define d_M        1500    //【Mode4】：标志点（y轴位点）
#define direction  0       //【Mode1】：方向（0为前进，1为后退）
                           //【Mode5】：顺时针/逆时针走圆（0为逆时针，1为顺时针）
#define Mode       6       //【模式选择】：
				           // 0调试状态（目前设置为静止）
				           // 1直行（r=0）||圆周运动 前进/后退; 
                           // 2直行（r=0）||多边形运动（此时r为多边形边长）（带自动校正）
                           // 3直线闭环
                           // 4
#define angle       0      //【Mode1】【Mode2】【Mode3】角度选择（度）
                           //【Mode1】：（无实际意义）                    
                           //【Mode2】：直线角度（自动校正）（车坐标）              
                           // Mode3：直线闭环角度选择                         >——————|                   |——> 直线角度（veh4坐标）
#define y_l         1000   //                  【Mode3】直线交y轴截距         >——————————【直线方程】———————> y_b
#define x_l        -1000   //                  【Mode3】直线平行y轴时与x轴交点>——————|                   |——> x_b
#define pi          3.14   // Π=3.14159265358979323846264338327950288
#define p_a         500    //                  【Mode3】直线闭环校正开始距离（mm）————————————————————————————————|
                           //                                                                                     |
//#define w_veh     300    // 角速度【删除】  150                                                                 |
                           //                                                                                     |
#define Kp_A        200    //【P】角度闭环————|| 300的Kp_A可能导致角度积分错误                                    |————【相关】
#define Ki_A        0      //【I】角度闭环————||                                                                  |
#define Kd_A        0      //【D】角度闭环————||                                                                  |
                           //                                                                                     |
#define Kp_l        0.03   //【D】直线闭环————||  0.09（Kp_l=90度/p_a(1000)）——————（Mode3直线用90ok）————————————|
#define Ki_l        0      //【I】直线闭环————||
#define Kd_l        0      //【D】直线闭环————||

//正方形闭环：1m/s: 200 500 0.03 （距离Kp过小导致机器人无法快速回到直线）
int v1,v2;         //两轮速度（2左 || 1右）
int x,y,angl;      //XY坐标（整数） 校正角度（整数）

//【其他参量】

//Move(V1,V2)               //运动函数定义参量
//v1,v2;                    //两轮速度（2左 || 1右）
//Veh                       //车号选择 【1】 || 【4】
//t                         //时间校正【1ms】           
//X,Y,Angl                  //【系统反馈数据】XY坐标（浮点） 校正角度（浮点型）
//x,y,angl                  //XY坐标（整数） 校正角度（整型）
//ex                        //交换变量（坐标反转函数）
//ANG                       //角度差（测量值angl-预期量ang）（角度闭环）
//Angle_Lock4(ang)          //角度闭环中给定【预期值】
//Line_Lock2(ang_l,y_b,x_b) //直线闭环中给定【直线角度】【直线—y轴截距】【直线-x轴截距】
//flo                       //数据反馈
//【数据反馈格式】：【角度校正量】【点到预定直线距离】【直线k值】  X:【x坐标】Y：【y坐标】A：【角度反馈】
//                                                                ————————————————————————————————————————（均取整数）
//float k;                  //直线斜率
//float d;                  //点到直线距离
//int x1=0,y1=0;            //正方形走位每边初始坐标记录
//int mo=1;                 //正方形走位模式

//====================================================================================
//                            定义车1、车4 【x】、【y】、【angl】（坐标翻转）
//====================================================================================
int ex;
float exf;
void Teh_Choose(void)          //坐标反转函数
{
if(Veh==1)
{
ex=(int)Y;
y=-(int)X;
x=ex;
angl=-(int)Angl;

exf=Y;
Y=-X;
X=exf;
}
else if(Veh==4)
{
x=(int)X;
y=(int)Y;
angl=(int)Angl;
}
else if(j==1)
{
USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%s%s%s%s\r\n","V","e","h","_","E","r","r","o","r");//Veh_Error
j=0;t=0;
}
}


/////////////////////////// 
//          |y /         //
//         0|0/          //
//          |/           //
// 90       /       -90  //
// ————————/|—————————— >//————————Veh【4】
//        / |          x //
//       /  |            //
//      /180|-180        //
//     /    |            //
///////////////////////////
/////////////////////////// 
//          |-x/         //
//         0|0/          //
//          |/           //
// -90      /        90  //
// ————————/|—————————— >//————————Veh【1】（已转换）
//        / |          y //
//       /  |            //
//      /   |            //
//     /-180| 180        //
///////////////////////////
//====================================================================================
//                                  PID（封装方案）
//======================================================================================
void PID_Step1()//————————|
{               //        |
}               //        |
void PID_Step2()//————————|——————【忽然不想写了】
{               //        |
}               //        |
void PID_Step3()//————————|
{
}
//====================================================================================
//                                角度闭环（方案）
//====================================================================================
/*
#define para 1/180

void Angle_Lock(int ang)//锁定角度方案一
{
	if(ang>0&&ang<=90)
	{
		if(angl<=ang+2&&angl>ang-2)
		{Move(v1,-v1);}
		if(angl<=-135)
		{Move((-0.005*v1*(360+angl-ang)),-v1);}//
		if(angl>ang+2)
		{Move(-0.005*v1*(angl-ang),-v1);}//
		if(angl<=ang-2&&angl>-135)
		{Move(v1,0.005*v1*(ang-angl));}//
	}
	if(ang>90&&ang<=180)
	{
		if(angl<=ang+2&&angl>ang-2)
		{Move(v1,-v1);}
		if(angl<=-45)
		{Move(-0.005*v1*(360+angl-ang),-v1);}
		if(angl>ang+2)
		{Move(-0.005*v1*(angl-ang),-v1);}
		if(angl<=ang-2&&angl>-45)
		{Move(v1,0.005*v1*(ang-angl));}
	}
	if(ang>-180&&ang<=-90)
	{
		if(angl<=ang+2&&angl>ang-2)
		{Move(v1,-v1);}
		if(angl<=45&&angl>ang+2)
		{Move(-0.005*v1*(angl-ang),-v1);}
		if(angl<=ang-2)
		{Move(v1,0.005*v1*(ang-angl));}
		if(angl>45)
		{Move(v1,0.005*v1*(360-angl+ang));}
	}
	
	if(ang>-90&&ang<=0)
	{
		if(angl<=ang+2&&angl>ang-2)
		{Move(v1,-v1);}
		if(angl<=135&&angl>ang+2)
		{Move(-0.005*v1*(angl-ang),-v1);}
		if(angl<=ang-2)
		{Move(v1,0.005*v1*(ang-angl));}
		if(angl>135)
		{Move(v1,0.005*v1*(360-angl+ang));}
	}
//if(angl<=A+2&&angl>=A-2)
//{Move(v1,-v1);}
//if(angl>A+2)
//{Move(0,-v1);}
//if(angl<A-2)
//{Move(v1,0);}
}

void Angle_Lock2(int ang)//锁定角度方案二
{
	if(ang>0&&ang<=90)
	{
	if(angl<=ang+2&&angl>ang-2)
	{Move(v1,-v1);}
	if(angl<=-135)
	{Move(-(0.005*(360+angl-ang))*(0.005*(360+angl-ang))*(0.005*(360+angl-ang))*(0.005*(360+angl-ang))*(0.005*(360+angl-ang))*v1,-v1);}//
	if(angl>ang+2)
	{Move(-0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*v1,-v1);}//
	if(angl<=ang-2&&angl>-135)
	{Move(v1,0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*v1);}//
	}
	if(ang>90&&ang<=180)
	{
	if(angl<=ang+2&&angl>ang-2)
	{Move(v1,-v1);}
	if(angl<=-45)
	{Move(-0.005*(360+angl-ang)*0.005*(360+angl-ang)*0.005*(360+angl-ang)*0.005*(360+angl-ang)*0.005*(360+angl-ang)*v1,-v1);}
	if(angl>ang+2)
	{Move(-0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*v1,-v1);}
	if(angl<=ang-2&&angl>-45)
	{Move(v1,0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*v1);}
	}
	if(ang>-180&&ang<=-90)
	{
	if(angl<=ang+2&&angl>ang-2)
	{Move(v1,-v1);}
	if(angl<=45&&angl>ang+2)
	{Move(-0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*v1,-v1);}
	if(angl<=ang-2)
	{Move(v1,0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*v1);}
	if(angl>45)
	{Move(v1,0.005*(360-angl+ang)*0.005*(360-angl+ang)*0.005*(360-angl+ang)*0.005*(360-angl+ang)*0.005*(360-angl+ang)*v1);}
	}
	
	if(ang>-90&&ang<=0)
	{
	if(angl<=ang+2&&angl>ang-2)
	{Move(v1,-v1);}
	if(angl<=135&&angl>ang+2)
	{Move(-0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*0.005*(angl-ang)*v1,-v1);}
	if(angl<=ang-2)
	{Move(v1,0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*0.005*(ang-angl)*v1);}
	if(angl>135)
	{Move(v1,0.005*(360-angl+ang)*0.005*(360-angl+ang)*0.005*(360-angl+ang)*0.005*(360-angl+ang)*0.005*(360-angl+ang)*v1);}
	}
}
*/
int v1_record=0,v2_record=0;


void Angle_Lock3(int ang)//锁定角度方案3【成功】
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
int v1_record,v2_record;
int ANG;                 //角度差（测量值angl-预期量ang）
void Angle_Lock4(int ang)//锁定角度方案4【成功】——————最终采纳方案4
{
//作差
if(ang<-180){ang=360+ang;}
if(ang>180){ang=ang-360;}
ANG=angl-ang;
//差值讨论
if(ANG>180){ANG=ANG-360;}
if(ANG<-180){ANG=ANG+360;}
//运行
Move(v1-Kp_A*ANG,-v1-Kp_A*ANG);

//记录v1、v2
v1_record=v1-Kp_A*ANG;
v2_record=-v1-Kp_A*ANG;
}

//====================================================================================
//                          多边形运动（方案）————凉凉【】失败
//====================================================================================
/*
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
*/
//====================================================================================
//                                直线闭环（方案）
//====================================================================================
/*
//void Line_Lock1(int ang)//直线闭环方案1
//{
//ANG=angl-ang;
//if(ANG>180){ANG=ANG-360;}
//if(ANG<-180){ANG=ANG+360;}
////{Move(v1-300*ANG,-v1-300*ANG);}

//	if(ang== 90){{Move(v1-Pa*ANG+Pl*y,-v1-Pa*ANG+Pl*y);}}
//	if(ang==-90){{Move(v1-300*ANG-Pl*y,-v1-300*ANG-Pl*y);}}
//	if(ang!=90&&ang!=-90)
//	{
//		k=tan (ang*pi/180);
//		b=y-k*x;
//		d=b/(1+k*k);
//		if(ang<=0){Move(v1-Pa*ANG-Pl*d,-v1-Pa*ANG-Pl*d);}
//		if(ang>0){Move(v1-Pa*ANG+Pl*d,-v1-Pa*ANG+Pl*d);}
//	}
//}
*/
float k;
float d;
/*
void Line_Lock2(int ang_l, float y_b, float x_b)
{
if(ang_l==0)
{
	if(x>(p_a+x_b)){Angle_Lock4(90);}
	if(x<(-p_a+x_b)){Angle_Lock4(-90);}
	if(x<=(p_a+x_b)&&x>=(-p_a+x_b)){Angle_Lock4(90*(x-x_b)/p_a);}
}
if(ang_l==180||ang_l==-180)
{
    if(x>(p_a+x_b)){Angle_Lock4(-90);}
	if(x<(-p_a+x_b)){Angle_Lock4(90);}
	if(x<=(p_a+x_b)&&x>=(-p_a+x_b)){Angle_Lock4(180-(90*(x-x_b))/p_a);}
}
if(ang_l!=0&&ang_l!=180&&ang_l!=-180)
{
	if(ang_l== 90)
    {
		if(y>(p_a+y_b)){Angle_Lock4(ang_l+90);}
		if(y<(-p_a+y_b)){Angle_Lock4(ang_l-90);}
		if(y<=(p_a+y_b)&&y>=(-p_a+y_b)){Angle_Lock4(ang_l+(90*(y-y_b))/p_a);}
	}
	if(ang_l==-90)
    {
		if(y>(p_a+y_b)){Angle_Lock4(ang_l-90);}
		if(y<(-p_a+y_b)){Angle_Lock4(ang_l+90);}
		if(y<=(p_a+y_b)&&y>=(-p_a+y_b)){Angle_Lock4(ang_l-(90*(y-y_b))/p_a);}
	}
	if(ang_l!=90&&ang_l!=-90)
	{
		if(ang_l>-90&&ang_l<90)
		{k=tan(pi*(ang_l+90)/180);}
		if(ang_l>90){k=tan(pi*(ang_l-90)/180);}
		if(ang_l<-90){k=tan(pi*(ang_l+270)/180);}
		d=(y-k*x-y_b)/(sqrt(1+k*k));
		if(ang_l>0)
		{
			if(d>p_a){Angle_Lock4(ang_l+90);}
			if(d<-p_a){Angle_Lock4(ang_l-90);}
			if(d<=p_a&&d>=-p_a){Angle_Lock4(ang_l+90*d/p_a);}
		}
		if(ang_l<0)
		{
			if(d>p_a){Angle_Lock4(ang_l-90);}
			if(d<-p_a){Angle_Lock4(ang_l+90);}
			if(d<=p_a&&d>=-p_a){Angle_Lock4(ang_l-90*d/p_a);}
		}
	}
}
}
*/
/*
void Line_Lock3(int ang_l, float y_b, float x_b)
{
if(ang_l==0)                {d=x-x_b;}
if(ang_l==180||ang_l==-180) {d=x_b-x;}
if(ang_l!=0&&ang_l!=180&&ang_l!=-180)  
{
	if(ang_l>=-90&&ang_l<=90){k=tan(pi*(ang_l+90)/180);}
	if(ang_l>90){k=tan(pi*(ang_l-90)/180);}
	if(ang_l<-90){k=tan(pi*(ang_l+270)/180);}
	d=(y-k*x-y_b)/(sqrt(1+k*k));
}
if(ang_l>=0)
{
	if(d>p_a){Angle_Lock4(ang_l+90);}
	if(d<-p_a){Angle_Lock4(ang_l-90);}
	if(d<=p_a&&d>=-p_a){Angle_Lock4(ang_l+Kp_l*d);}
}
if(ang_l<0)
{
	if(d>p_a){Angle_Lock4(ang_l-90);}
	if(d<-p_a){Angle_Lock4(ang_l+90);}
	if(d<=p_a&&d>=-p_a){Angle_Lock4(ang_l-Kp_l*d);}
}
}
*/

void Line_Lock4(int ang_l, float y_b, float x_b)
{
if(ang_l==0)                {d=x-x_b;}
if(ang_l==180||ang_l==-180) {d=x_b-x;}
if(ang_l!=0&&ang_l!=180&&ang_l!=-180)  
{
	if(ang_l>=-180&&ang_l<=0){k=tan(pi*(ang_l+90)/180);}
	if(ang_l>0){k=tan(pi*(ang_l-90)/180);}
//	if(ang_l<-90){k=tan(pi*(ang_l+270)/180);}
	d=(y-k*x-y_b)/(sqrt(1+k*k));
}
if(ang_l>=0)
{
	if(d>p_a){Angle_Lock4(ang_l+90);}
	if(d<-p_a){Angle_Lock4(ang_l-90);}
	if(d<=p_a&&d>=-p_a){Angle_Lock4(ang_l+Kp_l*d);}
}
if(ang_l<0)
{
	if(d>p_a){Angle_Lock4(ang_l-90);}
	if(d<-p_a){Angle_Lock4(ang_l+90);}
	if(d<=p_a&&d>=-p_a){Angle_Lock4(ang_l-Kp_l*d);}
}
}

/*
float Kp_line;
void Line_Lock5(int ang_l, float y_b, float x_b)
{
if(ang_l==0)                {d=x-x_b;}
if(ang_l==180||ang_l==-180) {d=x_b-x;}
if(ang_l!=0&&ang_l!=180&&ang_l!=-180)  
{
	if(ang_l>=-180&&ang_l<=0){k=tan(pi*(ang_l+90)/180);}
	if(ang_l>0){k=tan(pi*(ang_l-90)/180);}
//	if(ang_l<-90){k=tan(pi*(ang_l+270)/180);}
	d=(y-k*x-y_b)/(sqrt(1+k*k));
}
Kp_line=((1/5000000)*d*d-(1/5000)*d+0.09);
if(ang_l>=0)
{
	if(d>p_a){Angle_Lock4(ang_l+90);}
	if(d<-p_a){Angle_Lock4(ang_l-90);}
	if(d<=p_a&&d>=-p_a){Angle_Lock4(ang_l+Kp_line*d);}
}
if(ang_l<0)
{
	if(d>p_a){Angle_Lock4(ang_l-90);}
	if(d<-p_a){Angle_Lock4(ang_l+90);}
	if(d<=p_a&&d>=-p_a){Angle_Lock4(ang_l-Kp_line*d);}
}
}
*/
//====================================================================================
//                                正方形闭环（方案）
//====================================================================================
//r为正方形半边长
//d_M为正方形半边长
//四条直线（顺时针）：【1】y=d_M-r（+） , 【2】x=r（+） , 【3】y=d_M+r（+） , 【4】x=-r（+）
//四条直线（逆时针）：【1】y=d_M-r（-） , 【4】x=-r（-） , 【3】y=d_M+r（-） , 【2】x=r（-）
//direction顺/逆时针（0为逆时针，1为顺时针）
/////////////////////////// 
//         0|0           //
//     【3】|y           //
//     |<———|————|【2】  //
//     |    |    |       //
//     |    +d_M |       //
//     |   r|    |       //
//【4】|————|———>|       //
// 90       |【1】  -90  //
// —————————|—————————— >//
//          |          x //
//          |            //
//       180|-180        //
//          |            //
///////////////////////////

int d1,d2,Cho=0;
//void D_Compare(void)
//{
//d1=abs(y-(d_M-r));
//}

void Square_Lock1(int d_M_b , int r_b)//回到原定路线
{
if(direction==0)//逆时针
{
if(Cho==0){Line_Lock4(-90,d_M_b-r_b,0);}
if(Cho==0&&x>=r_b-p_a){Cho=1;Line_Lock4(0,0,r_b);}
if(Cho==1){Line_Lock4(0,0,r_b);}
if(Cho==1&&y>=(d_M_b+r_b-p_a)){Cho=2;Line_Lock4(90,d_M_b+r_b,0);}
if(Cho==2){Line_Lock4(90,d_M_b+r_b,0);}
if(Cho==2&&x<-r_b+p_a){Cho=3;Line_Lock4(180,0,-r_b);}
if(Cho==3){Line_Lock4(180,0,-r_b);}
if(Cho==3&&y<=d_M_b-r_b+p_a){Cho=0;Line_Lock4(-90,d_M_b-r_b,0);}
}
}


void Square_Lock3(int d_M_b , int r_b)//回到最优路线
{}

//====================================================================================
//                                正圆闭环（方案）
//====================================================================================
float angle_C;   //机器人最终行进角度
float angle_Cl;	 //机器人与圆心连线角度
float dangle_C;  //机器人（切线）偏差角度
float dangle_Cd; //机器人偏差距离校正
float d_C;       //机器人到圆心距离
//#define v          1       //【Mode5】车身速度（m/s）
//#define r          1       //【Mode5】：定圆半径
//#define x_C        1000    //【Mode5】：圆心坐标x
//#define y_C        2000    //【Mode5】：圆心坐标y
	
	
void Circle_Lock1(void)//【角度闭环】沿切线方向走(过点做切线)
{
d_C=sqrt((x_C-x)*(x_C-x)+(y_C-y)*(y_C-y));

	if(x_C!=x)
	{angle_Cl=180*(atan((y_C-Y)/(x_C-X)))/pi;}
	if(x_C==x)
	{
	if(y_C>y){angle_Cl=90;}
	if(y_C<y){angle_Cl=-90;}
	}
	if(y_C-y>0&&x_C-x<0){angle_Cl=angle_Cl+180;}
	if(y_C-y<0&&x_C-x<0){angle_Cl=angle_Cl-180;}

if(direction==0)//逆时针
{
	if(d_C>1000*r)
	{
		dangle_C=180*(asin(1000*r/d_C))/pi;
		if(dangle_C<0){dangle_C=-dangle_C;}
		
		if(d_C-1000*r<=p_a){dangle_Cd=(d_C-1000*r)*dangle_C/p_a;}
		if(d_C-1000*r>p_a){dangle_Cd=dangle_C;}
		
		angle_C=angle_Cl+dangle_Cd-dangle_C-90;
		
		Angle_Lock4(angle_C);
	}
	if(d_C==1000*r)
	{Angle_Lock4(angle_Cl-180);}
	if(d_C<1000*r)
	{
	dangle_Cd=90*(1000*r-d_C)/1000*r;	
	angle_C=angle_Cl-dangle_Cd-180;
	Angle_Lock4(angle_C);
	}
}
if(direction==1)//顺时针
{}
}


/*
void Circle_Lock2(void)//【角度闭环】距离决定角偏差量（大于r为锐角、小于r为钝角）
{}
*/

void Circle_Lock3(void)//【角度闭环】（过圆心交圆一点做切线）
{
d_C=sqrt((x_C-x)*(x_C-x)+(y_C-y)*(y_C-y));
angle_Cl=180*(atan((y_C-Y)/(x_C-X)))/pi;
if(direction==0)
{
	USART_OUT(UART4,(uint8_t*)"%d  ",(int)(X));

	USART_OUT(UART4,(uint8_t*)"%d  ",(int)(y_C-Y));
	USART_OUT(UART4,(uint8_t*)"%d  ",(int)(x_C-X));
	USART_OUT(UART4,(uint8_t*)"%d  ",(int)(x_C-X));

	USART_OUT(UART4,(uint8_t*)"%d  ",(int)angle_Cl);
		
	if(x_C-x<=0){angle_Cl=angle_Cl+90;}
	if(x_C>x) {angle_Cl=angle_Cl-90;}

	if(angle_Cl>180){angle_Cl-=360;}
	if(angle_Cl<-180){angle_Cl+=360;}

	d_C=d_C-r*1000;
	if(d_C>0)
	{angle_C=angle_Cl-90/(1+d_C/400);}
	if(d_C<0)
	{angle_C=angle_Cl-180+90/(1-d_C/400);}

	if(angle_C>180){angle_C-=360;}
	if(angle_C<-180){angle_C+=360;}
	Angle_Lock4(angle_C);
}
if(direction==1)//（未尝试/理论可行）
{
	if(x_C-x<=0){angle_Cl=angle_Cl-90;}
	if(x_C>x) {angle_Cl=angle_Cl+90;}

	if(angle_Cl>180){angle_Cl-=360;}
	if(angle_Cl<-180){angle_Cl+=360;}

	d_C=d_C-r*1000;
	if(d_C>0)
	{angle_C=angle_Cl+90/(1+d_C/400)-180;}
	if(d_C<0)
	{angle_C=angle_Cl-90/(1-d_C/400);}
		
	if(angle_C>180){angle_C-=360;}
	if(angle_C<-180){angle_C+=360;}
	Angle_Lock4(angle_C);
}
}
/*
void Circle_Lock4(void)//【改变基础分频】基础圆++变轨角（距离决定）
{
d_C=sqrt((x_C-x)*(x_C-x)+(y_C-y)*(y_C-y));
if(d_C>r+1000)
{}
if(d_C<=r+1000)
{
	if(d_C>=r&&d_C<r+1000)
	{}
	if(d_C<r)
	{}
}
}
void Circle_Lock5(void)//【圆闭环】
{}
*/

//====================================================================================
//                                正方形扫荡（方案）
//====================================================================================

int square_edg=2000;
int sweep_mode=0;
void Square_Sweep_Right1(int square_m , int square_e)//回到原定路线
{
if(square_edg>=2000) {sweep_mode=0;}
if(square_edg<=500)  {sweep_mode=1;}

if(Cho==0){Line_Lock4(-90,square_m-square_e,0);}
if(Cho==0&&x>=square_e-p_a){Cho=1;Line_Lock4(0,0,square_e);}
if(Cho==1){Line_Lock4(0,0,square_e);}
if(Cho==1&&y>=(square_m +square_e-p_a)){Cho=2;Line_Lock4(90,square_m +square_e,0);}
if(Cho==2){Line_Lock4(90,square_m +square_e,0);}
if(Cho==2&&x<-square_e+p_a){Cho=3;Line_Lock4(180,0,-square_e);}
if(Cho==3){Line_Lock4(180,0,-square_e);}
if(Cho==3&&y<=square_m -square_e+p_a)
{
if(sweep_mode==0){Cho=0;Line_Lock4(-90,square_m -square_e,0);square_edg=square_edg-300;}
if(sweep_mode==1){Cho=0;Line_Lock4(-90,square_m -square_e,0);square_edg=square_edg+300;}
}
}

void Square_Sweep_Left1(int square_m , int square_e)
{
if(square_edg>=2000) {sweep_mode=0;}
if(square_edg<=500)  {sweep_mode=1;}
	
if(Cho==0){Line_Lock4(90,square_m-square_e,0);}
if(Cho==0&&x<=-square_e+p_a){Cho=1;Line_Lock4(0,0,-square_e);}
if(Cho==1){Line_Lock4(0,0,-square_e);}
if(Cho==1&&y>=(square_m +square_e-p_a)){Cho=2;Line_Lock4(-90,square_m +square_e,0);}
if(Cho==2){Line_Lock4(-90,square_m +square_e,0);}
if(Cho==2&&x>square_e-p_a){Cho=3;Line_Lock4(180,0,square_e);}
if(Cho==3){Line_Lock4(180,0,square_e);}
if(Cho==3&&y<=square_m -square_e+p_a)
{
if(sweep_mode==0)
{Cho=0;Line_Lock4(90,square_m -square_e,0);square_edg=square_edg-300;}
if(sweep_mode==1)
{Cho=0;Line_Lock4(90,square_m -square_e,0);square_edg=square_edg+300;}	
}
}
//====================================================================================
//                                 ADC激光
//====================================================================================
float G_Adc_A4;
float G_Adc_A5;
float V4;
float V4_f;
int V4_i;

float V5;
float V5_f;
int V5_i;

void Adc(void)
{
V4=G_Adc_A4*10/4096;
V4_f=V4;
while(V4_f-1>=0)
{V4_f--;}
V4_i=V4-V4_f;
V4_f*=1000;

V5=G_Adc_A5*10/4096;
V5_f=V5;
while(V5_f-1>=0)
{V5_f--;}
V5_i=V5-V5_f;
V5_f*=1000;

USART_OUT(UART4,(uint8_t*)"%d",V4_i);
// 发字符
USART_OUT(UART4,(uint8_t*)"%s",".");
USART_OUT(UART4,(uint8_t*)"%d  ",(int) V4_f);


USART_OUT(UART4,(uint8_t*)"%d",V5_i);
// 发字符
USART_OUT(UART4,(uint8_t*)"%s",".");
USART_OUT(UART4,(uint8_t*)"%d  ",(int) V5_f);
}

//====================================================================================
//                                 死亡重启
//====================================================================================
float x_p10=0;
float x_p20=0;
float y_p10=0;
float y_p20=0;

int t_col=0;
int square_break=0;

float v_cal;
float v_real;

void square_edg_jump(void)
{


}	
	
void Collision_Processing(void)
{
v_cal=(v1_record-v2_record)/(2*10865);
v_real=0.1*sqrt((X-x_p10)*(X-x_p10)+(Y-y_p10)*(Y-y_p10));
if(v_real<0.5*v_cal){t_col++;}
if(v_real>0.5*v_cal){t_col=0;}
if(t_col>=100){square_break=1;square_edg_jump();}
}

//===================================================================================================================================================
//===================================================================================================================================================
 
//                             运动模块

//===================================================================================================================================================
//===================================================================================================================================================
int   x1=0,y1=0;    //mode2正方形边初始坐标记录
int   mo=1;         //mode2正方形边初始坐标记录
	
float Angle_p10=0;  //10ms前角度值
float Angle_p20=0;  //20ms前角度值
float d_Angle;      //角度微分（10ms）

int   flo;          //【反馈】输出检测
int t_adc=0;
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem,0,&os_err);
	OSSemPend(PeriodSem, 0, &os_err);
	while (1)
	{
		G_Adc_A4=Get_Adc_Average(14,10);
		G_Adc_A5=Get_Adc_Average(15,10);
		
		
		OSSemPend(PeriodSem, 0, &os_err);
		Teh_Choose();//坐标反转函数（一定要放在while开始，OSSemPend后一行）	
		
        //记录10ms前、20ms前角度值
		{
		Angle_p20=Angle_p10;
		Angle_p10=Angl;
			
		x_p20=x_p10;
		x_p10=X;
		y_p20=y_p10;
		y_p10=Y;
		}
		

/*		
//		x=(int)X;
//		y=(int)Y;
//		angl=(int)Angl;
*/		
//		delay1();
//		if(j==1)
//		{
//			delay1();
			USART_OUT(UART4,(uint8_t*)"%s%s","X",":");
			USART_OUT(UART4,(uint8_t*)"%d  ",x);
			USART_OUT(UART4,(uint8_t*)"%s%s","Y",":");
			USART_OUT(UART4,(uint8_t*)"%d  ",y);
			USART_OUT(UART4,(uint8_t*)"%s%s","A",":");
			USART_OUT(UART4,(uint8_t*)"%d  ",angl);
//		}
		
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
/*
//				//【方案一】
//				{
//					if(angl>=-2&&angl<=1)
//					{Move(v1,-v1);}
//					if(angl>1&&angl<=180)
//					//{Move(v1,0);}//1左转
//					{Move(0,-v1);}//4右转
//					if(angl>=-180&&angl<-2)
//					//{Move(0,-v1);}//1右转
//					{Move(v1,0);}//4左转
//				}
//				//【方案2】（失败）——摇头丸【失败】
//				{
//				Angle_Lock2(90);
//				}
//				//【方案3】（已改）——摇头丸【失败】
//				{
//				Angle_Lock2(179);
//				}
//				//【方案4】（尝试）————可行---
//				{
//				Angle_Lock3(90);
//				}
*/
                //【方案5】（尝试）————可行---【最终方案】
				{
				Angle_Lock4(angle);
				}
			}
			if(r!=0)//正方形//4车
			{
				if(y<=(y1+750*r)&&mo==1){Angle_Lock4(0);}
				if(y>(y1+750*r)&&mo==1){x1=x;y1=y;Angle_Lock4(-90);mo=2;}
				
				if(x<=(x1+750*r)&&mo==2){Angle_Lock4(-90);}
				if(x>(x1+750*r)&&mo==2){x1=x;y1=y;Angle_Lock4(179);mo=3;}
				
				if(y>(y1-750*r)&&mo==3){Angle_Lock4(179);}
				if(y<=(y1-750*r)&&mo==3){x1=x;y1=y;Angle_Lock4(90);mo=4;}
				
				if(x>(x1-750*r)&&mo==4){Angle_Lock4(90);}
				if(x<(x1-750*r)&&mo==4){x1=x;y1=y;Angle_Lock4(0);mo=1;}
			}
/*
//			时钟方案【不可行】
//			使用PID控制！！！
//			if(t<=1000*r/v)                              //直行r(m)
//			{Move(v1,-v1);}
//			if(t>(1000*r/v)&&t<=1000*r/v+3.8*angle/v)    //旋转angle度
//			{Move(v1,v1);}
//			if(t>(1000*r/v+3.8*angle/v))                 //t归0
//			{t=0;}
*/			
		}
		if(Mode==3)//蛇皮走位（直线闭环）
		{
			v1=(int)10865*v;
			Line_Lock4(angle,y_l,x_l);
//			while(j)//【反馈数据】
//			{
//			flo=(int)(90*d/p_a);
//			USART_OUT(UART4,(uint8_t*)"%d  ",flo);
//			flo=(int)d;
//			USART_OUT(UART4,(uint8_t*)"%d  ",flo);	
//			flo=(int)k;
//			USART_OUT(UART4,(uint8_t*)"%d  ",flo);		
//			t=0;j=0;
//			}
		}
		if(Mode==4)//正方形闭环
		//【思路】在车正前方d处建立一个标志点，过标志点沿x、y轴正、负方向等距建立四条直线（正方形）（方便以后调参）正方形边长的一半为r
		{
		v1=(int)10865*v;
		
		Square_Lock1(d_M , r);
			
		    while(j)//【反馈数据】
			{
			flo=Kp_A;
			USART_OUT(UART4,(uint8_t*)"%d  ",flo);
			flo=p_a;
			USART_OUT(UART4,(uint8_t*)"%d  ",flo);
			flo=Kp_l*100;
			USART_OUT(UART4,(uint8_t*)"%d  ",flo);
			flo=(int)(Kp_l*d);
			USART_OUT(UART4,(uint8_t*)"%d  ",flo);
			flo=(int)d;
			USART_OUT(UART4,(uint8_t*)"%d  ",flo);	
			t=0;j=0;
			}
		}
		
		if(Mode==5)
		{
			
			v1=(int)10865*v;
			Circle_Lock1();
			
//			while(j)//反馈数据
//			{	
			flo=(int)d_C;
			USART_OUT(UART4,(uint8_t*)"%d  ",flo);
			
			flo=(int)angle_Cl;
			USART_OUT(UART4,(uint8_t*)"%d  ",flo);			
			flo=(int)angle_C;
			USART_OUT(UART4,(uint8_t*)"%d\r\n",flo);	
			
//			j=0;t=0;
//			}
		}
		if(Mode==6)//正方形扫荡+ADC激光爆炸
		//
		{
			Adc();                                 //adc收到数据处理、反馈

			int left=0,right=0;
			if(V5<1){t_adc--;}
			if(V4<1){t_adc++;}
			if(t_adc>=100){right=1;}
			if(t_adc<=-100){left=1;}               //判断左、右
			
				flo=(int)left;                     //反馈数据
				USART_OUT(UART4,(uint8_t*)"%d  ",flo);
				flo=(int)right;
				USART_OUT(UART4,(uint8_t*)"%d  ",flo);		
				flo=(int)t_adc;
				USART_OUT(UART4,(uint8_t*)"%d  ",flo);
			if(left==1||right==1)
			{
				v1=(int)10865*v;                         //基础速度
				//判断是否被卡住
				
				//如果被卡住
				
				//如果没有被卡住
				{
				if(right==1)                             //右
				{
				{Square_Sweep_Right1(2200 , square_edg);}
					flo=(int)square_edg;
					USART_OUT(UART4,(uint8_t*)"%d  ",flo);	
				}
				
				else if(left==1)	                      //左
				{
				{Square_Sweep_Left1(2200 , square_edg);}  //【Square_Sweep_Right1还没写】！！！！！
					flo=(int)square_edg;
					USART_OUT(UART4,(uint8_t*)"%d  ",flo);	
				}
			    }
		    }
       		
			USART_OUT(UART4,(uint8_t*)"\r\n");	//换行（独列）
		}
		
//		OSSemPend(PeriodSem, 0, &os_err);
	}

}	
