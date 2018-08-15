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

//#define Veh 1         //车号选择 【1】 || 【4】
extern int t;           //时间校正【1ms】           
extern float X,Y,Angl;  //【系统反馈数据】XY坐标（浮点） 校正角度（浮点）
int j=0;
//void delay10(void)
//{
//if(t<10000){i=1;}
//if(t>=10000){i=0;}
//}

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
/*
===================================================================================================================================================
===================================================================================================================================================
 
                           信号量定义

===================================================================================================================================================
===================================================================================================================================================
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
===================================================================================================================================================
===================================================================================================================================================
 
                             初始化任务

===================================================================================================================================================
===================================================================================================================================================
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
	delay_s(2);
	driveGyro();
	while(asd);
	}///////////////////////////////////////////////////////////////////////
	if(Veh==4)
	{
	delay_s(10);
	delay_s(5);
	}
//	delay_s(5);
	
	OSTaskSuspend(OS_PRIO_SELF);
	
}
/*====================================================================================   
 _   _   _   _   _   _   _   _   _                _   _   _   _   _   _   _   _   _   _
| |_| |_| |_| |_| |_| |_| |_| |_| |_【参数选择】_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |

======================================================================================*/   
#define v          1       //【Mode1】【Mode2】【Mode3】车身速度（m/s）
#define r          0       //【Mode1】【Mode2】         半径 || 边长（m）
                           // Mode1：车身旋转半径（填入0则直行）（r>0逆时针运动；r<0顺时针运动）
                           // Mode2：正方形边长（r=0为直行）
#define direction  0       //【Mode1】                  方向（0为前进，1为后退————Mode1适用
#define Mode       3       //【模式选择】：
				           //0调试状态（目前设置为静止）
				           //1直行（r=0）||圆周运动 前进/后退; 
                           //2直行（r=0）||多边形运动（此时r为多边形边长）（带自动校正）
                           //直线闭环
#define angle       -135   //【Mode1】【Mode2】【Mode3】角度选择（度）
                           //Mode1：（无实际意义）                    
                           //Mode2：直线角度（自动校正）              
                           //Mode3：直线闭环角度选择                          >——————|                   |——> 直线角度（veh4坐标）
#define y_l         -1000  //                  【Mode3】直线交y轴截距         >——————————【直线方程】———————> y_b
#define x_l         0      //                  【Mode3】直线平行y轴时与x轴交点>——————|                   |——> x_b
#define pi          3.14   //Π=3.14159265358979323846264338327950288
#define p_a         1000   //                  【Mode3】直线闭环校正开始距离（mm）
//#define w_veh     150    //角速度【删除】

#define Kp_A        150    //【P】角度闭环————|| 300的Kp_A可能导致角度积分错误
#define Ki_A        0      //【I】角度闭环————|| 
#define Kd_A        0      //【D】角度闭环————||

#define Kp_l        0.09   //【D】直线闭环————|| 最大值为0.09 （Kp_l=90度/p_a(1000)）
#define Ki_l        0      //【I】直线闭环————||
#define Kd_l        0      //【D】直线闭环————||


int v1,v2;         //两轮速度（2左 || 1右）
int x,y,angl;      //XY坐标（整数） 校正角度（整数）

//【其他参量】
/*
Move(V1,V2)               //运动函数定义参量
v1,v2;                    //两轮速度（2左 || 1右）
Veh                       //车号选择 【1】 || 【4】
t                         //时间校正【1ms】           
X,Y,Angl                  //【系统反馈数据】XY坐标（浮点） 校正角度（浮点型）
x,y,angl                  //XY坐标（整数） 校正角度（整型）
ex                        //交换变量（坐标反转函数）
ANG                       //角度差（测量值angl-预期量ang）（角度闭环）
Angle_Lock4(ang)          //角度闭环中给定【预期值】
Line_Lock2(ang_l,y_b,x_b) //直线闭环中给定【直线角度】【直线—y轴截距】【直线-x轴截距】
flo                       //数据反馈
【数据反馈格式】：【角度校正量】【点到预定直线距离】【直线k值】  X:【x坐标】Y：【y坐标】A：【角度反馈】
                                                                ————————————————————————————————————————（均取整数）
float k;                  //直线斜率
float d;                  //点到直线距离
int x1=0,y1=0;            //正方形走位每边初始坐标记录
int mo=1;                 //正方形走位模式
*/

/*====================================================================================
                            定义车1、车4 【x】、【y】、【angl】（坐标翻转）
======================================================================================*/
int ex;
void Teh_Choose(void)          //坐标反转函数
{
if(Veh==1)
{
ex=(int)Y;
y=-(int)X;
x=ex;
angl=-(int)Angl;
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
/*====================================================================================
                                  PID（封装方案）
======================================================================================*/
void PID_Step1()
{
}
void PID_Step2()
{
}
void PID_Step3()
{
}
/*====================================================================================
                                角度闭环（方案）
======================================================================================*/
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

int ANG;                 //角度差（测量值angl-预期量ang）
void Angle_Lock4(int ang)//锁定角度方案4【成功】——————最终采纳方案3
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
}

/*====================================================================================
                          多边形运动（方案）————凉凉【】失败
======================================================================================*/
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
/*====================================================================================
                                直线闭环（方案）
======================================================================================*/

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

float k;
float d;

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



/*
===================================================================================================================================================
===================================================================================================================================================
 
                             运动模块

===================================================================================================================================================
===================================================================================================================================================
*/
int x1=0,y1=0; //mode2正方形边初始坐标记录
int mo=1;      //mode2正方形边初始坐标记录
int flo;       //输出检测
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

//			时钟方案【不可行】
//			使用PID控制！！！
//			if(t<=1000*r/v)                              //直行r(m)
//			{Move(v1,-v1);}
//			if(t>(1000*r/v)&&t<=1000*r/v+3.8*angle/v)    //旋转angle度
//			{Move(v1,v1);}
//			if(t>(1000*r/v+3.8*angle/v))                 //t归0
//			{t=0;}
			
		}
		if(Mode==3)//蛇皮走位（直线闭环）
		{
		v1=(int)10865*v;
		Line_Lock3(angle,y_l,x_l);
		while(j)
		{
		flo=(int)(90*d/p_a);
		USART_OUT(UART4,(uint8_t*)"%d  ",flo);
		flo=(int)d;
		USART_OUT(UART4,(uint8_t*)"%d  ",flo);	
		flo=(int)k;
		USART_OUT(UART4,(uint8_t*)"%d  ",flo);		
		t=0;j=0;
		}
		}
		OSSemPend(PeriodSem, 0, &os_err);
//		vel_radious(500.0,500.0);			//半径为0.5m，速度为0.5m/s
	}
}
