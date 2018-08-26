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
#include "fort.h"
                          //圆周率                 3.1415
#define One_Meter_Per_Second (10865.0)            //车轮一米每秒的设定值   4096*(1000/120π)
#define BaseVelocity (0.5 * One_Meter_Per_Second) //基础速度               0.5m/s
//#define CarOne 1                                  //一号车编号             1
//#define CarFour 4                                 //四号车编号             4
#define Side_Length (2000)                        //方形边长               2m
#define Angle_Error_Range (3)                     //角度误差范围           3 

#define Pulse2mm COUNTS_PER_ROUND/(WHEEL_DIAMETER*PI)

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
一个脉冲是4096/(120*Pi)
定义输入速度mm/s和半径mm
*/
float ratio1,ratio2;
void vel_radious(float vel,float radious)
{
	ratio1=(radious+WHEEL_TREAD/2)/radious;
	ratio2=(radious-WHEEL_TREAD/2)/radious;
	VelCrl(CAN2,1,ratio1*vel*Pulse2mm);
	VelCrl(CAN2,2,-ratio2*vel*Pulse2mm);
}

uint8_t adcFlag=0;
extern uint8_t squareFlag;
struct usartValue_{
	uint32_t cnt;//用于检测是否数据丢失
	float xValue;//串口输出x坐标
	float yValue;//串口输出y坐标
	float angleValue;//串口输出角度值
	float pidValueOut;//PID输出
	float d;
	float turnAngleValue;//
	uint8_t flagValue;
	float shootangle;
}usartValue;


/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

extern Pos_t pos;

//目标坐标信息结构体，x为横坐标，y为纵坐标，angle为与0°的相对角度
typedef struct{ 
	float x;
	float y;
	float angle;
}Target;

typedef union
{
    //这个32位整型数是给电机发送的速度（脉冲/s）
    int32_t Int32 ;
    //通过串口发送数据每次只能发8位
    uint8_t Uint8[4];

}num_t;

extern char buffer[20];
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
	OSTaskSuspend(OS_PRIO_SELF);
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
	Init();
	OSTaskSuspend(OS_PRIO_SELF);

}

//10ms 运行一次；
int cntTurn = 0, cntSendTime = 0;
char switchNextModeFlag = 1, adjustFlag = 0, turnFlag = 0;
float adjustVelocity, baseVelocity;

//炮台发回的值
extern FortType fort;

void WalkTask(void)
{
	int cntSendTime;
	CPU_INT08U os_err;
	os_err = os_err;
	
//	//新底盘
//	Angle_PidPara(20,0,0);
//	Distance_PidPara(1,0,0); 
	
	//旧底盘
	Angle_PidPara(25,0,300);
	Distance_PidPara(0.09,0,0); 
	squareFlag=0;
	
	float laserAValue=0;
	float laserBValue=0;
	uint16_t cnt=0;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{

		
		OSSemPend(PeriodSem, 0, &os_err);
		
		laserAValue=GetLaserAValueReceive();
		laserBValue=GetLaserAValueReceive();
//		
//		//发射摩擦轮转速
//		ShooterVelCtrl(50);
//		
		//走位
		Walk(&adcFlag);
		Shoot(adcFlag); 
//		WorkTwo(&adcFlag);
//		RoundTwo(0,2350,2000,0,1000);
////
////		

		
//		cnt++;
//		if(cnt == 50)
//		{
//			// 推球	
//			PosCrl(CAN1, 0x06,ABSOLUTE_MODE,4500);
//		}
//		else if(cnt > 100)
//		{
//			// 复位
//			PosCrl(CAN1, 0x06,ABSOLUTE_MODE,5);
//			cnt=0;
//		}
		//新底盘闭环转弯
//		Turn2(-135,250);
		
		//新底盘开环圆
//		Round3();
		
		//新底盘闭环圆
//		Round2(500,1000,400,0,250);

		//新底盘闭环直线
//		straightLine2(1,-1,-400,0);

		//收球电机
//		VelCrl(CAN1,COLLECT_BALL_ID,60*4096); 
//		
//		//航向电机
//		YawAngleCtr(40);
//		
//		SendUint8();
//	
		USART_OUT(UART4, " %d\t", (int)usartValue.flagValue);
//		USART_OUT(UART4, " %d\t", (int)usartValue.turnAngleValue);
		USART_OUT(UART4, " %d\t", (int)fort.shooterVelReceive);
//		USART_OUT(UART4, " %d\t", (int)usartValue.pidValueOut);
//		USART_OUT(UART4, " %d\t", (int)adcFlag);
//		USART_OUT(UART4, " %d\t", (int)usartValue.angleValue);
		USART_OUT(UART4, " %d\t", (int)usartValue.shootangle);
//		USART_OUT(UART4, " %d\t", (int)fort.yawPosReceive);
//		USART_OUT(UART4, " %d\t", (int)fort.shooterVelReceive);
		USART_OUT(UART4, " %d\t", (int)usartValue.xValue);
		USART_OUT(UART4, " %d\r\n", (int)usartValue.yValue);

//		USART_OUT(UART4, " %d\t", (int)laserAValue);
//		USART_OUT(UART4, " %d\r\n", (int)laserBValue);
//		USART_OUT(USART1, " %d\t", (int)usartValue.d);
//		USART_OUT(USART1, " %d\t", (int)usartValue.turnAngleValue);
//		USART_OUT(USART1, " %d\t", (int)usartValue.angleValue);
//		USART_OUT(USART1, " %d\t", (int)usartValue.xValue);
//		USART_OUT(USART1, " %d\r\n", (int)usartValue.yValue);


	
	}
}
 
//初始化函数
void Init(void)
{
	TIM_Init(TIM2, 999, 84, 0x01, 0x03);
	USART3_Init(115200);
	USART1_Init(921600);
	UART4_Init(921600);
	USART3_Init(115200);
	UART5_Init(921600);
	Adc_Init();
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	ElmoInit(CAN2);
	
	//右轮电机初始化
	VelLoopCfg(CAN2, 0x01, 20000, 20000);
	
	//左轮电机初始化
	VelLoopCfg(CAN2, 0x02, 20000, 20000);
	
	//收球电机初始化
	VelLoopCfg(CAN1, 0x08, 50000, 500000);
//	
	//推球电机初始化
	PosLoopCfg(CAN1, PUSH_BALL_ID, 5000000,5000000,20000);
////	
//	//航向电机初始化
//	PosLoopCfg(CAN1, GUN_YAW_ID, 50000,50000,20000);
	
	//新底盘后轮电机初始化
//	VelLoopCfg(CAN2, 0x05, 10000000, 10000000);
//	
//	//新底盘前轮电机初始化
//	VelLoopCfg(CAN2, 0x06, 10000000, 10000000);
	MotorOn(CAN2, 0x01);
	MotorOn(CAN2, 0x02);
	MotorOn(CAN1, 0x08);
	MotorOn(CAN1, PUSH_BALL_ID);
//	MotorOn(CAN1, GUN_YAW_ID);

//	MotorOn(CAN2, 0x05);
//	MotorOn(CAN2, 0x06);
	
	delay_ms(5000);
	PosConfig();
//	
	//收球电机
	VelCrl(CAN1,COLLECT_BALL_ID,60*4096); 
	
	GetDirection(&adcFlag);
}


//车1定位系统初始化
extern uint8_t sendFlag;
uint8_t isOKFlag;

void PosConfig(void)
{
	while(!isOKFlag)
	{
		delay_ms(1);
		USART_SendData(USART3,'A');
		USART_SendData(USART3,'T');
		USART_SendData(USART3,'\r');
		USART_SendData(USART3,'\n');

	}
	isOKFlag=0;
	while(!sendFlag);
	sendFlag=0;
}




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

// 将角度转换为脉冲
float YawTransform(float yawAngle)
{
	return (yawAngle * YAW_REDUCTION_RATIO * COUNT_PER_DEGREE);
}

//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
void YawAngleCtr(float yawAngle)
{
	PosCrl(CAN1, GUN_YAW_ID, ABSOLUTE_MODE, YawTransform(yawAngle));
}
// 同样要配置位置环
