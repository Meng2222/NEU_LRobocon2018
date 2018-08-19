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
#define Pi 3.1416                                 //π值                    3.1416
#define One_Meter_Per_Second (10865.0)            //车轮一米每秒的设定值   4096*(1000/120π)
#define BaseVelocity (1.0)                        //基础速度               1.0m/s

#define Round_Center_x (-1.5)                     //圆心横轴坐标x
#define Round_Center_y (1.5)                      //圆心纵轴坐标y
#define Round_Radius (1.0)                        //圆半径
#define Round_Compensation_Value 0.1              //圆半径补偿值 实际半径 = 圆半径 - 圆半径补偿值

#define Distance_coefficient (0.8)                //距离敏感度             数值越大 靠近直线越快 转向直线方向越慢，数值越小 靠近直线越慢 转向直线方向越快
#define PID_Round_Angle_Kp (0.03)                 //画圆闭环PID Kp参数     0.03
#define PID_Round_Angle_Ki (0.0)                  //画圆闭环PID Ki参数     0.0
#define PID_Round_Angle_Kd (0.001)                //画圆闭环PID Kd参数     0.001


/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

Pos_t pos;
extern Pos_t posTmp;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];

#if CarNum == CarOne
int isOKFlag = 0;    //定位系统初始化完毕标志位
uint8_t opsFlag = 0; //收到定位系统数据标志位

//一直以5ms间隔发“AT/r/n”给定位系统，等待定位系统回数 OK定位系统初始化结束
int IsSendOK(void)
{
	return isOKFlag;
}
void SetOKFlagZero(void)
{
	isOKFlag = 0;
}
void driveGyro(void)
{
	while(!IsSendOK())
	{
		delay_ms(5);
		USART_SendData(USART3, 'A');
		USART_SendData(USART3, 'T');
		USART_SendData(USART3, '\r');
		USART_SendData(USART3, '\n');
	}
	SetOKFlagZero();
}
#endif


/**
* @brief  PID控制参数数据结构体
* @author 陈昕炜
* @note   定义一个PID_Value类型，此时PID_Value为一个数据类型标识符，数据类型为结构体
*/
typedef struct{
	float setValue;      //系统待调节量的设定值 
    float feedbackValue; //系统待调节量的反馈值，就是传感器实际测量的值    
     
    float Kp;            //比例系数
    float Ki;            //积分系数
    float Kd;            //微分系数
     
    float error;         //当前偏差
    float error_1;       //前一步的偏差
	float error_sum;     //偏差累计
	float error_def;     //当前偏差与上一偏差的差值
    
    float output;        //PID控制器的输出
    float out_max;       //输出上限
    float out_min;       //输出下限
}PID_Value;
PID_Value PID_Round_Angle;

/**
* @brief  PID控制器
* @param  *p 要进行计算输出的PID控制数据结构体指针
* @author 陈昕炜
* @note   用于PID中
*/
float PID_operation(PID_Value *p)
{
//  p->error = p->setValue - p->feedbackValue;
	p->error_sum = p->error_sum + p->error;
	p->error_def = p->error - p->error_1;
	p->error_1 = p->error;
	p->output = p->Kp * p->error + p->Ki * p->error_sum + p->Kd * p->error_def;
	return p->output;
}

/**
* @brief  浮点数限幅
* @param  amt：需要进行限幅的数
* @param  high：输出上限
* @param  low：输出下限
* @author 陈昕炜
* @note   constrain ->约束，限制
*/
float constrain_float(float amt, float high, float low) 
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**
* @brief  转速差实现转弯函数
* @param  speed：两车轮圆心连线中点的移动速度 单位米
* @param  radius：两车轮圆心连线中点的转弯半径 单位米
* @param  mode：转弯模式 1为右转弯 2为左转弯 3为向前直走
* @param  slowwheel：内侧轮的设定值
* @param  fastwheel：外侧轮的设定值
* @author 陈昕炜
* @note   用于WalkTask中
*/
void Go(float velocity, float radius, char mode)
{
	float slowwheel, fastwheel;
	//设定值 = 速度 * ( 转弯半径 - 车轮距离的一半）/ 转弯半径 * 车轮一米每秒的设定值
	slowwheel = velocity * (1000 * radius - WHEEL_TREAD / 2) / (1000 * radius) * One_Meter_Per_Second;
    fastwheel = velocity * (1000 * radius + WHEEL_TREAD / 2) / (1000 * radius) * One_Meter_Per_Second;
	if(mode == 1)//右转弯
	{
		VelCrl(CAN2, 1, (int)slowwheel);
		VelCrl(CAN2, 2, (int)(fastwheel * -1));
	}
	if(mode == 2)//左转弯
	{
		VelCrl(CAN2, 1, (int)fastwheel);
		VelCrl(CAN2, 2, (int)(slowwheel * -1));
	}
	if(mode == 3)//直走
	{
		VelCrl(CAN2, 1, (int)(velocity * One_Meter_Per_Second));
		VelCrl(CAN2, 2, (int)(velocity * One_Meter_Per_Second * -1));
	}
}

/**
* @brief  角度限幅函数
* @param  angle：输入的角度值 单位°
* @author 陈昕炜
* @note   用于WalkTask中
*/
float Constrain_float_Angle(float angle)
{
	if(angle > 180)
	{
		angle = angle - 360;
	}
	if(angle < -180)
	{
		angle = angle + 360;
	}
	return angle;
}


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
	
	TIM_Init(TIM2, 999, 83, 0, 0);
	USART3_Init(115200);
	UART4_Init(921600);
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);
	
	ElmoInit(CAN2);
	VelLoopCfg(CAN2, 1, 53333333, 53333333);
	VelLoopCfg(CAN2, 2, 53333333, 53333333);
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	
	//定位系统初始化延时
    #if CarNum == CarOne        
		delay_s(2);
		driveGyro();
		while(!opsFlag);
    #elif CarNum == CarFour
		delay_s(10);
		delay_s(5);
    #endif
	OSTaskSuspend(OS_PRIO_SELF);
}


void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	int cntSendTime = 0, Round_Mode = 0;
	float adjustVelocityRoundAngle;
	float Line_A, Line_B, Line_C, x_meter, y_meter;
	float Line_Angle, Line_Distance, Point_Location, Line_K, Line_b;

	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		
		//将四号车的坐标系转化为一号车的标准坐标系
		#if CarNum == CarOne
		    pos.x = posTmp.y;
		    pos.y = posTmp.x * -1;
		    pos.angle = posTmp.angle * -1;
		#elif CarNum == CarFour
		    pos.x = posTmp.x;
		    pos.y = posTmp.y;
		    pos.angle = posTmp.angle;
        #endif
		
		//使用x_meter, y_meter统一单位，以用于直线相关参数计算	
		x_meter = pos.x / 1000.0;
		y_meter = pos.y / 1000.0;	
		
		//直线相关参数计算
		//直线公式               Line_A            Ax + By + C = 0
		//                       Line_B
		//                       Line_C
		//直线斜率               Line_K            单位m
		//直线截距               Line_b            单位m
		//直线角度               Line_Angle        范围(-π/2, π/2)
		//当前坐标到圆心的距离   Line_Distance     单位m
		//当前坐标到圆弧的距离   Point_Location
        //直线方向               Line_Mode = 1时   范围[-π/2, π/2]
        //                       Line_Mode = 0时   范围[π/2, 3π/2]			

        //求圆心到当前坐标的直线的A、B值
		Line_A = y_meter - Round_Center_y;
		Line_B = Round_Center_x - x_meter;
		//角度换算
		//当直线斜率绝对值小于1时，使用y = kx + b的格式
		if(fabs(Line_A) <= fabs(Line_B))
		{
			Line_K = Line_A / Line_B * -1.0;
			Line_Angle = ((atan(Line_K) * 180.0 / Pi) - 90.0);
		}
		//当直线斜率绝对值大于1时，使用x = ky + b的格式
        if(fabs(Line_A) > fabs(Line_B))
		{
			Line_K = Line_B / Line_A * -1.0;
			//当直线斜率大于0时
			if(atan(Line_K) * 180.0 / Pi >= 0)
			{
				Line_Angle = atan(Line_K) * 180.0 / Pi * -1.0;
			}
			//当直线斜率小于0时
			if(atan(Line_K) * 180.0 / Pi < 0)
			{
				Line_Angle = atan(Line_K) * 180.0 / Pi * -1.0 - 180.0;
			}			
		}
		//当圆心到当前坐标的直线方向为[π/2, 3π/2]时，直线角度 + 180°
		if(x_meter < Round_Center_x)
		{
			Line_Angle = Line_Angle + 180;
		}
        		
		//当前坐标到圆心的距离
		Line_Distance = sqrt((x_meter - Round_Center_x) * (x_meter - Round_Center_x) + (y_meter - Round_Center_y) * (y_meter - Round_Center_y));
		//当前坐标到圆弧的距离
		Point_Location = Line_Distance - (Round_Radius - Round_Compensation_Value);		
        
		//顺时针                     
		if(Round_Mode == 1)
		{
			//在圆外时
			if(Point_Location >= 0)
			{
				PID_Round_Angle.setValue = Line_Angle - 90.0 - 90.0 * (1.0 - 1.0 / (fabs(Point_Location) * Distance_coefficient + 1.0));
		    }
			//在圆内时
		    if(Point_Location < 0)
		    {
			    PID_Round_Angle.setValue = Line_Angle - 90.0 + 90.0 * (1.0 - 1.0 / (fabs(Point_Location) * Distance_coefficient + 1.0));
		    }
		}
		//逆时针	
		if(Round_Mode == 0)
		{
			//在圆外时
			if(Point_Location >= 0)
			{
				PID_Round_Angle.setValue = Line_Angle + 90.0 + 90.0 * (1.0 - 1.0 / (fabs(Point_Location) * Distance_coefficient + 1.0));
		    }
			//在圆内时
		    if(Point_Location < 0)
		    {
			    PID_Round_Angle.setValue = Line_Angle + 90.0 - 90.0 * (1.0 - 1.0 / (fabs(Point_Location) * Distance_coefficient + 1.0));
		    }
		}
		
		
		//PID参数赋值
		PID_Round_Angle.Kp = PID_Round_Angle_Kp;
        PID_Round_Angle.Ki = PID_Round_Angle_Ki;
        PID_Round_Angle.Kd = PID_Round_Angle_Kd;
		PID_Round_Angle.setValue = Constrain_float_Angle(PID_Round_Angle.setValue);
		PID_Round_Angle.feedbackValue = pos.angle;
		PID_Round_Angle.error = PID_Round_Angle.setValue - PID_Round_Angle.feedbackValue;
		PID_Round_Angle.error = Constrain_float_Angle(PID_Round_Angle.error);
		adjustVelocityRoundAngle = PID_operation(&PID_Round_Angle);
		
		
		VelCrl(CAN2, 1, (int)((BaseVelocity + adjustVelocityRoundAngle) * One_Meter_Per_Second));//右轮
		VelCrl(CAN2, 2, (int)((BaseVelocity - adjustVelocityRoundAngle) * One_Meter_Per_Second * -1.0));//左轮
		
		
		//以20 * 10ms为间隔发送数据
		cntSendTime++;
		cntSendTime = cntSendTime % 20;
		if(cntSendTime == 1)
		{
            USART_OUT(UART4, (uint8_t*)"x=%d,y=%d,ang=%d,LD=%d,PL=%d,LA=%d,Aset=%d,Aerr=%d,aVLRA=%d\r\n", \
			         (int)pos.x, (int)pos.y, (int)pos.angle, (int)(Line_Distance * 1000), (int)((Point_Location +  Round_Compensation_Value)* 1000), (int)Line_Angle, (int)PID_Round_Angle.setValue, (int)PID_Round_Angle.error, (int)(adjustVelocityRoundAngle * 1000));
		}
		OSSemSet(PeriodSem, 0, &os_err);
	}
}
