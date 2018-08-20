/**
  ******************************************************************************
  * @file	  moveBase.c
  * @author	  Action
  * @version   V1.0.0
  * @date	  2018/08/09
  * @brief	 2018省赛底盘运动控制部分
  ******************************************************************************
  * @attention
  *			None
  ******************************************************************************
  */
/* Includes -------------------------------------------------------------------------------------------*/

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

/* Private typedef ------------------------------------------------------------------------------------*/
PID_Value PID_Angle, PID_Line, PID_Round;
Line_Value Line;
Round_Value Round;
extern Pos_t pos;
/* Private define -------------------------------------------------------------------------------------*/
#define Pi 3.1416                           //π值                    3.1416
#define One_Meter_Per_Second (10865.0)      //车轮一米每秒的设定值   4096*(1000/120π)
#define Distance_coefficient (0.8)          //距离敏感度             数值越大 靠近直线越快 转向直线方向越慢，数值越小 靠近直线越慢 转向直线方向越快

#define PID_Angle_Kp (0.02)                 //角度闭环PID Kp参数     0.02
#define PID_Angle_Ki (0.0)                  //角度闭环PID Ki参数     0.0
#define PID_Angle_Kd (0.001)                //角度闭环PID Kd参数     0.001

#define PID_Line_Kp (0.025)                 //直线闭环PID Kp参数     0.025
#define PID_Line_Ki (0.0)                   //直线闭环PID Ki参数     0.0
#define PID_Line_Kd (0.001)                 //直线闭环PID Kd参数     0.001
#define Side_Length (2000)                  //方形边长               2m
#define Advance_Length (618.0)              //切换直线提前量         618.0mm

#define PID_Round_Kp (0.03)                 //画圆闭环PID Kp参数     0.03
#define PID_Round_Ki (0.0)                  //画圆闭环PID Ki参数     0.0
#define PID_Round_Kd (0.001)                //画圆闭环PID Kd参数     0.001
#define Round_Compensation_Value (0.1)      //圆半径补偿值 实际半径 = 圆半径 - 圆半径补偿值
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/**
* @brief  浮点数限幅
* @param  amt：需要进行限幅的数
* @param  high：输出上限
* @param  low：输出下限
* @author 陈昕炜
* @note   大于上限输出上限值，小于下限输出下限值，处于限幅区间内输出原数
* @note   constrain ->约束，限制
*/
float constrain_float(float amt, float high, float low) 
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


/**
* @brief  角度限幅函数
* @param  angle：输入的角度值 单位度°
* @author 陈昕炜
* @note   用于WalkTask中
*/
float Constrain_float_Angle(float angle)
{
	if(angle > 180.0)
	{
		angle = angle - 360.0;
	}
	if(angle < -180.0)
	{
		angle = angle + 360.0;
	}
	return angle;
}


/**
* @brief  PID控制器
* @param  *p 要进行计算输出的PID控制数据结构体指针
* @author 陈昕炜
* @note   用于PID
*/
float PID_Operation(PID_Value *p)
{
	p->setValue = Constrain_float_Angle(p->setValue);
	
	p->error = p->setValue - p->feedbackValue;
	p->error = Constrain_float_Angle(p->error);
	
	p->error_sum = p->error_sum + p->error;	
	p->error_def = p->error - p->error_1;	
	p->error_1 = p->error;
	
	p->output = p->Kp * p->error + p->Ki * p->error_sum + p->Kd * p->error_def;
	return p->output;
}

/**
* @brief  直线闭环数据处理
* @param  *p 要进行计算的直线数据结构体指针
* @author 陈昕炜
* @note   用于直线闭环
*/
void Line_Operation(Line_Value *p)
{
	//使用x_meter, y_meter统一单位，以用于直线相关参数计算	
	p->x_meter = pos.x / 1000.0;
    p->y_meter = pos.y / 1000.0;
	
	
	//当直线斜率绝对值小于1时，使用y = kx + b的格式
	if(fabs(p->Line_A) <= fabs(p->Line_B))
	{
		p->Line_K = p->Line_A / p->Line_B * -1.0;
		p->Line_b = p->Line_C / p->Line_B * -1.0;
		//角度换算
		p->Line_Angle = ((atan(p->Line_K) * 180.0 / Pi) - 90.0);
		p->Line_Distance = ((p->Line_K * p->x_meter - p->y_meter + p->Line_b) / sqrt(p->Line_K * p->Line_K + 1.0));
		//大于0时点在直线的下方，小于0时点在直线上方
		p->Line_Location = (p->Line_K * p->x_meter + p->Line_b - p->y_meter);
	}
	
	
	//当直线斜率绝对值大于1时，使用x = ky + b的格式
    if(fabs(p->Line_A) > fabs(p->Line_B))
	{
		p->Line_K = p->Line_B / p->Line_A * -1.0;
		p->Line_b = p->Line_C / p->Line_A * -1.0;
		//角度换算
		//当直线斜率大于0时
		if(atan(p->Line_K) * 180.0 / Pi >= 0)
		{
			p->Line_Angle = atan(p->Line_K) * 180.0 / Pi * -1.0;
		}
		//当直线斜率小于0时
		if(atan(p->Line_K) * 180.0 / Pi < 0)
		{
			p->Line_Angle = atan(p->Line_K) * 180.0 / Pi * -1.0 - 180.0;
		}	
		p->Line_Distance = ((p->Line_K * p->y_meter - p->x_meter + p->Line_b) / sqrt(p->Line_K * p->Line_K + 1.0));
		//大于0时点在直线的右方，小于0时点在直线左方
		p->Line_Location = (p->x_meter - p->Line_K * p->y_meter - p->Line_b);
	}   	
}

/**
* @brief  画圆闭环数据处理
* @param  *p 要进行计算的画圆数据结构体指针
* @author 陈昕炜
* @note   用于画圆闭环
*/
void Round_Operation(Round_Value *p)
{
	//使用x_meter, y_meter统一单位，以用于直线相关参数计算	
	p->x_meter = pos.x / 1000.0;
	p->y_meter = pos.y / 1000.0;
	
	//求圆心到当前坐标的直线的A、B值
	p->Line_A = p->y_meter - p->Round_Center_y;
	p->Line_B = p->Round_Center_x - p->x_meter;
	
	//角度换算
	//当直线斜率绝对值小于1时，使用y = kx + b的格式
	if(fabs(p->Line_A) <= fabs(p->Line_B))
	{
		p->Line_K = p->Line_A / p->Line_B * -1.0;
		p->Line_Angle = ((atan(p->Line_K) * 180.0 / Pi) - 90.0);
	}
	//当直线斜率绝对值大于1时，使用x = ky + b的格式
    if(fabs(p->Line_A) > fabs(p->Line_B))
	{
		p->Line_K = p->Line_B / p->Line_A * -1.0;
		//当直线斜率大于0时
		if(atan(p->Line_K) * 180.0 / Pi >= 0)
		{
			p->Line_Angle = atan(p->Line_K) * 180.0 / Pi * -1.0;
		}
		//当直线斜率小于0时
		if(atan(p->Line_K) * 180.0 / Pi < 0)
		{
			p->Line_Angle = atan(p->Line_K) * 180.0 / Pi * -1.0 - 180.0;
		}			
	}
	
	//当圆心到当前坐标的直线方向为[π/2, 3π/2]时，直线角度 + 180°
	if(p->x_meter < p->Round_Center_x)
	{
		p->Line_Angle = p->Line_Angle + 180;
	}
        		
	//当前坐标到圆心的距离
	p->Round_Distance = sqrt((p->x_meter - p->Round_Center_x) * (p->x_meter - p->Round_Center_x) + (p->y_meter - p->Round_Center_y) * (p->y_meter - p->Round_Center_y));
	//当前坐标到圆弧的距离
	p->Round_Location = p->Round_Distance - (p->Round_Radius - Round_Compensation_Value);			
}
/* Private functions ----------------------------------------------------------------------------------*/
/**
  * @brief  
  * @note
  * @param  
  * @retval None
  */


/**
* @brief  基础速度
* @param  baseVelocity：开环基础移动速度 单位米
* @author 陈昕炜
* @note   用于WalkTask中
*/
float Go(float baseVelocity)
{
	return baseVelocity * One_Meter_Per_Second;
}


/**
* @brief  开环画圆
* @param  velocity：两车轮圆心连线中点的移动速度 单位米
* @param  radius：两车轮圆心连线中点的转弯半径 单位米
* @param  mode：0为左转弯 1为右转弯
* @param  turnVelocity：开环画圆车速 单位米
* @author 陈昕炜
* @note   用于WalkTask中
*/
float Turn(float velocity, float radius, int mode)
{
	//差速 = 画圆速度 * 车轮距离的一半/ 转弯半径
	float turnVelocity = velocity * (WHEEL_TREAD / 2000.0) / radius ;
	if(mode == 1)//左转弯
	{
		turnVelocity = turnVelocity;
	}
	if(mode == 2)//右转弯
	{
		turnVelocity = turnVelocity * -1.0;
	}
	return turnVelocity * One_Meter_Per_Second;
}


/**
* @brief  角度闭环
* @param  PID_Angle_setValue：目标角度
* @param  PID_Angle_feedbackValue：当前角度
* @param  PID_Value *Pid: 角度PID结构体PID_Angle
* @param  PID_Operation(Pid): 角度PID的控制差速
* @author 陈昕炜
* @note   用于WalkTask中
*/
float PID_Angle_Operation(float PID_Angle_setValue, float PID_Angle_feedbackValue, PID_Value *Pid)
{
	Pid->Kp = PID_Angle_Kp;
	Pid->Ki = PID_Angle_Ki;
	Pid->Kd = PID_Angle_Kd;
	
	Pid->setValue = PID_Angle_setValue;
	Pid->feedbackValue = PID_Angle_feedbackValue;
	return PID_Operation(Pid) * One_Meter_Per_Second;
}

/**
* @brief  直线闭环
* @param  PID_Angle_feedbackValue：当前角度
* @param  Line_Value *Line：直线数据处理结构体Line
* @param  PID_Value *Pid: 直线PID结构体PID_Line
* @param  Line_Mode: 直线方向  
          Line_Mode = 1时   范围[-π/2, π/2]
          Line_Mode = 0时   范围[π/2, 3π/2]
* @param  PID_Operation(Pid): 直线PID的控制差速
* @author 陈昕炜
* @note   用于WalkTask中
*/
float PID_Line_Operation(float PID_Angle_feedbackValue, Line_Value *Line, PID_Value *Pid)
{
	Pid->Kp = PID_Line_Kp;
	Pid->Ki = PID_Line_Ki;
	Pid->Kd = PID_Line_Kd;

	
    //Line_Mode = 1时，直线方向范围[-π/2, π/2]                      
	if(Line->Line_Mode == 1)
	{
		//当点在直线右/下方时，离直线无限远时目标角度为直线方向 + 90°
		if(Line->Line_Location >= 0)
		{
			Pid->setValue = Line->Line_Angle + 90.0 * (1.0 - 1.0 / (fabs(Line->Line_Distance) * Distance_coefficient + 1.0));
		}
		//当点在直线左/上方时，离直线无限远时目标角度为直线方向 - 90°
		if(Line->Line_Location < 0)
		{
			Pid->setValue = Line->Line_Angle - 90.0 * (1.0 - 1.0 / (fabs(Line->Line_Distance) * Distance_coefficient + 1.0));
		}
	}
	// Line_Mode = 0时，直线方向范围[π/2, 3π/2]	
	if(Line->Line_Mode == 0)
	{
		//当点在直线右/下方时，离直线无限远时目标角度为直线方向的另一边(180°)再 - 90°
		if(Line->Line_Location >= 0)
		{
			Pid->setValue = Line->Line_Angle + 180.0 - 90.0 * (1.0 - 1.0 / (fabs(Line->Line_Distance) * Distance_coefficient + 1.0));
		}
		//当点在直线左/上方时，离直线无限远时目标角度为直线方向的另一边(180°)再 + 90°
		if(Line->Line_Location < 0)
		{
			Pid->setValue = Line->Line_Angle + 180.0 + 90.0 * (1.0 - 1.0 / (fabs(Line->Line_Distance) * Distance_coefficient + 1.0));
		}
	}
	Pid->feedbackValue = PID_Angle_feedbackValue;
	return PID_Operation(Pid) * One_Meter_Per_Second;  	
}

/**
* @brief  直线闭环
* @param  PID_Angle_feedbackValue：当前角度
* @param  Line_Value *Round：画圆数据处理结构体Round
* @param  PID_Value *Pid: 画圆PID结构体PID_Round
* @param  Round_Mode: 画圆方向  
          Round_Mode = 1时   顺时针
          Round_Mode = 0时   逆时针
* @param  PID_Operation(Pid): 画圆PID的控制差速
* @author 陈昕炜
* @note   用于WalkTask中
*/
float PID_Round_Operation(float PID_Angle_feedbackValue, Round_Value *Round, PID_Value *Pid)
{
	Pid->Kp = PID_Round_Kp;
	Pid->Ki = PID_Round_Ki;
	Pid->Kd = PID_Round_Kd;
	
	
	//顺时针                     
	if(Round->Round_Mode == 1)
	{
		//在圆外时
		if(Round->Round_Location >= 0)
		{
			Pid->setValue = Round->Line_Angle - 90.0 - 90.0 * (1.0 - 1.0 / (fabs(Round->Round_Location) * Distance_coefficient + 1.0));
		}
		//在圆内时
		if(Round->Round_Location < 0)
		{
			Pid->setValue = Round->Line_Angle - 90.0 + 90.0 * (1.0 - 1.0 / (fabs(Round->Round_Location) * Distance_coefficient + 1.0));
		}
	}
	//逆时针	
	if(Round->Round_Mode == 0)
	{
		//在圆外时
		if(Round->Round_Location >= 0)
		{
			Pid->setValue = Round->Line_Angle + 90.0 + 90.0 * (1.0 - 1.0 / (fabs(Round->Round_Location) * Distance_coefficient + 1.0));
		}
		//在圆内时
		if(Round->Round_Location < 0)
		{
			Pid->setValue = Round->Line_Angle + 90.0 - 90.0 * (1.0 - 1.0 / (fabs(Round->Round_Location) * Distance_coefficient + 1.0));
		}
	}
	Pid->feedbackValue = PID_Angle_feedbackValue;
	return PID_Operation(Pid) * One_Meter_Per_Second;
}
/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
