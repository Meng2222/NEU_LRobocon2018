/**
  ******************************************************************************
  * @file    .h
  * @author  ACTION_2017
  * @version V0.0.0._alpha
  * @date    2017//
  * @brief   This file contains all the functions prototypes for 
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOVEBASE_H
#define __MOVEBASE_H


/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  
  */
/**
* @brief  PID控制参数数据结构体
* @author 陈昕炜
* @note   定义一个PID_Value类型，此时PID_Value为一个数据类型标识符，数据类型为结构体
*/
typedef struct{
	float setValue;             //系统待调节量的设定值 
    float feedbackValue;        //系统待调节量的反馈值，就是传感器实际测量的值    
     
    float Kp;                   //比例系数
    float Ki;                   //积分系数
    float Kd;                   //微分系数
     
    float error;                //当前偏差
    float error_1;              //前一步的偏差
	float error_sum;            //偏差累计
	float error_def;            //当前偏差与上一偏差的差值
    
    float output;               //PID控制器的输出
    float out_max;              //输出上限
    float out_min;              //输出下限
}PID_Value;


/**
* @brief  直线参数数据结构体
* @author 陈昕炜
* @note   单位米(m)
* @note   直线公式为Ax + By + C = 0
* @note   定义一个Line_Value类型，此时Line_Value为一个数据类型标识符，数据类型为结构体
*/
typedef struct{
	float x_meter;              //转化为米单位 以用于参数计算
	float y_meter;              //转化为米单位 以用于参数计算
	float Line_A;               //A
	float Line_B;               //B
    float Line_C;               //C
	
	float Line_K;               //直线斜率
	float Line_b;               //直线截距
	float Line_Angle;           //直线角度                    范围(-π/2, π/2)	
	float Line_Distance;        //当前点到直线的距离
	float Line_Location;        //当前点相对直线的位置
	
	int Line_Mode;              //直线方向                    Line_Mode = 1时  范围[-π/2, π/2] 
                                //                            Line_Mode = 0时  范围[π/2, 3π/2]
}Line_Value;



/**
* @brief  圆周参数数据结构体
* @author 陈昕炜
* @note   单位米(m)
* @note   圆心到当前位置直线公式为Ax + By + C = 0
* @note   定义一个Round_Value类型，此时Round_Value为一个数据类型标识符，数据类型为结构体
*/
typedef struct{
	float x_meter;              //转化为米单位 以用于参数计算
	float y_meter;              //转化为米单位 以用于参数计算
	float Line_A;               //A
	float Line_B;               //B
    float Line_C;               //C
	
	float Line_K;               //圆心到当前位置直线斜率
	float Line_b;               //圆心到当前位置直线截距
	float Line_Angle;           //圆心到当前位置直线角度      范围(-π/2, π/2)
	
	float Round_Center_x;       //圆心横坐标x          -1.5m
	float Round_Center_y;       //圆心纵坐标y          1.5m
	float Round_Radius;         //半径r                1.0m
	float Round_Distance;       //当前坐标到圆心的距离
	float Round_Location;       //当前坐标到圆弧的距离
	
	int Round_Mode;             //画圆方向                    Line_Mode = 1时  顺时针 
                                //                            Line_Mode = 0时  逆时针
}Round_Value;


/**
* @brief  目标坐标信息结构体
* @author 陈昕炜
* @note   定义一个Target类型，此时Target为一个数据类型标识符，数据类型为结构体
*/
typedef struct{ 
	float x;                    //目标点横坐标
	float y;                    //目标点纵坐标
	float angle;                //目标点角度
}Target;

 
/* Exported constants --------------------------------------------------------*/



/** @defgroup 
  * @{
  */

//#define 

//电机旋转一周的脉冲数
#define COUNTS_PER_ROUND (4096.0f)
//轮子直径（单位：mm）
#define WHEEL_DIAMETER (120.0f)
//调试小车车长（单位：mm）
#define MOVEBASE_LENGTH (492.0f)
//调试小车车宽(单位：mm)
#define MOVEBASE_WIDTH (490.0f)
//轮子宽度（单位：mm）
#define WHEEL_WIDTH (40.0f)
//两个轮子中心距离（单位：mm）
#define WHEEL_TREAD (434.0f)


/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
float Constrain_float_Angle(float angle);
float PID_Operation(PID_Value *p);
void Line_Operation(Line_Value *p);
void Round_Operation(Round_Value *p);

float Go(float baseVelocity);
float Turn(float velocity, float radius, int mode);
float PID_Angle_Operation(float PID_Angle_setValue, float PID_Angle_feedbackValue, PID_Value *Pid);
float PID_Line_Operation(float PID_Angle_feedbackValue, Line_Value *Line, PID_Value *Pid);
float PID_Round_Operation(float PID_Angle_feedbackValue, Round_Value *Round, PID_Value *Pid);

#endif /* ___H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

