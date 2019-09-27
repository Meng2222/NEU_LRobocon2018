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

#define PI 3.14159265359f

/* Includes ------------------------------------------------------------------*/
void OpenLoopLine(double V,float Ang);	//units : ° 
int OpenLoopCircle(float velRotate,float cenX,float cenY);
void CloseLoopLine(float vel, float ward, float exPos, 
							float actPos, float curPosX, float curPosY, float pointOnLineX, float pointOnLineY);
void CloseLoopCircle(float velRotate, float cenX, float cenY,
					float beginWard, float endWard, float exPos,
				    float actPos, float rRotate);
/*
 *关于底盘相关参数的宏定义
 */
//单位：m
#define MOVEBASERADIUS 0.29132f
#define WHEELRADIUS    (0.127/2)f
/* Exported types ------------------------------------------------------------*/
//弧度制和角度制相互转换
#define ANGTORAD(x) (x)/180.0f*3.14159f   
#define RADTOANG(x) (x)/3.14159f*180.0f

//速度单位转换
//轮子半径 0.127/2 m,转一圈为2000个脉冲；
//减速比91/6
//速度为2000脉冲/s时，在国际标准单位制中为 (2*PI*0.127/2)/1s *6/91=0.0263065f

//脉冲/s 转 m/s
#define VELTOSTD(x)   (x)/2000*0.0263065f
//m/s 转 脉冲/s
#define VELTOPULSE(x) (x)/0.0263065f*2000

//加速度单位转换
//加速度为 a 脉冲/s^2时，1s内速度从0加到 a 脉冲/s(国标a/4096*0.3114159f m/s)
#define ACCTOSTD(x)   (x)/2000*0.0263065f
#define ACCTOPULSE(x) (x)/0.0263065f*2000
/** 
  * @brief  
  */
//关于加速度的结构体
typedef struct
{
	float wheel1;
	float wheel2;
	float wheel3;
}motorAcc_t;

//关于三个轮速度的结构体类型
//单位：脉冲/s
typedef struct
{
	float v1;
	float v2;
	float v3;
	int status;
}wheelSpeed_t;

 
/* Exported constants --------------------------------------------------------*/
//函数正常运行
#define RETURNOK 		 1
//无效变量
#define IVLDPARM   		 0
//有效变量
#define VALIDPARM 		 1
//变量超出范围
#define OUTOFRANGE 	    -2
/** @defgroup 
  * @{
  */

#define R_BASE  291.32f

//电机旋转一周的脉冲数
#define COUNTS_PER_ROUND (4096)
//轮子直径（单位：mm）
#define WHEEL_DIAMETER (127.0f)
//3508电机减速比，相当于给出去的脉冲要多乘上减速比
#define REDUCTION_RATIO  (91/6)                                             (19.2f)
//
/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

wheelSpeed_t CalcVel(float vel, float ward, float velRotate,float selfPos);




#endif /* ___H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

