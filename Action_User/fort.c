/**
  ******************************************************************************
  * @file     
  * @author  Dantemiwa
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @attention
  *  
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/

#include "string.h"
#include "timer.h"
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
#include "pps.h"
#include "moveBase.h"
#include "math.h"
#include "adc.h"
#include "fort.h"

//对应的收发串口
#define USARTX UART5

#define FORT_TO_BACK_WHEEL (10.5f)                              //炮台到后轮两轮轴中点距离    10.5mm
#define G (9800.0f)                                             //重力加速度                  9800mm/s2
#define FORT_ELEVATION_DEG (60.0f)                              //炮台仰角                    60°
#define FORT_ELEVATION_RAD (FORT_ELEVATION_DEG * Pi / 180.0)    //炮台仰角                    1/3πrad
#define FORT_HEIGHT (128.0f + 60.0f)                            //炮台高度                    188.0mm
#define BUCKET_HEIGHT (800.0f)                                  //桶高度                      800.0mm
#define FORT_TO_BUCKET_HEIGHT (BUCKET_HEIGHT - FORT_HEIGHT)     //炮台到桶高度差              (800.0 - 188.0)mm

FortType fort;
GunneryData Gundata;
int bufferI = 0;
char buffer[20] = {0};
/**
* @brief 炮台航向控制
* @param  ang:转台航向角度，范围为0~360度
* @retval none
* @attention none
*/
void YawPosCtrl(float ang)
{
		fort.usartTransmitData.dataFloat = ang;
		USART_SendData(USARTX,'Y');
		USART_SendData(USARTX,'A');
		USART_SendData(USARTX,fort.usartTransmitData.data8[0]);
		USART_SendData(USARTX,fort.usartTransmitData.data8[1]);
		USART_SendData(USARTX,fort.usartTransmitData.data8[2]);
		USART_SendData(USARTX,fort.usartTransmitData.data8[3]);
		USART_SendData(USARTX,'\r');
		USART_SendData(USARTX,'\n');
		fort.usartTransmitData.data32 = 0;
		
}

/**
* @brief 发射电机转速控制
* @param  rps:发射电机速度，单位转每秒
* @retval none
* @attention none
*/
void ShooterVelCtrl(float rps)
{
		fort.usartTransmitData.dataFloat = rps;
		USART_SendData(USARTX,'S');
		USART_SendData(USARTX,'H');
		USART_SendData(USARTX,fort.usartTransmitData.data8[0]);
		USART_SendData(USARTX,fort.usartTransmitData.data8[1]);
		USART_SendData(USARTX,fort.usartTransmitData.data8[2]);
		USART_SendData(USARTX,fort.usartTransmitData.data8[3]);
		USART_SendData(USARTX,'\r');
		USART_SendData(USARTX,'\n');
		fort.usartTransmitData.data32 = 0;
}


void bufferInit()
{
	for(int i = 0; i < 20; i++)
	{
		buffer[i] = 0;
	}
	bufferI = 0;
}

/**
* @brief 接收炮台返回的数据
* @param data：串口每次中断接收到的一字节数据
* @retval none
* @attention 该函数请插入到对应的串口中断中
							注意清除标志位
*/
void GetValueFromFort(uint8_t data)
{
	buffer[bufferI] = data;
	bufferI++;
	if(bufferI >= 20)
	{
		bufferInit();
	}
	if(buffer[bufferI - 2] == '\r' && buffer[bufferI - 1] == '\n')
	{ 
		if(bufferI > 2 && strncmp(buffer,"PO",2) == 0)//接收航向位置
		{
				for(int i = 0; i < 4; i++)
					fort.usartReceiveData.data8[i] = buffer[i + 2];
				fort.yawPosReceive = fort.usartReceiveData.dataFloat;
		}
		else if(bufferI > 2 && strncmp(buffer,"VE",2) == 0)//接收发射电机转速
		{
				for(int i = 0; i < 4; i++)
					fort.usartReceiveData.data8[i] = buffer[i + 2];
				fort.shooterVelReceive = fort.usartReceiveData.dataFloat;
		}
		else if(bufferI > 2 && strncmp(buffer,"LA",2) == 0)//接收A激光的ADC值
		{
			for(int i = 0; i < 4; i++)
					fort.usartReceiveData.data8[i] = buffer[i + 2];
				fort.laserAValueReceive = fort.usartReceiveData.dataFloat;
		}
		else if(bufferI > 2 && strncmp(buffer,"LB",2) == 0)//接收B激光的ADC值
		{
			for(int i = 0; i < 4; i++)
					fort.usartReceiveData.data8[i] = buffer[i + 2];
				fort.laserBValueReceive = fort.usartReceiveData.dataFloat;
		}
		bufferInit();
	}
}

/**
* @brief  定位系统坐标系角度限幅函数
* @param  angle：输入的角度值 单位度°
* @author 陈昕炜
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
* @brief  计算射击诸元
* @param  Square_Mode：        方形闭环方向
          Square_Mode = 1时    顺时针
          Square_Mode = 0时    逆时针
* @param  *Gun：               射击诸元结构体指针
* @param  *Pos:                定位系统数据结构体指针
* @author 陈昕炜
* @note   
*/
void GunneryData_Operation(GunneryData *Gun, PID_Value *Pos)
{
	//读取炮台航向角和射球电机转速	
	Gun->YawPosAngleRec = fort.yawPosReceive;
	Gun->ShooterVelRec = fort.shooterVelReceive;
	
	if(Pos->direction == ACW){Gun->Square_Mode = 0;}
	if(Pos->direction == CW){Gun->Square_Mode = 1;}
	
	//读取炮台激光探测到的距离值
	Gun->Distance_LaserA = 2.5114 * fort.laserAValueReceive + 51.623;
	Gun->Distance_LaserB = 2.4269 * fort.laserBValueReceive + 371.39;
	
	//炮台坐标系补偿
	Gun->Fort_X = Pos->X - (FORT_TO_BACK_WHEEL) * sin(Pos->Angle * Pi / 180.0);
	Gun->Fort_Y = Pos->Y + (FORT_TO_BACK_WHEEL) * cos(Pos->Angle * Pi / 180.0);
	Gun->Fort_Angle = Constrain_float_Angle(Pos->Angle - fort.yawPosReceive + Gun->Yaw_Zero_Offset);
	
	//初始化目标桶到炮台坐标的横纵轴距离和距离值
	Gun->Distance_Fort_X = Gun->Bucket_X[Gun->BucketNum] - Gun->Fort_X;
	Gun->Distance_Fort_Y = Gun->Bucket_Y[Gun->BucketNum] - Gun->Fort_Y;
	Gun->Distance_Fort = sqrt(Gun->Distance_Fort_X * Gun->Distance_Fort_X + Gun->Distance_Fort_Y * Gun->Distance_Fort_Y);
	
	//初始化车移动的横纵轴距离
	Gun->Distance_Car_X = 0;
	Gun->Distance_Car_Y = 0;
	
	//初始化射球实际移动的横纵轴距离为炮台到目标点的横纵轴距离
	Gun->Distance_Shoot_X = Gun->Distance_Fort_X;
	Gun->Distance_Shoot_Y = Gun->Distance_Fort_Y;
	
	//初始化迭代计算次数
	Gun->cntIteration = 0;
	
	//全方位全向速度补偿迭代
	do
	{
		//迭代次数计数
		Gun->cntIteration++;
		
		//计算射球实际移动的横纵轴距离和距离
		Gun->Distance_Shoot_X = Gun->Distance_Fort_X - Gun->Distance_Car_X;
		Gun->Distance_Shoot_Y = Gun->Distance_Fort_Y - Gun->Distance_Car_Y;
		Gun->Distance_Shoot = sqrt(Gun->Distance_Shoot_X * Gun->Distance_Shoot_X + Gun->Distance_Shoot_Y * Gun->Distance_Shoot_Y);
		
		//计算射球水平速度和飞行时间
		Gun->ShooterVelSet_H = sqrt((Gun->Distance_Shoot * Gun->Distance_Shoot * G) / (2.0 * (Gun->Distance_Shoot * tan(FORT_ELEVATION_RAD) - FORT_TO_BUCKET_HEIGHT)));
		Gun->ShooterTime = Gun->Distance_Shoot / Gun->ShooterVelSet_H;
		
		//计算射球飞行时间中车移动的横轴轴距离和距离
		Gun->Distance_Car_X = Pos->X_Speed * Gun->ShooterTime;		
		Gun->Distance_Car_Y = Pos->Y_Speed * Gun->ShooterTime;
		Gun->Distance_Car = sqrt(Gun->Distance_Car_X * Gun->Distance_Car_X + Gun->Distance_Car_Y * Gun->Distance_Car_Y);

		//计算当前计算值与目标桶的在横轴轴上偏差值
		//射球飞行时间中车移动的距离 + 射球实际移动距离 - 炮台到目标点的距离
		Gun->Distance_Deviation_X = fabs(Gun->Distance_Shoot_X + Gun->Distance_Car_X - Gun->Distance_Fort_X);
		Gun->Distance_Deviation_Y = fabs(Gun->Distance_Shoot_Y + Gun->Distance_Car_Y - Gun->Distance_Fort_Y);
	}
	//当差值大于精度要求时，继续迭代
	while((Gun->Distance_Deviation_X > Gun->Distance_Accuracy) || (Gun->Distance_Deviation_Y > Gun->Distance_Accuracy));
	
    //根据射球电机转速与静止炮台到桶距离经验公式计算射球电机转速
	Gun->ShooterVelSet = 0.0133 * Gun->Distance_Shoot + 35.267 + Gun->Shooter_Vel_Offset[Gun->BucketNum + Gun->Square_Mode * 4];
	Gun->No_Offset_ShooterVel = 0.0133 * Gun->Distance_Fort + 35.267 + Gun->Shooter_Vel_Offset[Gun->BucketNum + Gun->Square_Mode * 4];
	
	//计算炮台偏向角 = arctan(射球实际移动的横轴距离 / 射球实际移动的纵轴距离)
	Gun->YawPosAngleTar = atan(Gun->Distance_Shoot_X / Gun->Distance_Shoot_Y) * 180.0 / Pi;
	Gun->No_Offset_Angle = atan(Gun->Distance_Fort_X / Gun->Distance_Fort_Y) * 180.0 / Pi;
	
	//将炮台航向角转化为与定位系统坐标系角度一致	
	switch(Gun->BucketNum)
	{
		case 0:
			Gun->YawPosAngleTar = -180.0 - Gun->YawPosAngleTar;
			Gun->No_Offset_Angle = -180.0 - Gun->No_Offset_Angle;
			break;
		case 1:
			Gun->YawPosAngleTar = Gun->YawPosAngleTar * -1.0;
			Gun->No_Offset_Angle = Gun->No_Offset_Angle * -1.0;
			break;		
		case 2:
			Gun->YawPosAngleTar = Gun->YawPosAngleTar * -1.0;
			Gun->No_Offset_Angle = Gun->No_Offset_Angle * -1.0;
			break;		
		case 3:
			Gun->YawPosAngleTar = 180.0 - Gun->YawPosAngleTar;
			Gun->No_Offset_Angle = 180.0 - Gun->No_Offset_Angle;
			break;
	}
	
	//计算炮台航向角实际值
	Gun->YawPosAngleSet = Constrain_float_Angle(Constrain_float_Angle(Pos->Angle - Gun->YawPosAngleTar + Gun->Yaw_Angle_Offset[Gun->BucketNum + Gun->Square_Mode * 4] + Gun->Yaw_Zero_Offset)\
						- (int)Gun->YawPosAngleRec % 360) + Gun->YawPosAngleRec;
	
	
//	Gun->YawPosAngleSet = Gun->YawPosAngleSet + YawPos_Angle_PID_Operation(Gun->YawPosAngleSet, Gun->YawPosAngleRec, &PID_YawPos_Angle);
//	Gun->ShooterVelSet = Gun->ShooterVelSet + Shooter_Vel_PID_Operation(Gun->ShooterVelSet, Gun->ShooterVelRec, &PID_Shooter_Vel);
}

/**
* @brief  PID控制器
* @param  *PID：    PID参数结构体指针
* @author 陈昕炜
* @note   用于PID控制
*/
float PID_Operation(PID_Value_Fort *PID)
{
	PID->setValue = Constrain_float_Angle(PID->setValue);
	
	PID->error = PID->setValue - PID->feedbackValue;
	PID->error = Constrain_float_Angle(PID->error);
	
	PID->error_sum = PID->error_sum + PID->error;	
	PID->error_def = PID->error - PID->error_pre;	
	PID->error_pre = PID->error;
	
	PID->output = PID->Kp * PID->error + PID->Ki * PID->error_sum + PID->Kd * PID->error_def;
	return PID->output;
}

/**
* @brief  航向电机位置闭环
* @param  YawPosAngle_Tar：    目标航向角度
* @param  YawPosAngle_Rec：    当前航向角度
* @param  *PID：               PID参数结构体指针
* @author 陈昕炜
* @note   用于Fort中
*/
float YawPos_Angle_PID_Operation(float YawPosAngle_Tar, float YawPosAngle_Rec, PID_Value_Fort *PID)
{
	PID->Kp = 1;
	PID->Ki = 0;
	PID->Kd = 0;
	
	PID->setValue = YawPosAngle_Tar;
	PID->feedbackValue = YawPosAngle_Rec;
	return PID_Operation(PID);
}

/**
* @brief  射球电机转速闭环
* @param  Shooter_Vel_Tar：    目标电机转速
* @param  Shooter_Vel_Rec：    当前电机转速
* @param  *PID：               PID参数结构体指针
* @author 陈昕炜
* @note   用于Fort中
*/
float Shooter_Vel_PID_Operation(float Shooter_Vel_Tar, float Shooter_Vel_Rec, PID_Value_Fort *PID)
{
	PID->Kp = 1;
	PID->Ki = 0;
	PID->Kd = 0;
	
	PID->setValue = Shooter_Vel_Tar;
	PID->feedbackValue = Shooter_Vel_Rec;
	return PID_Operation(PID);
}
