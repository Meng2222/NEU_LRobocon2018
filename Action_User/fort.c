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
#include "PID.h"

//对应的收发串口
#define USARTX UART5


#define Yaw_Zero_Offset (170.0f)                                //炮台角度指向车头方向置零    170°
#define Fort_TO_BACK_WHEEL (65.0f)                              //炮台到车轴中点距离          65.0mm
#define BACK_WHEEL_TO_WALL (95.0f)                              //车轴中点到墙面距离          95.0mm
#define G (9800.0f)                                             //重力加速度                  9800mm/s2
#define Fort_Elevation_Deg (60.0f)                              //炮台仰角                    60°
#define Fort_Elevation_Rad (Fort_Elevation_Deg * Pi / 180.0)    //炮台仰角                    1/3πrad
#define Fort_Height (150.0f)                                    //炮台高度                    150.0mm
#define Bucket_Height (800.0f)                                  //桶高度                      800.0mm
#define Fort_To_Bucket_Height (Bucket_Height - Fort_Height)     //炮台到桶高度差              (800.0 - 150.0)mm

FortType fort;
GunneryData gundata;
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
		if(bufferI > 2 &&  strncmp(buffer,"PO",2) == 0)//接收航向位置
		{
				for(int i = 0; i < 4; i++)
					fort.usartReceiveData.data8[i] = buffer[i + 2];
				fort.yawPosReceive = fort.usartReceiveData.dataFloat;
		}
		else if(bufferI > 2 &&  strncmp(buffer,"VE",2) == 0)//接收发射电机转速
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
* @brief  计算射击诸元
* @param  Square_Mode：顺/逆时针模式
* @param  BucketNum：目标桶编号
* @param  GunneryData *Gun：射击诸元结构体指针
* @param  pos_t *pos: 定位系统数据结构体指针
* @author 陈昕炜
* @note   在使用该函数之前需输入当前速度的检测值 单位mm/s
*/
void GunneryData_Operation(PID_Value *pos,GunneryData *Gun)
{
	//炮台坐标补偿
	Gun->Fort_x = pos->X - (Fort_TO_BACK_WHEEL) * sin(pos->Angle * Pi / 180.0);
	Gun->Fort_y = pos->Y + (Fort_TO_BACK_WHEEL) * cos(pos->Angle * Pi / 180.0) + BACK_WHEEL_TO_WALL;
	Gun->Fort_angle = Constrain_float_Angle(Yaw_Zero_Offset - fort.yawPosReceive + pos->Angle);
	
	//以直线方向为y轴正向建立坐标系，计算炮台到目标点的横轴坐标相对距离
	switch(Gun->BucketNum)
	{
		case 0:
			if(pos->direction == ACW)
			{
				Gun->Distance_X = fabs(Gun->Fort_y - 200.0);
				Gun->Distance_Y = fabs(Gun->Fort_x - 2200.0);
			}
			if(pos->direction == CW)
			{
				Gun->Distance_X = fabs(Gun->Fort_x - 2200.0);
				Gun->Distance_Y = fabs(Gun->Fort_y - 200.0);				
			}
			break;
		case 1:
			if(pos->direction == ACW)
			{
				Gun->Distance_X = fabs(Gun->Fort_x - 2200.0);
				Gun->Distance_Y = fabs(Gun->Fort_y - 4600.0);
			}
			if(pos->direction == CW)
			{
				Gun->Distance_X = fabs(Gun->Fort_y - 4600.0);
				Gun->Distance_Y = fabs(Gun->Fort_x - 2200.0);			
			}
			break;		
		case 2:
			if(pos->direction == ACW)
			{
				Gun->Distance_X = fabs(Gun->Fort_y - 4600.0);
				Gun->Distance_Y = fabs(Gun->Fort_x + 2200.0);
			}
			if(pos->direction == CW)
			{
				Gun->Distance_X = fabs(Gun->Fort_x + 2200.0);		
				Gun->Distance_Y = fabs(Gun->Fort_y - 4600.0);
			}
			break;		
		case 3:
			if(pos->direction == ACW)
			{
				Gun->Distance_X = fabs(Gun->Fort_x + 2200.0);
				Gun->Distance_Y = fabs(Gun->Fort_y - 200.0);
			}
			if(pos->direction == CW)
			{
				Gun->Distance_X = fabs(Gun->Fort_y - 200.0);
				Gun->Distance_Y = fabs(Gun->Fort_x + 2200.0);
			}
			break;
	}
	
	Gun->Distance_Car_Y = 0;                    //初始化射球飞行时间中车移动的距离
	Gun->Distance_Shoot_X = Gun->Distance_X;    //初始化射球实际移动的横坐标距离为炮台到目标点的横坐标距离
	Gun->Distance_Shoot_Y = Gun->Distance_Y;    //初始化射球实际移动的纵坐标距离为炮台到目标点的纵坐标距离
	//当前迭代只考虑直线方向的车速度
	do
	{
		//计算射球实际移动的纵坐标距离 = 炮台到桶纵坐标距离 - 射球飞行时间中车移动的距离
		Gun->Distance_Shoot_Y = Gun->Distance_Y - Gun->Distance_Car_Y;
		//计算炮台偏向角 = arctan(射球实际移动的横坐标距离 / 射球实际移动的纵坐标距离)
		Gun->YawPosTarAngle = atan(Gun->Distance_Shoot_X / Gun->Distance_Shoot_Y) * 180.0 / Pi;
		
		//计算射球实际移动距离
		Gun->Distance_Fort = sqrt(Gun->Distance_Shoot_X * Gun->Distance_Shoot_X + Gun->Distance_Shoot_Y * Gun->Distance_Shoot_Y);
		//计算射球水平速度
		Gun->ShooterVelSetA = sqrt((Gun->Distance_Fort * Gun->Distance_Fort * G) / (2.0 * (Gun->Distance_Fort * tan(Fort_Elevation_Rad) - Fort_To_Bucket_Height)));
		//计算射球飞行时间
		Gun->ShooterTime = Gun->Distance_Fort / Gun->ShooterVelSetA;
		//计算射球飞行时间中车移动的距离
		Gun->Distance_Car_Y = Gun->detVel * Gun->ShooterTime;
		//计算射球飞行时间中车移动的距离 加上 射球实际移动纵坐标距离 与 炮台到目标点的纵坐标距离 的差值
		Gun->Distance_Deviation = fabs(Gun->Distance_Shoot_Y + Gun->Distance_Car_Y - Gun->Distance_Y);
	}
	//当差值大于精度要求时，继续迭代
	while(Gun->Distance_Deviation > Gun->Distance_Accuracy);
    //根据射球电机转速与静止炮台到桶距离经验公式计算
	Gun->ShooterVelSet = 40.0 / 3102.505 * Gun->Distance_Fort + 35.7 + Gun->Shooter_Vel_Offset;
	Gun->YawPosTarAngle = Gun->YawPosTarAngle + Gun->Yaw_Angle_Offset;
	
	if(pos->direction == CW)
	{
		Gun->YawPosTarAngle = Gun->YawPosTarAngle * -1.0;
	}
	//设定炮台航向角
	switch(Gun->BucketNum)
	{
		case 0:
			if(pos->direction == ACW)
			{
				Gun->YawPosTarActAngle = Yaw_Zero_Offset + (pos->Angle + 90.0) + Gun->YawPosTarAngle;
			}
			if(pos->direction == CW)
			{
				if(pos->Angle > 0)
				{
					Gun->YawPosTarActAngle = Yaw_Zero_Offset + (pos->Angle - 180.0) + Gun->YawPosTarAngle;
				}
				if(pos->Angle < 0)
				{
					Gun->YawPosTarActAngle = Yaw_Zero_Offset + (pos->Angle + 180.0) + Gun->YawPosTarAngle;
				}
			}
			break;
		case 1:
			if(pos->direction == ACW)
			{
				Gun->YawPosTarActAngle = Yaw_Zero_Offset + (pos->Angle - 0) + Gun->YawPosTarAngle;
			}
			if(pos->direction == CW)
			{
				Gun->YawPosTarActAngle = Yaw_Zero_Offset + (pos->Angle + 90.0) + Gun->YawPosTarAngle;				
			}
			break;		
		case 2:
			if(pos->direction == ACW)
			{
				Gun->YawPosTarActAngle = Yaw_Zero_Offset + (pos->Angle - 90.0) + Gun->YawPosTarAngle;
			}
			if(pos->direction == CW)
			{
				Gun->YawPosTarActAngle = Yaw_Zero_Offset + (pos->Angle - 0) + Gun->YawPosTarAngle;				
			}
			break;		
		case 3:
			if(pos->direction == ACW)
			{
				if(pos->Angle > 0)
				{
					Gun->YawPosTarActAngle = Yaw_Zero_Offset + (pos->Angle - 180.0) + Gun->YawPosTarAngle;
				}
				if(pos->Angle < 0)
				{
					Gun->YawPosTarActAngle = Yaw_Zero_Offset + (pos->Angle + 180.0) + Gun->YawPosTarAngle;
				}
			}
			if(pos->direction == CW)
			{
				Gun->YawPosTarActAngle = Yaw_Zero_Offset + (pos->Angle - 90.0) + Gun->YawPosTarAngle;				
			}
			break;
	}
}




