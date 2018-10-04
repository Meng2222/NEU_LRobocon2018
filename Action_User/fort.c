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

FortType fort;
GunneryData Gundata;
ScanData Scan;
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
* @brief  浮点数限幅
* @param  amt：     需要进行限幅的数
* @param  high：    输出上限
* @param  low：     输出下限
* @author 陈昕炜
* @note   大于上限输出上限值，小于下限输出下限值，处于限幅区间内输出原数
* @note   constrain -> 约束，限制
*/
float constrain0(float amt, float high, float low)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


/**
* @brief  定位系统坐标系角度限幅函数
* @param  angle：    输入的角度值    单位度°
* @author 陈昕炜
*/
float Constrain_float_Angle(float angle)
{
	while(angle > 180.0f)
	{
		angle = angle - 360.0f;
	}
	while(angle < -180.0f)
	{
		angle = angle + 360.0f;
	}
	return angle;
}


/**
* @brief  计算射击诸元
* @param  *Gun：               射击诸元结构体指针
* @param  *Pos:                定位系统数据结构体指针
* @author 陈昕炜
* @note   
*/
void GunneryData_Operation(GunneryData *Gun, PID_Value const *Pos)
{
	//读取炮台航向角实际值和射球电机转速实际值	
	Gun->YawPosAngleRec = fort.yawPosReceive;
	Gun->ShooterVelRec  = fort.shooterVelReceive;
	
	if(Pos->direction == ACW){Gun->Square_Mode = 0;}
	if(Pos->direction == CW) {Gun->Square_Mode = 1;}
	
	//读取两侧炮台激光探测到的距离
	Gun->Distance_Laser_Left  = 2.4482f * fort.laserAValueReceive + 50.697f;
	Gun->Distance_Laser_Right = 2.4443f * fort.laserBValueReceive + 41.860f;
	
	//炮台坐标系补偿
	Gun->Fort_X = Pos->X - (FORT_TO_BACK_WHEEL) * sin(Pos->Angle * Pi / 180.0f);
	Gun->Fort_Y = Pos->Y + (FORT_TO_BACK_WHEEL) * cos(Pos->Angle * Pi / 180.0f);
	Gun->Fort_Angle = Constrain_float_Angle(Pos->Angle - fort.yawPosReceive + Gun->Yaw_Zero_Offset);
	
	//初始化目标桶到炮台的横纵轴距离和距离
	Gun->Distance_Fort_X = Gun->Bucket_X[Gun->BucketNum] - Gun->Fort_X;
	Gun->Distance_Fort_Y = Gun->Bucket_Y[Gun->BucketNum] - Gun->Fort_Y;
	Gun->Distance_Fort = sqrt(Gun->Distance_Fort_X * Gun->Distance_Fort_X + Gun->Distance_Fort_Y * Gun->Distance_Fort_Y);
	
	//初始化车移动的横纵轴距离
	Gun->Distance_Car_X = 0;
	Gun->Distance_Car_Y = 0;
	
	//初始化射球实际移动的横纵轴距离为炮台到目标桶的横纵轴距离
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
		Gun->ShooterVelSet_H = sqrt((Gun->Distance_Shoot * Gun->Distance_Shoot * G) / (2.0f * (Gun->Distance_Shoot * tan(FORT_ELEVATION_RAD) - FORT_TO_BUCKET_HEIGHT)));
		Gun->ShooterTime = Gun->Distance_Shoot / Gun->ShooterVelSet_H;
		
		//计算射球飞行时间中车移动的横纵轴距离和距离
		Gun->Distance_Car_X = Pos->X_Speed * Gun->ShooterTime;
		Gun->Distance_Car_Y = Pos->Y_Speed * Gun->ShooterTime;
		Gun->Distance_Car = sqrt(Gun->Distance_Car_X * Gun->Distance_Car_X + Gun->Distance_Car_Y * Gun->Distance_Car_Y);
		
		//计算当前计算值与目标桶的在横纵轴上偏差值
		//射球飞行时间中车移动的距离 + 射球实际移动距离 - 炮台到目标桶的距离
		Gun->Distance_Deviation_X = fabs(Gun->Distance_Shoot_X + Gun->Distance_Car_X - Gun->Distance_Fort_X);
		Gun->Distance_Deviation_Y = fabs(Gun->Distance_Shoot_Y + Gun->Distance_Car_Y - Gun->Distance_Fort_Y);
	}
	//当差值大于精度要求时，继续迭代
	while(((Gun->Distance_Deviation_X > Gun->Distance_Accuracy) || (Gun->Distance_Deviation_Y > Gun->Distance_Accuracy)) && (Gun->cntIteration < 10));
	
    //根据射球电机转速与静止炮台到桶距离经验公式计算射球电机转速
	Gun->ShooterVelSet = 0.0136f * Gun->Distance_Shoot + 30.274f + Gun->Shooter_Vel_Offset[Gun->BucketNum + Gun->Square_Mode * 4];
	Gun->No_Offset_Vel = 0.0136f * Gun->Distance_Fort  + 30.274f + Gun->Shooter_Vel_Offset[Gun->BucketNum + Gun->Square_Mode * 4];
	
	//计算炮台航向角目标值
	Gun->YawPosAngleTar  = Tar_Angle_Operation(Gun->Distance_Shoot_X, Gun->Distance_Shoot_Y);
	Gun->No_Offset_Angle = Tar_Angle_Operation(Gun->Distance_Fort_X , Gun->Distance_Fort_Y);
	
	//计算炮台航向角设定值
	Gun->YawPosAngleSet  = Constrain_float_Angle(Constrain_float_Angle(Pos->Angle - Gun->YawPosAngleTar  + Gun->Yaw_Angle_Offset[Gun->BucketNum + Gun->Square_Mode * 4] + Gun->Yaw_Zero_Offset) - Constrain_float_Angle(Gun->YawPosAngleRec))\
						 + Gun->YawPosAngleRec;
	Gun->No_Offset_Angle = Constrain_float_Angle(Constrain_float_Angle(Pos->Angle - Gun->No_Offset_Angle + Gun->Yaw_Angle_Offset[Gun->BucketNum + Gun->Square_Mode * 4] + Gun->Yaw_Zero_Offset) - Constrain_float_Angle(Gun->YawPosAngleRec))\
						 + Gun->YawPosAngleRec;			
	
	/*ZYJ Predictor*/
	if(Pos->direction == ACW)
	{
		Gun->YawPosAngleSetAct = Gun->YawPosAngleSet;
		Gun->YawPosAngleSetAct = constrain0(Gun->YawPosAngleSetAct,90,0);
		if(Gun->YawPosAngleSetAct > 79) Gun->YawPosAngleSetAct = 0;
		Gun->ShooterVelSetAct = Gun->ShooterVelSet;
		Gun->ShooterVelSetAct = constrain0(Gun->ShooterVelSetAct,90,50);
		if(Gun->ShooterVelSetAct < 54) Gun->ShooterVelSetAct = 80;
	}
	else
	{
		Gun->YawPosAngleSetAct = Gun->YawPosAngleSet;
		Gun->YawPosAngleSetAct = constrain0(Gun->YawPosAngleSetAct,0,-90);
		if(Gun->YawPosAngleSetAct < -79) Gun->YawPosAngleSetAct = 0;
		Gun->ShooterVelSetAct = Gun->ShooterVelSet;
		Gun->ShooterVelSetAct = constrain0(Gun->ShooterVelSetAct,90,50);
		if(Gun->ShooterVelSetAct < 54) Gun->ShooterVelSetAct = 80;
	}
}
/**
* @brief  扫描数据处理
* @param  *Scan：              扫描参数结构体指针
* @param  *Pos:                定位系统数据结构体指针
* @param  *Squ：               方形参数结构体指针
* @author 陈昕炜
* @note   用于Fort中
*/
void Scan_Operation(ScanData *Scan, PID_Value const *Pos, int targets[])
{
	//读取炮台航向角实际值
	Scan->YawPosAngleRec = fort.yawPosReceive;
	Scan->ShooterVelRec  = fort.shooterVelReceive;
	
	//炮台坐标系补偿	
	Scan->Fort_X = Pos->X - (FORT_TO_BACK_WHEEL) * sin(Pos->Angle * Pi / 180.0f);
	Scan->Fort_Y = Pos->Y + (FORT_TO_BACK_WHEEL) * cos(Pos->Angle * Pi / 180.0f);
	Scan->Fort_Angle = Constrain_float_Angle(Pos->Angle - fort.yawPosReceive + Scan->Yaw_Zero_Offset);
	
	//扫描时间计时，4s时强制将ScanStatus置为0
	if(Scan->DelayFlag)
	{
		Scan->CntDelayTime++;
		if(Scan->CntDelayTime > 800)
		{
			Scan->DelayFlag = 0;
			Scan->CntDelayTime = 0;
			Scan->ScanStatus = 0;
			Scan->FirePermitFlag = 0;
		}
	}
	
	//ScanStatus = 0航向角转到扫描起始点
	if(Scan->ScanStatus == 0)
	{
		//设定目标桶号
		Scan->BucketNum++;
		Scan->BucketNum = Scan->BucketNum % 4;
//		for(Scan->i = 0; Scan->i < 4; Scan->i++)
//		{
//			if(targets[Scan->i] == 0)
//			{
//				Scan->BucketNum = Scan->i;
//			}
//		}
		
		//炮塔到目标桶挡板边缘横轴距离以及计算指向挡板边缘航向角设定值A、B
		Scan->FortToBorder_Distance_X = Scan->Bucket_Border_X[Scan->BucketNum * 2] - Scan->Fort_X;
		Scan->FortToBorder_Distance_Y = Scan->Bucket_Border_Y[Scan->BucketNum * 2] - Scan->Fort_Y;
		Scan->ScanAngleTar = Tar_Angle_Operation(Scan->FortToBorder_Distance_X, Scan->FortToBorder_Distance_Y);
		Scan->ScanAngleEnd = Constrain_float_Angle(Pos->Angle - Scan->ScanAngleTar + Scan->Yaw_Zero_Offset - Constrain_float_Angle(Scan->YawPosAngleRec)) + Scan->YawPosAngleRec + 10.0f;	

		Scan->FortToBorder_Distance_X = Scan->Bucket_Border_X[Scan->BucketNum * 2 + 1] - Scan->Fort_X;
		Scan->FortToBorder_Distance_Y = Scan->Bucket_Border_Y[Scan->BucketNum * 2 + 1] - Scan->Fort_Y;	
		Scan->ScanAngleTar = Tar_Angle_Operation(Scan->FortToBorder_Distance_X, Scan->FortToBorder_Distance_Y);
		Scan->ScanAngleStart  = Constrain_float_Angle(Pos->Angle - Scan->ScanAngleTar + Scan->Yaw_Zero_Offset - Constrain_float_Angle(Scan->YawPosAngleRec)) + Scan->YawPosAngleRec - 10.0f;		
		
		//航向角设定为位于最左侧的扫描起始点
		Scan->YawPosAngleSet = Scan->ScanAngleStart;	
		Scan->ShooterVelSet = 60.0f;

		if(fabs(Scan->YawPosAngleRec - Scan->ScanAngleStart) < 2.0f)
		{
			Scan->DelayFlag = 1;
			Scan->ScanStatus = 1;
			Scan->ScanPermitFlag = 1;
		}
	}	
	
	
	//ScanStatus = 1扫描状态
	if(Scan->ScanStatus == 1) 
	{  
		if(fabs(Scan->YawPosAngleRec - Scan->ScanAngleStart) < 1.0f)
		{
			Scan->ScanPermitFlag = 1;
		}
		
		if(Scan->ScanPermitFlag == 1)
		{
			//如果在扫描状态时，炮台航向角设定值每10ms增加0.2°
			Scan->YawPosAngleSet = Scan->YawPosAngleSet + 0.2f;
			Scan->ShooterVelSet = 60.0f;
		
			//计算左右侧激光探测距离和探测点横轴坐标
			Scan->Probe_Left_Distance  = 2.4482f * fort.laserAValueReceive + 50.697f;
			Scan->Laser_Left_X  = Scan->Fort_X - (FORT_TO_LASER_X) * sin((Scan->Fort_Angle + 90.0f) * Pi / 180.0f) - (FORT_TO_LASER_Y) * sin(Scan->Fort_Angle * Pi / 180.0f);
			Scan->Laser_Left_Y  = Scan->Fort_Y + (FORT_TO_LASER_X) * cos((Scan->Fort_Angle + 90.0f) * Pi / 180.0f) + (FORT_TO_LASER_Y) * cos(Scan->Fort_Angle * Pi / 180.0f);					
			Scan->Probe_Left_X  = Scan->Laser_Left_X  - Scan->Probe_Left_Distance  * sin(Scan->Fort_Angle * Pi / 180.0f);
			Scan->Probe_Left_Y  = Scan->Laser_Left_Y  + Scan->Probe_Left_Distance  * cos(Scan->Fort_Angle * Pi / 180.0f);	
			
			Scan->Probe_Right_Distance = 2.4443f * fort.laserBValueReceive + 41.860f;
			Scan->Laser_Right_X = Scan->Fort_X - (FORT_TO_LASER_X) * sin((Scan->Fort_Angle - 90.0f) * Pi / 180.0f) - (FORT_TO_LASER_Y) * sin(Scan->Fort_Angle * Pi / 180.0f);
			Scan->Laser_Right_Y = Scan->Fort_Y + (FORT_TO_LASER_X) * cos((Scan->Fort_Angle - 90.0f) * Pi / 180.0f) + (FORT_TO_LASER_Y) * cos(Scan->Fort_Angle * Pi / 180.0f);	
			Scan->Probe_Right_X = Scan->Laser_Right_X - Scan->Probe_Right_Distance * sin(Scan->Fort_Angle * Pi / 180.0f);
			Scan->Probe_Right_Y = Scan->Laser_Right_Y + Scan->Probe_Right_Distance * cos(Scan->Fort_Angle * Pi / 180.0f);		

			
			//识别目标桶两侧挡板边缘，统一为从左往右扫描
			//当右侧激光距离值突然变短，标记左侧挡板边缘坐标点
			if((Scan->Probe_Left_Distance - Scan->Probe_Right_Distance) > 1500.0f)
			{
				if((-3000.0f < Scan->Probe_Right_X && Scan->Probe_Right_X < -1400.0f) || (1400.0f < Scan->Probe_Right_X && Scan->Probe_Right_X < 3000.0f))
				{
					if((-600.0f < Scan->Probe_Right_Y && Scan->Probe_Right_Y < 1000.0f) || (3800.0f < Scan->Probe_Right_Y && Scan->Probe_Right_Y < 5400.0f))
					{
						Scan->Probe_Border_Left_X = Scan->Probe_Right_X;
						Scan->Probe_Border_Left_Y = Scan->Probe_Right_Y;
						Scan->Probe_Border_Left_Angle = Scan->YawPosAngleSet;
					}
				}
			}
			//当右侧激光距离值突然变长，标记右侧挡板边缘坐标点	
			if((Scan->Probe_Right_Distance - Scan->Probe_Left_Distance) > 1500.0f)
			{
				if((-3000.0f < Scan->Probe_Left_X && Scan->Probe_Left_X < -1400.0f) || (1400.0f < Scan->Probe_Left_X && Scan->Probe_Left_X < 3000.0f))
				{
					if((-600.0f < Scan->Probe_Left_Y && Scan->Probe_Left_Y < 1000.0f) || (3800.0f < Scan->Probe_Left_Y && Scan->Probe_Left_Y < 5400.0f))
					{
						Scan->Probe_Border_Right_X = Scan->Probe_Left_X;
						Scan->Probe_Border_Right_Y = Scan->Probe_Left_Y;
						Scan->Probe_Border_Right_Angle = Scan->YawPosAngleSet;
					}
				}
			}
		
		
			//如果炮台航向角设定值大于较大的航向角设定值
			if(Scan->YawPosAngleSet > Scan->ScanAngleEnd)
			{
				if(Scan->Probe_Border_Left_X != Scan->Probe_Border_Left_X_Temp)
				{
					Scan->GetBorderLeftFlag  = 1;
				}
				if(Scan->Probe_Border_Right_X != Scan->Probe_Border_Right_X_Temp)
				{
					Scan->GetBorderRightFlag = 1;
				} 
				
				
				if(Scan->GetBorderLeftFlag == 1 && Scan->GetBorderRightFlag == 1)
				{
					Scan->Probe_Border_Left_X_Temp  = Scan->Probe_Border_Left_X;
					Scan->Probe_Border_Left_Y_Temp  = Scan->Probe_Border_Left_Y;
					Scan->Probe_Border_Right_X_Temp = Scan->Probe_Border_Right_X;	
					Scan->Probe_Border_Right_Y_Temp = Scan->Probe_Border_Right_Y;
					
					Scan->GetBorderLeftFlag  = 0;
					Scan->GetBorderRightFlag = 0;			
					Scan->ScanStatus = 2;
				}
				else
				{
					if(Scan->GetBorderLeftFlag == 0 && Scan->GetBorderRightFlag == 0)
					{
						Scan->ScanAngleStart = Scan->ScanAngleStart - 10.0f;
						Scan->ScanAngleEnd   = Scan->ScanAngleEnd   + 10.0f;
					}
					if(Scan->GetBorderLeftFlag == 0 && Scan->GetBorderRightFlag == 1)
					{				
						Scan->ScanAngleStart = Scan->ScanAngleStart - 10.0f;
						Scan->ScanAngleEnd   = Scan->ScanAngleEnd   - 10.0f;					
					}
					if(Scan->GetBorderLeftFlag == 1 && Scan->GetBorderRightFlag == 0)
					{				
						Scan->ScanAngleStart = Scan->ScanAngleStart + 10.0f;
						Scan->ScanAngleEnd   = Scan->ScanAngleEnd   + 10.0f;					
					}
					Scan->YawPosAngleSet = Scan->ScanAngleStart;
					Scan->ScanPermitFlag = 0;
				}				
			}
		}			
	}

	//ScanStatus = 2射球状态
	if(Scan->ScanStatus == 2)
	{
		//根据标记的两侧挡板边缘坐标值相加除二得到桶的坐标
		switch(Scan->BucketNum)
		{
			case 0:
				Scan->Probe_Bucket_X = (Scan->Probe_Border_Left_X + Scan->Probe_Border_Right_X - 54.0f) / 2.0f;
				Scan->Probe_Bucket_Y = (Scan->Probe_Border_Left_Y + Scan->Probe_Border_Right_Y + 54.0f) / 2.0f;
			case 1:
				Scan->Probe_Bucket_X = (Scan->Probe_Border_Left_X + Scan->Probe_Border_Right_X - 54.0f) / 2.0f;
				Scan->Probe_Bucket_Y = (Scan->Probe_Border_Left_Y + Scan->Probe_Border_Right_Y - 54.0f) / 2.0f;
			case 2:
				Scan->Probe_Bucket_X = (Scan->Probe_Border_Left_X + Scan->Probe_Border_Right_X + 54.0f) / 2.0f;
				Scan->Probe_Bucket_Y = (Scan->Probe_Border_Left_Y + Scan->Probe_Border_Right_Y - 54.0f) / 2.0f;				
			case 3:
				Scan->Probe_Bucket_X = (Scan->Probe_Border_Left_X + Scan->Probe_Border_Right_X + 54.0f) / 2.0f;
				Scan->Probe_Bucket_Y = (Scan->Probe_Border_Left_Y + Scan->Probe_Border_Right_Y + 54.0f) / 2.0f;
		}
		Scan->FortToBucket_Distance_X = Scan->Probe_Bucket_X - Scan->Fort_X;
		Scan->FortToBucket_Distance_Y = Scan->Probe_Bucket_Y - Scan->Fort_Y;
		
		//根据射球电机转速与静止炮台到桶距离经验公式计算射球电机转速
		Scan->Probe_Bucket_Distance = sqrt(Scan->FortToBucket_Distance_X * Scan->FortToBucket_Distance_X + Scan->FortToBucket_Distance_Y * Scan->FortToBucket_Distance_Y);
		Scan->ShooterVelSet = 0.0136f * Scan->Probe_Bucket_Distance + 30.274f + Scan->Shooter_Vel_Offset;
		
		//计算炮台航向角目标值和设定值
		Scan->YawPosAngleTar = Tar_Angle_Operation(Scan->FortToBucket_Distance_X, Scan->FortToBucket_Distance_Y);
		Scan->YawPosAngleSet = Constrain_float_Angle(Pos->Angle - Scan->YawPosAngleTar + Scan->Yaw_Zero_Offset + Scan->YawPosAngle_Offset - Constrain_float_Angle(Scan->YawPosAngleRec)) + Scan->YawPosAngleRec;	

		//到达航向角和射球电机转速设定值时允许开火
		if((fabs(Scan->YawPosAngleRec - Scan->YawPosAngleSet) < 1.0f) && (fabs(Scan->ShooterVelRec - Scan->ShooterVelSet) < 1.0f))
		{
			Scan->FirePermitFlag = 1;
		}
		
		//判断数组0是否变1
		if(targets[Scan->i] == 1)
		{
			Scan->CntDelayTime = 750;
		}
	}
}


/**
* @brief  计算当前点指向目标点的角度（按定位系统坐标系角度规定）
* @param  Distance_X：		目标点到当前坐标的横轴距离（目标点横坐标 - 当前点横坐标）
* @param  Distance_Y：		目标点到当前坐标的纵轴距离（目标点纵坐标 - 当前点纵坐标）
* @param  Tar_Quadrant：	目标点位于的象限
* @param  Tar_Angle：		当前点指向目标点的角度
* @author 陈昕炜
* @note   用于Fort中
*/
float Tar_Angle_Operation(float Distance_X, float Distance_Y)
{
	int Tar_Quadrant;
	//未转化的炮台航向角目标值
	float Tar_Angle = atan(Distance_X / Distance_Y) * 180.0f / Pi;
	
	//判断目标桶位于的象限
	if(Distance_X > 0 && Distance_Y > 0) {Tar_Quadrant = 1;}
	if(Distance_X < 0 && Distance_Y > 0) {Tar_Quadrant = 2;}	
	if(Distance_X < 0 && Distance_Y < 0) {Tar_Quadrant = 3;}	
	if(Distance_X > 0 && Distance_Y < 0) {Tar_Quadrant = 4;}

	//转换为定位系统坐标系角度规定的炮台航向角目标值
	switch(Tar_Quadrant)
	{
		case 1:
			Tar_Angle = Tar_Angle * -1.0f;
			break;		
		case 2:
			Tar_Angle = Tar_Angle * -1.0f;
			break;		
		case 3:
			Tar_Angle = 180.0f - Tar_Angle;
			break;
		case 4:
			Tar_Angle = -180.0f - Tar_Angle;
			break;
	}
	return Tar_Angle;	
}
