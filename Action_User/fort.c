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
CalibrationData Cal;
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
float Constrain_Float(float amt, float high, float low)
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
	//读取炮塔航向角实际值和射球电机转速实际值	
	Gun->YawAngle_Rec = fort.yawPosReceive;
	Gun->ShooterVel_Rec  = fort.shooterVelReceive;
	
	if(Pos->direction == ACW){Gun->Square_Mode = 0;}
	if(Pos->direction == CW) {Gun->Square_Mode = 1;}
	
	//读取两侧炮塔激光探测到的距离
	Gun->Dist_Laser_Left  = 2.4637f * fort.laserAValueReceive + 32.629f;
	Gun->Dist_Laser_Right = 2.4617f * fort.laserBValueReceive + 22.944f;
	
	//炮塔坐标系补偿
	Gun->Pos_Fort_X = Pos->X - (FORT_TO_BACK_WHEEL) * sin(Pos->Angle * Pi / 180.0f);
	Gun->Pos_Fort_Y = Pos->Y + (FORT_TO_BACK_WHEEL) * cos(Pos->Angle * Pi / 180.0f);
	Gun->Pos_Fort_Angle = Constrain_float_Angle(Pos->Angle - fort.yawPosReceive - Gun->YawAngle_Zero_Offset);
	
	//初始化炮塔到目标桶的横纵轴距离和距离
	Gun->Dist_FToBucket_X = Gun->Pos_Bucket_X[Gun->BucketNum] - Gun->Pos_Fort_X;
	Gun->Dist_FToBucket_Y = Gun->Pos_Bucket_Y[Gun->BucketNum] - Gun->Pos_Fort_Y;
	Gun->Dist_FToBucket = sqrt(Gun->Dist_FToBucket_X * Gun->Dist_FToBucket_X + Gun->Dist_FToBucket_Y * Gun->Dist_FToBucket_Y);
	
	//初始化车移动的横纵轴距离
	Gun->Dist_Move_X = 0;
	Gun->Dist_Move_Y = 0;
	
	//初始化射球实际移动的横纵轴距离为炮塔到目标桶的横纵轴距离
	Gun->Dist_Shoot_X = Gun->Dist_FToBucket_X;
	Gun->Dist_Shoot_Y = Gun->Dist_FToBucket_Y;
	
	//初始化迭代计算次数
	Gun->CntIteration = 0;
	
	//全方位全向速度补偿迭代
	do
	{
		//迭代次数计数
		Gun->CntIteration++;
		
		//计算射球实际移动的横纵轴距离和距离
		Gun->Dist_Shoot_X = Gun->Dist_FToBucket_X - Gun->Dist_Move_X;
		Gun->Dist_Shoot_Y = Gun->Dist_FToBucket_Y - Gun->Dist_Move_Y;
		Gun->Dist_Shoot = sqrt(Gun->Dist_Shoot_X * Gun->Dist_Shoot_X + Gun->Dist_Shoot_Y * Gun->Dist_Shoot_Y);
		
		//计算射球水平速度和飞行时间
		Gun->ShooterVel_Set_H = sqrt((Gun->Dist_Shoot * Gun->Dist_Shoot * G) / (2.0f * (Gun->Dist_Shoot * tan(FORT_ELEVATION_RAD) - FORT_TO_BUCKET_HEIGHT)));
		Gun->Shooter_Time = Gun->Dist_Shoot / Gun->ShooterVel_Set_H;
		
		//计算射球飞行时间中车移动的横纵轴距离和距离
		Gun->Dist_Move_X = Pos->X_Speed * Gun->Shooter_Time;
		Gun->Dist_Move_Y = Pos->Y_Speed * Gun->Shooter_Time;
		Gun->Dist_Move = sqrt(Gun->Dist_Move_X * Gun->Dist_Move_X + Gun->Dist_Move_Y * Gun->Dist_Move_Y);
		
		//计算当前计算值与目标桶的在横纵轴上偏差值
		//射球飞行时间中车移动的距离 + 射球实际移动距离 - 炮台到目标桶的距离
		Gun->Dist_Error_X = fabs(Gun->Dist_Move_X + Gun->Dist_Shoot_X - Gun->Dist_FToBucket_X);
		Gun->Dist_Error_Y = fabs(Gun->Dist_Move_Y + Gun->Dist_Shoot_Y - Gun->Dist_FToBucket_Y);
	}
	//当差值大于精度要求时，继续迭代
	while((Gun->Dist_Error_X > Gun->Dist_Error_Accuracy) || (Gun->Dist_Error_Y > Gun->Dist_Error_Accuracy));
	
    //根据射球电机转速与静止炮塔到桶距离经验公式计算射球电机转速
	Gun->ShooterVel_Set = 0.0128f * Gun->Dist_Shoot + 31.577f + Gun->ShooterVel_Offset[Gun->BucketNum + Gun->Square_Mode * 4];
	Gun->ShooterVel_No_Offset = 0.0128f * Gun->Dist_FToBucket  + 31.577f + Gun->ShooterVel_Offset[Gun->BucketNum + Gun->Square_Mode * 4];
	
	//计算炮塔航向角目标值
	Gun->YawAngle_Tar  = Tar_Angle_Operation(Gun->Dist_Shoot_X, Gun->Dist_Shoot_Y);
	Gun->YawAngle_No_Offset = Tar_Angle_Operation(Gun->Dist_FToBucket_X , Gun->Dist_FToBucket_Y);
	
	//计算炮塔航向角设定值
	Gun->YawAngle_Set  = Constrain_float_Angle(Constrain_float_Angle(Pos->Angle - Gun->YawAngle_Tar  + Gun->YawAngle_Offset[Gun->BucketNum + Gun->Square_Mode * 4] + Gun->YawAngle_Zero_Offset) - Constrain_float_Angle(Gun->YawAngle_Rec))\
						 + Gun->YawAngle_Rec;
	Gun->YawAngle_No_Offset = Constrain_float_Angle(Constrain_float_Angle(Pos->Angle - Gun->YawAngle_No_Offset + Gun->YawAngle_Offset[Gun->BucketNum + Gun->Square_Mode * 4] + Gun->YawAngle_Zero_Offset) - Constrain_float_Angle(Gun->YawAngle_Rec))\
						 + Gun->YawAngle_Rec;			
	
	/*ZYJ Predictor*/
	if(Pos->direction == ACW)
	{
		Gun->YawAngle_SetAct = Constrain_Float(Gun->YawAngle_Set, 90,  0);
		if(Gun->YawAngle_SetAct > 70)
		{
			Gun->YawAngle_SetAct = 0;
		}
	}
	if(Pos->direction == CW)
	{
		Gun->YawAngle_SetAct = Constrain_Float(Gun->YawAngle_Set, 0, -90);
		if(Gun->YawAngle_SetAct < -70)
		{
			Gun->YawAngle_SetAct = 0;
		}
	}
	Gun->ShooterVel_SetAct  = Constrain_Float(Gun->ShooterVel_Set , 90, 50);
	if(Gun->ShooterVel_SetAct  < 54)
	{
		Gun->ShooterVel_SetAct = 80;
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
void Scan_Operation(ScanData *Scan, PID_Value *Pos, int targets[])
{
	//读取炮塔航向角实际值
	Scan->YawAngle_Rec = fort.yawPosReceive;
	Scan->ShooterVel_Rec  = fort.shooterVelReceive;
	
	//炮塔坐标系补偿	
	Scan->Pos_Fort_X = Pos->X - (FORT_TO_BACK_WHEEL) * sin(Pos->Angle * Pi / 180.0f);
	Scan->Pos_Fort_Y = Pos->Y + (FORT_TO_BACK_WHEEL) * cos(Pos->Angle * Pi / 180.0f);
	Scan->Pos_Fort_Angle = Constrain_float_Angle(Pos->Angle - fort.yawPosReceive - Scan->YawAngle_Zero_Offset);
	
	//扫描时间计时，6s时强制将ScanStatus置为0
	if(Scan->DelayFlag)
	{
		Scan->CntDelayTime++;
		if(Scan->CntDelayTime > 600)
		{
			Scan->CntDelayTime = 0;
			Scan->DelayFlag = 0;
			Scan->FirePermitFlag = 0;
			Scan->GetLeftFlag = 0;
			Scan->GetRightFlag = 0;
			Scan->ScanStatus = 0;
			Scan->SetTimeFlag = 1;
			Scan->SetFireFlag = 1;
			Scan->ScanVel = 0.1f;
		}
	}
	
	//ScanStatus = 0航向角转到扫描起始点
	if(Scan->ScanStatus == 0)
	{
		//设定目标桶号
		for(Scan->i = 1, Scan->min = 0; Scan->i < 4; Scan->i++)
		{
			if(targets[Scan->i] < targets[Scan->min])
			{
				Scan->min = Scan->i;
			}
		}
		Pos->target_Num = Scan->min;
		Scan->BucketNum = Scan->min;
		
		//炮塔到目标桶左右挡板边缘横轴距离以及计算指向左右挡板边缘航向角设定值
		Scan->Dist_FToBorder_Left_X = Scan->Pos_Border_X[Scan->BucketNum * 2 + 1] - Scan->Pos_Fort_X;
		Scan->Dist_FToBorder_Left_Y = Scan->Pos_Border_Y[Scan->BucketNum * 2 + 1] - Scan->Pos_Fort_Y;	
		Scan->ScanAngle_Tar_Left = Tar_Angle_Operation(Scan->Dist_FToBorder_Left_X, Scan->Dist_FToBorder_Left_Y);
		Scan->ScanAngle_Set_Left  = Constrain_float_Angle(Pos->Angle - Scan->ScanAngle_Tar_Left + Scan->YawAngle_Zero_Offset - Constrain_float_Angle(Scan->YawAngle_Rec)) + Scan->YawAngle_Rec;	
		
		Scan->Dist_FToBorder_Right_X = Scan->Pos_Border_X[Scan->BucketNum * 2] - Scan->Pos_Fort_X;
		Scan->Dist_FToBorder_Right_Y = Scan->Pos_Border_Y[Scan->BucketNum * 2] - Scan->Pos_Fort_Y;
		Scan->ScanAngle_Tar_Right = Tar_Angle_Operation(Scan->Dist_FToBorder_Right_X, Scan->Dist_FToBorder_Right_Y);
		Scan->ScanAngle_Set_Right = Constrain_float_Angle(Pos->Angle - Scan->ScanAngle_Tar_Right + Scan->YawAngle_Zero_Offset - Constrain_float_Angle(Scan->YawAngle_Rec)) + Scan->YawAngle_Rec;	

		Scan->ScanAngle_Start = Scan->ScanAngle_Set_Left - 15.0f;
		Scan->ScanAngle_End = Scan->ScanAngle_Set_Right + 15.0f;
		
		//炮塔航向角设定为位于最左侧的扫描起始点
		Scan->YawAngle_Set = Scan->ScanAngle_Start;
		Scan->ShooterVel_Set = 60.0f;

		if(fabs(Scan->YawAngle_Rec - Scan->ScanAngle_Start) < 1.0f)
		{
			Scan->DelayFlag = 1;
			Scan->ScanStatus = 1;
			Scan->ScanPermitFlag = 1;
		}
	}		
	
	
	//ScanStatus = 1扫描状态
	if(Scan->ScanStatus == 1) 
	{  
		if(fabs(Scan->YawAngle_Rec - Scan->ScanAngle_Start) < 1.0f)
		{
			Scan->ScanPermitFlag = 1;
		}
		
		if(Scan->ScanPermitFlag == 1)
		{
			//如果在扫描状态时，炮塔航向角设定值每10ms增加0.2°
			Scan->YawAngle_Set = Scan->YawAngle_Set + Scan->ScanVel;
			Scan->ShooterVel_Set = 60.0f;
			
			//计算左右侧激光探测距离和探测点横轴坐标
			Scan->Pro_Left_Dist  = 2.4637f * fort.laserAValueReceive + 32.629f;
			Scan->Pos_Laser_Left_X  = Scan->Pos_Fort_X - (FORT_TO_LASER_X) * sin((Scan->Pos_Fort_Angle + 90.0f) * Pi / 180.0f) - (FORT_TO_LASER_Y) * sin(Scan->Pos_Fort_Angle * Pi / 180.0f);
			Scan->Pos_Laser_Left_Y  = Scan->Pos_Fort_Y + (FORT_TO_LASER_X) * cos((Scan->Pos_Fort_Angle + 90.0f) * Pi / 180.0f) + (FORT_TO_LASER_Y) * cos(Scan->Pos_Fort_Angle * Pi / 180.0f);					
			Scan->Pro_Left_X  = Scan->Pos_Laser_Left_X  - Scan->Pro_Left_Dist  * sin(Scan->Pos_Fort_Angle * Pi / 180.0f);
			Scan->Pro_Left_Y  = Scan->Pos_Laser_Left_Y  + Scan->Pro_Left_Dist  * cos(Scan->Pos_Fort_Angle * Pi / 180.0f);	
			
			Scan->Pro_Right_Dist = 2.4617f * fort.laserBValueReceive + 22.944f;
			Scan->Pos_Laser_Right_X = Scan->Pos_Fort_X - (FORT_TO_LASER_X) * sin((Scan->Pos_Fort_Angle - 90.0f) * Pi / 180.0f) - (FORT_TO_LASER_Y) * sin(Scan->Pos_Fort_Angle * Pi / 180.0f);
			Scan->Pos_Laser_Right_Y = Scan->Pos_Fort_Y + (FORT_TO_LASER_X) * cos((Scan->Pos_Fort_Angle - 90.0f) * Pi / 180.0f) + (FORT_TO_LASER_Y) * cos(Scan->Pos_Fort_Angle * Pi / 180.0f);	
			Scan->Pro_Right_X = Scan->Pos_Laser_Right_X - Scan->Pro_Right_Dist * sin(Scan->Pos_Fort_Angle * Pi / 180.0f);
			Scan->Pro_Right_Y = Scan->Pos_Laser_Right_Y + Scan->Pro_Right_Dist * cos(Scan->Pos_Fort_Angle * Pi / 180.0f);		
				
			//识别目标桶两侧挡板边缘，统一为从左往右扫描
			//当右侧激光距离值突然变短，标记左侧挡板边缘坐标点

			if((Scan->Pro_Left_Dist - Scan->Pro_Right_Dist) > 1200.0f)
			{
				if((-3000.0f < Scan->Pro_Right_X && Scan->Pro_Right_X < -1400.0f) || (1400.0f < Scan->Pro_Right_X && Scan->Pro_Right_X < 3000.0f))
				{
					if((-600.0f < Scan->Pro_Right_Y && Scan->Pro_Right_Y < 1000.0f) || (3800.0f < Scan->Pro_Right_Y && Scan->Pro_Right_Y < 5400.0f))
					{
						Scan->Pro_Border_Left_X = Scan->Pro_Right_X;
						Scan->Pro_Border_Left_Y = Scan->Pro_Right_Y;
						Scan->Pro_Border_Left_Angle = Scan->YawAngle_Set;
					}
				}
			}
			//当右侧激光距离值突然变长，标记右侧挡板边缘坐标点	
			if((Scan->Pro_Right_Dist - Scan->Pro_Left_Dist) > 1200.0f)
			{
				if((-3000.0f < Scan->Pro_Left_X && Scan->Pro_Left_X < -1400.0f) || (1400.0f < Scan->Pro_Left_X && Scan->Pro_Left_X < 3000.0f))
				{
					if((-600.0f < Scan->Pro_Left_Y && Scan->Pro_Left_Y < 1000.0f) || (3800.0f < Scan->Pro_Left_Y && Scan->Pro_Left_Y < 5400.0f))
					{
						Scan->Pro_Border_Right_X = Scan->Pro_Left_X;
						Scan->Pro_Border_Right_Y = Scan->Pro_Left_Y;
						Scan->Pro_Border_Right_Angle = Scan->YawAngle_Set;
					}
				}
			}
		
		
			//如果炮台航向角设定值大于较大的航向角设定值
			if(Scan->YawAngle_Set > Scan->ScanAngle_End)
			{
				if(Scan->Pro_Border_Left_X != Scan->Pro_Border_Left_X_Last)
				{
					Scan->GetLeftFlag  = 1;
				}
				if(Scan->Pro_Border_Right_X != Scan->Pro_Border_Right_X_Last)
				{
					Scan->GetRightFlag = 1;
				} 
				
				if(Scan->GetLeftFlag == 1 && Scan->GetRightFlag == 1)
				{
					Scan->Pro_Border_Left_X_Last  = Scan->Pro_Border_Left_X;
					Scan->Pro_Border_Left_Y_Last  = Scan->Pro_Border_Left_Y;
					Scan->Pro_Border_Right_X_Last = Scan->Pro_Border_Right_X;	
					Scan->Pro_Border_Right_Y_Last = Scan->Pro_Border_Right_Y;
					
					Scan->GetLeftFlag  = 0;
					Scan->GetRightFlag = 0;	
					Scan->GetBucketFlag = 1;					
					Scan->ScanStatus = 2;
				}
				else
				{
					if(Scan->GetLeftFlag == 0 && Scan->GetRightFlag == 0)
					{
						Scan->ScanAngle_Start = Scan->ScanAngle_Start - 20.0f;
						Scan->ScanAngle_End   = Scan->ScanAngle_End   + 20.0f;
					}
					if(Scan->GetLeftFlag == 0 && Scan->GetRightFlag == 1)
					{				
						Scan->ScanAngle_Start = Scan->ScanAngle_Start - 20.0f;
						Scan->ScanAngle_End   = Scan->ScanAngle_End   - 20.0f;					
					}
					if(Scan->GetLeftFlag == 1 && Scan->GetRightFlag == 0)
					{				
						Scan->ScanAngle_Start = Scan->ScanAngle_Start + 20.0f;
						Scan->ScanAngle_End   = Scan->ScanAngle_End   + 20.0f;		

					}
					Scan->YawAngle_Set = Scan->ScanAngle_Start;
					Scan->ScanPermitFlag = 0;
					Scan->ScanVel = 0.1f;
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
				Scan->Pro_Bucket_X = (Scan->Pro_Border_Left_X + Scan->Pro_Border_Right_X - 54.0f) / 2.0f;
				Scan->Pro_Bucket_Y = (Scan->Pro_Border_Left_Y + Scan->Pro_Border_Right_Y + 54.0f) / 2.0f;
			case 1:
				Scan->Pro_Bucket_X = (Scan->Pro_Border_Left_X + Scan->Pro_Border_Right_X - 54.0f) / 2.0f;
				Scan->Pro_Bucket_Y = (Scan->Pro_Border_Left_Y + Scan->Pro_Border_Right_Y - 54.0f) / 2.0f;
			case 2:
				Scan->Pro_Bucket_X = (Scan->Pro_Border_Left_X + Scan->Pro_Border_Right_X + 54.0f) / 2.0f;
				Scan->Pro_Bucket_Y = (Scan->Pro_Border_Left_Y + Scan->Pro_Border_Right_Y - 54.0f) / 2.0f;				
			case 3:
				Scan->Pro_Bucket_X = (Scan->Pro_Border_Left_X + Scan->Pro_Border_Right_X + 54.0f) / 2.0f;
				Scan->Pro_Bucket_Y = (Scan->Pro_Border_Left_Y + Scan->Pro_Border_Right_Y + 54.0f) / 2.0f;
		}
		Scan->Dist_FToBucket_X_Pro = Scan->Pro_Bucket_X - Scan->Pos_Fort_X;
		Scan->Dist_FToBucket_Y_Pro = Scan->Pro_Bucket_Y - Scan->Pos_Fort_Y;
		
		//根据射球电机转速与静止炮台到桶距离经验公式计算射球电机转速
		Scan->Pro_Bucket_Dist = sqrt(Scan->Dist_FToBucket_X_Pro * Scan->Dist_FToBucket_X_Pro + Scan->Dist_FToBucket_Y_Pro * Scan->Dist_FToBucket_Y_Pro);
		Scan->ShooterVel_Set = 0.0128f * Scan->Pro_Bucket_Dist + 31.577f + Scan->ShooterVel_Offset;
		
		//计算炮塔航向角目标值和设定值
		Scan->YawAngle_Tar_Pro = Tar_Angle_Operation(Scan->Dist_FToBucket_X_Pro, Scan->Dist_FToBucket_Y_Pro);
		Scan->YawAngle_Set = Constrain_float_Angle(Pos->Angle - Scan->YawAngle_Tar_Pro + Scan->YawAngle_Zero_Offset + Scan->YawAngle_Offset - Constrain_float_Angle(Scan->YawAngle_Rec)) + Scan->YawAngle_Rec;	
		
		//到达航向角和射球电机转速设定值时允许开火
		if((fabs(Scan->YawAngle_Rec - Scan->YawAngle_Set) < 1.0f) && (fabs(Scan->ShooterVel_Rec - Scan->ShooterVel_Set) < 2.0f) && Scan->SetFireFlag == 1)
		{
			Scan->FirePermitFlag = 1;
			Scan->SetFireFlag = 0;
		}	
		
		//判断数组0是否变1
		if(targets[Scan->BucketNum] > 0 && Scan->SetTimeFlag == 1)
		{
			Scan->SetTimeFlag = 0;
			Scan->CntDelayTime = 500;
			Scan->FirePermitFlag = 0;
		}
	}
}


void Calibration_Operation(CalibrationData *Cal, ScanData *Scan, GunneryData *Gun, PID_Value const *Pos)
{
	Cal->LToR_Act_Dist_X = Scan->Pro_Border_Right_X - Scan->Pro_Border_Left_X;
	Cal->LToR_Act_Dist_Y = Scan->Pro_Border_Right_Y - Scan->Pro_Border_Left_Y;
	Cal->LToR_Act_Angle = Tar_Angle_Operation(Cal->LToR_Act_Dist_X, Cal->LToR_Act_Dist_Y);
	
	Cal->LToR_The_Dist_X = Scan->Pos_Border_X[Scan->BucketNum * 2] - Scan->Pos_Border_X[Scan->BucketNum * 2 + 1];
	Cal->LToR_The_Dist_Y = Scan->Pos_Border_Y[Scan->BucketNum * 2] - Scan->Pos_Border_Y[Scan->BucketNum * 2 + 1];
	Cal->LToR_The_Angle = Tar_Angle_Operation(Cal->LToR_The_Dist_X, Cal->LToR_The_Dist_Y);	

	Cal->Coor_Error_Angle = Cal->LToR_The_Angle - Cal->LToR_Act_Angle;
	
	Cal->OToRight_Angle = Tar_Angle_Operation(Scan->Pro_Border_Right_X, Scan->Pro_Border_Right_Y);
	Cal->OToRight_Dist = sqrt(Scan->Pro_Border_Right_X * Scan->Pro_Border_Right_X + Scan->Pro_Border_Right_Y * Scan->Pro_Border_Right_Y);
	Cal->Pos_Border_Right_X = Cal->OToRight_Dist * sin((Cal->OToRight_Angle + Cal->Coor_Error_Angle) * Pi / 180.0f) * -1.0f;
	Cal->Pos_Border_Right_Y = Cal->OToRight_Dist * cos((Cal->OToRight_Angle + Cal->Coor_Error_Angle) * Pi / 180.0f);

	Cal->OToLeft_Angle = Tar_Angle_Operation(Scan->Pro_Border_Left_X, Scan->Pro_Border_Left_Y);
	Cal->OToLeft_Dist = sqrt(Scan->Pro_Border_Left_X * Scan->Pro_Border_Left_X + Scan->Pro_Border_Left_Y * Scan->Pro_Border_Left_Y);	
	Cal->Pos_Border_Left_X = Cal->OToLeft_Dist * sin((Cal->OToLeft_Angle + Cal->Coor_Error_Angle) * Pi / 180.0f) * -1.0f;
	Cal->Pos_Border_Left_Y = Cal->OToLeft_Dist * cos((Cal->OToLeft_Angle + Cal->Coor_Error_Angle) * Pi / 180.0f);
	
	switch(Scan->BucketNum)
	{
		case 0:
			Cal->Pos_Bucket_X = (Cal->Pos_Border_Left_X + Cal->Pos_Border_Right_X - 54.0f) / 2.0f;
			Cal->Pos_Bucket_Y = (Cal->Pos_Border_Left_Y + Cal->Pos_Border_Right_Y + 54.0f) / 2.0f;
		case 1:                                               
			Cal->Pos_Bucket_X = (Cal->Pos_Border_Left_X + Cal->Pos_Border_Right_X - 54.0f) / 2.0f;
			Cal->Pos_Bucket_Y = (Cal->Pos_Border_Left_Y + Cal->Pos_Border_Right_Y - 54.0f) / 2.0f;
		case 2:
			Cal->Pos_Bucket_X = (Cal->Pos_Border_Left_X + Cal->Pos_Border_Right_X + 54.0f) / 2.0f;
			Cal->Pos_Bucket_Y = (Cal->Pos_Border_Left_Y + Cal->Pos_Border_Right_Y - 54.0f) / 2.0f;				
		case 3:
			Cal->Pos_Bucket_X = (Cal->Pos_Border_Left_X + Cal->Pos_Border_Right_X + 54.0f) / 2.0f;
			Cal->Pos_Bucket_Y = (Cal->Pos_Border_Left_Y + Cal->Pos_Border_Right_Y + 54.0f) / 2.0f;
	}	
	
	Cal->Coor_Error_X = Gun->Pos_Bucket_X[Scan->BucketNum] - Cal->Pos_Bucket_X;
	Cal->Coor_Error_Y = Gun->Pos_Bucket_Y[Scan->BucketNum] - Cal->Pos_Bucket_Y;
	
	Cal->Car_Angle = Pos->Angle + Cal->Coor_Error_Angle;
	Cal->Car_X = Pos->X + Cal->Coor_Error_X;
	Cal->Car_Y = Pos->Y + Cal->Coor_Error_Y;
	
	if(fabs(Cal->Coor_Error_Angle) > 5.0f || fabs(Cal->Coor_Error_X) > 200.0f || fabs(Cal->Coor_Error_Y) > 200.0f)
	{
		CorrectAngle(Cal->Car_Angle);
		CorrectX(Cal->Car_X);
		CorrectY(Cal->Car_Y);
	}
}


/**
* @brief  计算当前点指向目标点的角度（按定位系统坐标系角度规定）
* @param  Dist_X：		目标点到当前坐标的横轴距离（目标点横坐标 - 当前点横坐标）
* @param  Diste_Y：		目标点到当前坐标的纵轴距离（目标点纵坐标 - 当前点纵坐标）
* @param  Quadrant：	目标点位于的象限
* @param  TarAngle：	当前点指向目标点的角度
* @author 陈昕炜
* @note   用于Fort中
*/
float Tar_Angle_Operation(float Dist_X, float Dist_Y)
{
	int Quadrant;
	//未转化的炮台航向角目标值
	float TarAngle = atan(Dist_X / Dist_Y) * 180.0f / Pi;
	
	//判断目标桶位于的象限
	if(Dist_X > 0 && Dist_Y > 0) {Quadrant = 1;}
	if(Dist_X < 0 && Dist_Y > 0) {Quadrant = 2;}	
	if(Dist_X < 0 && Dist_Y < 0) {Quadrant = 3;}	
	if(Dist_X > 0 && Dist_Y < 0) {Quadrant = 4;}

	//转换为定位系统坐标系角度规定的炮台航向角目标值
	switch(Quadrant)
	{
		case 1:
			TarAngle = TarAngle * -1.0f;
			break;		
		case 2:
			TarAngle = TarAngle * -1.0f;
			break;		
		case 3:
			TarAngle = 180.0f - TarAngle;
			break;
		case 4:
			TarAngle = -180.0f - TarAngle;
			break;
	}
	return TarAngle;	
}
