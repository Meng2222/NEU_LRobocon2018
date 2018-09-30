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
		angle -= 360.0f;
	}
	while(angle < -180.0f)
	{
		angle -= 360.0f;
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
	Gun->Distance_Laser_Left  = 2.4621f * fort.laserAValueReceive + 29.234f;
	Gun->Distance_Laser_Right = 2.4706f * fort.laserBValueReceive + 11.899f;
	
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
	Gun->ShooterVelSet = 0.0123f * Gun->Distance_Shoot + 35.224f + Gun->Shooter_Vel_Offset[Gun->BucketNum + Gun->Square_Mode * 4];
	Gun->No_Offset_Vel = 0.0123f * Gun->Distance_Fort  + 35.224f + Gun->Shooter_Vel_Offset[Gun->BucketNum + Gun->Square_Mode * 4];
	
	//计算炮台航向角目标值
	Gun->YawPosAngleTar  = YawPosAngleTar_Operation(Gun->Distance_Shoot_X, Gun->Distance_Shoot_Y);
	Gun->No_Offset_Angle = YawPosAngleTar_Operation(Gun->Distance_Fort_X , Gun->Distance_Fort_Y);
	
	//计算炮台航向角设定值
	Gun->YawPosAngleSet  = Constrain_float_Angle(Constrain_float_Angle(Pos->Angle - Gun->YawPosAngleTar  + Gun->Yaw_Angle_Offset[Gun->BucketNum + Gun->Square_Mode * 4] + Gun->Yaw_Zero_Offset)\
						- (int)Gun->YawPosAngleRec % 360) + Gun->YawPosAngleRec;
	Gun->No_Offset_Angle = Constrain_float_Angle(Constrain_float_Angle(Pos->Angle - Gun->No_Offset_Angle + Gun->Yaw_Angle_Offset[Gun->BucketNum + Gun->Square_Mode * 4] + Gun->Yaw_Zero_Offset)\
						- (int)Gun->YawPosAngleRec % 360) + Gun->YawPosAngleRec;	
	
	/*ZYJ Predictor*/
	if(Pos->direction == ACW)
	{
		Gun->YawPosAngleSetAct = Gun->YawPosAngleSet;
		Gun->YawPosAngleSetAct = constrain0(Gun->YawPosAngleSetAct,80,0);
		if(Gun->YawPosAngleSetAct > 79) Gun->YawPosAngleSetAct = 0;
		Gun->ShooterVelSetAct = Gun->ShooterVelSet;
		Gun->ShooterVelSetAct = constrain0(Gun->ShooterVelSetAct,90,50);
		if(Gun->ShooterVelSetAct < 54) Gun->ShooterVelSetAct = 80;
	}
	else
	{
		Gun->YawPosAngleSetAct = Gun->YawPosAngleSet;
		Gun->YawPosAngleSetAct = constrain0(Gun->YawPosAngleSetAct,0,-80);
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
void Scan_Operation(ScanData *Scan, PID_Value const *Pos)
{
	//读取炮台航向角实际值
	Scan->YawPosAngleRec = fort.yawPosReceive;
	Scan->ShooterVelRec = fort.shooterVelReceive;
	
	if(Pos->direction == ACW){Scan->Square_Mode = 0;}
	if(Pos->direction == CW) {Scan->Square_Mode = 1;}

	//炮台坐标系补偿	
	Scan->Fort_X = Pos->X - (FORT_TO_BACK_WHEEL) * sin(Pos->Angle * Pi / 180.0f);
	Scan->Fort_Y = Pos->Y + (FORT_TO_BACK_WHEEL) * cos(Pos->Angle * Pi / 180.0f);
	Scan->Fort_Angle = Constrain_float_Angle(Pos->Angle - fort.yawPosReceive + Scan->Yaw_Zero_Offset);
	
	//炮塔到目标桶挡板边缘横轴距离以及计算指向挡板边缘航向角设定值A、B
	Scan->FortToBorder_Distance_X = Scan->Bucket_Border_X[Pos->target_Num * 2] - Scan->Fort_X;
	Scan->FortToBorder_Distance_Y = Scan->Bucket_Border_Y[Pos->target_Num * 2] - Scan->Fort_Y;
	Scan->BorderAngleTar = YawPosAngleTar_Operation(Scan->FortToBorder_Distance_X, Scan->FortToBorder_Distance_Y);
	Scan->BorderAngleA = Constrain_float_Angle(Constrain_float_Angle(Pos->Angle - Scan->BorderAngleTar + Scan->Yaw_Zero_Offset) - (int)Scan->YawPosAngleRec % 360) + Scan->YawPosAngleRec;	

	Scan->FortToBorder_Distance_X = Scan->Bucket_Border_X[Pos->target_Num * 2 + 1] - Scan->Fort_X;
	Scan->FortToBorder_Distance_Y = Scan->Bucket_Border_Y[Pos->target_Num * 2 + 1] - Scan->Fort_Y;	
	Scan->BorderAngleTar = YawPosAngleTar_Operation(Scan->FortToBorder_Distance_X, Scan->FortToBorder_Distance_Y);
	Scan->BorderAngleB = Constrain_float_Angle(Constrain_float_Angle(Pos->Angle - Scan->BorderAngleTar + Scan->Yaw_Zero_Offset) - (int)Scan->YawPosAngleRec % 360) + Scan->YawPosAngleRec;

	//比较大小得出较小的航向角设定值和较大的航向角设定值
	Scan->BorderAngleUpper = (((Scan->BorderAngleA) > (Scan->BorderAngleB))?(Scan->BorderAngleA):(Scan->BorderAngleB)) + 10.0f;
	Scan->BorderAngleLower = (((Scan->BorderAngleA) < (Scan->BorderAngleB))?(Scan->BorderAngleA):(Scan->BorderAngleB)) - 10.0f;
	
	if(Scan->ScanStatus == 0)
	{	
		Scan->YawPosAngleSet = Scan->BorderAngleLower;	
		Scan->ShooterVelSet = 60.0f;
	}

	if(Scan->ScanStatus == 1) 
	{
		//如果在扫描状态时，炮台航向角设定值每10ms增加0.3°
		Scan->YawPosAngleSet = Scan->YawPosAngleSet + 0.3f;
		Scan->ShooterVelSet = 60.0f;
		
		//如果炮台航向角设定值大于较大的航向角设定值时，进入射球状态
		if(Scan->YawPosAngleSet > Scan->BorderAngleUpper)
		{
			Scan->ScanStatus = 2;
		}	
		
		//计算左右侧激光探测距离和探测点横轴坐标
		Scan->Probe_Left_Distance  = 2.4621f * fort.laserAValueReceive + 29.234f;
		Scan->Laser_Left_X  = Scan->Fort_X - (FORT_TO_LASER_X) * sin((Scan->Fort_Angle + 90.0f) * Pi / 180.0f) - (FORT_TO_LASER_Y) * sin(Scan->Fort_Angle * Pi / 180.0f);
		Scan->Laser_Left_Y  = Scan->Fort_Y + (FORT_TO_LASER_X) * cos((Scan->Fort_Angle + 90.0f) * Pi / 180.0f) + (FORT_TO_LASER_Y) * cos(Scan->Fort_Angle * Pi / 180.0f);					
		Scan->Probe_Left_X  = Scan->Laser_Left_X  - Scan->Probe_Left_Distance  * sin(Scan->Fort_Angle * Pi / 180.0f);
		Scan->Probe_Left_Y  = Scan->Laser_Left_Y  + Scan->Probe_Left_Distance  * cos(Scan->Fort_Angle * Pi / 180.0f);	
		
		Scan->Probe_Right_Distance = 2.4706f * fort.laserBValueReceive + 11.899f;
		Scan->Laser_Right_X = Scan->Fort_X - (FORT_TO_LASER_X) * sin((Scan->Fort_Angle - 90.0f) * Pi / 180.0f) - (FORT_TO_LASER_Y) * sin(Scan->Fort_Angle * Pi / 180.0f);
		Scan->Laser_Right_Y = Scan->Fort_Y + (FORT_TO_LASER_X) * cos((Scan->Fort_Angle - 90.0f) * Pi / 180.0f) + (FORT_TO_LASER_Y) * cos(Scan->Fort_Angle * Pi / 180.0f);	
		Scan->Probe_Right_X = Scan->Laser_Right_X - Scan->Probe_Right_Distance * sin(Scan->Fort_Angle * Pi / 180.0f);
		Scan->Probe_Right_Y = Scan->Laser_Right_Y + Scan->Probe_Right_Distance * cos(Scan->Fort_Angle * Pi / 180.0f);		
		
		//识别目标桶两侧挡板边缘，统一为从左往右扫描
		//当右侧激光距离值突然变短，标记左侧挡板边缘坐标点
		if((Scan->Probe_Left_Distance - Scan->Probe_Right_Distance) > 1500.0f)
		{
			if((-2700.0f < Scan->Probe_Right_X && Scan->Probe_Right_X < -1800.0f) || (1800.0f < Scan->Probe_Right_X && Scan->Probe_Right_X < 2700.0f))
			{
				if((-300.0f < Scan->Probe_Right_Y && Scan->Probe_Right_Y < 600.0f) || (4200.0f < Scan->Probe_Right_Y && Scan->Probe_Right_Y < 5100.0f))
				{
					Scan->BucketBorder_Lower_X = Scan->Probe_Right_X;
					Scan->BucketBorder_Lower_Y = Scan->Probe_Right_Y;
				}
			}
		}
		//当右侧激光距离值突然变长，标记右侧挡板边缘坐标点	
		if((Scan->Probe_Right_Distance - Scan->Probe_Left_Distance) > 1500.0f)
		{
			if((-2700.0f < Scan->Probe_Left_X && Scan->Probe_Left_X < -1800.0f) || (1800.0f < Scan->Probe_Left_X && Scan->Probe_Left_X < 2700.0f))
			{
				if((-300.0f < Scan->Probe_Left_Y && Scan->Probe_Left_Y < 600.0f) || (4200.0f < Scan->Probe_Left_Y && Scan->Probe_Left_Y < 5100.0f))
				{
					Scan->BucketBorder_Upper_X = Scan->Probe_Left_X;
					Scan->BucketBorder_Upper_Y = Scan->Probe_Left_Y;
				}
			}
		}	
	}
	
	
	if(Scan->ScanStatus == 2)
	{
		//根据标记的两侧挡板边缘坐标值相加除二得到桶的坐标
		switch(Pos->target_Num)
		{
			case 0:
				Scan->Probe_Bucket_X = (Scan->BucketBorder_Lower_X + Scan->BucketBorder_Upper_X - 54.0f) / 2.0f;
				Scan->Probe_Bucket_Y = (Scan->BucketBorder_Lower_Y + Scan->BucketBorder_Upper_Y + 54.0f) / 2.0f;
			case 1:
				Scan->Probe_Bucket_X = (Scan->BucketBorder_Lower_X + Scan->BucketBorder_Upper_X - 54.0f) / 2.0f;
				Scan->Probe_Bucket_Y = (Scan->BucketBorder_Lower_Y + Scan->BucketBorder_Upper_Y - 54.0f) / 2.0f;
			case 2:
				Scan->Probe_Bucket_X = (Scan->BucketBorder_Lower_X + Scan->BucketBorder_Upper_X + 54.0f) / 2.0f;
				Scan->Probe_Bucket_Y = (Scan->BucketBorder_Lower_Y + Scan->BucketBorder_Upper_Y - 54.0f) / 2.0f;				
			case 3:
				Scan->Probe_Bucket_X = (Scan->BucketBorder_Lower_X + Scan->BucketBorder_Upper_X + 54.0f) / 2.0f;
				Scan->Probe_Bucket_Y = (Scan->BucketBorder_Lower_Y + Scan->BucketBorder_Upper_Y + 54.0f) / 2.0f;
		}
		Scan->FortToBucket_Distance_X = Scan->Probe_Bucket_X - Scan->Fort_X;
		Scan->FortToBucket_Distance_Y = Scan->Probe_Bucket_Y - Scan->Fort_Y;
		
		//根据射球电机转速与静止炮台到桶距离经验公式计算射球电机转速
		Scan->Probe_Bucket_Distance = sqrt(Scan->FortToBucket_Distance_X * Scan->FortToBucket_Distance_X + Scan->FortToBucket_Distance_Y * Scan->FortToBucket_Distance_Y);
		Scan->ShooterVelSet = 0.0123f * Scan->Probe_Bucket_Distance + 35.224f;
		
		//计算炮台航向角目标值和设定值
		Scan->YawPosAngleTar = YawPosAngleTar_Operation(Scan->FortToBucket_Distance_X, Scan->FortToBucket_Distance_Y);
		Scan->YawPosAngleSet = Constrain_float_Angle(Constrain_float_Angle(Pos->Angle - Scan->YawPosAngleTar + Scan->Yaw_Zero_Offset) - (int)Scan->YawPosAngleRec % 360) + Scan->YawPosAngleRec;	

		if((fabs(Scan->YawPosAngleRec - Scan->YawPosAngleSet) < 1.0f) && (fabs(Scan->ShooterVelRec - Scan->ShooterVelSet) < 2.0f))
		{
			Scan->FirePermitFlag = 1;
		}
	}	
}


/**
* @brief  计算按定位系统坐标系角度规定的炮台航向角目标值
* @param  Distance_X：        目标点到炮台的横轴距离
* @param  Distance_Y：        目标点到炮台的纵轴距离
* @param  BucketQuadrant：    目标桶位于的象限
* @param  YawPosAngleTar：    炮台航向角目标值
* @author 陈昕炜
* @note   用于Fort中
*/
float YawPosAngleTar_Operation(float Distance_X, float Distance_Y)
{
	int BucketQuadrant;
	//未转化的炮台航向角目标值
	float YawPosAngleTar = atan(Distance_X / Distance_Y) * 180.0f / Pi;
	
	//判断目标桶位于的象限
	if(Distance_X > 0 && Distance_Y > 0) {BucketQuadrant = 1;}
	if(Distance_X < 0 && Distance_Y > 0) {BucketQuadrant = 2;}	
	if(Distance_X < 0 && Distance_Y < 0) {BucketQuadrant = 3;}	
	if(Distance_X > 0 && Distance_Y < 0) {BucketQuadrant = 4;}

	//转换为定位系统坐标系角度规定的炮台航向角目标值
	switch(BucketQuadrant)
	{
		case 1:
			YawPosAngleTar = YawPosAngleTar * -1.0f;
			break;		
		case 2:
			YawPosAngleTar = YawPosAngleTar * -1.0f;
			break;		
		case 3:
			YawPosAngleTar = 180.0f - YawPosAngleTar;
			break;
		case 4:
			YawPosAngleTar = -180.0f - YawPosAngleTar;
			break;
	}
	return YawPosAngleTar;	
}
