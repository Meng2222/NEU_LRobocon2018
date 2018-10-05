#ifndef FORT__H
#define FORT__H

#include "stdint.h"
#include "pps.h"
#include "moveBase.h"

#define FORT_TO_BACK_WHEEL (10.5f)								//炮台到后轮两轮轴中点距离		10.5mm
#define G (9800.0f)												//重力加速度					9800.0mm/s2
#define FORT_ELEVATION_DEG (60.0f)								//炮台仰角						60°
#define FORT_ELEVATION_RAD (FORT_ELEVATION_DEG * Pi / 180.0f)	//炮台仰角						1/3πrad
#define FORT_HEIGHT (128.0f + 60.0f)							//炮台高度						188.0mm
#define BUCKET_HEIGHT (800.0f)									//桶高度						800.0mm
#define FORT_TO_BUCKET_HEIGHT (BUCKET_HEIGHT - FORT_HEIGHT)		//炮台到桶高度差				800.0 - 188.0mm
#define FORT_TO_LASER_X (40.0f)									//炮台到激光器横轴距离			40.0mm
#define FORT_TO_LASER_Y (23.32f)								//炮台到激光器纵轴距离			23.32mm
#define LASER_LEFT (0)											//左侧激光编号					0
#define LASER_RIGHT (1)											//右侧激光编号					1

typedef union
{
	uint8_t	data8[4];
	int		data32;
	float	dataFloat;
}usartData_t;


typedef struct
{
	usartData_t usartTransmitData;
	usartData_t usartReceiveData;
	float yawPosReceive ;
	float shooterVelReceive;
	float laserAValueReceive ;
	float laserBValueReceive ; 
}FortType;


/**
* @brief  射击诸元结构体
* @author 陈昕炜
* @note   定义一个GunneryData类型
*/
typedef struct							//										2  _   _  1   
{										//										  |     |
	int BucketNum;						//目标桶号
	int BucketQuadrant;					//目标桶位于的象限						  |_   _|
	int CntIteration;           	    //迭代次数								3         0
	int Square_Mode;            	    //方形走行方向							Square_Mode = 0时    逆时针
										//										Square_Mode = 1时    顺时针
	
	float Pos_Fort_X;					//炮塔横坐标							单位mm
	float Pos_Fort_Y;					//炮塔纵坐标							单位mm
	float Pos_Fort_Angle;				//炮塔航向角实际值换算定位系统角度		单位°

	float Pos_Bucket_X[4];				//目标桶横坐标							单位mm
	float Pos_Bucket_Y[4];				//目标桶纵坐标							单位mm

	
	float YawAngle_Rec;					//炮塔航向角实际值						单位°
	float YawAngle_Tar;					//炮塔航向角目标值						单位° 
	float YawAngle_Set;					//炮塔航向角设定值						单位°     
	float YawAngle_SetAct;				//炮塔航向角实际设定值					单位°           
	float YawAngle_No_Offset;			//炮塔航向角无补偿设定值				单位°
	float YawAngle_Zero_Offset;			//炮塔航向角归零补偿值					单位°
	float YawAngle_Offset[8];			//炮塔航向角补偿值						单位°
	
	
	float ShooterVel_Rec;				//射球电机转速实际值					单位rad/s
	float ShooterVel_Set;				//射球电机转速设定值					单位rad/s
	float ShooterVel_Set_H;				//射球水平速度							单位mm/s
	float ShooterVel_Set_V;				//射球竖直速度							单位mm/s
	float ShooterVel_SetAct;			//射球电机转速实际设定值				单位rad/s
	float ShooterVel_No_Offset;			//射球电机转速无补偿设定值				单位rad/s
	float ShooterVel_Offset[8];			//射球电机转速补偿值					单位rad/s	
	float Shooter_Time;					//飞行时间								单位s


	float Dist_FToBucket;				//炮塔到目标桶的距离					单位mm	
	float Dist_FToBucket_X;				//炮塔到目标桶的横轴距离				单位mm      
	float Dist_FToBucket_Y;				//炮塔到目标桶的纵轴距离				单位mm
	
	float Dist_Shoot;					//射球实际飞行距离						单位mm
	float Dist_Shoot_X;					//射球实际飞行横轴距离					单位mm
	float Dist_Shoot_Y;					//射球实际飞行纵轴距离					单位mm
	
	float Dist_Move;					//射球飞行时间中车移动的距离			单位mm
	float Dist_Move_X;					//射球飞行时间中车移动的横轴距离		单位mm
	float Dist_Move_Y;					//射球飞行时间中车移动的纵轴距离		单位mm
	
	float Dist_Error_Accuracy;			//迭代精度要求							单位mm
	float Dist_Error_X;					//横轴距离偏差							单位mm
	float Dist_Error_Y;					//纵轴距离偏差							单位mm 

	float Dist_Laser_Left;				//左侧激光探测点距离					单位mm
	float Dist_Laser_Right;				//右侧激光探测点距离					单位mm
}GunneryData;


/**
* @brief  扫描参数结构体
* @author 陈昕炜
* @note   定义一个ScanData类型
*/
typedef struct{
	int BucketNum;						//目标桶号 
	int CntDelayTime;					//延时计数
	int DelayFlag;						//延时标志位
	int FirePermitFlag;					//允许射球标志位
	int GetLeftFlag;					//扫描到左侧挡板边缘标志位
	int GetRightFlag;					//扫描到右侧挡板边缘标志位
	int ScanStatus;						//扫描状态                              0走行	1扫描	2射球
	int ScanPermitFlag;					//允许开始扫描标志位
	int ScanEndFlag;
	int SetTimeFlag;					//允许设定延时标志位	
	int SetFireFlag;					//允许设定射球标志位
	int i;								//计数
	int GetBucketFlag;					//
	float ScanVel;						//扫描速度								单位°/10ms
	
	
	float Pos_Fort_X;					//炮塔横坐标							单位mm
	float Pos_Fort_Y;					//炮塔纵坐标							单位mm
	float Pos_Fort_Angle;				//炮塔航向角检测值换算定位系统角度		单位°

	float Pos_Laser_Left_X;				//左侧激光横坐标						单位mm  
	float Pos_Laser_Left_Y;				//左侧激光纵坐标						单位mm
	float Pos_Laser_Right_X;			//右侧激光横坐标						单位mm
	float Pos_Laser_Right_Y;			//右侧激光纵坐标						单位mm
										//													  _4 3_
	float Pos_Border_X[8];				//逆时针排序的挡板边缘横坐标			单位mm		5|     |2
	float Pos_Border_Y[8];				//逆时针排序的挡板边缘纵坐标			单位mm
										//													6|_   _|1 
										//													   7 0
	float Dist_FToBorder_Left_X;		//炮塔到目标桶左挡板边缘横轴距离		单位mm
	float Dist_FToBorder_Left_Y;		//炮塔到目标桶左挡板边缘纵轴距离		单位mm
	float Dist_FToBorder_Right_X;		//炮塔到目标桶右挡板边缘横轴距离		单位mm
	float Dist_FToBorder_Right_Y;		//炮塔到目标桶右挡板边缘纵轴距离		单位mm

	float Dist_FToBucket_X_Pro;			//炮塔到目标桶横轴距离探测值			单位mm
	float Dist_FToBucket_Y_Pro;			//炮塔到目标桶纵轴距离探测值			单位mm
	float Dist_FToBucket_X_The;			//炮塔到目标桶横轴距离理论值			单位mm
	float Dist_FToBucket_Y_The;			//炮塔到目标桶纵轴距离理论值			单位mm

	float ScanAngle_Tar_Left;			//指向左挡板边缘航向角目标值			单位°
	float ScanAngle_Tar_Right;			//指向右挡板边缘航向角目标值			单位°
	float ScanAngle_Set_Left;			//指向左挡板边缘航向角目标值			单位°
	float ScanAngle_Set_Right;			//指向右挡板边缘航向角目标值			单位°
	float ScanAngle_End;				//位于最右侧的扫描起始航向角设定值		单位°
	float ScanAngle_Start;				//位于最左侧的扫描终止航向角设定值		单位°

	float Pro_Left_Dist;				//左侧激光探测点距离值					单位mm	
	float Pro_Left_X;					//左侧激光探测点横坐标					单位mm
	float Pro_Left_Y;					//左侧激光探测点纵坐标					单位mm
	float Pro_Right_Dist;				//右侧激光探测点距离值					单位mm	
	float Pro_Right_X;					//右侧激光探测点横坐标					单位mm
	float Pro_Right_Y;					//右侧激光探测点纵坐标					单位mm
	
	float Pro_Border_Left_X_Last;		//探测到的上一个左侧挡板边缘横坐标		单位mm	
	float Pro_Border_Left_Y_Last;		//探测到的上一个左侧挡板边缘纵坐标		单位mm
	float Pro_Border_Right_X_Last;		//探测到的上一个右侧挡板边缘横坐标		单位mm
	float Pro_Border_Right_Y_Last;		//探测到的上一个右侧挡板边缘纵坐标		单位mm
	
	float Pro_LToR_Dist_X;				//左侧挡板边缘到右侧挡板边缘横轴距离	单位mm
	float Pro_LToR_Dist_Y;				//左侧挡板边缘到右侧挡板边缘纵轴距离	单位mm
	float Pro_LToR_Dist;				//左侧挡板边缘到右侧挡板边缘距离		单位mm
	
	float Pro_Border_Left_X;			//探测的左侧挡板边缘横坐标				单位mm
	float Pro_Border_Left_Y;			//探测的左侧挡板边缘纵坐标				单位mm
	float Pro_Border_Left_Angle;		//探测的左侧挡板边缘的定位系统角度		单位mm
	float Pro_Border_Right_X;			//探测的右侧挡板边缘横坐标				单位mm
	float Pro_Border_Right_Y;			//探测的右侧挡板边缘纵坐标				单位mm
	float Pro_Border_Right_Angle;		//探测的右侧挡板边缘的定位系统角度		单位mm
	
	float Pro_Bucket_X;					//探测到的桶横坐标						单位mm
	float Pro_Bucket_Y;					//探测到的桶纵坐标						单位mm
	float Pro_Bucket_Dist;				//探测到的桶距离						单位mm

	float ShooterVel_Rec;				//射球电机转速实际值					单位rad/s
	float ShooterVel_Set;				//射球电机转速设定值					单位rad/s
	float ShooterVel_Offset;			//射球电机转速补偿值					单位rad/s
	
	float YawAngle_Rec;					//炮塔航向角实际值						单位°
	float YawAngle_Tar_Pro;				//炮塔航向角探测目标值					单位°
	float YawAngle_Tar_The;				//炮塔航向角理论目标值					单位°	
	float YawAngle_Set;					//炮塔航向角设定值						单位°
	float YawAngle_Zero_Offset;			//炮塔航向角归零补偿值					单位°
	float YawAngle_Offset;				//炮塔航向角补偿值						单位°
}ScanData;


typedef struct{
	float LToR_Act_Dist_X;
	float LToR_Act_Dist_Y;	
	float LToR_Act_Angle;

	float LToR_The_Dist_X;
	float LToR_The_Dist_Y;	
	float LToR_The_Angle;
	
	float Coor_Error_X;
	float Coor_Error_Y;	
	float Coor_Error_Angle;

	float OToRight_Angle;
	float OToRight_Dist;
	float OToLeft_Angle;
	float OToLeft_Dist;
	
	float Pos_Border_Left_X;
	float Pos_Border_Left_Y;
	float Pos_Border_Right_X;
	float Pos_Border_Right_Y;
	
	float Pos_Bucket_X;
	float Pos_Bucket_Y;

	float Car_Angle;
	float Car_X;
	float Car_Y;
}CalibrationData;


void YawPosCtrl(float ang);
void ShooterVelCtrl(float rps);
void ReadShooterVel(void);
void ReadYawPos(void);
void ReadLaserAValue(void);
void ReadLaserBValue(void);
void GetValueFromFort(uint8_t data);

void GunneryData_Operation(GunneryData *Gun, PID_Value const *Pos);
void Scan_Operation(ScanData *Scan, PID_Value *Pos, int targets[]);
void Calibration_Operation(CalibrationData *Cal, ScanData *Scan, GunneryData *Gun, PID_Value const *Pos);
float Tar_Angle_Operation(float Dist_X, float Dist_Y);

extern FortType fort;

#endif
