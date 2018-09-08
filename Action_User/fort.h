#ifndef FORT__H
#define FORT__H

#include "stdint.h"
#include "pps.h"
typedef union
{	
	uint8_t data8[4];
	int		data32;
	float   dataFloat;
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
* @note   定义一个GunneryData类型，此时GunneryData为一个数据类型标识符，数据类型为结构体
*/
typedef struct
{
	int BucketNum;                 //目标桶号
	int cntIteration;              //迭代次数
	
	float Fort_X;                  //炮台横坐标                          单位mm
	float Fort_Y;                  //炮台纵坐标                          单位mm
	float Fort_Angle;              //炮台航向角检测值换算绝对角度        单位°
	float Bucket_X;                //当前目标桶横坐标                    单位mm
	float Bucket_Y;                //当前目标桶纵坐标                    单位mm

	float YawPosAngleRec;          //炮台航向角检测值                    单位°
	float YawPosAngleTar;          //炮台相对偏移角度                    单位° 
	float YawPosAngleSet;          //炮台设定角度                        单位°
	
	float ShooterVelRec;           //射球电机转速检测值                  单位rad/s
	float ShooterVelSet;           //射球设定转速                        单位mm/s               
	float ShooterVelSet_H;         //射球水平速度                        单位mm/s
	float ShooterVelSet_V;         //射球竖直速度                        单位mm/s
	float ShooterTime;             //飞行时间                            单位s
	
	float Distance_Fort;           //炮台到目标点的距离                  单位mm	
	float Distance_Fort_X;         //炮台到目标点的横轴距离              单位mm      
	float Distance_Fort_Y;         //炮台到目标点的纵纵距离              单位mm
	
	float Distance_Shoot;          //射球实际飞行距离                    单位mm
	float Distance_Shoot_X;        //射球实际飞行横轴距离                单位mm
	float Distance_Shoot_Y;        //射球实际飞行纵轴距离                单位mm
	
	float Distance_Car;            //射球飞行时间中车移动的距离          单位mm
	float Distance_Car_X;          //射球飞行时间中车移动的横轴距离      单位mm
	float Distance_Car_Y;          //射球飞行时间中车移动的纵轴距离      单位mm
	
	float Distance_Accuracy;       //精度要求                            单位mm
	float Distance_Deviation_X;    //横轴距离偏差                        单位mm
	float Distance_Deviation_Y;    //纵轴距离偏差                        单位mm 

	float Distance_LaserA;         //激光A探测到的距离值                 单位mm
	float Distance_LaserB;         //激光B探测到的距离值                 单位mm

	float No_Offset_Angle;         //无补偿航向角                        单位°
	float No_Offset_ShooterVel;    //无补偿射球电机转速                  单位rad/s

	float Yaw_Angle_Offset;        //航向角补偿                          单位°
	float Shooter_Vel_Offset;      //射球转速补偿                        单位rad/s
}GunneryData;

void YawPosCtrl(float ang);
void ShooterVelCtrl(float rps);
void ReadShooterVel(void);
void ReadYawPos(void);
void ReadLaserAValue(void);
void ReadLaserBValue(void);
void GetValueFromFort(uint8_t data);
void GunneryData_Operation(GunneryData *Gun, PID_Value *pos);

extern FortType fort;

#endif


