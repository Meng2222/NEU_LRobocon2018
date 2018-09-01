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
	int cntIteration;
	
	float Fort_X;                  //炮台横坐标                      单位mm
	float Fort_Y;                  //炮台纵坐标                      单位mm
	float Fort_Angle;              //炮台绝对角度                    单位°
	float CarVel_X;
    float CarVel_Y;

	float Distance_X;              //炮台到目标点的相对横坐标距离    单位mm      
	float Distance_Y;              //炮台到目标点的相对纵坐标距离    单位mm  
	float Distance_Fort;           //炮台到目标点的相对距离          单位mm	
	
	float YawPos;
	float YawPosTarAngle;          //炮台相对偏移角度                单位° 
	float YawPosTarActAngle;       //炮台设定角度                    单位° 
	
	float ShooterVel;
	float ShooterVelSet;           //射球设定转速                    单位mm/s               
	float ShooterVelSet_H;         //射球水平速度                    单位mm/s
	float ShooterVelSet_V;         //射球竖直速度                    单位mm/s
	float ShooterTime;             //飞行时间                        单位s
		
	float Distance_Shoot;
	float Distance_Shoot_X;        //射球实际飞行横坐标距离          单位mm
	float Distance_Shoot_Y;        //射球实际飞行纵坐标距离          单位mm
	float Distance_Car;
	float Distance_Car_X;
	float Distance_Car_Y;          //射球飞行时间中车移动的距离      单位mm

	float Angle_Deviation;
	float Distance_Accuracy;       //精度要求                        单位mm
	float Distance_Deviation_X;    //距离偏差                        单位mm
	float Distance_Deviation_Y;
	float Yaw_Angle_Offset;        //航向角补偿                      单位°
	float Shooter_Vel_Offset;      //射球转速补偿                    单位rad/s
	
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


