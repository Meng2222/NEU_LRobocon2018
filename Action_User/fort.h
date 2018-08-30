#ifndef FORT__H
#define FORT__H

#include "stdint.h"
#include "pps.h"
#include "PID.h"
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
	int BucketNum;            //目标桶号
	
	float Fort_x;             //炮台横坐标                      单位mm
	float Fort_y;             //炮台纵坐标                      单位mm
	float Fort_angle;         //炮台角度                        单位°
	float detVel;             //检测当前车速                    单位mm/s


	float Distance_X;         //炮台到目标点的相对横坐标距离    单位mm      
	float Distance_Y;         //炮台到目标点的相对纵坐标距离    单位mm  
	float Distance_Fort;      //炮台到目标点的相对距离          单位mm      
	
	float YawPosTarAngle;     //炮台相对偏移角度                单位° 
	float YawPosTarActAngle;  //炮台设定角度                    单位° 
	
	float ShooterVelSet;      //射球设定转速                    单位mm/s               
	float ShooterVelSetA;     //射球水平速度                    单位mm/s
	float ShooterVelSetA_Y;   //射球水平速度y轴分量             单位mm/s
	float ShooterVelSetB;     //射球竖直速度                    单位mm/s
	float ShooterTime;        //飞行时间                        单位s
}GunneryData;

void YawPosCtrl(float ang);
void ShooterVelCtrl(float rps);
void ReadShooterVel(void);
void ReadYawPos(void);
void ReadLaserAValue(void);
void ReadLaserBValue(void);
void GetValueFromFort(uint8_t data);
void GunneryData_Operation(GunneryData *Gun,PID_Value *pos);

extern FortType fort;

#endif


