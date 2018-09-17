#ifndef FORT__H
#define FORT__H

#include "stdint.h"
#include "pps.h"
#include "moveBase.h"
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
* @note   定义一个GunneryData类型
*/
typedef struct
{
	int BucketNum;                  //目标桶号
	int cntIteration;               //迭代次数
	int Square_Mode;
	
	float Fort_X;                   //炮台横坐标                          单位mm
	float Fort_Y;                   //炮台纵坐标                          单位mm
	float Fort_Angle;               //炮台航向角检测值换算绝对角度        单位°
	
	float Bucket_X[4];              //当前目标桶横坐标                    单位mm
	float Bucket_Y[4];              //当前目标桶纵坐标                    单位mm

	float YawPosAngleRec;           //炮台航向角检测值                    单位°
	float YawPosAngleTar;           //炮台相对偏移角度                    单位° 
	float YawPosAngleSet;           //炮台设定角度                        单位°
	float YawPosAngleSetAct;
	
	float ShooterVelRec;            //射球电机转速检测值                  单位rad/s
	float ShooterVelSet;            //射球设定转速                        单位mm/s               
	float ShooterVelSet_H;          //射球水平速度                        单位mm/s
	float ShooterVelSet_V;          //射球竖直速度                        单位mm/s
	float ShooterVelSetAct;
	float ShooterTime;              //飞行时间                            单位s
	
	float Distance_Fort;            //炮台到目标点的距离                  单位mm	
	float Distance_Fort_X;          //炮台到目标点的横轴距离              单位mm      
	float Distance_Fort_Y;          //炮台到目标点的纵纵距离              单位mm
	
	float Distance_Shoot;           //射球实际飞行距离                    单位mm
	float Distance_Shoot_X;         //射球实际飞行横轴距离                单位mm
	float Distance_Shoot_Y;         //射球实际飞行纵轴距离                单位mm
	
	float Distance_Car;             //射球飞行时间中车移动的距离          单位mm
	float Distance_Car_X;           //射球飞行时间中车移动的横轴距离      单位mm
	float Distance_Car_Y;           //射球飞行时间中车移动的纵轴距离      单位mm
	
	float Distance_Accuracy;        //精度要求                            单位mm
	float Distance_Deviation_X;     //横轴距离偏差                        单位mm
	float Distance_Deviation_Y;     //纵轴距离偏差                        单位mm 

	float Distance_LaserA;          //激光A探测到的距离值                 单位mm
	float Distance_LaserB;          //激光B探测到的距离值                 单位mm
	
	float No_Offset_Angle;          //无补偿航向角                        单位°
	float No_Offset_ShooterVel;     //无补偿射球电机转速                  单位rad/s
	
	float Yaw_Zero_Offset;          //航向角归零补偿                      单位°
	float Yaw_Angle_Offset[8];      //航向角补偿                          单位°
	float Shooter_Vel_Offset[8];    //射球转速补偿                        单位rad/s
}GunneryData;

/**
* @brief  PID控制参数
* @author 陈昕炜
*/
typedef struct{
	float setValue;             //系统待调节量的设定值 
    float feedbackValue;        //系统待调节量的反馈值，就是传感器实际测量的值    
     
    float Kp;                   //比例系数
    float Ki;                   //积分系数
    float Kd;                   //微分系数
     
    float error;                //当前偏差
    float error_pre;            //前一步的偏差
	float error_sum;            //偏差累计
	float error_def;            //当前偏差与上一偏差的差值
    
    float output;               //PID控制器的输出
    float flag;              //输出上限
    float out_min;              //输出下限
}PID_Value_Fort;

void YawPosCtrl(float ang);
void ShooterVelCtrl(float rps);
void ReadShooterVel(void);
void ReadYawPos(void);
void ReadLaserAValue(void);
void ReadLaserBValue(void);
void GetValueFromFort(uint8_t data);

void GunneryData_Operation(GunneryData *Gun, PID_Value *Pos);
float YawPos_Angle_PID_Operation(float YawPosAngle_Tar, float YawPosAngle_Rec, PID_Value_Fort *PID);
float Shooter_Vel_PID_Operation(float Shooter_Vel_Tar, float Shooter_Vel_Rec, PID_Value_Fort *PID);
float PID_Operation(PID_Value_Fort *PID);

extern FortType fort;

#endif


