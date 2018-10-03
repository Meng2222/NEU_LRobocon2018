#ifndef FORT__H
#define FORT__H

#include "stdint.h"
#include "pps.h"
#include "moveBase.h"

#define FORT_TO_BACK_WHEEL (10.5f)                               //炮台到后轮两轮轴中点距离    10.5mm
#define G (9800.0f)                                              //重力加速度                  9800.0mm/s2
#define FORT_ELEVATION_DEG (60.0f)                               //炮台仰角                    60°
#define FORT_ELEVATION_RAD (FORT_ELEVATION_DEG * Pi / 180.0f)    //炮台仰角                    1/3πrad
#define FORT_HEIGHT (128.0f + 60.0f)                             //炮台高度                    188.0mm
#define BUCKET_HEIGHT (800.0f)                                   //桶高度                      800.0mm
#define FORT_TO_BUCKET_HEIGHT (BUCKET_HEIGHT - FORT_HEIGHT)      //炮台到桶高度差              (800.0 - 188.0)mm
#define FORT_TO_LASER_X (40.0f)                                  //炮台到激光器横轴距离        40.0mm
#define FORT_TO_LASER_Y (23.32f)                                 //炮台到激光器纵轴距离        23.32mm
#define LASER_LEFT (0)                                           //左侧激光编号                0
#define LASER_RIGHT (1)                                          //右侧激光编号                1

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
typedef struct                      //                                    2  _   _  1   
{                                   //                                      |     |
	int BucketNum;                  //目标桶号
	int cntIteration;               //迭代次数                              |_   _|
	int BucketQuadrant;             //目标桶位于的象限                    3         0
	int Square_Mode;                //方形走行方向                        Square_Mode = 0时    逆时针
                                    //                                    Square_Mode = 1时    顺时针
	
	float Fort_X;                   //炮台横坐标                          单位mm
	float Fort_Y;                   //炮台纵坐标                          单位mm
	float Fort_Angle;               //炮台航向角实际值换算定位系统角度    单位°
	
	float Bucket_X[4];              //当前目标桶横坐标                    单位mm
	float Bucket_Y[4];              //当前目标桶纵坐标                    单位mm

	float YawPosAngleRec;           //炮台航向角实际值                    单位°
	float YawPosAngleTar;           //炮台航向角目标值                    单位° 
	float YawPosAngleSet;           //炮台航向角设定值                    单位°     
	float YawPosAngleSetAct;        //炮台航向角实际设定值                单位°           
	
	float ShooterVelRec;            //射球电机转速实际值                  单位rad/s
	float ShooterVelSet;            //射球电机转速设定值                  单位rad/s
	float ShooterVelSet_H;          //射球水平速度                        单位mm/s
	float ShooterVelSet_V;          //射球竖直速度                        单位mm/s
	float ShooterVelSetAct;         //射球电机转速实际设定值              单位rad/s
	float ShooterTime;              //飞行时间                            单位s
	
	float Distance_Fort;            //目标桶到炮台的距离                  单位mm	
	float Distance_Fort_X;          //目标桶到炮台的横轴距离              单位mm      
	float Distance_Fort_Y;          //目标桶到炮台的纵轴距离              单位mm
	
	float Distance_Shoot;           //射球实际飞行距离                    单位mm
	float Distance_Shoot_X;         //射球实际飞行横轴距离                单位mm
	float Distance_Shoot_Y;         //射球实际飞行纵轴距离                单位mm
	
	float Distance_Car;             //射球飞行时间中车移动的距离          单位mm
	float Distance_Car_X;           //射球飞行时间中车移动的横轴距离      单位mm
	float Distance_Car_Y;           //射球飞行时间中车移动的纵轴距离      单位mm
	
	float Distance_Accuracy;        //精度要求                            单位mm
	float Distance_Deviation_X;     //横轴距离偏差                        单位mm
	float Distance_Deviation_Y;     //纵轴距离偏差                        单位mm 

	float Distance_Laser_Left;      //左侧激光探测到的距离                单位mm
	float Distance_Laser_Right;     //右侧激光探测到的距离                单位mm

	float No_Offset_Angle;          //无补偿航向角设定值                  单位°
	float No_Offset_Vel;            //无补偿射球电机转速设定值            单位rad/s

	float Yaw_Zero_Offset;          //航向角归零补偿值                    单位°
	float Yaw_Angle_Offset[8];      //航向角补偿值                        单位°
	float Shooter_Vel_Offset[8];    //射球电机转速补偿值                  单位rad/s
}GunneryData;


/**
* @brief  扫描参数结构体
* @author 陈昕炜
* @note   定义一个ScanData类型
*/
typedef struct{    
	int ScanStatus;                   //扫描状态                              0走行   1扫描 2射球
	int FirePermitFlag;               //允许射球                              0不允许 1允许
	int ScanPermitFlag;
	int BucketNum;                    //目标桶号
	int i;
	int DelayFlag;
	int CntDelayTime;	
	int GetBorderLeftFlag;
	int GetBorderRightFlag;
	
	float Fort_X;                     //炮台横坐标                            单位mm
	float Fort_Y;                     //炮台纵坐标                            单位mm
	float Fort_Angle;                 //炮台航向角检测值换算绝对角度          单位°
                                      //                                                   _4 3_
	float Bucket_Border_X[8];         //逆时针排序的挡板边缘横坐标            单位mm     5|     |2
	float Bucket_Border_Y[8];         //逆时针排序的挡板边缘纵坐标            单位mm
	float FortToBorder_Distance_X;    //炮塔到目标桶挡板边缘横轴距离          单位mm     6|_   _|1 
	float FortToBorder_Distance_Y;    //炮塔到目标桶挡板边缘纵轴距离          单位mm        7 0
	
	float ScanAngleTar;               //指向挡板边缘航向角目标值			  单位°
	float ScanAngleEnd;               //位于最右侧的扫描起始航向角设定值	  单位°
	float ScanAngleStart;             //位于最左侧的扫描终止航向角设定值	  单位°

	float Laser_Left_X;               //左侧激光横坐标                        单位mm  
	float Laser_Left_Y;               //左侧激光纵坐标                        单位mm
	float Laser_Right_X;              //右侧激光横坐标                        单位mm
	float Laser_Right_Y;              //右侧激光纵坐标                        单位mm
	
	float Probe_Left_Distance;        //左侧激光探测距离值                    单位mm	
	float Probe_Left_X;               //左侧激光探测点横坐标                  单位mm
	float Probe_Left_Y;	              //左侧激光探测点纵坐标                  单位mm
	float Probe_Right_Distance;       //右侧激光探测距离值                    单位mm	
	float Probe_Right_X;              //右侧激光探测点横坐标                  单位mm
	float Probe_Right_Y;	          //右侧激光探测点纵坐标                  单位mm
	
	float Probe_Border_Left_X_Temp;
	float Probe_Border_Left_Y_Temp;
	float Probe_Border_Right_X_Temp;
	float Probe_Border_Right_Y_Temp;
	
	float Probe_Border_Left_X;        //左侧挡板边缘横坐标					  单位mm
	float Probe_Border_Left_Y;        //左侧挡板边缘纵坐标					  单位mm
	float Probe_Border_Left_Angle;
	float Probe_Border_Right_X;       //右侧挡板边缘横坐标					  单位mm
	float Probe_Border_Right_Y;		  //右侧挡板边缘纵坐标					  单位mm
	float Probe_Border_Right_Angle;
	
	float FortToBucket_Distance_X;    //炮塔到目标桶横轴距离                  单位mm
	float FortToBucket_Distance_Y;    //炮塔到目标桶纵轴距离                  单位mm	

	float Probe_Bucket_X;             //探测到的目标桶横坐标                  单位mm
	float Probe_Bucket_Y;             //探测到的目标桶纵坐标                  单位mm
	float Probe_Bucket_Distance;      //探测到的目标桶距离                    单位mm

	float ShooterVelRec;              //射球电机转速实际值                    单位rad/s
	float ShooterVelSet;              //射球电机转速设定值                    单位rad/s
	float YawPosAngleRec;             //炮台航向角实际值                      单位°
	float YawPosAngleTar;             //炮台航向角目标值                      单位°              
	float YawPosAngleSet;             //炮台航向角设定值                      单位°
	float Yaw_Zero_Offset;            //航向角归零补偿值                      单位°
	float YawPosAngle_Offset;         //射球电机转速补偿值                    单位rad/s		
	float Shooter_Vel_Offset;         //射球电机转速补偿值                    单位rad/s		
}ScanData;


void YawPosCtrl(float ang);
void ShooterVelCtrl(float rps);
void ReadShooterVel(void);
void ReadYawPos(void);
void ReadLaserAValue(void);
void ReadLaserBValue(void);
void GetValueFromFort(uint8_t data);

void GunneryData_Operation(GunneryData *Gun, PID_Value const *Pos);
void Scan_Operation(ScanData *Scan, PID_Value const *Pos, int targets[]);
float Tar_Angle_Operation(float Distance_X, float Distance_Y);

extern FortType fort;

#endif


