#ifndef FORT__H  
#define FORT__H

#include "stdint.h"
#include "app_cfg.h"
// 宏定义推球电机ID
#define PUSH_BALL_ID (7)

//四个桶所在坐标
#define BUCKET_ONE_X (-2200.0)
#define BUCKET_ONE_Y 135.0
#define BUCKET_TWO_X (-2200.0)
#define BUCKET_TWO_Y 4535.0
#define BUCKET_THR_X 2200.0
#define BUCKET_THR_Y 4535.0
#define BUCKET_FOR_X 2200.0
#define BUCKET_FOR_Y 135.0

//由距离算射球电机转速的比例项
#define SHOOT_KP    0.013f

//由距离算射球电机转速的截距
#define SHOOT_INTERCEPT  39.859f

//激光A的参数
#define LASER_SCALE_A  2.4694f //一脉冲值代表2.4817毫米
#define LASER_INTERCEPT_A  30.992f

//激光B的参数
#define LASER_SCALE_B  2.4811f //一脉冲值代表2.4777毫米
#define LASER_INTERCEPT_B  18.299f

//射球距离的限定，外圈在该距离内才打球
#define SHOOT_D_ONE 2000.0f
#define SHOOT_D_TWO 3800.0f

//车速的限定，中圈达到该速度才打球
#define SPEED_ONE 1300.0f

//延时时间
#define DELAY_TIME 200

//车速的限定，定点小于该速度才打球
#define SPEED_TWO 30.0f

//转速误差范围
#define SHOOT_ERR_1 2
#define SHOOT_ERR_2 3

//识别没有球
#define NO_BALL    0

typedef union
{	
	uint8_t data8[4];
	int			data32;
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

struct comend{
	usartData_t uv4Data;
	float shoot;
	float turn;

};


void YawPosCtrl(float ang);
void ShooterVelCtrl(float rps);
void ReadShooterVel(void);
void ReadYawPos(void);
void ReadLaserAValue(void);
void ReadLaserBValue(void);
void GetValueFromFort(uint8_t data);
void UARTCmd(uint8_t data);
void Shoot(uint8_t flg);
void CarStuck(void);
void NormalShootOne(void);
void Laser_Aim(void);
void Laser_Aim_Two(void);
void BallColorRecognition(void);
extern FortType fort;


#endif


