#ifndef FORT__H  
#define FORT__H

#include "stdint.h"


#define BUCKET_ONE_X (-2200.0)
#define BUCKET_ONE_Y 105.0
#define BUCKET_TWO_X (-2200.0)
#define BUCKET_TWO_Y 4600.0
#define BUCKET_THR_X 2200.0
#define BUCKET_THR_Y 4600.0
#define BUCKET_FOR_X 2200.0
#define BUCKET_FOR_Y 105.0
#define SHOOOT_KP    0.012
#define LASER_SHOOOT_KP  (SHOOOT_KP*LASER_SCALE)
#define LASER_SCALE  2.48545 //一脉冲值代表2.48545毫米
#define SHOOT_SPEED_INTEGRAL 48
#define TIME_DIFF_1_ONE 50
#define TIME_DIFF_1_TWO 65
#define TIME_DIFF_1_THR 60
#define TIME_DIFF_1_FOR 65

#define TIME_DIFF_2_ONE 50
#define TIME_DIFF_2_TWO 65
#define TIME_DIFF_2_THR 65
#define TIME_DIFF_2_FOR 65

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

void YawPosCtrl(float ang);
void ShooterVelCtrl(float rps);
void ReadShooterVel(void);
void ReadYawPos(void);
void ReadLaserAValue(void);
void ReadLaserBValue(void);
void GetValueFromFort(uint8_t data);
void Shoot(uint8_t flg,uint16_t pushTime);
void CarStuck(void);
void NormalShootOne(uint16_t getPushTime);
void NormalShootTwo(uint16_t getPushTime);
void LaserCalibration(float getreferenceDistance);
extern FortType fort;


#endif


