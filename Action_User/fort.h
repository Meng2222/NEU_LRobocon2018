#ifndef FORT__H  
#define FORT__H

#include "stdint.h"


#define BUCKET_ONE_X (-2200.0)
#define BUCKET_ONE_Y 105.0
#define BUCKET_TWO_X (-2200.0)
#define BUCKET_TWO_Y 4500.0
#define BUCKET_THR_X 2150.0
#define BUCKET_THR_Y 4600.0
#define BUCKET_FOR_X 2200.0
#define BUCKET_FOR_Y 105.0
#define SHOOOT_KP    0.012
#define LASER_SCALE  2.48545 //一脉冲值代表2.48545毫米


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
extern FortType fort;


#endif


