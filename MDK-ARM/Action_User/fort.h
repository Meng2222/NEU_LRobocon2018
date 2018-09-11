#ifndef FORT__H
#define FORT__H

#include "stdint.h"

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
float ReadShooterVel(void);
float ReadYawPos(void);
float ReadLaserAValue(void);
float ReadLaserBValue(void);
float ReadRps(void);
void GetValueFromFort(uint8_t data);

extern FortType fort;

#endif


