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
void ReadShooterVel(void);
void ReadYawPos(void);
void ReadLaserAValue(void);
void ReadLaserBValue(void);
void GetValueFromFort(uint8_t data);

extern FortType fort;
struct member
{  
   uint32_t pos_v;
   uint32_t get_number_vorpos;
};
union push_ball_ActualPos_v
{
	
	uint8_t datas[2];
	uint32_t member[2];
	
};

#endif


