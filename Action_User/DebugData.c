#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "pps.h"
#include "fort.h"
#include "Walk.h" 
#include "PushBall.h" 
#include "Point.h" 
#include "PushBall.h"
#include "DebugData.h"

/***************************************/
/*****    extern 变量区            *****/
/***************************************/
extern   pos_t action;
extern int roundCnt;
extern int err_kind;
extern float goRealAng;
extern union push_p push_position;
extern int pushBallpos;
extern int barrel,errorTime,errorLockTime,aimErrorFlag,aimErrorTime,errorLockFlag;
extern int ifShoot[],ifShootFlag[];
extern int normalPush;
extern int stableFlg;
extern float AimAngle;
extern float adcAngle;
extern int Ballcolor[5];
void debugdata(void)
{
	USART_OUT( UART4, (uint8_t*)"%d\t", roundCnt);
	USART_OUT( UART4, (uint8_t*)"%d\t", stableFlg);
	USART_OUT( UART4, (uint8_t*)"%d\t%d\t%d\t", (int)action.x,(int)action.y,(int)action.angle);
	USART_OUT( UART4, (uint8_t*)"%d\t", err_kind);
	USART_OUT( UART4, (uint8_t*)"%d\t%d\t%d\t%d\t%d\t%d\t",barrel,errorTime,aimErrorTime,aimErrorFlag,errorLockTime,errorLockFlag);	
	USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t%d\t",ifShoot[ 0],ifShoot[1],ifShoot[2],ifShoot[3]);
	USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t%d\t%d\t",ifShootFlag[0],ifShootFlag[1],ifShootFlag[2],ifShootFlag[3],(int)adcAngle);
	USART_OUT( UART4, (uint8_t*)"%d  %d\t",(int)ReadLaserAValue(),(int)ReadLaserBValue());	
	USART_OUT( UART4, (uint8_t*)"%d ", (int)GetWZ());
	USART_OUT( UART4, (uint8_t*)"%d ", (int)pushBallpos);
	USART_OUT( UART4, (uint8_t*)"%d ", (int)push_position.push_pos[1]);
	USART_OUT( UART4, (uint8_t*)"%d ", (int)goRealAng);
	USART_OUT( UART4, (uint8_t*)"%d ", (int)fort.yawPosReceive);
	USART_OUT( UART4, (uint8_t*)"%d ", (int)GetRollV());
	USART_OUT( UART4, (uint8_t*)"%d ", (int)fort.shooterVelReceive);
	USART_OUT( UART4, (uint8_t*)"%d ", (int)normalPush);	
	USART_OUT( UART4, (uint8_t*)"%d ", (int)goRealAng);
	USART_OUT( UART4, (uint8_t*)"%d ", (int)Ballcolor[2]);
	
}