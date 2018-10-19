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
extern float go_real_send_angle;
extern union push_p push_position;
extern int PushBallPosition;
extern int Barrel,errorTime,errorLockTime,aimErrorFlag,aimErrorTime,errorLockFlag;
extern int if_shoot[],if_shootFlag[];
extern int normal_push;
extern int stable_flag;
extern float AimAngle;
extern float AdcAngle;
extern int Ballcolor[5];
void debugdata(void)
{
	USART_OUT( UART4, (uint8_t*)"%d\t", roundCnt);
	USART_OUT( UART4, (uint8_t*)"%d\t", stable_flag);
	USART_OUT( UART4, (uint8_t*)"%d\t%d\t%d\t", (int)action.x,(int)action.y,(int)action.angle);
	USART_OUT( UART4, (uint8_t*)"%d\t", err_kind);
	USART_OUT( UART4, (uint8_t*)"%d\t%d\t%d\t%d\t%d\t%d\t",Barrel,errorTime,aimErrorTime,aimErrorFlag,errorLockTime,errorLockFlag);	
	USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t%d\t",if_shoot[ 0],if_shoot[1],if_shoot[2],if_shoot[3]);
	USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t%d\t%d\t",if_shootFlag[0],if_shootFlag[1],if_shootFlag[2],if_shootFlag[3],(int)AdcAngle);
	USART_OUT( UART4, (uint8_t*)"%d  %d\t",(int)ReadLaserAValue(),(int)ReadLaserBValue());	
	USART_OUT( UART4, (uint8_t*)"%d ", (int)GetWZ());
	USART_OUT( UART4, (uint8_t*)"%d ", (int)PushBallPosition);
	USART_OUT( UART4, (uint8_t*)"%d ", (int)push_position.push_pos[1]);
	USART_OUT( UART4, (uint8_t*)"%d ", (int)go_real_send_angle);
	USART_OUT( UART4, (uint8_t*)"%d ", (int)fort.yawPosReceive);
	USART_OUT( UART4, (uint8_t*)"%d ", (int)get_roll_v());
	USART_OUT( UART4, (uint8_t*)"%d ", (int)fort.shooterVelReceive);
	USART_OUT( UART4, (uint8_t*)"%d ", (int)normal_push);	
	USART_OUT( UART4, (uint8_t*)"%d ", (int)go_real_send_angle);
	USART_OUT( UART4, (uint8_t*)"%d ", (int)Ballcolor[2]);
	
}