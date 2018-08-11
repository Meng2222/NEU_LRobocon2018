#include "status.h"
#include "elmo.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/**
* @brief  速度转对应脉冲
* @param  v；设定速度
* @author ACTION
* @note 单位 m/s
*/
int exchange(float v)
{
	int pulse=0;
	pulse=v*4096/0.335352;
	return pulse;
}

/**
* @brief  直走
* @param  v；设定速度
* @author ACTION
* @note 单位 m/s
*/
void straight(float v)
{
	int pulse=0;
	pulse=exchange(v);
	VelCrl(CAN2,0x01,pulse);
	VelCrl(CAN2,0x02,-pulse);
}

/**
* @brief  转圈
* @param  v；设定速度
* @param  r；设定半径
* @param  direction：左右转向（Left左转，Right右转）
* @author ACTION
* @note 单位 v:m/s，r：m 半径为车轴中心到圆心距离，r>=l/2,l=0.434m
*/
void circular(float v,float r,char direction)
{
	float v_big=0,v_small=0,pulse_big=0,pulse_small=0;
	v_big=v+0.434*v/(2*r);
	v_small=v-0.434*v/(2*r);
	pulse_big=exchange(v_big);
	pulse_small=exchange(v_small);
	switch(direction)
	{
		case Left:	VelCrl(CAN2,0x01,pulse_big);
								VelCrl(CAN2,0x02,-pulse_small);
								break;
		case Right: VelCrl(CAN2,0x02,pulse_big);
								VelCrl(CAN2,0x01,-pulse_small);
								break;
	}
}