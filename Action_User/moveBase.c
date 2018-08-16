/**
  ******************************************************************************
  * @file	  moveBase.c
  * @author	  Action
  * @version   V1.0.0
  * @date	  2018/08/09
  * @brief	 2018省赛底盘运动控制部分
  ******************************************************************************
  * @attention
  *			None
  ******************************************************************************
  */
/* Includes -------------------------------------------------------------------------------------------*/

#include "moveBase.h"
#include <math.h>
#include "elmo.h"

/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/

/*****************定义的一些全局变量用于串口返回值****************************/

struct usartValue_{
	uint32_t cnt;//用于检测是否数据丢失
	float xValue;//串口输出x坐标
	float yValue;//串口输出y坐标
	float angleValue;//串口输出角度值
	float pidValueOut;//PID输出
	float d;//距离
	float turnAngleValue;//
	uint8_t flagValue;
}usartValue;

/**
  * @brief  开环走直线
  * @note	
  * @param  speed：给定速度
  * @retval None
  */

void Straight(float speed)
{
	int32_t pulseNum=(speed*4095*1000)/(PI*WHEEL_DIAMETER);
	
	VelCrl(CAN2, 0x01,pulseNum);
	VelCrl(CAN2, 0x02,-pulseNum);
}

/**
  * @brief  开环转圈
  * @note	
* @param  rightSpeed：给定右轮速度,正数为逆时针转,为外侧速度；负数顺时针转，为内侧速度
  * @param  radius：给定半径
  * @retval None
  */

void Round(float rightSpeed,float radius)
{
	float leftSpeed;
	int32_t rightPulseNum;
	int32_t leftPulseNum;
	//逆时针转
	if(rightSpeed > 0.0)
	{
		rightPulseNum=(rightSpeed*4095*1000)/(PI*WHEEL_DIAMETER);
		leftSpeed=rightSpeed*(radius-(WHEEL_TREAD/1000))/(radius+(WHEEL_TREAD/1000));
		leftPulseNum=-(leftSpeed*4095*1000)/(PI*WHEEL_DIAMETER);
		
		VelCrl(CAN2, 0x01,rightPulseNum);
		VelCrl(CAN2, 0x02,leftPulseNum);
		
	}
	//顺时针转
	else
	{
		rightPulseNum=-(rightSpeed*4095*1000)/(PI*WHEEL_DIAMETER);
		leftSpeed=rightSpeed*(radius+(WHEEL_TREAD/1000))/(radius-(WHEEL_TREAD/1000));
		leftPulseNum=(leftSpeed*4095*1000)/(PI*WHEEL_DIAMETER);
		
		VelCrl(CAN2, 0x01,rightPulseNum);
		VelCrl(CAN2, 0x02,leftPulseNum);
	}
}

/**
  * @brief  PID 转弯
  * @note	
* @param  angle：给定角度,为正左转，为负右转
  * @param  getAngle：现在的角度
  * @retval None
  */

void Turn(float angle)
{
	int32_t pulseNum=0;
	int32_t bPulseNum=(1000*4095)/(PI*WHEEL_DIAMETER);
	float getAngle=0;
	float speed=0;
	getAngle=GetAngle();
	
	speed=AnglePid(angle,getAngle);	
	usartValue.pidValueOut=speed;
	
	pulseNum=(speed*4095)/(PI*WHEEL_DIAMETER);
	VelCrl(CAN2, 0x01,bPulseNum+pulseNum);
	VelCrl(CAN2, 0x02,pulseNum-bPulseNum);

}	
 
/**
  * @brief  PID 回正
  * @note	
  * @param  angle：给定角度
  * @param  getAngle：现在的角度
  * @retval None
  */

void BTP(float angle)
{
	int32_t pulseNum=0;
	int32_t bPulseNum=(700*4095)/(PI*WHEEL_DIAMETER);
	float getAngle=0;
	float speed=0;
	getAngle=GetAngle();
	
	speed=AnglePid(angle,getAngle);
	usartValue.pidValueOut=speed;
	
	pulseNum=(speed*4095)/(PI*WHEEL_DIAMETER);
	
	VelCrl(CAN2, 0x01,bPulseNum+pulseNum);
	VelCrl(CAN2, 0x02,pulseNum-bPulseNum);

}

/**
  * @brief  走方形
  * @note	
  * @param  
  * @retval None
  */

void Square(void)
{
	float x;
	float y;
	float angle;
	
	static uint8_t flg=0;
	
	x=GetPosX();
	y=GetPosY();
	angle=GetAngle();
	
	usartValue.xValue=x;
	usartValue.yValue=y;
	usartValue.angleValue=angle;
	
	if(flg == 0)
	{
		if(y > 1600.0)
		{
			flg++;
		}
		else
		{
			Turn(0.0);
		}
	}
	else if(flg == 1)
	{
		if(x > 1600.0)
		{
			flg++;
		}
		else
		{
			Turn(-90.0);
		}
	}
	else if(flg == 2)
	{
		if(y < 400.0)
		{
			flg++;
		}
		else
		{
			Turn(-180.0);
		}
	}
	else if(flg == 3)
	{
		if(x < 400.0)
		{
			flg=0;
		}
		else
		{
			Turn(90.0);
		}
	}
	
}

/**
  * @brief  沿直线走，能回到直线
  * @note	给定直线以Ax+By+C=0形式
  * @param  A：
  * @param  B:
  * @param  C:
  * @param  dir:dir为0，向右；dir为1，向左
  * @retval None
  */

void straightLine(float A,float B,float C,uint8_t dir)
{
	float setAngle=0;
	float getAngle=GetAngle();
	float getX=GetPosX();
	float getY=GetPosY();
	float distance=((A*getX)+(B*getY)+C)/sqrt(A*A+B*B);
	float angleAdd=distance*0.1;
	if(angleAdd > 90)
	{
		angleAdd=90;
	}
	else if(angleAdd < -90)
	{
		angleAdd=-90;
	}
	
	
	if((B > -0.005) && (B < 0.005))
	{
		if(!dir)
		{
			setAngle=0;
		}
		else
		{
				if(A>0)
				{
					setAngle=-180;
				}
				else
				{
					setAngle=180;
				}
		}
	}
	else
	{
		if(!dir)
		{
			setAngle=(atan(-A/B)*180/PI)-90;
		}
		else
		{
			setAngle=(atan(-A/B)*180/PI)+90;
		}
	}

	
	
	usartValue.d=distance;
	usartValue.xValue=getX;
	usartValue.yValue=getY;
	usartValue.angleValue=getAngle;
	if(distance > 10)
	{
		if(setAngle < 0)
		{
			Turn(setAngle-angleAdd);
			usartValue.turnAngleValue=setAngle-angleAdd;
		}
		else
		{
			Turn(setAngle+angleAdd);
			usartValue.turnAngleValue=setAngle+angleAdd;			
		}
	}
	else if(distance < -10)
	{
		if(setAngle >= 0)
		{
			Turn(setAngle+angleAdd);
			usartValue.turnAngleValue=setAngle+angleAdd;
		}
		else
		{
			Turn(setAngle-angleAdd);
			usartValue.turnAngleValue=setAngle-angleAdd;			
		}
	}
	else
	{
		Turn(setAngle);
		usartValue.turnAngleValue=setAngle;
	}
	
}

/**
  * @brief  走方形，面积渐渐减小，偏离轨道后能回去
  * @note	
  * @param 
  * @retval None
  */

void ShrinkSquare(void)
{
	float sAngle=GetAngle();
	float sX=GetPosX();
	float sY=GetPosY();
	static uint8_t flag=0;
	usartValue.flagValue=flag;
	switch(flag)
	{
		case 0:
			if(sY < 2500)
				straightLine(1,0,0,0);
			else
				flag++;
				break;
		case 1:
			if(sX < 2500)
				straightLine(0,1,-3000,0);
			else
				flag++;
				break;
		case 2:
			if(sY > 1000)
				straightLine(1,0,-3000,1);
			else
				flag++;
				break;
		case 3:
			if(sX > 1000)
				straightLine(0,1,-500,1);
			else
				flag++;
				break;
		case 4:
			if(sY < 2000)
				straightLine(1,0,-500,0);
			else
				flag++;
				break;
		case 5:
			if(sX < 2000)
				straightLine(0,1,-2500,0);
			else
				flag++;
				break;
		case 6:
			if(sY > 1500)
				straightLine(1,0,-2500,1);
			else
				flag++;
				break;
		case 7:
			if(sX > 1500)
				straightLine(0,1,-1000,1);
			else
				flag++;
				break;
		case 8:
			if(sY < 1500)
				straightLine(1,0,-1000,0);
			else
				flag++;
				break;
		case 9:
			if(sX < 1500)
				straightLine(0,1,-2000,0);
			else
				flag++;
				break;
		case 10:
			if(sY > 1000)
				straightLine(1,0,-2000,1);
			else
				flag=0;
				break;	
		default: break;
			
	}

}

/**
  * @brief  直线闭环方形 2000*2000
  * @note	
  * @param 
  * @retval None
  */

void SquareTwo(void)
{
	static uint8_t flagOne=0;
	float sTAngle=GetAngle();
	float sTX=GetPosX();
	float sTY=GetPosY();
	usartValue.flagValue=flagOne;
	switch(flagOne)
	{
		case 0:
			if(sTY < 1300)
				straightLine(1,0,0,0);
			else
				flagOne++;
			break;
		case 1:
			if(sTX < 1350)
				straightLine(0,1,-2000,0);
			else
				flagOne++;
			break;
		case 2:
			if(sTY > 650)
				straightLine(1,0,-2000,1);
			else
				flagOne++;
			break;
		case 3:
			if(sTX > 650)
				straightLine(0,1,0,1);
			else
				flagOne=0;
			break;
		default: flagOne=0;break;
	}

}
/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
