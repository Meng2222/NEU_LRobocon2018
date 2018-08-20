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
	float leftSpeed=0;
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

void Turn(float angle,float gospeed)
{
	int32_t pulseNum=0;
	int32_t bPulseNum=(gospeed*4095)/(PI*WHEEL_DIAMETER);
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
  * @brief  PID 转弯
  * @note	
* @param  angle：给定角度,为正左转，为负右转
  * @param  getAngle：现在的角度
  * @retval None
  */

void BackTurn(float angle,float gospeed)
{
	int32_t pulseNum=0;
	int32_t bPulseNum=-(gospeed*4095)/(PI*WHEEL_DIAMETER);
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
			Turn(0.0,1000);
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
			Turn(-90.0,1000);
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
			Turn(-180.0,1000);
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
			Turn(90.0,1000);
		}
	}
	
}

/**
  * @brief  后退走直线
  * @note	
  * @param  
  * @retval None
  */

uint8_t BackstraightLine(float A2,float B2,float C2,uint8_t dir)
{
	float setAngle=0;
	float getAngle=GetAngle();
	float getX=GetPosX();
	float getY=GetPosY();
	float distance=((A2*getX)+(B2*getY)+C2)/sqrt(A2*A2+B2*B2);
	float angleAdd=DistancePid(distance,0);
	if(angleAdd > 90)
	{
		angleAdd=90;
	}
	else if(angleAdd < -90)
	{
		angleAdd=-90;
	}
	
	
	if((B2 > -0.005) && (B2 < 0.005))
	{
		if(!dir)
		{
			if(A2 > 0)
				{
					setAngle=-180;
				}
				else
				{
					setAngle=180;
				}
		}
		else
		{
				setAngle=0;
		}
	}
	else
	{
		if(!dir)
		{
			setAngle=(atan(-A2/B2)*180/PI)-90;
		}
		else
		{
			setAngle=(atan(-A2/B2)*180/PI)+90;
		}
	}

	
	
	usartValue.d=distance;
	usartValue.xValue=getX;
	usartValue.yValue=getY;
	usartValue.angleValue=getAngle;
	if(distance > 10)
	{
		if(setAngle >= 0)
		{
			BackTurn(setAngle-angleAdd,1000);
			usartValue.turnAngleValue=setAngle-angleAdd;
		}
		else
		{
			BackTurn(setAngle+angleAdd,1000);
			usartValue.turnAngleValue=setAngle+angleAdd;			
		}
	}
	else if(distance < -10)
	{
		if(setAngle <= 0)
		{
			BackTurn(setAngle+angleAdd,1000);
			usartValue.turnAngleValue=setAngle+angleAdd;
		}
		else
		{
			BackTurn(setAngle-angleAdd,1000);
			usartValue.turnAngleValue=setAngle-angleAdd;			
		}
	}
	else
	{
		BackTurn(setAngle,1000);
		usartValue.turnAngleValue=setAngle;
	}
	if((distance < 20) && (distance > -20))
		return 1;
	else
		return 0; 
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

void straightLine(float A1,float B1,float C1,uint8_t dir)
{
	float setAngle=0;
	float getAngle=GetAngle();
	float getX=GetPosX();
	float getY=GetPosY();
	float distance=((A1*getX)+(B1*getY)+C1)/sqrt(A1*A1+B1*B1);
	float angleAdd=DistancePid(distance,0);
	if(angleAdd > 90)
	{
		angleAdd=90;
	}
	else if(angleAdd < -90)
	{
		angleAdd=-90;
	}
	
	
	if((B1 > -0.005) && (B1 < 0.005))
	{
		if(!dir)
		{
			setAngle=0;
		}
		else
		{
				if(A1 > 0)
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
			setAngle=(atan(-A1/B1)*180/PI)-90;
		}
		else
		{
			setAngle=(atan(-A1/B1)*180/PI)+90;
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
			Turn(setAngle-angleAdd,1000);
			usartValue.turnAngleValue=setAngle-angleAdd;
		}
		else
		{
			Turn(setAngle+angleAdd,1000);
			usartValue.turnAngleValue=setAngle+angleAdd;			
		}
	}
	else if(distance < -10)
	{
		if(setAngle >= 0)
		{
			Turn(setAngle+angleAdd,1000);
			usartValue.turnAngleValue=setAngle+angleAdd;
		}
		else
		{
			Turn(setAngle-angleAdd,1000);
			usartValue.turnAngleValue=setAngle-angleAdd;			
		}
	}
	else
	{
		Turn(setAngle,1000);
		usartValue.turnAngleValue=setAngle;
	}
	
}

/**
  * @brief  顺时针走方形，面积渐渐变大，偏离轨道后能回去
  * @note	
  * @param 
  * @retval None
  */
uint8_t squareFlag=0;

void BiggerSquareOne(void)
{
	float sAngle=GetAngle();
	float sX=GetPosX();
	float sY=GetPosY();

	switch(squareFlag)
	{
		case 0:
			if(sY < 2300)
				straightLine(1,0,600,0);
			else
				squareFlag++;
				break;
		case 1:
			if(sX < 100)
				straightLine(0,1,-2800,0);
			else
				squareFlag++;
				break;
		case 2:
			if(sY > 2100)
				straightLine(1,0,-600,1);
			else
				squareFlag++;
				break;
		case 3:
			if(sX > -600)
				straightLine(0,1,-1600,1);
			else
				squareFlag++;
				break;
		case 4:
			if(sY < 2800)
				straightLine(1,0,1100,0);
			else
				squareFlag++;
				break;
		case 5:
			if(sX < 600)
				straightLine(0,1,-3300,0);
			else
				squareFlag++;
				break;
		case 6:
			if(sY > 1600)
				straightLine(1,0,-1100,1);
			else
				squareFlag++;
				break;
		case 7:
			if(sX > -1100)
				straightLine(0,1,-1100,1);
			else
				squareFlag++;
				break;
		case 8:
			if(sY < 3300)
				straightLine(1,0,1600,0);
			else
				squareFlag++;
				break;
		case 9:
			if(sX < 1100)
				straightLine(0,1,-3800,0);
			else
				squareFlag++;
				break;
		case 10:
			if(sY > 1100)
				straightLine(1,0,-1600,1);
			else
				squareFlag++;
		case 11:
			if(sX > -1100)
				straightLine(0,1,-600,1);
			else
				squareFlag=8;
				break;	
		default: squareFlag=0;break;
			
	}
	usartValue.flagValue=squareFlag;
}

/**
  * @brief  逆时针走方形，面积渐渐变大，偏离轨道后能回去
  * @note	
  * @param 
  * @retval None
  */

void BiggerSquareTwo(void)
{
	float sAngle=GetAngle();
	float sX=GetPosX();
	float sY=GetPosY();
	
	switch(squareFlag)
	{
		case 0:
			if(sY < 2300)
				straightLine(1,0,-600,0);
			else
				squareFlag++;
				break;
		case 1:
			if(sX > -100)
				straightLine(0,1,-2800,1);
			else
				squareFlag++;
				break;
		case 2:
			if(sY > 2100)
				straightLine(1,0,600,1);
			else
				squareFlag++;
				break;
		case 3:
			if(sX < 600)
				straightLine(0,1,-1600,0);
			else
				squareFlag++;
				break;
		case 4:
			if(sY < 2800)
				straightLine(1,0,-1100,0);
			else
				squareFlag++;
				break;
		case 5:
			if(sX > -600)
				straightLine(0,1,-3300,1);
			else
				squareFlag++;
				break;
		case 6:
			if(sY > 1600)
				straightLine(1,0,1100,1);
			else
				squareFlag++;
				break;
		case 7:
			if(sX < 1100)
				straightLine(0,1,-1100,0);
			else
				squareFlag++;
				break;
		case 8:
			if(sY < 3300)
				straightLine(1,0,-1600,0);
			else
				squareFlag++;
				break;
		case 9:
			if(sX > -1100)
				straightLine(0,1,-3800,1);
			else
				squareFlag++;
				break;
		case 10:
			if(sY > 1100)
				straightLine(1,0,1600,1);
			else
				squareFlag++;
				break;	
		case 11:
			if(sX < 1100)
				straightLine(0,1,-600,0);
			else
				squareFlag=8;
				break;	
		default:squareFlag=0; break;
			
	}
	usartValue.flagValue=squareFlag;
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
			if(sTY < 1400)
				straightLine(1,0,0,0);
			else
				flagOne++;
			break;
		case 1:
			if(sTX < 1400)
				straightLine(0,1,-2000,0);
			else
				flagOne++;
			break;
		case 2:
			if(sTY > 600)
				straightLine(1,0,-2000,1);
			else
				flagOne++;
			break;
		case 3:
			if(sTX > 600)
				straightLine(0,1,0,1);
			else
				flagOne=0;
			break;
		default: flagOne=0;break;
	}

}


/**
  * @brief  闭环圆形 
  * @note	
  * @param centerX:圆心x坐标
  * @param centerY:圆心y坐标
  * @param r:半径
  * @param speed:速度
  * @retval None
  */
void RoundTwo(float centerX,float centerY,float r,uint8_t o,float speed)
{
	float rX=GetPosX();
	float rY=GetPosY();
	float d=sqrt(((rX-centerX)*(rX-centerX))+((rY-centerY)*(rY-centerY)));
	float angleK=atan((rY-centerY)/(rX-centerX))*180/PI;
	float angleErr=-DistancePid(r,d);
	float angleSet=0;
	if(centerX == rX)
	{
		if(o == 0)
		{
			if(rY > centerY)
				angleSet=angleErr-90;
			else
				angleSet=90-angleErr;
		}
		else if(o == 1)
		{
			if(rY > centerY)
				angleSet=90+angleErr;
			else
				angleSet=angleErr-90;
		}
	}
	else if(centerX > rX)
	{
		if(o == 0)
			angleSet=-angleErr+angleK;
		else if(o == 1)
			angleSet=angleErr+angleK-180;
	}
	else
	{
		if(o == 0)
			angleSet=-angleErr+angleK+180;
		else if(o == 1)
			angleSet=angleErr+angleK;
	}
	usartValue.d=d;
	usartValue.xValue=rX;
	usartValue.yValue=rY;
	usartValue.turnAngleValue=angleSet;
	Turn(angleSet,speed);
}

/**
  * @brief  x方向速度
  * @note	
  * @param 
  * @retval None
  */

float Speed_X(void)
{
	static float tXLast=0;
	float tX=GetPosX();
	float speedX=(tX-tXLast)*100;
	tXLast=tX;
	return speedX;
}


/**
  * @brief  y方向速度
  * @note	
  * @param 
  * @retval None
  */

float Speed_Y(void)
{
	static float tYLast=0;
	float tY=GetPosY();
	float speedY=(tY-tYLast)*100;
	tYLast=tY;
	return speedY;
}
/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
