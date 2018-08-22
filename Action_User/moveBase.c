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

extern struct trans{
float oldPosX;
float oldPosY;
float newPosX;
float newPosY;
float t_angle;
}transform;

extern float outMax2;
extern float outMin2;
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
  * @param  gospeed：基础速度
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
  * @param  angle：给定角度
  * @param  gospeed：基础速度
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
	float angleAdd=0.09*distance;
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
		if(setAngle >= 0)
		{
			BackTurn(setAngle-angleAdd,1000);
			usartValue.turnAngleValue=setAngle+angleAdd;
		}
		else
		{
			BackTurn(setAngle+angleAdd,1000);
			usartValue.turnAngleValue=setAngle-angleAdd;			
		}
	}
	else
	{
		BackTurn(setAngle,1000);
		usartValue.turnAngleValue=setAngle;
	}
	if((distance < 50) && (distance > -50))
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
	outMax2=90;
	outMin2=90;
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
			if(sY < 2200)
				straightLine(1,0,600,0);
			else
				squareFlag++;
				break;
		case 1:
			if(sX < 0)
				straightLine(0,1,-2850,0);
			else
				squareFlag++;
				break;
		case 2:
			if(sY > 2200)
				straightLine(1,0,-650,1);
			else
				squareFlag++;
				break;
		case 3:
			if(sX > -500)
				straightLine(0,1,-1600,1);
			else
				squareFlag++;
				break;
		case 4:
			if(sY < 2700)
				straightLine(1,0,1100,0);
			else
				squareFlag++;
				break;
		case 5:
			if(sX < 500)
				straightLine(0,1,-3300,0);
			else
				squareFlag++;
				break;
		case 6:
			if(sY > 1700)
				straightLine(1,0,-1100,1);
			else
				squareFlag++;
				break;
		case 7:
			if(sX > -1000)
				straightLine(0,1,-1100,1);
			else
				squareFlag++;
				break;
		case 8:
			if(sY < 3200)
				straightLine(1,0,1600,0);
			else
				squareFlag++;
				break;
		case 9:
			if(sX < 1000)
				straightLine(0,1,-3800,0);
			else
				squareFlag++;
				break;
		case 10:
			if(sY > 1200)
				straightLine(1,0,-1600,1);
			else
				squareFlag++;
				break;
		case 11:
			if(sX > -1200)
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
			if(sY < 2200)
				straightLine(1,0,-600,0);
			else
				squareFlag++;
				break;
		case 1:
			if(sX > 0)
				straightLine(0,1,-2850,1);
			else
				squareFlag++;
				break;
		case 2:
			if(sY > 2200)
				straightLine(1,0,750,1);
			else
				squareFlag++;
				break;
		case 3:
			if(sX < 500)
				straightLine(0,1,-1600,0);
			else
				squareFlag++;
				break;
		case 4:
			if(sY < 2700)
				straightLine(1,0,-1100,0);
			else
				squareFlag++;
				break;
		case 5:
			if(sX > -500)
				straightLine(0,1,-3300,1);
			else
				squareFlag++;
				break;
		case 6:
			if(sY > 1500)
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
			if(sY < 3200)
				straightLine(1,0,-1600,0);
			else
				squareFlag++;
				break;
		case 9:
			if(sX > -1000)
				straightLine(0,1,-3800,1);
			else
				squareFlag++;
				break;
		case 10:
			if(sY > 1200)
				straightLine(1,0,1600,1);
			else
				squareFlag++;
				break;	
		case 11:
			if(sX < 1000)
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
	outMax2=90;
	outMin2=90;
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


void Walk(uint8_t *getAdcFlag)
{
	uint8_t ready=0;
	float speedx=0;
	float speedy=0;
	static uint8_t errFlag=0;
	static float X_Now=0;
	static float Y_Now=0;
	speedx=Speed_X();
	speedy=Speed_Y();
	
	if((speedx < 100) && (speedx > -100) && (speedy < 100) && (speedy > -100))
	{
		usartValue.cnt++;
		if(usartValue.cnt > 100)
		{
			errFlag=!errFlag;
			X_Now=usartValue.xValue;
			Y_Now=usartValue.yValue;
			usartValue.cnt=0;
		}
	}
	
	if(errFlag)
	{
		if((X_Now > -900) && (X_Now < 900))
		{
			if(Y_Now > 3400)
			{
				ready=BackstraightLine(0,1,500-Y_Now,(*getAdcFlag));
			}
			else if((Y_Now <= 3400) && (Y_Now > 2200))
			{
				ready=BackstraightLine(0,1,-500-Y_Now,(*getAdcFlag));
			}
			else if((Y_Now <= 2200) && (Y_Now > 1100))
			{
				ready=BackstraightLine(0,1,500-Y_Now,!(*getAdcFlag));
			}
			else if(Y_Now <= 1100)
			{
				ready=BackstraightLine(0,1,-500-Y_Now,!(*getAdcFlag));
			}
		
		}
		else if(X_Now <= -900 && X_Now > -1400)
		{
			ready=BackstraightLine(1,0,500-X_Now,!(*getAdcFlag));
		}
		else if(X_Now >= 900 && X_Now < 1400)
		{
			ready=BackstraightLine(1,0,-500-X_Now,(*getAdcFlag));
		}
		else if(X_Now <= -1400)
		{
			ready=BackstraightLine(1,0,-500-X_Now,!(*getAdcFlag));
		}
		else
		{
			ready=BackstraightLine(1,0,500-X_Now,(*getAdcFlag));
		}
		
		if(ready)
		{
			if(squareFlag >= 8)
				squareFlag=squareFlag-4;
			else
				squareFlag=squareFlag+4;
			errFlag=!errFlag;
		}
		else;
	}
	
	else
	{
		if(!(*getAdcFlag))
		{
			BiggerSquareOne();
		}
		else
		{
			BiggerSquareTwo();
		}
	}

}

/**
  * @brief  新底盘角度闭环
  * @note	
  * @param 
  * @retval None
  */
void Turn2(float setAngle1,float tSpeed)
{
	float t2X=transform.newPosX;
	float t2Y=transform.newPosY;
	int32_t pulseNum=0;
	int32_t bPulseNum=(tSpeed*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*WHEEL_DIAMETER);
	float getAngle=0;
	float speed=0;
	getAngle=GetAngle();
	
	usartValue.xValue=transform.newPosX;
	usartValue.yValue=transform.newPosY;
	usartValue.angleValue=getAngle;
	
	speed=AnglePid(setAngle1,getAngle);
	usartValue.pidValueOut=speed;
	
	pulseNum=-(speed*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*TURN_AROUND_WHEEL_DIAMETER );
	
	VelCrl(CAN2, 0x06,pulseNum);
	VelCrl(CAN2, 0x05,bPulseNum);
	
}

/**
  * @brief  新底盘直线闭环
  * @note	
  * @param A1
  * @param B1
  * @param C1
  * @param dir:为0 往上或右走
  * @retval None
  */
void straightLine2(float A1,float B1,float C1,uint8_t dir)
{
	float setAngle=0;
	float getAngle=GetAngle();
	float getX=GetPosX();
	float getY=GetPosY();
	float distance=((A1*getX)+(B1*getY)+C1)/sqrt(A1*A1+B1*B1);
	float rearWheelSpeed=0;
	float angleAdd=0;
	outMax2=300;
	outMin2=-300;
	if(DistancePid(distance,0) < 0)
		rearWheelSpeed=-DistancePid(distance,0);
	else
		rearWheelSpeed=DistancePid(distance,0);
	
	angleAdd=0.5*DistancePid(distance,0);
	if(angleAdd > 90)
		angleAdd=90;
	else if(angleAdd < -90)
		angleAdd=-90;
	
	usartValue.d=distance;
	usartValue.xValue=getX;
	usartValue.yValue=getY;
	usartValue.angleValue=getAngle;
	usartValue.turnAngleValue=setAngle+angleAdd;
	
	if((B1 > -0.005) && (B1 < 0.005))
	{
		if(!dir)
		{
			setAngle=0;
			Turn2(setAngle+angleAdd,200+rearWheelSpeed);
		}
		else
		{
			if(A1 > 0)
			{
				setAngle=-180;
				Turn2(setAngle-angleAdd,200+rearWheelSpeed);
			}
			else
			{
				setAngle=180;
				Turn2(setAngle+angleAdd,200+rearWheelSpeed);
			}
		}
	}
	else
	{
		if(!dir)
		{
			setAngle=(atan(-A1/B1)*180/PI)-90;
			Turn2(setAngle-angleAdd,200+rearWheelSpeed);
		}
		else
		{
			setAngle=(atan(-A1/B1)*180/PI)+90;
			Turn2(setAngle+angleAdd,200+rearWheelSpeed);
		}
		
	}
	
	


	
	
	
}
/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
