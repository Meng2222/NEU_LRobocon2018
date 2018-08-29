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



extern struct usartValue_{
	uint32_t cnt;//用于检测是否数据丢失
	float xValue;//串口输出x坐标
	float yValue;//串口输出y坐标
	float angleValue;//串口输出角度值
	float pidValueOut;//PID输出
	float d;
	float turnAngleValue;//
	uint8_t flagValue;
	float shootangle;
}usartValue;


uint8_t flag=0;
extern float outMax2;
extern float outMin2;
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
  * @brief  PID 后退转弯
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
	float angleAdd=0.1*distance;
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
	if((distance < 100) && (distance > -100))
		return 1;
	else
		return 0; 
}


/**
  * @brief  沿直线走，能回到直线,距离速度pid
  * @note	给定直线以Ax+By+C=0形式
  * @param  A：
  * @param  B:
  * @param  C:
  * @param  dir:dir为0，向右；dir为1，向左
  * @retval None
  */

void HighSpeedStraightLine(float A1,float B1,float C1,uint8_t dir,float highSpeed)
{
	float setAngle=0;
	float getAngle=GetAngle();
	float getX=GetPosX();
	float getY=GetPosY();
	float distance=((A1*getX)+(B1*getY)+C1)/sqrt(A1*A1+B1*B1);
	float angleAdd=DistancePid(distance,0);
	float speedAdd=__fabs(distance)*0.3;
	
	

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
			Turn(setAngle-angleAdd,highSpeed+speedAdd);
			usartValue.turnAngleValue=setAngle-angleAdd;
		}
		else
		{
			Turn(setAngle+angleAdd,highSpeed+speedAdd);
			usartValue.turnAngleValue=setAngle+angleAdd;			
		}
	}
	else if(distance < -10)
	{
		if(setAngle >= 0)
		{
			Turn(setAngle+angleAdd,highSpeed+speedAdd);
			usartValue.turnAngleValue=setAngle+angleAdd;
		}
		else
		{
			Turn(setAngle-angleAdd,highSpeed+speedAdd);
			usartValue.turnAngleValue=setAngle-angleAdd;			
		}
	}
	else
	{
		Turn(setAngle,highSpeed);
		usartValue.turnAngleValue=setAngle;
	}
	
}

/**
  * @brief  顺时针走方形，面积渐渐变大，偏离轨道后能回去
  * @note	
  * @param 
  * @retval None
  */
uint8_t flagOne=0;

void BiggerSquareOne(void)
{
	float sTAngle=GetAngle();
	float sTX=GetPosX();
	float sTY=GetPosY();
	float speed1=0;
	usartValue.flagValue=flagOne;
	switch(flagOne)
	{
		case 0:
			if(sTY < 2000)
			{
				HighSpeedStraightLine(1,0,700,0,1500);
			}
			else
				flagOne++;
			break;
		case 1:
			if(sTX < 0)
			{
				HighSpeedStraightLine(0,1,-2900,0,1500);
			}
			else
				flagOne++;
			break;
		case 2:
			if(sTY > 2500)
			{
				HighSpeedStraightLine(1,0,-700,1,1500);
			}
			else
				flagOne++;
			break;
		case 3:
			if(sTX > 300)
			{
				HighSpeedStraightLine(0,1,-1600,1,1500);
			}
			else
				flagOne++;
			break;
			
		case 4:
			if(sTY < 2600)
			{
				if(sTY < 2275)
					speed1=SpeedPid(sTY,1000);
				else
					speed1=SpeedPid(3550,sTY);
				HighSpeedStraightLine(1,0,1250,0,speed1);
			}
			else
				flagOne++;
			break;
		case 5:
			if(sTX < -300)
			{
				if(sTX < 0)
					speed1=SpeedPid(1250,sTX);
				else
					speed1=SpeedPid(sTX,-1250);
				HighSpeedStraightLine(0,1,-3550,0,speed1);
			}
			else
				flagOne++;
			break;
		case 6:
			if(sTY > 1800)
			{
				if(sTY < 2275)
					speed1=SpeedPid(sTY,1000);
				else
					speed1=SpeedPid(3500,sTY);
				HighSpeedStraightLine(1,0,-1250,1,speed1);
			}
			else
				flagOne++;
			break;
		case 7:
			if(sTX > 600)
			{
				if(sTX > 0)
					speed1=SpeedPid(1250,sTX);
				else
					speed1=SpeedPid(sTX,-1250);
				HighSpeedStraightLine(0,1,-1000,1,speed1);
			}
			else
				flagOne++;
			break;
			
		case 8:
			if(sTY < 2800)
			{
				if(sTY < 2300)
					speed1=SpeedPid(sTY,400);
				else
					speed1=SpeedPid(4200,sTY);
				HighSpeedStraightLine(1,0,1800,0,speed1);
			}
			else
				flagOne++;
			break;
		case 9:
			if(sTX < -500)
			{
				if(sTX > 0)
					speed1=SpeedPid(1800,sTX);
				else
					speed1=SpeedPid(sTX,-1700);
				HighSpeedStraightLine(0,1,-4200,0,speed1);
			}
			else
				flagOne++;
			break;
		case 10:
			if(sTY > 1700)
			{
				if(sTY < 2300)
					speed1=SpeedPid(sTY,400);
				else
					speed1=SpeedPid(4200,sTY);
				HighSpeedStraightLine(1,0,-1800,1,speed1);
			}
			else
				flagOne++;
			break;
		case 11:
			if(sTX > 600)
			{
				if(sTX > 0)
					speed1=SpeedPid(1800,sTX);
				else
					speed1=SpeedPid(sTX,-1800);
				HighSpeedStraightLine(0,1,-400,1,speed1);
			}
			else
				flagOne=8;
			break;
		default: flagOne=0;break;
	}
}

/**
  * @brief  逆时针走方形，面积渐渐变大，偏离轨道后能回去
  * @note	
  * @param 
  * @retval None
  */


void BiggerSquareTwo(void)
{
	float sTAngle=GetAngle();
	float sTX=GetPosX();
	float sTY=GetPosY();
	float speed1=0;
	usartValue.flagValue=flagOne;
	switch(flagOne)
	{
		case 0:
			if(sTY < 2000)
			{
				HighSpeedStraightLine(1,0,-700,0,1500);
			}
			else
				flagOne++;
			break;
		case 1:
			if(sTX > 0)
			{
				HighSpeedStraightLine(0,1,-2900,1,1500);
			}
			else
				flagOne++;
			break;
		case 2:
			if(sTY > 2500)
			{
				HighSpeedStraightLine(1,0,700,1,1500);
			}
			else
				flagOne++;
			break;
		case 3:
			if(sTX < 300)
			{
				HighSpeedStraightLine(0,1,-1600,0,1500);
			}
			else
				flagOne++;
			break;
			
		case 4:
			if(sTY < 2600)
			{
				if(sTY < 2275)
					speed1=SpeedPid(sTY,1000);
				else
					speed1=SpeedPid(3550,sTY);
				HighSpeedStraightLine(1,0,-1250,0,speed1);
			}
			else
				flagOne++;
			break;
		case 5:
			if(sTX > -300)
			{
				if(sTX > 0)
					speed1=SpeedPid(1250,sTX);
				else
					speed1=SpeedPid(sTX,-1250);
				HighSpeedStraightLine(0,1,-3550,1,speed1);
			}
			else
				flagOne++;
			break;
		case 6:
			if(sTY > 1800)
			{
				if(sTY < 2275)
					speed1=SpeedPid(sTY,1000);
				else
					speed1=SpeedPid(3500,sTY);
				HighSpeedStraightLine(1,0,1250,1,speed1);
			}
			else
				flagOne++;
			break;
		case 7:
			if(sTX < 600)
			{
				if(sTX > 0)
					speed1=SpeedPid(1250,sTX);
				else
					speed1=SpeedPid(sTX,-1250);
				HighSpeedStraightLine(0,1,-1000,0,speed1);
			}
			else
				flagOne++;
			break;
			
		case 8:
			if(sTY < 2800)
			{
				if(sTY < 2300)
					speed1=SpeedPid(sTY,400);
				else
					speed1=SpeedPid(4200,sTY);
				HighSpeedStraightLine(1,0,-1800,0,speed1);
			}
			else
				flagOne++;
			break;
		case 9:
			if(sTX > -500)
			{
				if(sTX > 0)
					speed1=SpeedPid(1800,sTX);
				else
					speed1=SpeedPid(sTX,-1700);
				HighSpeedStraightLine(0,1,-4200,1,speed1);
			}
			else
				flagOne++;
			break;
		case 10:
			if(sTY > 1700)
			{
				if(sTY < 2300)
					speed1=SpeedPid(sTY,400);
				else
					speed1=SpeedPid(4200,sTY);
				HighSpeedStraightLine(1,0,1800,1,speed1);
			}
			else
				flagOne++;
			break;
		case 11:
			if(sTX < 600)
			{
				if(sTX > 0)
					speed1=SpeedPid(1800,sTX);
				else
					speed1=SpeedPid(sTX,-1800);
				HighSpeedStraightLine(0,1,-400,0,speed1);
			}
			else
				flagOne=8;
			break;
		default: flagOne=0;break;
	}
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
	static float X_Now=0;
	static float Y_Now=0;
	static uint8_t errFlag=0;
	
	static uint16_t walkCnt=0;
	float speedx=0;
	float speedy=0;
	
	
	speedx=Speed_X();
	speedy=Speed_Y();
	
	if((speedx < 200) && (speedx > -200) && (speedy < 200) && (speedy > -200))
	{
		walkCnt++;
		if(walkCnt >= 100)
		{
			errFlag=!errFlag;
			X_Now=GetPosX();
			Y_Now=GetPosY();
			walkCnt=0;
		} 
		if(errFlag == 1)
		{
			if(flag >= 3)
				flag=3;
			else
				flag++;
			
		}
	}
	else
		flag=0;
	
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
			if(flagOne >= 8)
				flagOne=flagOne-4;
			else
				flagOne=flagOne+4;
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


/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
