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


uint8_t flagOne=0;
uint8_t errFlg=0;
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
	int32_t bPulseNum=-(gospeed*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*WHEEL_DIAMETER*TRANSMISSION_RATIO);
	float getAngle=GetAngle();
	float speed=0;

	speed=AnglePid(angle,getAngle);
	usartValue.d=speed;
	pulseNum=-(speed*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*TURN_AROUND_WHEEL_DIAMETER*TRANSMISSION_RATIO);
	
	VelCrl(CAN1, TURN_AROUND_WHEEL_ID,pulseNum);
	VelCrl(CAN1, BACK_WHEEL_ID,bPulseNum);

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
	int32_t bPulseNum=(gospeed*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*WHEEL_DIAMETER*TRANSMISSION_RATIO);
	float getAngle=0;
	float speed=0;
	getAngle=GetAngle();
	
	speed=AnglePid(angle,getAngle);	
	
	pulseNum=-(speed*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*TURN_AROUND_WHEEL_DIAMETER*TRANSMISSION_RATIO);
	VelCrl(CAN1, TURN_AROUND_WHEEL_ID,pulseNum);
	VelCrl(CAN1, BACK_WHEEL_ID,bPulseNum);

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
uint8_t straightLine(float A1,float B1,float C1,uint8_t dir,float setSpeed)
{
	
	
	float setAngle=0;
	float getAngleNow=GetAngle();
	float getX=GetPosX();
	float getY=GetPosY();
	float distance=((A1*getX)+(B1*getY)+C1)/sqrt((A1*A1)+(B1*B1));
	float angleAdd=DistancePid(distance,0);
	
	if((B1 > -0.005) && (B1 < 0.005))
	{
		if(!dir)
		{
			setAngle=0;
			Turn(setAngle+angleAdd,setSpeed);
		}
		else
		{
			if(A1 > 0)
			{
				setAngle=-180;
				Turn(setAngle-angleAdd,setSpeed);
			}
			else
			{
				setAngle=180;
				Turn(setAngle+angleAdd,setSpeed);
			}
		}
	}
	else
	{
		if(!dir)
		{
			setAngle=(atan(-A1/B1)*180/PI)-90;
			Turn(setAngle-angleAdd,setSpeed);
		}
		else
		{
			setAngle=(atan(-A1/B1)*180/PI)+90;
			Turn(setAngle+angleAdd,setSpeed);
		}
		
	}
	if(flagOne < 7)
	{
		if((distance < 32) && (distance > -32))
			return 1;
		else
			return 0; 
	}
	else
	{
		if((distance < 45) && (distance > -45))
			return 1;
		else
			return 0; 
	}


}


/**
  * @brief  后退走直线
  * @note	
  * @param  
  * @retval None
  */

uint8_t BackstraightLine(float A1,float B1,float C1,uint8_t dir,float setSpeed)
{
	float setAngle=0;
	float getX=GetPosX();
	float getY=GetPosY();
	float distance=((A1*getX)+(B1*getY)+C1)/sqrt(A1*A1+B1*B1);
	float angleAdd=DistancePid(distance,0);;
	
	if((B1 > -0.005) && (B1 < 0.005))
	{
		if(!dir)
		{
			setAngle=0;
			BackTurn(setAngle-angleAdd,setSpeed);
		}
		else
		{
			if(A1 > 0)
			{
				setAngle=-180;
				BackTurn(setAngle+angleAdd,setSpeed);
			}
			else
			{
				setAngle=180;
				BackTurn(setAngle-angleAdd,setSpeed);
			}
		}
	}
	else
	{
		if(!dir)
		{
			setAngle=(atan(-A1/B1)*180/PI)+90;
			BackTurn(setAngle-angleAdd,setSpeed);
		}
		else
		{
			setAngle=(atan(-A1/B1)*180/PI)-90;
			BackTurn(setAngle+angleAdd,setSpeed);
		}
	}
	if((distance < 100) && (distance > -100))
		return 1;
	else
		return 0; 	
}


/**
  * @brief  走方形
  * @note	
  * @param 
  * @retval None
  */
void Squre(void)
{
	static uint8_t sFlag=0;
	float speed2=0;
	float sTAngle=GetAngle();
	float sTX=GetPosX();
	float sTY=GetPosY();
	Angle_PidPara((600*KP_A),0,0);
	switch(sFlag)
	{
		case 0:
			if(sTY < 2500)
			{
				speed2=1000;
				straightLine(1,0,1200,0,speed2);
			}
			else
			{
				speed2=600;
				if(straightLine(0,1,-3400,0,speed2) == 1)
					sFlag++;
			}
				
			break;
		case 1:
			if(sTX < 250)
			{
				speed2=2000;
				straightLine(0,1,-3400,0,speed2);
				
			}
			else
			{
				speed2=600;
				if(straightLine(1,0,-1200,1,speed2) == 1)
					sFlag++;
			}
			break;
		case 2:
			if(sTY > 2150)
			{
				speed2=2000;
				straightLine(1,0,-1200,1,speed2);
				
			}
			else
			{
				speed2=600;
				if(straightLine(0,1,-1200,1,speed2) == 1)
					sFlag++;
			}
			break;
		case 3:
			if(sTX > -250)
			{
				speed2=2000;
				straightLine(0,1,-1200,1,speed2);
				
			}
			else
			{
				speed2=600;
				if(straightLine(1,0,1200,0,speed2) == 1)
					sFlag++;
			}
			break;
			
		case 4:
			if(sTY < 2450)
			{
				speed2=2000;
				straightLine(1,0,1200,0,speed2);
				
			}
			else
			{
				speed2=600;
				if(straightLine(0,1,-3400,0,speed2) == 1)
					sFlag=1;
			}
			break;
		default: sFlag=0;
			break;
	}
}


void Squre2(void)
{
	static uint8_t sFlag=0;
	static float speed3=2500;
	static float speed4=600;
	float sTAngle=GetAngle();
	float sTX=GetPosX();
	float sTY=GetPosY();
	
	switch(sFlag)
	{
		case 0:
			if(sTY < 2850)
			{
				straightLine(1,0,1900,0,1000);
				
			}
			else
			{
				if(straightLine(0,1,-4100,0,speed4) == 1)
					sFlag++;
			}
				
			break;
		case 1:
			if(sTX < 650)
			{ 
				straightLine(0,1,-4100,0,speed3);
				
			}
			else
			{

				if(straightLine(1,0,-1900,1,speed4) == 1)
					sFlag++;
			}
			break;
		case 2:
			if(sTY > 1550)
			{
				straightLine(1,0,-1900,1,speed3);
				
			}
			else
			{
				if(straightLine(0,1,-300,1,speed4) == 1)
					sFlag++;
			}
			break;
		case 3:
			if(sTX > -650)
			{
				straightLine(0,1,-300,1,speed3);
				
			}
			else
			{
				if(straightLine(1,0,1900,0,speed4) == 1)
					sFlag++;
			}
			break;
			
		case 4:
			if(sTY < 2850)
			{
				straightLine(1,0,1900,0,speed3);
				
			}
			else
			{
				if(straightLine(0,1,-4100,0,speed4) == 1)
					sFlag=1;
			}
			break;
		default: sFlag=0;
			break;
	}
}

/**
  * @brief  顺时针走方形，面积渐渐变大，偏离轨道后能回去
  * @note	
  * @param 
  * @retval None
  */


void BiggerSquareOne(void)
{
	float sTAngle=GetAngle();
	float sTX=GetPosX();
	float sTY=GetPosY();
	float speed1=600;
	float speed2=2000;
	float speed3=2400;
	
	
	switch(flagOne)
	{
		case 0:
			if(sTY < 2000)
			{
				straightLine(1,0,700,0,800);
				
			}
			else
				flagOne++;
			break;
		case 1:
			if(sTX < -200)
			{
				straightLine(0,1,-2900,0,800);
				
			}
			else
				flagOne++;
			break;
		case 2:
			if(sTY > 2500)
			{
				straightLine(1,0,-700,1,800);
				
			}
			else
				flagOne++;
			break;
		case 3:
			if(sTX > -400)
			{
				straightLine(0,1,-1700,1,800);
				
			}
			else
			{
				if(straightLine(1,0,1200,0,speed1) == 1)
					flagOne++;
			}
			break;
			
		case 4:
			if(sTY < 2500)
			{
				straightLine(1,0,1200,0,speed2);
				
			}
			else
			{
				if(straightLine(0,1,-3400,0,speed1) == 1)
					flagOne++;
			}
			break;
		case 5:
			if(sTX < 250)
			{
				Angle_PidPara((600*(266.55/16000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(0,1,-3400,0,speed2);
				
			}
			else
			{
				if(straightLine(1,0,-1200,1,speed1) == 1)
					flagOne++;
			}
			break;
		case 6:
			if(sTY > 2100)
			{
				Angle_PidPara((600*(266.55/16000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(1,0,-1200,1,speed2);
				
			}
			else
			{
				if(straightLine(0,1,-1200,1,speed1) == 1)
					flagOne++;
			}
			break;
		case 7:
			if(sTX > -650)
			{
				Angle_PidPara((600*(266.55/16000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(0,1,-1200,1,speed3);
				
			}
			else
			{
				if(straightLine(1,0,1900,0,speed1) == 1)
					flagOne++;
			}
			break;
			
		case 8:
			if(sTY < 2850)
			{
				Angle_PidPara((600*(266.55/17000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(1,0,1900,0,speed3);
				
			}
			else
			{
				
				if(straightLine(0,1,-4100,0,speed1) == 1)
					flagOne++;
			}
			break;
		case 9:
			if(sTX < 650)
			{ 
				Angle_PidPara((600*(266.55/17000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(0,1,-4100,0,speed3);
				
			}
			else
			{

				if(straightLine(1,0,-1900,1,speed1) == 1)
					flagOne++;
			}
			break;
		case 10:
			if(sTY > 1550)
			{
				Angle_PidPara((600*(266.55/17000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(1,0,-1900,1,speed3);
				
			}
			else
			{
				if(straightLine(0,1,-300,1,speed1) == 1)
					flagOne++;
			}
			break;
		case 11:
			if(sTX > -650)
			{
				Angle_PidPara((600*(266.55/17000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(0,1,-300,1,speed3);
				
			}
			else
			{
				if(straightLine(1,0,1900,0,speed1) == 1)
					flagOne++;
			}
			break;
		case 12:
			if(sTY < 2850)
			{
				Angle_PidPara((600*(266.55/17000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(1,0,1900,0,speed3);
				
			}
			else
			{
				if(straightLine(0,1,-4100,0,speed1) == 1)
					flagOne=9;
			}
			break;
		default: flagOne=0;
			break;
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
	float speed1=600;
	float speed2=2000;
	float speed3=2500;
	

	switch(flagOne)
	{
		case 0:
			if(sTY < 2000)
			{
				straightLine(1,0,-700,0,800);
				
			}
			else
				flagOne++;
			break;
		case 1:
			if(sTX > 200)
			{
				straightLine(0,1,-2900,1,800);
				
			}
			else
				flagOne++;
			break;
		case 2:
			if(sTY > 2500)
			{
				straightLine(1,0,700,1,800);
				
			}
			else
				flagOne++;
			break;
		case 3:
			if(sTX < 400)
			{
				straightLine(0,1,-1700,0,800);
				
			}
			else
			{
				if(straightLine(1,0,-1200,0,speed1) == 1)
					flagOne++;
			}
			break;
			
		case 4:
			if(sTY < 2500)
			{
				straightLine(1,0,1200,0,speed2);
				
			}
			else
			{
				if(straightLine(0,1,-3400,1,speed1) == 1)
					flagOne++;
			}
			break;
		case 5:
			if(sTX > -300)
			{
				Angle_PidPara((600*(266.55/16000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(0,1,-3400,1,speed2);
				
			}
			else
			{
				if(straightLine(1,0,1200,1,speed1) == 1)
					flagOne++;
			}
			break;
		case 6:
			if(sTY > 2100)
			{
				Angle_PidPara((600*(266.55/16000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(1,0,1200,1,speed2);
				
			}
			else
			{
				if(straightLine(0,1,-1200,0,speed1) == 1)
					flagOne++;
			}
			break;
		case 7:
			if(sTX < 650)
			{
				Angle_PidPara((600*(266.55/16000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(0,1,-1200,0,speed3);
				
			}
			else
			{
				if(straightLine(1,0,-1900,0,speed1) == 1)
					flagOne++;
			}
			break;
			
		case 8:
			if(sTY < 2850)
			{
				Angle_PidPara((600*(266.55/16000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(1,0,-1900,0,speed3);
				
			}
			else
			{
				if(straightLine(0,1,-4100,1,speed1) == 1)
					flagOne++;
			}
			break;
		case 9:
			if(sTX > -650)
			{ 
				Angle_PidPara((600*(266.55/16000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(0,1,-4100,1,speed3);
				
			}
			else
			{

				if(straightLine(1,0,1900,1,speed1) == 1)
					flagOne++;
			}
			break;
		case 10:
			if(sTY > 1550)
			{
				Angle_PidPara((600*(266.55/16000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(1,0,1900,1,speed3);
				
			}
			else
			{
				if(straightLine(0,1,-300,0,speed1) == 1)
					flagOne++;
			}
			break;
		case 11:
			if(sTX < 650)
			{
				Angle_PidPara((600*(266.55/16000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(0,1,-300,0,speed3);
				
			}
			else
			{
				if(straightLine(1,0,-1900,0,speed1) == 1)
					flagOne++;
			}
			break;
		case 12:
			if(sTY < 2850)
			{
				Angle_PidPara((600*(266.55/16000)),0,0);
				Distance_PidPara(KP_D,0,20);
				straightLine(1,0,-1900,0,speed3);
				
			}
			else
			{
				if(straightLine(0,1,-4100,1,speed1) == 1)
					flagOne=9;
			}
			break;
		default: flagOne=0;
			break;
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


void Walk(uint8_t getAdcFlag)
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
	
	//故障判断
	if((speedx < 100) && (speedx > -100) && (speedy < 100) && (speedy > -100))
	{
		walkCnt++;
		if(walkCnt >= 120)
		{
			errFlag=!errFlag;
			X_Now=GetPosX();
			Y_Now=GetPosY();
			walkCnt=0;
		} 
		if(errFlag == 1)
		{
			if(errFlg > 3)
				errFlg=3;
			else
				errFlg++;
			
		}
	}
	else
		errFlg=0;
	
	//故障处理，后退
	if(errFlag)
	{
		if(((X_Now+2300) <= Y_Now) && ((2300-X_Now) <= Y_Now))
		{
			if(Y_Now > 3200)
			{
				ready=BackstraightLine(0,1,500-Y_Now,getAdcFlag,1000);
			}
			else if((Y_Now <= 3200) && (Y_Now > 2300))
			{
				ready=BackstraightLine(0,1,-500-Y_Now,getAdcFlag,1000);
			}
		}
		else if(((X_Now+2300) > Y_Now) && ((2300-X_Now) > Y_Now))
		{
			if((Y_Now <= 2300) && (Y_Now > 1200))
			{
				ready=BackstraightLine(0,1,500-Y_Now,!getAdcFlag,1000);
			}
			else if(Y_Now <= 1200)
			{
				ready=BackstraightLine(0,1,-500-Y_Now,!getAdcFlag,1000);
			}
		}
		else if(((X_Now+2300) < Y_Now) && ((2300-X_Now) >= Y_Now))
		{
			if(X_Now <= 0 && X_Now > -1000)
			{
				ready=BackstraightLine(1,0,500-X_Now,!getAdcFlag,1000);
			}
			else if(X_Now <= -1000)
			{
				ready=BackstraightLine(1,0,-500-X_Now,!getAdcFlag,1000);
			}
		}
		else if(((X_Now+2300) >= Y_Now) && ((2300-X_Now) < Y_Now))
		{
		
			if(X_Now >= 0 && X_Now < 1000)
			{
				ready=BackstraightLine(1,0,-500-X_Now,getAdcFlag,1000);
			}
			
			else if(X_Now >= 1000)
			{
				ready=BackstraightLine(1,0,500-X_Now,getAdcFlag,1000);
			}
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
	
	//
	else
	{
		if(!getAdcFlag)
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
