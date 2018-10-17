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



uint8_t step=0;
uint8_t flagOne=0;
uint8_t errFlg=0;
extern usartValue uv4;
extern uint8_t shootReady[4];
extern uint8_t shootFlagOne;
extern uint8_t stopFlg;
float judgeSpeed=0;
//uint8_t malFlg=0;
extern uint8_t isBallRight;
extern uint8_t noRightBall;

typedef enum {
	innerLine,innerRound,middleLineOne,middleLineTwo,middleLineThr,middleLineFor,middleOutLine,
	stopShootLineOne,stopShootLineTwo,stopShootLineThr,stopShootLineFor,
	collectOutRound,collectMiddleRound,collectInnerRound,stopShoot,recollect,semicircle
}walkstatus;
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
  * @param dir:为0 往上或右走，为1 往下或往左走
  * @param setSpeed：速度
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
		if((distance < 35) && (distance > -35))
			return 1;
		else
			return 0; 
	}
	else
	{
		if((distance < 52) && (distance > -52))
			return 1;
		else
			return 0; 
	}


}


/**
  * @brief  新底盘直线闭环
  * @note	
  * @param x：圆心x坐标
  * @param y：圆心y坐标
  * @param R：半径
  * @param clock:为0 顺时针，为1 逆时针
  * @param backspeed：速度
  * @retval None
  */
void N_SET_closeRound(float x,float y,float R,float clock,float backspeed)
{
	float Distance=0;
	float k=0;
	float setangle=0,Agl=0,frontspeed=0;
	Distance=sqrt(pow(GetPosX()-x,2)+pow(GetPosY()-y,2))-R;
	k=(GetPosX()-x)/(y-GetPosY());
	//顺1逆2
	if(clock==1)
	{
		if(GetPosY()>y)
		  Agl=-90+atan(k)*180/PI;
	  else if(GetPosY()<y)
		  Agl=90+atan(k)*180/PI;
	  else if(GetPosY()==y&&GetPosX()>=x)
		  Agl=180;
	  else if(GetPosY()==y&&GetPosX()<x)
		  Agl=0;
		setangle=Agl-DistancePid(Distance,0);
		frontspeed=AnglePid(setangle,GetAngle());
	}
	else if(clock==2)
	{
		if(GetPosY()>y)
		  Agl=90+atan(k)*180/PI;
	  else if(GetPosY()<y)
		  Agl=-90+atan(k)*180/PI;
	  else if(GetPosY()==y&&GetPosX()>=x)
		  Agl=0;
	  else if(GetPosY()==y&&GetPosX()<x)
		  Agl=180;
		setangle=Agl+DistancePid(Distance,0);
		frontspeed=AnglePid(setangle,GetAngle());
	}
	VelCrl(CAN1,1,-backspeed*REDUCTION_RATIO*NEW_CAR_COUNTS_PER_ROUND/(PI*WHEEL_DIAMETER));//后轮
	VelCrl(CAN1,2,-frontspeed*REDUCTION_RATIO*NEW_CAR_COUNTS_PER_ROUND/(PI*TURN_AROUND_WHEEL_DIAMETER));//前轮
}

uint8_t roundFlg=0;
/**
  * @brief  顺时针圆收球
  * @note	
  * @param  
  * @retval None
  */
void Round(void)
{
	switch(flagOne)
	{
		
		case collectOutRound:
			Angle_PidPara(40,0,0);
			Distance_PidPara(0.1,0,0);
			N_SET_closeRound(0,2335,1700,1,2000);
		  if(GetPosY()>-0.06*GetPosX()+2335&&GetPosY()<2335)
				flagOne++;
			break;
		case collectMiddleRound:
			Angle_PidPara(80,0,0);
			Distance_PidPara(0.08,0,0);
			N_SET_closeRound(0,2335,1100,1,2000);
			if(GetPosX()>0&&GetPosX()<0.13*(GetPosY()-2335))
				flagOne++;
			break;
		case collectInnerRound:
			noRightBall=0;
			Angle_PidPara(70,0,0);
			Distance_PidPara(0.08,0,0);
			N_SET_closeRound(0,2335,450,1,1500);
			if(GetPosY()<-0.06*GetPosX()+2335&&GetPosY()>2335)
				flagOne++;
			break;
		case stopShoot:
			errFlg=3;
			VelCrl(CAN1,1,0);
			VelCrl(CAN1,2,0);
			if(noRightBall == 1)
			{
				flagOne++;
				if(roundFlg < 2)
				{
					roundFlg++;
				}
				else
					roundFlg=0;
				errFlg=0;
			}
			break;
		case recollect:
			Angle_PidPara(80,0,0);
			Distance_PidPara(0.08,0,0);
			N_SET_closeRound(0,2335,1100,1,2000);
			if(GetPosX()<0&&GetPosX()>0.13*(GetPosY()-2335))
			{
				if(roundFlg == 0)
					flagOne=collectInnerRound;
				else if(roundFlg == 1)
					flagOne=collectMiddleRound;
				else if(roundFlg == 2)
					flagOne=collectOutRound;
			}
			break;
		case semicircle:
			noRightBall=0;
			Angle_PidPara(70,0,0);
			Distance_PidPara(0.08,0,0);
			N_SET_closeRound(0,2335,450,1,1500);
			if(GetPosY()>-0.06*GetPosX()+2335&&GetPosY()<2335)
				flagOne=14;
			break;
	}
}
/**
  * @brief  逆时针圆收球
  * @note	
  * @param  
  * @retval None
  */
void Round2(void)
{
	switch(flagOne)
	{
		case collectOutRound:
			Angle_PidPara(40,0,0);
			Distance_PidPara(0.1,0,0);
			N_SET_closeRound(0,2335,1700,2,2000);
			if(GetPosY()>0.06*GetPosX()+2335&&GetPosY()<2335)
				flagOne++;
			break;
		case collectMiddleRound:
			Angle_PidPara(80,0,0);
			Distance_PidPara(0.08,0,0);
			N_SET_closeRound(0,2335,1100,2,2000);
			if(GetPosX()<0&&GetPosX()>-0.13*(GetPosY()-2335))
				flagOne++;
			break;
		case collectInnerRound:
			noRightBall=0;
			Angle_PidPara(70,0,0);
			Distance_PidPara(0.08,0,0);
			N_SET_closeRound(0,2335,450,2,1500);
			if(GetPosY()<0.06*GetPosX()+2335&&GetPosY()>2335)
				flagOne++;
			break;
		case stopShoot:
			errFlg=3;
			VelCrl(CAN1,1,0);
			VelCrl(CAN1,2,0);
			if(noRightBall == 1)
			{
				flagOne++;
				if(roundFlg < 2)
				{
					roundFlg++;
				}
				else
					roundFlg=0;
				errFlg=0;
			}
			break;
		case recollect:
			Angle_PidPara(80,0,0);
			Distance_PidPara(0.08,0,0);
			N_SET_closeRound(0,2335,1100,2,2000);
			if(GetPosX()<0&&GetPosX()>0.13*(GetPosY()-2335))
			{
				if(roundFlg == 0)
					flagOne=13;
				else if(roundFlg == 1)
					flagOne=12;
				else if(roundFlg == 2)
					flagOne=11;
			}
			break;
		case semicircle:
			noRightBall=0;
			Angle_PidPara(70,0,0);
			Distance_PidPara(0.08,0,0);
			N_SET_closeRound(0,2335,450,2,1500);
			if(GetPosY()>0.06*GetPosX()+2335&&GetPosY()<2335)
				flagOne=14;
			break;
		
	}
}
/**
  * @brief  后退走直线
  * @note	
  * @param  
  * @retval 0 未退完全，1 退完全
  */

uint8_t BackstraightLine(float A1,float B1,float C1,uint8_t dir,float setSpeed)
{
	Angle_PidPara(10,0,5);
	Distance_PidPara(0.18,0,15); 
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
  * @brief  四个桶分别都投过后刷新
  * @note	
  * @param  
  * @retval None
  */
void shootjudge(void)
{
	if(shootReady[0] == 1 && shootReady[1] == 1 && shootReady[2] == 1 && shootReady[3] == 1)
	{
		for(int i=0;i<4;i++)
		{
			shootReady[i]=0;
		}
	}
}
/**
  * @brief  顺时针走方形，内圈圆，中圈方形，从内圈开始
  * @note	
  * @param 
  * @retval None
  */

void BiggerSquareOne(void)
{
	static int Tangencyflag=0,Tangencyflag2=0,lastTangencyflag=0,sureflag=0;
	float sTAngle=GetAngle();
	float sTX=GetPosX();
	float sTY=GetPosY();
	float speed1=1300;
	float speed2=1500;
	float speed3=1800;
	
	switch(flagOne)
	{
		case innerLine:
			shootFlagOne=2;
			judgeSpeed=0;
			if(sTY < 1900)
			{
				shootFlagOne=4;
				Angle_PidPara(27,0,0);
				Distance_PidPara(0.1,0,4);
				straightLine(1,0,700,0,1500);
			}
			else
			{
				flagOne++;
			}
			break;
		case innerRound:
			judgeSpeed=0;
			Angle_PidPara(80,0,0);
		    Distance_PidPara(0.1,0,0);
				N_SET_closeRound(0,2300,450,1,1500);
			if(GetPosX() > 100 && GetPosY() < 1800)  
				Tangencyflag2=1;
				if(GetAngle() > 88 && GetAngle() < 92)
					Tangencyflag=1;
				else 
					Tangencyflag=0;
		    if(GetPosY()>2300)
			    sureflag=1;
		    if(Tangencyflag == 1 && lastTangencyflag == 0 && sureflag && Tangencyflag2)
			{
			    flagOne++;
			}
		    lastTangencyflag=Tangencyflag;
			break;
		case middleLineOne:
			judgeSpeed=0;
			if(sTY < 2600)
			{
				
				Angle_PidPara(30,0,0);
				Distance_PidPara(0.10,0,3);
				straightLine(1,0,1200,0,1500);
			}
			else
			{
				shootFlagOne=2;
				Angle_PidPara((800*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,25); 
				if(straightLine(0,1,-3400,0,speed1) == 1)
					flagOne++;
			}
			break;
		case middleLineTwo:
			if(sTX < 160)
			{
				judgeSpeed=fabs(GetSpeeedX());
				straightLine(0,1,-3400,0,speed2);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTX > 800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=3;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				if(straightLine(1,0,-1200,1,speed1) == 1)
					flagOne++;
			}
			break;
		case middleLineThr:
			if(sTY > 2320)
			{
				Angle_PidPara((600*(266.55/18000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedY());
				straightLine(1,0,-1200,1,speed2);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTY < 1600)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=0;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				if(straightLine(0,1,-1200,1,speed1) == 1)
					flagOne++;
			}
			break;
		case middleLineFor:
			if(sTX > -400)
			{
				Angle_PidPara((600*(266.55/20000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedX());
				straightLine(0,1,-1200,1,speed2);
				
			}
			else
			{
				
				Angle_PidPara((800*KP_A),0,30);
				Distance_PidPara(KP_D,0,KD_D2);
				if(sTX < -1000)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=1;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				if(straightLine(1,0,1900,0,speed1) == 1)
					flagOne++;
			}
			break;
			
		case middleOutLine:
			if(sTY < 2900)
			{
				Angle_PidPara((600*(266.55/18000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedY());
				straightLine(1,0,1900,0,speed3);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTY > 3800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=2;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				if(straightLine(0,1,-4300,0,speed1) == 1)
				{
					flagOne++;
					step++;
				}
			}
			break;
		default: flagOne=0;
			break;
	}
}


/**
  * @brief  顺时针走方形，外圈定点投球
  * @note	
  * @param 
  * @retval None
  */
void StopShootOne(void)
{
	float sTAngle=GetAngle();
	float sTX=GetPosX();
	float sTY=GetPosY();
	float speed1=1200;
	float speed3=1800;
	switch(flagOne)
	{
			
		case stopShootLineOne:
			judgeSpeed=0;
			if(sTX < 450)
			{ 
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/18000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(0,1,-4300,0,speed3);
				}
				else
				{
					if(sTX < -750)
					{
						Angle_PidPara((600*(266.55/18000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(0,1,-4300,0,speed3);
					}
					else
					{
						VelCrl(CAN1,1,0);
						VelCrl(CAN1,2,0);
					}
				}
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTX > 1400)
				{
					shootjudge();
					shootFlagOne=3;
				}
				if(straightLine(1,0,-1900,1,speed1) == 1)
					flagOne++;
			}
			break;
			
		case stopShootLineTwo:
			if(sTY > 1800)
			{
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/18000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(1,0,-1900,1,speed3);
				}
				else
				{
					if(sTY > 3200)
					{
						Angle_PidPara((600*(266.55/18000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(1,0,-1900,1,speed3);
					}
					else
					{
						VelCrl(CAN1,1,0);
						VelCrl(CAN1,2,0);
					}
				}
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTY < 800)
				{
					shootjudge();
					shootFlagOne=0;
				}
				
				if(straightLine(0,1,-300,1,speed1) == 1)
					flagOne++;
			}
			break;
			
		case stopShootLineThr:
			if(sTX > -400)
			{
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/18000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(0,1,-300,1,speed3);
				}
				else
				{
					if(sTX > 1000)
					{
						Angle_PidPara((600*(266.55/18000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(0,1,-300,1,speed3);
					}
					else
					{
						VelCrl(CAN1,1,0);
						VelCrl(CAN1,2,0);
					}
				}
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTX < -1400)
				{
					shootjudge();
					shootFlagOne=1;	
				}	
				if(straightLine(1,0,1900,0,speed1) == 1)
				{
					flagOne++;
				}
			}
			break;
		case stopShootLineFor:
			if(sTY < 2800)
			{
				judgeSpeed=fabs(GetSpeeedY());
				if(!stopFlg)
				{
					
					Angle_PidPara((600*(266.55/18000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(1,0,1900,0,speed3);
				}
				else
				{
					if(sTY < 1500)
					{
						Angle_PidPara((600*(266.55/18000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(1,0,1900,0,speed3);
					}
					else
					{
						VelCrl(CAN1,1,0);
						VelCrl(CAN1,2,0);
					}						
				}
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTY > 3800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=2;
				}
				else 
					judgeSpeed=fabs(GetSpeeedY());
				if(straightLine(0,1,-4300,0,speed1) == 1)
				{
					flagOne++;
					step++;
				}
			}
			break;	
		default: flagOne=0;
			break;
	}
	
}

/**
  * @brief  逆时针走方形，内圈圆，中圈方形，从内圈开始
  * @note	
  * @param 
  * @retval None
  */
void BiggerSquareThr(void)
{
	float sTAngle=GetAngle();
	float sTX=GetPosX();
	float sTY=GetPosY();
	float speed1=1200;
	float speed2=1500;
	

	switch(flagOne)
	{
		static int Tangencyflag=0,Tangencyflag2=0,lastTangencyflag=0,sureflag=0;
		case innerLine:
			if(GetPosY()<1900)
			{
				shootFlagOne=1;
				judgeSpeed=0;
				shootFlagOne=4;
				Angle_PidPara(27,0,0);
				Distance_PidPara(0.1,0,4);
				straightLine(1,0,-700,0,1500);
			}
			else
			{
				flagOne++;
			}
			break;
		case innerRound:
			judgeSpeed=0;
			Angle_PidPara(80,0,0);
			Distance_PidPara(0.1,0,0);
				N_SET_closeRound(0,2300,450,2,1500);
			  if(GetPosX()<-100&&GetPosY()<1800)  
					Tangencyflag2=1;
				if(GetAngle()>-92&&GetAngle()<-88)
					Tangencyflag=1;
		    else Tangencyflag=0;
		    if(GetPosY()>2300)
			    sureflag=1;
		    if(Tangencyflag==1&&lastTangencyflag==0&&sureflag&&Tangencyflag2)
			{
			    flagOne++;
			}
		    lastTangencyflag=Tangencyflag;
			  break;
	 case middleLineOne:
			if(GetPosY()<2700)
			{
				judgeSpeed=0;
				Angle_PidPara(30,0,0);
				Distance_PidPara(0.1,0,3);
				straightLine(1,0,-1200,0,1500);
			}
			else
			{
				shootFlagOne=1;
				Angle_PidPara((800*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,25); 
				if(straightLine(0,1,-3400,1,speed1) == 1)
				flagOne++;
			}
			break;
		case middleLineTwo:
			if(sTX > -320)
			{
				judgeSpeed=fabs(GetSpeeedX());
				straightLine(0,1,-3400,1,speed2);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,20);
				if(sTX < -400)
				{
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=0;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				if(straightLine(1,0,1200,1,speed1) == 1)
					flagOne++;
			}
			break;
		case middleLineThr:
			if(sTY > 2320)
			{
				judgeSpeed=fabs(GetSpeeedY());
				Angle_PidPara((600*(266.55/17000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				straightLine(1,0,1200,1,speed2);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,20);
				if(sTY < 1600)
				{
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=3;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				if(straightLine(0,1,-1200,0,speed1) == 1)
					flagOne++;
			}
			break;
		case middleLineFor:
			if(sTX < 800)
			{
				judgeSpeed=fabs(GetSpeeedX());
				Angle_PidPara((600*(266.55/19000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				straightLine(0,1,-1200,0,speed2);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,18);
				if(sTX > 1600)
				{
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=2;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				if(straightLine(1,0,-1900,0,speed1) == 1)
					flagOne++;
			}
			break;
			
		case middleOutLine:
			if(sTY < 3200)
			{
				Angle_PidPara((600*(266.55/17000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedY());
				straightLine(1,0,-1900,0,speed2);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTY > 3800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=1;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				if(straightLine(0,1,-4300,1,speed1) == 1)
				{
					step++;
					flagOne++;
				}
			}
			break;
		default: flagOne=0;
			break;
	}
}

/**
  * @brief  逆时针走方形，外圈定点投球
  * @note	
  * @param 
  * @retval None
  */
void StopShootTwo(void)
{
	float sTAngle=GetAngle();
	float sTX=GetPosX();
	float sTY=GetPosY();
	float speed1=1200;
	float speed3=1800;
	
	switch(flagOne)
	{
			
		case stopShootLineOne:
			judgeSpeed=0;
			if(sTX > -800)
			{ 
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/17000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(0,1,-4300,1,speed3);
				}
				else
				{
					if(sTX > 750)
					{
						Angle_PidPara((600*(266.55/17000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(0,1,-4300,1,speed3);
					}
					else
					{
						VelCrl(CAN1,1,0);
						VelCrl(CAN1,2,0);
					}
				}
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTX < -1400)
				{
					shootjudge();
					shootFlagOne=0;
				}
				if(straightLine(1,0,1900,1,speed1) == 1)
					flagOne++;
			}
			break;
			
		case stopShootLineTwo:
			if(sTY > 1400)
			{
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/17000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(1,0,1900,1,speed3);
				}
				else
				{
					if(sTY > 3600)
					{
						Angle_PidPara((600*(266.55/18000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(1,0,1900,1,speed3);
					}
					else
					{
						VelCrl(CAN1,1,0);
						VelCrl(CAN1,2,0);
					}
				}
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTY < 800)
				{
					shootjudge();
					shootFlagOne=3;
				}
				if(straightLine(0,1,-300,0,speed1) == 1)
					flagOne++;
			}
			break;
			
		case stopShootLineThr:
			if(sTX < 700)
			{
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/17000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(0,1,-300,0,speed3);
				}
				else
				{
					if(sTX < -1000)
					{
						Angle_PidPara((600*(266.55/17000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(0,1,-300,0,speed3);
					}
					else
					{
						VelCrl(CAN1,1,0);
						VelCrl(CAN1,2,0);
					}
				}
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTX > 1400)
				{
					shootjudge();
					shootFlagOne=2;
				}
				if(straightLine(1,0,-1900,0,speed1) == 1)
					flagOne++;
			}
			break;
		case stopShootLineFor:
			if(sTY < 3200)
			{
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/17000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(1,0,-1900,0,speed3);
				}
				else
				{
					if(sTY < 1500)
					{
						Angle_PidPara((600*(266.55/17000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(1,0,-1900,0,speed3);
					}
					else
					{
						VelCrl(CAN1,1,0);
						VelCrl(CAN1,2,0);
					}						
				}
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTY > 3800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
				}
				if(straightLine(0,1,-4300,1,speed1) == 1)
				{
					flagOne++;
					step++;
				}
			}
			break;	
		default: flagOne=0;
			break;
	}
	
}
/**
  * @brief  顺时针走方形，从中圈开始
  * @note	
  * @param 
  * @retval None
  */
void BiggerSquareFiv(void)
{
	float sTAngle=GetAngle();
	float sTX=GetPosX();
	float sTY=GetPosY();
	float speed1=1200;
	float speed2=1500;
	float speed3=1800;
	switch(flagOne)
	{
		case 0:
			flagOne=middleLineOne;
			shootFlagOne=1;
		case middleLineOne:
			if(sTY < 2600)
			{
				
				judgeSpeed=fabs(GetSpeeedY());
				Angle_PidPara(30,0,0);
				Distance_PidPara(0.10,0,3);
				straightLine(1,0,1200,0,speed2);
			}
			else
			{
				Angle_PidPara((800*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,25); 
				if(sTY > 2900)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=2;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				if(straightLine(0,1,-3400,0,speed1) == 1)
					flagOne++;
			}
			break;
		case middleLineTwo:
			if(sTX < 160)
			{
				judgeSpeed=fabs(GetSpeeedX());
				straightLine(0,1,-3400,0,speed2);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTX > 800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=3;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				if(straightLine(1,0,-1200,1,speed1) == 1)
					flagOne++;
			}
			break;
		case middleLineThr:
			if(sTY > 2320)
			{
				Angle_PidPara((600*(266.55/18000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedY());
				straightLine(1,0,-1200,1,speed2);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTY < 1600)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=0;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				if(straightLine(0,1,-1200,1,speed1) == 1)
					flagOne++;
			}
			break;
		case middleLineFor:
			if(sTX > -400)
			{
				Angle_PidPara((600*(266.55/20000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedX());
				straightLine(0,1,-1200,1,speed2);
				
			}
			else
			{
				
				Angle_PidPara((800*KP_A),0,30);
				Distance_PidPara(KP_D,0,KD_D2);
				if(sTX < -1000)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=1;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				if(straightLine(1,0,1900,0,speed1) == 1)
					flagOne++;
			}
			break;
			
		case middleOutLine:
			if(sTY < 2900)
			{
				Angle_PidPara((600*(266.55/18000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedY());
				straightLine(1,0,1900,0,speed3);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTY > 3800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=2;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				if(straightLine(0,1,-4300,0,speed1) == 1)
				{
					flagOne++;
					step++;
				}
			}
			break;
		default: flagOne=0;
			break;
	}
}
/**
  * @brief  逆时针走方形，从中圈开始
  * @note	
  * @param 
  * @retval None
  */
void BiggerSquareSix(void)
{
	float sTAngle=GetAngle();
	float sTX=GetPosX();
	float sTY=GetPosY();
	float speed1=1200;
	float speed2=1500;
	float speed3=1800;
	switch(flagOne)
	{
		case 0:
				flagOne=middleLineOne;
				shootFlagOne=middleLineOne;
		case middleLineOne:
			if(GetPosY() < 2700)
			{
				judgeSpeed=fabs(GetSpeeedY());
				Angle_PidPara(30,0,0);
				Distance_PidPara(0.1,0,3);
				straightLine(1,0,-1200,0,speed2);
			}
			else
			{
				Angle_PidPara((800*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,25);
				if(sTY > 2900)
				{
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=1;
				}
				else
				judgeSpeed=fabs(GetSpeeedY());
				if(straightLine(0,1,-3400,1,speed1) == 1)
				flagOne++;
			}
			break;
		case middleLineTwo:
			if(sTX > -320)
			{
				judgeSpeed=fabs(GetSpeeedX());
				straightLine(0,1,-3400,1,speed2);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,20);
				if(sTX < -400)
				{
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=0;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				if(straightLine(1,0,1200,1,speed1) == 1)
					flagOne++;
			}
			break;
		case middleLineThr:
			if(sTY > 2320)
			{
				judgeSpeed=fabs(GetSpeeedY());
				Angle_PidPara((600*(266.55/17000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				straightLine(1,0,1200,1,speed2);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,20);
				if(sTY < 1600)
				{
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=3;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				if(straightLine(0,1,-1200,0,speed1) == 1)
					flagOne++;
			}
			break;
		case middleLineFor:
			if(sTX < 800)
			{
				judgeSpeed=fabs(GetSpeeedX());
				Angle_PidPara((600*(266.55/19000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				straightLine(0,1,-1200,0,speed2);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,18);
				if(sTX > 1600)
				{
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=2;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				if(straightLine(1,0,-1900,0,speed1) == 1)
					flagOne++;
			}
			break;
			
		case middleOutLine:
			if(sTY < 3200)
			{
				Angle_PidPara((600*(266.55/17000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedY());
				straightLine(1,0,-1900,0,speed3);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(sTY > 3800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=1;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				if(straightLine(0,1,-4300,1,speed1) == 1)
				{
					step++;
					flagOne++;
				}
			}
			break;
		
		default: flagOne=0;
			break;
	}
}

/**
  * @brief  顺时针内圈收一圈球定点
  * @note	
  * @param 
  * @retval None
  */
void BiggerSquareSte(void)
{
	static int Tangencyflag=0,Tangencyflag2=0,lastTangencyflag=0,sureflag=0;
	float sTAngle=GetAngle();
	float sTX=GetPosX();
	float sTY=GetPosY();
	
	switch(flagOne)
	{
		case innerLine:
			shootFlagOne=2;
			judgeSpeed=0;
			if(sTY < 1900)
			{    
				shootFlagOne=4;
				Angle_PidPara(27,0,0);
				Distance_PidPara(0.1,0,4);
				straightLine(1,0,700,0,1500);
			}
			else
			{
				flagOne++;
			}
			break;
		case innerRound:
			judgeSpeed=0;
			Angle_PidPara(80,0,0);
		    Distance_PidPara(0.1,0,0);
				N_SET_closeRound(0,2300,450,1,1500);
			if(GetPosX() > 100 && GetPosY() < 1800)  
				Tangencyflag2=1;
				if(GetAngle() > 88 && GetAngle() < 92)
					Tangencyflag=1;
				else 
					Tangencyflag=0;
		    if(GetPosY()>2300)
			    sureflag=1;
		    if(Tangencyflag == 1 && lastTangencyflag == 0 && sureflag && Tangencyflag2)
			{
			    flagOne=stopShoot;
				step=2;
			}
		    lastTangencyflag=Tangencyflag;
			break;
		default: flagOne=0;
			break;
	}
}
/**
  * @brief  逆时针内圈收一圈球定点
  * @note	
  * @param 
  * @retval 0 正常，1 避障
  */
void BiggerSquareEit(void)
{
	static int Tangencyflag=0,Tangencyflag2=0,lastTangencyflag=0,sureflag=0;
	switch(flagOne)
	{
		
		case innerLine:
			if(GetPosY()<1900)
			{
				shootFlagOne=1;
				judgeSpeed=0;
				shootFlagOne=4;
				Angle_PidPara(27,0,0);
				Distance_PidPara(0.1,0,4);
				straightLine(1,0,-700,0,1500);
			}
			else
			{
				flagOne++;
			}
			break;
		case innerRound:
			judgeSpeed=0;
			Angle_PidPara(80,0,0);
			Distance_PidPara(0.1,0,0);
				N_SET_closeRound(0,2300,450,2,1500);
			  if(GetPosX()<-100&&GetPosY()<1800)  
					Tangencyflag2=1;
				if(GetAngle()>-92&&GetAngle()<-88)
					Tangencyflag=1;
		    else Tangencyflag=0;
		    if(GetPosY()>2300)
			    sureflag=1;
		    if(Tangencyflag==1&&lastTangencyflag==0&&sureflag&&Tangencyflag2)
			{
			    flagOne=stopShoot;
				step=2;
			}
		    lastTangencyflag=Tangencyflag;
			  break;
		default: flagOne=0;
			break;
	}
}
/**
  * @brief  避障处理
  * @note	 
  * @param getAdcFlag：得到模式标志位
  * @retval None
  */
uint8_t Troubleshoot(uint8_t *getAdcFlag)
{
	//后退角度
	static float angleTurn=0;
	
	//故障类型
	static uint8_t errFlag=0;
	
	//往后推标志位
	static uint8_t backErrFlag=0;
	
	//侧推标志位
	static uint8_t sidePushErrFlag=0;
	
	//卡住标志位
	static uint8_t stuckErrFlag=0;
	
	//定点被撞标志位
	static uint8_t stopErrFlag=0;
	
	//故障反应切换计时
	static uint16_t walkCnt=0;
	
	//速度与车身角度差值
	float angleErr=0;
	
	//速度的角度
	float speedAngle=0;
	
	//被推标志位置一延时，防止误判
	static uint16_t pushCnt=0;
	
	//卡住标志位置一延时，防止误判
	static uint16_t stuckCnt=0;
	
	//定点打被撞标志位置一延时，防止误判
	static uint16_t stopCnt=0;
	
	if(GetSpeeedY() < 1 && GetSpeeedY() > -1)
	{
		if(GetSpeeedX() < 0)
			speedAngle=90;
		else 
			speedAngle=-90;
	}
	else
		speedAngle=-atan(GetSpeeedX()/GetSpeeedY())*180/PI;
	//速度方向与车头方向不同
	if(GetSpeeedY() < 0)
		speedAngle=speedAngle+180;	
	if(speedAngle > 180)
		speedAngle=speedAngle-360;
	else if(speedAngle < -180)
		speedAngle=speedAngle+360;
	
	angleErr=fabs(speedAngle-GetAngle());
	if(angleErr > 180)
		angleErr=fabs(angleErr-360);

	//故障判断
	
	if(angleErr > 30 && errFlag == 0 && flagOne != 14 && stuckErrFlag == 0 && (fabs(GetSpeeedY()) > 200 || fabs(GetSpeeedX()) > 200) && (fabs(GetSpeeedY()) > 0 && fabs(GetSpeeedX()) > 0))
	{
		pushCnt++;
		if(pushCnt > 200)
		{
			
			//被往后推
			if(angleErr > 165 && sidePushErrFlag != 1)
			{
				pushCnt=0;
				backErrFlag=1;
				
				if(((*getAdcFlag == 3 || *getAdcFlag == 4 || *getAdcFlag == 5) && flagOne > 5) || ((*getAdcFlag == 0 || *getAdcFlag == 1 || *getAdcFlag == 2) && flagOne <= 5))
				{
					angleTurn=GetAngle()-90;
					if(angleTurn < -180)
						angleTurn=angleTurn+360;
				}
				else if(((*getAdcFlag == 3 || *getAdcFlag == 4 || *getAdcFlag == 5) && flagOne <= 5) || ((*getAdcFlag == 0 || *getAdcFlag == 1 || *getAdcFlag == 2) && flagOne > 5))
				{
					angleTurn=GetAngle()+90;
					if(angleTurn > 180)
						angleTurn=angleTurn-360;
				}
			}
			//横向被撞
			else if(angleErr <= 165 && backErrFlag != 1)
			{
				pushCnt=0;
				sidePushErrFlag=1;
			}
		}
	}
	else 
	{
		pushCnt=0;
	}
		
	//卡住
	if((fabs(GetSpeeedY()) < 300 && fabs(GetSpeeedX()) < 300) && flagOne != 14 && stopFlg != 1 && backErrFlag == 0 && sidePushErrFlag == 0)
	{
		stuckCnt++;
		if(stuckCnt > 100)
		{
			stuckErrFlag= 1;
			stuckCnt=0;
			
			//在赛场边缘
			if((GetPosX() < -2000 || GetPosX() > 2000) && (GetPosY() > 4335 || GetPosY() < 335))
			{
				errFlag=2;
			}
			
			//在赛场中间
			else if(GetPosX() < 1000 && GetPosX() > -1000 && GetPosY() < 3335 && GetPosY() > 1335)
				errFlag=3;
			else
			{
				//后转弯角度为当前角度加或减90度
				if(((*getAdcFlag == 3 || *getAdcFlag == 4 || *getAdcFlag == 5) && flagOne > 5) || ((*getAdcFlag == 0 || *getAdcFlag == 1 || *getAdcFlag == 2) && flagOne <= 5))
				{
					angleTurn=GetAngle()-90;
					if(angleTurn < -180)
						angleTurn=angleTurn+360;
				}
				else if(((*getAdcFlag == 3 || *getAdcFlag == 4 || *getAdcFlag == 5) && flagOne <= 5) || ((*getAdcFlag == 0 || *getAdcFlag == 1 || *getAdcFlag == 2) && flagOne > 5))
				{
					angleTurn=GetAngle()+90;
					if(angleTurn > 180)
						angleTurn=angleTurn-360;
				}
				errFlag=1;
			}
		}
		
	}
	else
	{
		errFlg=0;
		stuckCnt=0;
	}
	
	//停止打球被撞
	if((stopFlg == 1 ||  errFlg == 3) && (fabs(GetSpeeedY()) > 100 || fabs(GetSpeeedX()) > 100))
	{
		stopCnt++;
		if(stopCnt > 100)
		{
			stopCnt=0;
			if(stopFlg == 1)
				stopFlg=0;
			else if(errFlg == 3)
				noRightBall=1;
				
		}
	}
	else
		stopCnt=0;
	
	//故障处理
	if((backErrFlag == 1 || sidePushErrFlag == 1)&& stopFlg == 0)
	{
		walkCnt++;
		if(walkCnt < 300)
			BackTurn(angleTurn,1500);
		else if(walkCnt < 600)
			Turn(-angleTurn,1500);
		if(sqrt(GetSpeeedX()*GetSpeeedX()+GetSpeeedY()*GetSpeeedY()) > 1400 || fabs(angleTurn-GetAngle()) < 10 || fabs(angleTurn-GetAngle()) > 350 || walkCnt >= 600)
		{
			sidePushErrFlag=0;
			backErrFlag=0;
			walkCnt=0;
			
			//顺时针变逆时针或逆时针变顺时针
			if(*getAdcFlag == 3 || *getAdcFlag == 4 || *getAdcFlag == 5)
			*getAdcFlag=1;
			else if(*getAdcFlag == 0 || *getAdcFlag == 1 || *getAdcFlag == 2)
			*getAdcFlag=4;
			
			if(step == 0)
			{	if((flagOne <= 6 && flagOne >= 2) || flagOne == 0)			
					flagOne=flagOne+4;
				else if(flagOne > 6 && flagOne <= 10)
					flagOne=flagOne-4;
				else;
			}
			else if(step == 1)
			{
				if(flagOne == 10)
					flagOne=8;
				if(flagOne == 8)
					flagOne=10;
			}
			else
			{
				if(flagOne < 14)
					flagOne++;
				else if(flagOne > 14)
					flagOne=12;
					
			}
		}
		else;
	}
	
	if(stuckErrFlag == 1)
	{
		walkCnt++;
		if(errFlag == 2)
		{
			//在桶下
			if(GetPosX() < -2000 && GetPosY() < 335)
			{
				if(GetAngle() > 45 && GetAngle() < -135)
				{
					if(walkCnt < 300)
						BackTurn(135,1000);
					else if(walkCnt < 600)
						BackTurn(135,2000);
					else if(walkCnt < 800)
					{
						VelCrl(CAN1, BACK_WHEEL_ID,(1500*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*WHEEL_DIAMETER*TRANSMISSION_RATIO));
						VelCrl(CAN1, TURN_AROUND_WHEEL_ID,0);
					}
				}
					
				else
				{
					if(walkCnt < 300)
						Turn(-45,1500);
					else if(walkCnt < 600)
						Turn(-45,2500);
					else if(walkCnt < 800)
					{
						VelCrl(CAN1, BACK_WHEEL_ID,(1500*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*WHEEL_DIAMETER*TRANSMISSION_RATIO));
						VelCrl(CAN1, TURN_AROUND_WHEEL_ID,0);
					}
				}
					
			}
			else if(GetPosX() > 2000 && GetPosY() < 335)
			{
				if(GetAngle() > 135 && GetAngle() < -45)
				{
					if(walkCnt < 300)
						BackTurn(-135,1000);
					else if(walkCnt < 600)
						BackTurn(-135,2000);
					else if(walkCnt < 800)
					{
						VelCrl(CAN1, BACK_WHEEL_ID,(1500*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*WHEEL_DIAMETER*TRANSMISSION_RATIO));
						VelCrl(CAN1, TURN_AROUND_WHEEL_ID,0);
					}
				}
					
				else
				{
					if(walkCnt < 300)
						Turn(45,1500);
					else if(walkCnt < 600)
						Turn(45,2500);
					else if(walkCnt < 800)
					{
						VelCrl(CAN1, BACK_WHEEL_ID,(1500*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*WHEEL_DIAMETER*TRANSMISSION_RATIO));
						VelCrl(CAN1, TURN_AROUND_WHEEL_ID,0);
					}
				}
			}
				
			else if(GetPosX() < -2000 && GetPosY() > 4335)
			{
				if(GetAngle() > -45 && GetAngle() < 135)
				{
					if(walkCnt < 300)
						BackTurn(45,1000);
					else if(walkCnt < 600)
						BackTurn(45,2000);
					else if(walkCnt < 800)
					{
						VelCrl(CAN1, BACK_WHEEL_ID,(1500*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*WHEEL_DIAMETER*TRANSMISSION_RATIO));
						VelCrl(CAN1, TURN_AROUND_WHEEL_ID,0);
					}
				}
					
				else
				{
					if(walkCnt < 300)
						Turn(-135,1000);
					else if(walkCnt < 600)
						Turn(-135,2000);
					else if(walkCnt < 800)
					{
						VelCrl(CAN1, BACK_WHEEL_ID,(1500*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*WHEEL_DIAMETER*TRANSMISSION_RATIO));
						VelCrl(CAN1, TURN_AROUND_WHEEL_ID,0);
					}
				}
			}
				
			else if(GetPosX() > 2000 && GetPosY() > 4335)
			{
				if(GetAngle() > -135 && GetAngle() < 45)
				{
					if(walkCnt < 300)
						BackTurn(-45,1000);
					else if(walkCnt < 400)
						BackTurn(-45,2000);
					else if(walkCnt < 800)
					{
						VelCrl(CAN1, BACK_WHEEL_ID,(1500*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*WHEEL_DIAMETER*TRANSMISSION_RATIO));
						VelCrl(CAN1, TURN_AROUND_WHEEL_ID,0);
					}
				}
					
				else
				{
					if(walkCnt < 300)
						Turn(135,1500);
					else if(walkCnt < 600)
						Turn(135,2500);
					else if(walkCnt < 800)
					{
						VelCrl(CAN1, BACK_WHEEL_ID,(1500*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(PI*WHEEL_DIAMETER*TRANSMISSION_RATIO));
						VelCrl(CAN1, TURN_AROUND_WHEEL_ID,0);
					}
				}
			}
			if(fabs(sqrt(GetSpeeedX()*GetSpeeedX()+GetSpeeedY()*GetSpeeedY())) > 950 || walkCnt >= 800)
			{
				walkCnt=0;
				errFlag=0;
				stuckErrFlag=0;
			}
		}
		
		//其他地方
		else if(errFlag == 1)	
		{
			walkCnt++;
			if(walkCnt < 200)
				BackTurn(angleTurn,1000);
			else if(walkCnt < 400)
				Turn(-angleTurn,1000);
			else 
			{
				errFlg=3;
				walkCnt=0;
			}
			
			if(sqrt(GetSpeeedX()*GetSpeeedX()+GetSpeeedY()*GetSpeeedY()) > 800 || fabs(angleTurn-GetAngle()) < 10 || fabs(angleTurn-GetAngle()) > 350)
			{
				errFlag=0;
				stuckErrFlag=0;
				walkCnt=0;
				if(*getAdcFlag == 3 || *getAdcFlag == 4 || *getAdcFlag == 5)
				*getAdcFlag=1;
				else if(*getAdcFlag == 0 || *getAdcFlag == 1 || *getAdcFlag == 2)
				*getAdcFlag=4;
				
				if(step == 0)
				{	if((flagOne <= 6 && flagOne >= 2) || flagOne == 0)			
						flagOne=flagOne+4;
					else if(flagOne > 6 && flagOne <= 10)
						flagOne=flagOne-4;
					else;
				}
				else if(step == 1)
				{
					if(flagOne == 10)
						flagOne=8;
					if(flagOne == 8)
						flagOne=10;
				}
				else
				{
					if(flagOne < 14)
						flagOne++;
					else if(flagOne >= 17)
						flagOne=12;
						
				}
			}
			else;
		}
		
		//在场地中间
		else if(errFlag == 3)
		{
			if(noRightBall == 0)
			{
				errFlg=3;
				VelCrl(CAN1,1,0);
				VelCrl(CAN1,2,0);
			}
			else
			{
				walkCnt++;
				if(walkCnt < 400)
					BackTurn(angleTurn,1000);
				else if(walkCnt < 800)
					Turn(angleTurn,1000);
				else 
					walkCnt=0;
				
				if(sqrt(GetSpeeedX()*GetSpeeedX()+GetSpeeedY()*GetSpeeedY()) > 800 || fabs(angleTurn-GetAngle()) < 10 || fabs(angleTurn-GetAngle()) > 350)
				{
					errFlag=0;
					stuckErrFlag=0;
					walkCnt=0;
					if(*getAdcFlag == 3 || *getAdcFlag == 4 || *getAdcFlag == 5)
					*getAdcFlag=1;
					else if(*getAdcFlag == 0 || *getAdcFlag == 1 || *getAdcFlag == 2)
					*getAdcFlag=4;
					
					if(step == 0)
					{	if((flagOne <= 6 && flagOne >= 2) || flagOne == 0)			
							flagOne=flagOne+4;
						else if(flagOne > 6 && flagOne <= 10)
							flagOne=flagOne-4;
						else;
					}
					else if(step == 1)
					{
						if(flagOne == 10)
							flagOne=8;
						if(flagOne == 8)
							flagOne=10;
					}
					else
					{
						if(flagOne < 14)
							flagOne++;
						else if(flagOne == 15)
							flagOne=12;
							
					}
				}
				else;
			}
				
		}			
		
	}
	
	if(sidePushErrFlag == 0 && stuckErrFlag == 0 && stopErrFlag == 0 && backErrFlag == 0)
		return 0;
	else 
		return 1;

}

/**
  * @brief  走形
  * @note	 
  * @param getAdcFlag：得到模式标志位
  * @retval None
  */

void Walk(uint8_t *getAdcFlag)
{
	uint8_t fault=0;
	errFlg=3;
	fault=Troubleshoot(getAdcFlag);
	if(fault == 0)
	{
		//顺内圈
		if(*getAdcFlag == 0)
		{
			switch(step)
			{
				case 0:
					BiggerSquareOne();
					break;
				case 1:
					StopShootOne();
					break;
				case 2:
					Round();
					break;
				default: break;
			}		
		}
		
		//顺中圈
		else if(*getAdcFlag == 1)
		{
			switch(step)
			{
				case 0:
					BiggerSquareFiv();
					break;
				case 1:
					StopShootOne();
					break;
				case 2:
					Round();
					break;
			}	
			
		}
		
		//顺定点
		else if(*getAdcFlag == 2)
		{
			switch(step)
			{
				case 0:
					BiggerSquareSte();
					break;
				case 2:
					Round();
					break;
			
			
			}
		}
		
		//逆外圈
		else if(*getAdcFlag == 3)
		{
			switch(step)
			{
				case 0:
					BiggerSquareThr();
					break;
				case 1:
					StopShootTwo();
					break;
				case 2:
					Round2();
					break;
				default: break;
			}	
		
		}
		
		//逆中圈
		else if(*getAdcFlag == 4)
		{
			switch(step)
			{
				case 0:
					BiggerSquareSix();
					break;
				case 1:
					StopShootTwo();
					break;
				case 2:
					Round2();
					break;
			}
		}
		
		//逆定点
		else if(*getAdcFlag == 5)
		{
			switch(step)
			{
				case 0:
					BiggerSquareEit();
					break;
				case 2:
					Round2();
					break;
			
			
			}
		}
	}
	else;
	
	//串口发数赋值
	uv4.judgeSp=judgeSpeed;
	uv4.flgOne=flagOne;
	uv4.errflg=fault;
}



/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/





