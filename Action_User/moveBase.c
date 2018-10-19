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


//用于大的步骤的连接
uint8_t step=0;

//用于小的步骤的连接
uint8_t flagOne=0;

//用于定点打球，errFlg=3时定点打球
uint8_t errFlg=0;
extern usartValue uv4;
extern uint8_t shootReady[4];
extern uint8_t shootFlagOne;
extern uint8_t stopFlg;

//用于存储当前前进方向的速度，x轴或y轴速度，用于直线打球判断是否到达要求速度
float judgeSpeed=0;
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
	
	//根据PID算出前轮转向的速度
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
	
	//根据PID算出前轮转向的速度
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
	
	//计算出直线角度，角度突变处理
	if((B1 > -0.005f) && (B1 < 0.005f))
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
	
	//离直线35 或 52以内时表示到达直线
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
		
		//外圈圆收球
		case collectOutRound:
			Angle_PidPara(40,0,0);
			Distance_PidPara(0.1f,0,0);
			N_SET_closeRound(0,2335,1700,1,2000);
		
			//在y>-0.06x+2335和y<2335区间换中圈圆
			if(GetPosY()>-0.06f*GetPosX()+2335&&GetPosY()<2335)
				flagOne++;
			break;
		  
		//中圈圆收球
		case collectMiddleRound:
			Angle_PidPara(80,0,0);
			Distance_PidPara(0.08f,0,0);
			N_SET_closeRound(0,2335,1100,1,2000);
		
			//在x<0.13x-2335和x>0区间换内圈圆
			if(GetPosX()>0&&GetPosX()<0.13f*(GetPosY()-2335))
				flagOne++;
			break;
			
		//里圈圆收球
		case collectInnerRound:
			noRightBall=0;
			Angle_PidPara(70,0,0);
			Distance_PidPara(0.08f,0,0);
			N_SET_closeRound(0,2335,450,1,1500);
		
			//在y<-0.06x+2335和y>2335区间换定点打
			if(GetPosY()<-0.06f*GetPosX()+2335&&GetPosY()>2335)
				flagOne++;
			break;
		case stopShoot:
			errFlg=3;
			VelCrl(CAN1,1,0);
			VelCrl(CAN1,2,0);
			
			//没球之后出去收球
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
			Distance_PidPara(0.08f,0,0);
			N_SET_closeRound(0,2335,1100,1,2000);
			
			//先收内圈，没有球后再收中圈，没有之后再大圈,
			if(GetPosX()<0&&GetPosX()>0.13f*(GetPosY()-2335))
			{
				if(roundFlg == 0)
					flagOne=collectInnerRound;
				else if(roundFlg == 1)
					flagOne=collectMiddleRound;
				else if(roundFlg == 2)
					flagOne=collectOutRound;
			}
			break;
			
		//某点扫不到挡板跑半圆再扫
		case semicircle:
			noRightBall=0;
			Angle_PidPara(70,0,0);
			Distance_PidPara(0.08f,0,0);
			N_SET_closeRound(0,2335,450,1,1500);
			
			//在y>-0.06x+2335和y<2335区间换定点打
			if(GetPosY()>-0.06f*GetPosX()+2335&&GetPosY()<2335)
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
		//外圈圆收球，
		case collectOutRound:
			Angle_PidPara(40,0,0);
			Distance_PidPara(0.1f,0,0);
			N_SET_closeRound(0,2335,1700,2,2000);
		
			//在y>-0.06x+2335和y<2335区间换中圈圆
			if(GetPosY()>0.06f*GetPosX()+2335&&GetPosY()<2335)
				flagOne++;
			break;
			
		//中圈圆收球，
		case collectMiddleRound:
			Angle_PidPara(80,0,0);
			Distance_PidPara(0.08f,0,0);
			N_SET_closeRound(0,2335,1100,2,2000);
		
			//在x>-0.13x-2335和x<0区间换内圈圆
			if(GetPosX()<0&&GetPosX()>-0.13f*(GetPosY()-2335))
				flagOne++;
			break;
		
		//内圈圆收球，
		case collectInnerRound:
			noRightBall=0;
			Angle_PidPara(70,0,0);
			Distance_PidPara(0.08f,0,0);
			N_SET_closeRound(0,2335,450,2,1500);
			
			//在y<0.06x+2335和y>2335区间换定点打球
			if(GetPosY()<0.06f*GetPosX()+2335&&GetPosY()>2335)
				flagOne++;
			break;
		case stopShoot:
			errFlg=3;
			VelCrl(CAN1,1,0);
			VelCrl(CAN1,2,0);
		
			//没球之后出去收球
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
			Distance_PidPara(0.08f,0,0);
			N_SET_closeRound(0,2335,1100,2,2000);
			
			
			if(GetPosX()<0&&GetPosX()>0.13f*(GetPosY()-2335))
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
			Distance_PidPara(0.08f,0,0);
			N_SET_closeRound(0,2335,450,2,1500);
		
			//在y>0.06x+2335和y<2335区间换定点打
			if(GetPosY()>0.06f*GetPosX()+2335&&GetPosY()<2335)
				flagOne=14;
			break;
		
	}
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
  * @brief  顺时针走方形，内圈圆，中圈方形，从内圈开始（BiggerSquareOne和BiggerSquareTwo，只是一个顺时针，一个逆时针）
  * @note	
  * @param 
  * @retval None
  */

void BiggerSquareOne(void)
{
	static int Tangencyflag=0,Tangencyflag2=0,lastTangencyflag=0,sureflag=0;
	float turnSpeed=1200;
	float middleSpeed=1500;
	
	switch(flagOne)
	{
		//向上走x=-700的直线
		case innerLine:
			shootFlagOne=2;
			
			//不打球，置为0
			judgeSpeed=0;
		
			if(GetPosY() < 1900)
			{
				
				//配置PID参数，由于场地各个地方可能不太一样，参数也不一样，速度不一样，参数也不一定一样
				Angle_PidPara(27,0,0);
				Distance_PidPara(0.1,0,4);
				
				//内圈先走直线，x=-700，向上，速度1500 
				straightLine(1,0,700,0,1500);
			}
			else
			{
				//走完后变圆，更快收球
				flagOne++;
			}
			break;
			
		//顺时针走半径为450的圆
		case innerRound:
			Angle_PidPara(80,0,0);
		    Distance_PidPara(0.1,0,0);
				N_SET_closeRound(0,2300,450,1,1500);
			
			//圆到位置后拐直线
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
			
		//向上走x=-1200的直线
		case middleLineOne:
			
			shootFlagOne=2;
			//不打球，置为0
			judgeSpeed=0;
			if(GetPosY() < 2600)
			{
				
				Angle_PidPara(30,0,0);
				Distance_PidPara(0.10,0,3);
				
				//中圈先走直线，x=-1200，向上
				straightLine(1,0,1200,0,middleSpeed);
			}
			//直线转到另一条直线，转弯要减速，并且要给一定转弯空间
			else
			{
				Angle_PidPara((800*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,25);
				judgeSpeed=fabs(GetSpeeedY());
				
				//转到y=3400的直线上，向右跑，==1，即已经转到位，可以走直线
				if(straightLine(0,1,-3400,0,turnSpeed) == 1)
					flagOne++;
			}
			break;
		//向右走y=3400的直线
		case middleLineTwo:
			if(GetPosX() < 160)
			{
				judgeSpeed=fabs(GetSpeeedX());
				straightLine(0,1,-3400,0,middleSpeed);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(GetPosX() > 800)
				{
					//直线走完判断一下四个桶是否都打过，刷新
					shootjudge();
					
					//提前改变judgeSpeed，从x轴速度变到y轴
					judgeSpeed=fabs(GetSpeeedY());
					
					//提前改变要打的桶，使枪提前转到下一个桶
					shootFlagOne=3;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				
				//转到x=1200的直线上，向下跑，==1，即已经转到位，可以走直线
				if(straightLine(1,0,-1200,1,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向下走x=1200的直线
		case middleLineThr:
			if(GetPosY() > 2320)
			{
				Angle_PidPara((600*(266.55/18000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedY());
				straightLine(1,0,-1200,1,middleSpeed);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(GetPosY() < 1600)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=0;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				
				//转y=1200的直线，向左走
				if(straightLine(0,1,-1200,1,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向左走y=1200的直线
		case middleLineFor:
			if(GetPosX() > -400)
			{
				Angle_PidPara((600*(266.55/20000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedX());
				straightLine(0,1,-1200,1,middleSpeed);
				
			}
			else
			{
				
				Angle_PidPara((800*KP_A),0,30);
				Distance_PidPara(KP_D,0,KD_D2);
				if(GetPosX() < -1000)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=1;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				
				//转x=-1900的直线，向上走
				if(straightLine(1,0,1900,0,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向上走x=-1900的直线
		case middleOutLine:
			if(GetPosY() < 2900)
			{
				Angle_PidPara((600*(266.55/18000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedY());
				straightLine(1,0,1900,0,middleSpeed);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(GetPosY() > 3800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=2;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				
				//转y=4300的直线，向右走
				if(straightLine(0,1,-4300,0,turnSpeed) == 1)
				{
					flagOne++;
					
					//内圈，中圈走完，跳到下一个阶段，外圈打球
					step++;
				}
			}
			break;
		default: flagOne=0;
			break;
	}
}


/**
  * @brief  顺时针走方形，外圈定点投球（StopShootOne和StopShootTwo结构一样，只是一个顺时针，一个逆时针）
  * @note	
  * @param 
  * @retval None
  */
void StopShootOne(void)
{
	float turnSpeed=1200;
	float outSpeed=1800;
	
	switch(flagOne)
	{
		
		case stopShootLineOne:
			judgeSpeed=0;
			
			//走y=4300的直线，向右走
			if(GetPosX() < 450)
			{ 
				//外圈没有得到打球的标志，即没有球或没有扫到挡板，车继续走
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/18000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(0,1,-4300,0,outSpeed);
				}
				
				//得到打球的标志，车在相应区间（x=-750右边一点）停止，扫描后打球
				else
				{
					//x>-750时停止
					if(GetPosX() < -750)
					{
						Angle_PidPara((600*(266.55/18000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(0,1,-4300,0,outSpeed);
					}
					else
					{
						VelCrl(CAN1,1,0);
						VelCrl(CAN1,2,0);
					}
				}
				
			}
			//x>450时转弯
			else
			{
				
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(GetPosX() > 1400)
				{
					shootjudge();
					shootFlagOne=3;
				}
				
				//转x=1900的直线，向下走
				if(straightLine(1,0,-1900,1,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向下走x=1900的直线
		case stopShootLineTwo:
			if(GetPosY() > 1800)
			{
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/18000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(1,0,-1900,1,outSpeed);
				}
				//得到打球的标志，车在相应区间（y=3200下边一点）停止，扫描后打球
				else
				{
					if(GetPosY() > 3200)
					{
						Angle_PidPara((600*(266.55/18000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(1,0,-1900,1,outSpeed);
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
				if(GetPosY() < 800)
				{
					shootjudge();
					shootFlagOne=0;
				}
				
				//转y=300的直线，向左走
				if(straightLine(0,1,-300,1,turnSpeed) == 1)
					flagOne++;
			}
			break;
		
		//向左走y=300的直线
		case stopShootLineThr:
			if(GetPosX() > -400)
			{
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/18000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(0,1,-300,1,outSpeed);
				}
				//得到打球的标志，车在相应区间（X=1000左边一点）停止，扫描后打球
				else
				{
					if(GetPosX() > 1000)
					{
						Angle_PidPara((600*(266.55/18000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(0,1,-300,1,outSpeed);
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
				if(GetPosX() < -1400)
				{
					shootjudge();
					shootFlagOne=1;	
				}	
				
				//转x=-1900的直线，向上走
				if(straightLine(1,0,1900,0,turnSpeed) == 1)
				{
					flagOne++;
				}
			}
			break;
			
		//向上走x=-1900的直线
		case stopShootLineFor:
			if(GetPosY() < 2800)
			{
				judgeSpeed=fabs(GetSpeeedY());
				if(!stopFlg)
				{
					
					Angle_PidPara((600*(266.55/18000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(1,0,1900,0,outSpeed);
				}
				
				//得到打球的标志，车在相应区间（y=1500上边一点）停止，扫描后打球
				else
				{
					if(GetPosY() < 1500)
					{
						Angle_PidPara((600*(266.55/18000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(1,0,1900,0,outSpeed);
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
				if(GetPosY() > 3800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=2;
				}
				else 
					judgeSpeed=fabs(GetSpeeedY());
				
				//转y=4300的直线，向右走
				if(straightLine(0,1,-4300,0,turnSpeed) == 1)
				{
					flagOne++;
					
					//外圈走完，跳到下一个阶段，圆形收球，到中间定点打
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
	static int Tangencyflag=0,Tangencyflag2=0,lastTangencyflag=0,sureflag=0;
	float turnSpeed=1200;
	float middleSpeed=1500;
	float outSpeed=1800;
	

	switch(flagOne)
	{
		//向上走x=700的直线
		case innerLine:
			shootFlagOne=1;
			judgeSpeed=0;
			if(GetPosY()<1900)
			{
				Angle_PidPara(27,0,0);
				Distance_PidPara(0.1,0,4);
				straightLine(1,0,-700,0,1500);
			}
			else
			{
				flagOne++;
			}
			break;
			
		//逆时针走半径为450的圆
		case innerRound:
			judgeSpeed=0;
			Angle_PidPara(80,0,0);
			Distance_PidPara(0.1,0,0);
			N_SET_closeRound(0,2300,450,2,1500);
		
			//角度和位置都到了相应的情况
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
			
		//向上走x=1200的直线	
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
				
				//转到y=3400的直线上，向左跑，==1，即已经转到位，可以走直线
				if(straightLine(0,1,-3400,1,turnSpeed) == 1)
				flagOne++;
			}
			break;
			
		//向左走y=3400的直线	
		case middleLineTwo:
			if(GetPosX() > -320)
			{
				judgeSpeed=fabs(GetSpeeedX());
				straightLine(0,1,-3400,1,middleSpeed);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,20);
				if(GetPosX() < -400)
				{
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=0;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				
				//转到x=-1200的直线上，向下跑，==1，即已经转到位，可以走直线
				if(straightLine(1,0,1200,1,turnSpeed) == 1)
					flagOne++;
			}
			break;
		
		//向下走x=-1200的直线	
		case middleLineThr:
			if(GetPosY() > 2320)
			{
				judgeSpeed=fabs(GetSpeeedY());
				Angle_PidPara((600*(266.55/17000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				straightLine(1,0,1200,1,middleSpeed);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,20);
				if(GetPosY() < 1600)
				{
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=3;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				
				//转到y=1200的直线上，向右跑，==1，即已经转到位，可以走直线
				if(straightLine(0,1,-1200,0,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向右走y=1200的直线	
		case middleLineFor:
			if(GetPosX() < 800)
			{
				judgeSpeed=fabs(GetSpeeedX());
				Angle_PidPara((600*(266.55/19000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				straightLine(0,1,-1200,0,middleSpeed);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,18);
				if(GetPosX() > 1600)
				{
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=2;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				//转到x=1900的直线上，向上跑，==1，即已经转到位，可以走直线
				if(straightLine(1,0,-1900,0,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向上走x=1900的直线	
		case middleOutLine:
			if(GetPosY() < 3200)
			{
				Angle_PidPara((600*(266.55/17000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedY());
				straightLine(1,0,-1900,0,outSpeed);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(GetPosY() > 3800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=1;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				
				//转到y=4300的直线上，向左跑，==1，即已经转到位，可以走直线
				if(straightLine(0,1,-4300,1,turnSpeed) == 1)
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
	float turnSpeed=1200;
	float outSpeed=1800;
	
	switch(flagOne)
	{	
		//走y=4300的直线，向左走
		case stopShootLineOne:
			judgeSpeed=0;
			if(GetPosX() > -800)
			{ 
				//外圈没有得到打球的标志，即没有球或没有扫到挡板，车继续走
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/17000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(0,1,-4300,1,outSpeed);
				}
				
				//得到打球的标志，车在相应区间（x=750左边一点）停止，扫描后打球
				else
				{
					if(GetPosX() > 750)
					{
						Angle_PidPara((600*(266.55/17000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(0,1,-4300,1,outSpeed);
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
				if(GetPosX() < -1400)
				{
					shootjudge();
					shootFlagOne=0;
				}
				//转x=-1900的直线，向下走
				if(straightLine(1,0,1900,1,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向下走x=-1900的直线	
		case stopShootLineTwo:
			if(GetPosY() > 1400)
			{
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/17000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(1,0,1900,1,outSpeed);
				}
				//得到打球的标志，车在相应区间（y=3600下边一点）停止，扫描后打球
				else
				{
					if(GetPosY() > 3600)
					{
						Angle_PidPara((600*(266.55/18000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(1,0,1900,1,outSpeed);
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
				if(GetPosY() < 800)
				{
					shootjudge();
					shootFlagOne=3;
				}
				//转y=300的直线，向右走
				if(straightLine(0,1,-300,0,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向右走y=300的直线
		case stopShootLineThr:
			if(GetPosX() < 700)
			{
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/17000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(0,1,-300,0,outSpeed);
				}
				//得到打球的标志，车在相应区间（x=-1000右边一点）停止，扫描后打球
				else
				{
					if(GetPosX() < -1000)
					{
						Angle_PidPara((600*(266.55/17000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(0,1,-300,0,outSpeed);
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
				if(GetPosX() > 1400)
				{
					shootjudge();
					shootFlagOne=2;
				}
				
				//转x=1900的直线，向上走
				if(straightLine(1,0,-1900,0,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向上走x=1900的直线
		case stopShootLineFor:
			if(GetPosY() < 3200)
			{
				if(!stopFlg)
				{
					Angle_PidPara((600*(266.55/17000)),0,0);
					Distance_PidPara(KP_D2,0,KD_D2);
					straightLine(1,0,-1900,0,outSpeed);
				}
				//得到打球的标志，车在相应区间（y=1500上边一点）停止，扫描后打球
				else
				{
					if(GetPosY() < 1500)
					{
						Angle_PidPara((600*(266.55/17000)),0,0);
						Distance_PidPara(KP_D2,0,KD_D2);
						straightLine(1,0,-1900,0,outSpeed);
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
				if(GetPosY() > 3800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
				}
				
				//转y=4300的直线，向左走
				if(straightLine(0,1,-4300,1,turnSpeed) == 1)
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
  * @brief  顺时针走方形，从中圈开始（和BiggerSquareOne差不多，只是没有了里圈收球）（BiggerSquareFiv和BiggerSquareSix结构一样，只是一个顺时针，一个逆时针）
  * @note	
  * @param 
  * @retval None
  */
void BiggerSquareFiv(void)
{
	float turnSpeed=1200;
	float middleSpeed=1500;
	float outSpeed=1800;
	switch(flagOne)
	{
		//没有里圈收球，直接从中圈开始走
		case 0:
			flagOne=middleLineOne;
			shootFlagOne=1;
		
		//向上走x=-1200的直线
		case middleLineOne:
			if(GetPosY() < 2600)
			{
				
				judgeSpeed=fabs(GetSpeeedY());
				Angle_PidPara(30,0,0);
				Distance_PidPara(0.10,0,3);
				straightLine(1,0,1200,0,middleSpeed);
			}
			else
			{
				Angle_PidPara((800*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,25); 
				if(GetPosY() > 2900)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=2;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				
				//转到y=3400的直线上，向右跑，==1，即已经转到位，可以走直线
				if(straightLine(0,1,-3400,0,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向右走y=3400的直线
		case middleLineTwo:
			if(GetPosX() < 160)
			{
				judgeSpeed=fabs(GetSpeeedX());
				straightLine(0,1,-3400,0,middleSpeed);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(GetPosX() > 800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=3;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				
				//转到x=1200的直线上，向下跑，==1，即已经转到位，可以走直线
				if(straightLine(1,0,-1200,1,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向下走x=1200的直线
		case middleLineThr:
			if(GetPosY() > 2320)
			{
				Angle_PidPara((600*(266.55/18000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedY());
				straightLine(1,0,-1200,1,middleSpeed);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(GetPosY() < 1600)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=0;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				
				//转y=1200的直线，向左走
				if(straightLine(0,1,-1200,1,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向左走y=1200的直线
		case middleLineFor:
			if(GetPosX() > -400)
			{
				Angle_PidPara((600*(266.55/20000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedX());
				straightLine(0,1,-1200,1,middleSpeed);
				
			}
			else
			{
				
				Angle_PidPara((800*KP_A),0,30);
				Distance_PidPara(KP_D,0,KD_D2);
				if(GetPosX() < -1000)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=1;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
			
				//转x=-1900的直线，向上走
				if(straightLine(1,0,1900,0,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向上走x=-1900的直线	
		case middleOutLine:
			if(GetPosY() < 2900)
			{
				Angle_PidPara((600*(266.55/18000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedY());
				straightLine(1,0,1900,0,outSpeed);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(GetPosY() > 3800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=2;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				
				//转y=4300的直线，向右走
				if(straightLine(0,1,-4300,0,turnSpeed) == 1)
				{
					flagOne++;
					
					//内圈，中圈走完，跳到下一个阶段，外圈打球
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
	float turnSpeed=1200;
	float middleSpeed=1500;
	float outSpeed=1800;
	switch(flagOne)
	{
		case 0:
				flagOne=middleLineOne;
				shootFlagOne=middleLineOne;
		
		//向上走x=1200的直线
		case middleLineOne:
			if(GetPosY() < 2700)
			{
				judgeSpeed=fabs(GetSpeeedY());
				Angle_PidPara(30,0,0);
				Distance_PidPara(0.1,0,3);
				straightLine(1,0,-1200,0,middleSpeed);
			}
			else
			{
				Angle_PidPara((800*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,25);
				if(GetPosY() > 2900)
				{
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=1;
				}
				else
				judgeSpeed=fabs(GetSpeeedY());
				
				//转到y=3400的直线上，向左跑，==1，即已经转到位，可以走直线
				if(straightLine(0,1,-3400,1,turnSpeed) == 1)
				flagOne++;
			}
			break;
			
		//向左走y=3400的直线	
		case middleLineTwo:
			if(GetPosX() > -320)
			{
				judgeSpeed=fabs(GetSpeeedX());
				straightLine(0,1,-3400,1,middleSpeed);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,20);
				if(GetPosX() < -400)
				{
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=0;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				
				//转到x=-1200的直线上，向下跑，==1，即已经转到位，可以走直线
				if(straightLine(1,0,1200,1,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向下走x=-1200的直线	
		case middleLineThr:
			if(GetPosY() > 2320)
			{
				judgeSpeed=fabs(GetSpeeedY());
				Angle_PidPara((600*(266.55/17000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				straightLine(1,0,1200,1,middleSpeed);
				
			}
			else
			{
				
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,20);
				if(GetPosY() < 1600)
				{
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=3;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				
				//转到y=1200的直线上，向右跑，==1，即已经转到位，可以走直线
				if(straightLine(0,1,-1200,0,turnSpeed) == 1)
					flagOne++;
			}
			break;
			
		//向右走y=1200的直线	
		case middleLineFor:
			if(GetPosX() < 800)
			{
				judgeSpeed=fabs(GetSpeeedX());
				Angle_PidPara((600*(266.55/19000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				straightLine(0,1,-1200,0,middleSpeed);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,18);
				if(GetPosX() > 1600)
				{
					judgeSpeed=fabs(GetSpeeedY());
					shootFlagOne=2;
				}
				else
					judgeSpeed=fabs(GetSpeeedX());
				
				//转到x=1900的直线上，向上跑，==1，即已经转到位，可以走直线
				if(straightLine(1,0,-1900,0,turnSpeed) == 1)
					flagOne++;
			}
			break;
		
		//向上走x=1900的直线			
		case middleOutLine:
			if(GetPosY() < 3200)
			{
				Angle_PidPara((600*(266.55/17000)),0,0);
				Distance_PidPara(KP_D2,0,KD_D2);
				judgeSpeed=fabs(GetSpeeedY());
				straightLine(1,0,-1900,0,outSpeed);
				
			}
			else
			{
				Angle_PidPara((600*KP_A),0,KD_A);
				Distance_PidPara(KP_D,0,KD_D);
				if(GetPosY() > 3800)
				{
					shootjudge();
					judgeSpeed=fabs(GetSpeeedX());
					shootFlagOne=1;
				}
				else
					judgeSpeed=fabs(GetSpeeedY());
				
				//转到y=4300的直线上，向左跑，==1，即已经转到位，可以走直线
				if(straightLine(0,1,-4300,1,turnSpeed) == 1)
				{
					//转到外部打球
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
  * @brief  顺时针内圈收一圈球定点（只有BiggerSquareOne的内圈，走完直接跳到Round的定点投球）
  * @note	
  * @param 
  * @retval None
  */
void BiggerSquareSte(void)
{
	static int Tangencyflag=0,Tangencyflag2=0,lastTangencyflag=0,sureflag=0;
	
	switch(flagOne)
	{
		//向上走x=-700的直线
		case innerLine:
			shootFlagOne=2;
			judgeSpeed=0;
			if(GetPosY() < 1900)
			{    
				shootFlagOne=4;
				Angle_PidPara(27,0,0);
				Distance_PidPara(0.1,0,4);
				straightLine(1,0,700,0,1500);
			}
			
			//转顺时针圆
			else
			{
				flagOne++;
			}
			break;
		
		//顺时针走半径为450的圆
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
			
			//圆到位置，收了一圈球，定点打球
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
  * @brief  逆时针内圈收一圈球定点（只有BiggerSquareTwo的内圈，走完直接跳到Round2的定点投球）
  * @note	
  * @param 
  * @retval 0 正常，1 避障
  */
void BiggerSquareEit(void)
{
	static int Tangencyflag=0,Tangencyflag2=0,lastTangencyflag=0,sureflag=0;
	switch(flagOne)
	{
		//向上走x=700的直线
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
		
		//逆时针走半径为450的圆
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
			
			//圆到位置，收了一圈球，定点打球
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
	
	//y轴速度为0，速度的角度为90或-90
	if(GetSpeeedY() < 1 && GetSpeeedY() > -1)
	{
		if(GetSpeeedX() < 0)
			speedAngle=90;
		else 
			speedAngle=-90;
	}
	else
		speedAngle=-atan(GetSpeeedX()/GetSpeeedY())*180/PI;
	
	//计算速度方向
	if(GetSpeeedY() < 0)
		speedAngle=speedAngle+180;	
	if(speedAngle > 180)
		speedAngle=speedAngle-360;
	else if(speedAngle < -180)
		speedAngle=speedAngle+360;
	
	//速度方向与车方向的角度的差值
	angleErr=fabs(speedAngle-GetAngle());
	if(angleErr > 180)
		angleErr=fabs(angleErr-360);

	//故障判断
	//速度角度与车的角度差大于30度，且没有卡住，而且有速度
	if(angleErr > 30 && errFlag == 0 && flagOne != 14 && stuckErrFlag == 0 && (fabs(GetSpeeedY()) > 200 || fabs(GetSpeeedX()) > 200))
	{
		pushCnt++;
		if(pushCnt > 200)
		{
			
			//角度差大于165度，判断为被往后推
			if(angleErr > 165 && sidePushErrFlag != 1)
			{
				pushCnt=0;
				backErrFlag=1;
				
				//顺时针的内圈或逆时针的外圈
				if(((*getAdcFlag == 3 || *getAdcFlag == 4 || *getAdcFlag == 5) && flagOne > 5) || ((*getAdcFlag == 0 || *getAdcFlag == 1 || *getAdcFlag == 2) && flagOne <= 5))
				{
					angleTurn=GetAngle()-90;
					if(angleTurn < -180)
						angleTurn=angleTurn+360;
				}
				
				//逆时针的内圈或顺时针的外圈
				else if(((*getAdcFlag == 3 || *getAdcFlag == 4 || *getAdcFlag == 5) && flagOne <= 5) || ((*getAdcFlag == 0 || *getAdcFlag == 1 || *getAdcFlag == 2) && flagOne > 5))
				{
					angleTurn=GetAngle()+90;
					if(angleTurn > 180)
						angleTurn=angleTurn-360;
				}
			}
			//角度差大于30度，小于165，判断为横向被撞
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
		
	//速度小于300mm/s，判断为卡住
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
			
			//在外圈停住打
			if(stopFlg == 1)
				stopFlg=0;
			
			//定点打
			else if(errFlg == 3)
				noRightBall=1;
				
		}
	}
	else
		stopCnt=0;
	
	//故障处理
	//车被向后推和从侧面被撞
	if((backErrFlag == 1 || sidePushErrFlag == 1)&& stopFlg == 0)
	{
		walkCnt++;
		if(walkCnt < 300)
			BackTurn(angleTurn,1500);
		else if(walkCnt < 600)
			Turn(-angleTurn,1500);
		
		//避开了障碍
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
	
	//卡住处理
	if(stuckErrFlag == 1)
	{
		walkCnt++;
		if(errFlag == 2)
		{
			//在左下桶下方
			if(GetPosX() < -2000 && GetPosY() < 335)
			{
				//车头朝里
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
				
				//车头朝外
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
			
			//在右下桶下方
			else if(GetPosX() > 2000 && GetPosY() < 335)
			{
				//车头朝里
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
				
				//车头朝外	
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
			
			//在左上桶下方	
			else if(GetPosX() < -2000 && GetPosY() > 4335)
			{
				//车头朝里
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
				
				//车头朝外					
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
			
			//在右上桶下方	
			else if(GetPosX() > 2000 && GetPosY() > 4335)
			{
				//车头朝里
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
				
				//车头朝外					
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
			
			//避开了障碍
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
			
			//避开了障碍
			if(sqrt(GetSpeeedX()*GetSpeeedX()+GetSpeeedY()*GetSpeeedY()) > 800 || fabs(angleTurn-GetAngle()) < 10 || fabs(angleTurn-GetAngle()) > 350)
			{
				errFlag=0;
				stuckErrFlag=0;
				walkCnt=0;
				
				//顺时针变逆时针或逆时针变顺时针
				if(*getAdcFlag == 3 || *getAdcFlag == 4 || *getAdcFlag == 5)
				*getAdcFlag=1;
				else if(*getAdcFlag == 0 || *getAdcFlag == 1 || *getAdcFlag == 2)
				*getAdcFlag=4;
				
				if(step == 0)
				{	
					//中圈变外圈
					if((flagOne <= 6 && flagOne >= 2) || flagOne == 0)			
						flagOne=flagOne+4;
					
					//外圈变中圈
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
				
				//避开了障碍
				if(sqrt(GetSpeeedX()*GetSpeeedX()+GetSpeeedY()*GetSpeeedY()) > 800 || fabs(angleTurn-GetAngle()) < 10 || fabs(angleTurn-GetAngle()) > 350)
				{
					errFlag=0;
					stuckErrFlag=0;
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
	fault=Troubleshoot(getAdcFlag);
	if(fault == 0)
	{
		//顺内圈.step=0,内圈收球，中圈跑投；step=1外圈停下投球；step=2，圆形收球，定点投球
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
		
		//顺中圈.step=0,中圈跑投；step=1外圈停下投球；step=2，圆形收球，定点投球
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
		
		//顺定点.step=2，圆形收球，定点投球
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
		
		//逆外圈.step=0,内圈收球，中圈跑投；step=1外圈停下投球；step=2，圆形收球，定点投球
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
		
		//逆中圈.step=0,中圈跑投；step=1外圈停下投球；step=2，圆形收球，定点投球
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
		
		//逆定点.step=2，圆形收球，定点投球
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





