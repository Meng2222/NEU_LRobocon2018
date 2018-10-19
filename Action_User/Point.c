#include "elmo.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "timer.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_gpio.h"
#include "math.h"
#include "movebase.h"
#include "pps.h"
#include "fort.h"
#include "string.h" 
#include "Point.h" 
#include "PushBall.h"
#include "Walk.h"
#include "PushBall.h"

/***********************************************************************************
**********************************定义变量区****************************************
************************************************************************************/
extern int clockFlg;
float point2carAngle;
float lastCalculateAngle=0;
float controlAngle;
float Last_bodyAngle=0;
int certainFlag=0;
float distanceA,distanceB;
float distanceFirst,distanceSecond,distanceThird,distanceFourth;
int pointDistance;
int barrel=0;
float adcAngle=0;
int findDirection=1;
float shootVel;
extern uint8_t Ballcolor[5];
int findingCnt=0;
extern int pushBallpos;
int needBallFlag=0;
int changeBarrelFlag=0;
int state=2,pmtCnt=0,changeFlag=0,lastState=0,lastKey_A=0;
int leftPosition=0,rightPosition=0,leftFlag=0,rightFlag=0;
float rollVel;
int prepareTime=0,prepareOkFlag=0,waitCnt=0,waitTime,sureNoBallFlag=0;
int shoot_over=0;
extern int ifShoot[];
int ifShootFlag[4]={1,1,1,1};
extern int roundCnt;
int turnAngle=0,findTime=0,findAngle=0,errorDistance=0;
float perTurnAngle=0;
int errorTime=0,errorLockTime=0;
int errorLockFlag=0;
extern int deadZone;
float aimAngle=0;
float vDiff=0;
int aimErrorFlag=0,aimErrorTime=0;
float beginAngle=0;
int needBallTime=0,noNeedBallTime=0,noBallTime=0;
int pushFlag=0;
float addAngle=0;
int addAngleFlag=0;
int errorFindFlag;
extern int normalPush;
float yPosAngle;    	
float calculateAngle;

/*******************************定点时扫描空中储藏室位置并投球************************************/


/**
* @brief  控制炮台转的角度位置 并且使炮台不受车身角度的影响 始终对准某个方向
* @param  point2carAngle：固定点与车的角度
		  controlAngle：最终控制炮台的角度
		  yPosAngle：固定点与车的连线 与y轴正方向的角度
		  calculateAngle：计算得到的炮台角度
		  adcAngle：激光改变的角度
* @author ACTION
*/
void AimPoint(float point_x,float point_y)
{
	//得到固定点与车的角度
	point2carAngle=CountAngle(GetY()-point_y,-GetX()+point_x,(GetY()-point_y)/fabs((GetY()-point_y)),clockFlg);
	
	//该角度与y轴正方向的角度,将角度限制在-180~180
	yPosAngle=make_angle_in_wide(90+point2carAngle,-180);
	
	//计算得到的炮台角度
	calculateAngle=make_angle_in_wide(0-yPosAngle+GetAngle(),-180);
	
	//把计算得到的角处理成劣弧
	controlAngle=controlAngle+make_angle_in_wide(calculateAngle-lastCalculateAngle,-180);
	lastCalculateAngle=calculateAngle;
	
	//控制炮台转的位置
	YawPosCtrl(controlAngle+adcAngle);
}

/**
* @brief  用激光进行扫描寻找桶的位置
* @param  certainFlag：激光确定桶的标志位
		  findAngle：激光的扫描范围
		  perTurnAngle：每个周期激光转的角度
		  leftPosition：激光扫到桶的左边记下的炮台角度
		  rightPosition：激光扫到桶的右边记下的炮台角度
		  findDirection：激光扫描的方向 1为逆时针
		  findTime：激光测得的距离一直在误差允许的范围内的计时
		  errorDistance：误差允许的距离范围
		  errorFindFlag：激光确定桶的位置后发现此位置超出可以容忍的误差范围的标志位
* @author ACTION
*/
void LightPoint(int Distance)
{
	//调参位置
	if(roundCnt==5)
	{
		turnAngle=3;
		findTime=3;
		findAngle=15;
		perTurnAngle=0.7f;
		errorDistance=300;
	}
	else if(roundCnt!=5)
	{
		turnAngle=5;
		findTime=10;
		findAngle=30;
		perTurnAngle=0.3f;
		errorDistance=400;
		addAngle=15;
	}
	//等炮台转到指定角度
	if((int)ReadYawPos()>=(int)(controlAngle+adcAngle-1)&&(int)ReadYawPos()<=(int)(controlAngle+adcAngle+1))
	{
		//还没锁定桶
		if(certainFlag==0)
		 {
			 errorLockTime=0;			 
			 //激光扫不到的故障处理
			 errorTime++;
			 
			 //出现一边扫到另一边扫不到的情况，加大扫描范围
			 if(errorTime>=250)
			 {	 				
				 if(leftFlag==1&&rightFlag==0&&addAngleFlag==0)
				{
					findAngle+=addAngle;
					addAngleFlag=1;
				}
				else if(leftFlag==0&&rightFlag==1&&addAngleFlag==0)
				{
					findAngle+=addAngle;
					addAngleFlag=1;
				}
			 }	
			 //一直扫不到就退出定点模式
			if(errorTime>=500)
			{
				shoot_over=1;
				 errorTime=0;
			}
			//在findAngle范围内寻找，到达范围的极限时换个方向
			if(ReadYawPos()>(controlAngle+findAngle))
			{
				findDirection=1;
			}
			if(ReadYawPos()<(controlAngle-findAngle))
			{
				findDirection=-1;
			}
			if(findDirection==1)	
			{
				adcAngle-=perTurnAngle;		
			}
			else if(findDirection==-1)
			{
				adcAngle+=perTurnAngle;
			}
			//在误差允许的范围内扫描到物体，记录连续扫到的时间
			if(distanceA<Distance+errorDistance&&distanceB<Distance+errorDistance)
			{
				findingCnt++;	
			}	
			//扫到左边记录此时炮台位置，改变扫描方向，并且往右边转一个角度以节省时间
			else if(distanceA>Distance+errorDistance&&distanceB<Distance+errorDistance&&findingCnt>findTime)
			{
				leftPosition=ReadYawPos();
				leftFlag=1;
				adcAngle+=turnAngle;
				findDirection=-1;
			}	
			//扫到右边记录此时炮台位置，改变扫描方向，并且往左边边转一个角度以节省时间
			else if(distanceB>Distance+errorDistance&&distanceA<Distance+errorDistance&&findingCnt>findTime)
			{
				rightPosition=ReadYawPos();
				rightFlag=1;
				adcAngle-=turnAngle;
				findDirection=1;
			}
			//如果左右两边都记录到了炮台位置，则对这两个位置取平均得到桶的位置，结束扫描
			if(rightFlag==1&&leftFlag==1)
			{
				adcAngle=(leftPosition+rightPosition)/2-controlAngle;
				rightFlag=0;
				leftFlag=0;
				certainFlag=1;
				findingCnt=0;
			}

		}
		 //确定了桶的位置后
		 if(certainFlag==1)
		 {				 
			 aimAngle=0;
			 errorTime=0;
			 errorLockTime++;
			 //确定了桶的位置一定时间后却一直没换桶，结束定点模式
			 if(errorLockTime>=800)
			 {
				shoot_over=1;
				 certainFlag=0;
				 errorLockTime=0;
			 }
			 //如果此时两个激光测得的距离大于可以容忍的误差范围，重新扫描，并且记个标志位
			if(distanceA>Distance+errorDistance+100&&distanceB>Distance+errorDistance+100)
			{
				certainFlag=0;
				errorFindFlag+=1;
			}
			//如果此时激光测得的距离小于可以容忍的误差范围，重新扫描，并且记个标志位
			else if(distanceA<Distance-errorDistance-100||distanceB<Distance-errorDistance-100)
			{
				certainFlag=0;
				errorFindFlag+=1;
			}
			//出现一边扫到另一边扫不到的情况，把扫描范围减回去
			if(addAngleFlag==1)
			{
				findAngle-=addAngle;
				addAngleFlag=0;
			}
			//如果确定桶后重新扫描次数达3次，退出定点模式
			if(errorFindFlag>2)
			{
				errorFindFlag=0;
				shoot_over=1;
			}
		 }
	}
		
		
}


/**
* @brief  计算得到包胶轮应该的转速
* @param  rollVel：计算得到的包胶轮的转速
		  roundCnt：此时所在的圈数
* @author ACTION
*/
float ShootBallVel(float Distance)
{   
	//拟合得到的包胶轮转速与激光测得的桶的距离的关系
	rollVel=2.0f*(1E-10f)*pow(Distance,3)-3.0f*(1E-6f)*pow(Distance,2)+0.0255f*Distance+19.304f;
	if(roundCnt!=5)
	{	
		//发现该拟合公式存在一定误差的补偿
		if(rollVel<80&&rollVel>=20)
		{		
			return(rollVel-3.5f);
		}
		else if(rollVel<110&&rollVel>=80)
		{
			return(rollVel-2.5f);
		}
		else if(rollVel<20)
		{
			return(20);
		}
		else return(50);
	}
	else
	{
		rollVel-=0.5f;
		if(rollVel<80&&rollVel>=20)
		{		
			return(rollVel);
		}
		else if(rollVel<20)
		{
			return(20);
		}
		else return(50);		
	}
}

/**
* @brief  定点扫描模式的所有部分的集成
* @param  distanceFirst：根据定位系统计算出的此时离桶的水平距离
		  distanceA(B)：左(右)激光测得的距离
		  shoot_over：置1退出定点模式
		  shootVel：给包胶轮的转速
		  pointDistance：两个激光得到的距离取平均
* @author ACTION
*/
void FixedPoint(void)
{
	//定位系统得到车与4个点的距离
	distanceFirst=GetDis(-2400,0);
	distanceSecond=GetDis(-2400,4800);
	distanceThird=GetDis(2400,4800);
	distanceFourth=GetDis(2400,0);
	//激光数值转距离（单位mm）
	distanceA=2.4541f*ReadLaserAValue()+82.744f;
	distanceB=2.3839f*ReadLaserBValue()+413.37f;
	//两个激光得到的距离取平均
	pointDistance=(distanceA+distanceB)/2;
	
	//投球标志位
	pushFlag=RecognizeBall();
	
	//出现卡球的情况退出定点模式
	if(normalPush==0)
	{
		shoot_over=1;
	}
	//定点模式的扫描
	if(roundCnt!=5)
	{
		//如果四个桶都打过一遍，重置标志位
		if(ifShoot[0]==0&&ifShoot[1]==0&&ifShoot[2]==0&&ifShoot[3]==0)
		{
			for(int i=0;i<4;i++)
			{
				ifShoot[i]=1;
			}
		}
		//定点打过的桶标志位，如果定点打过四个桶，重置标志位
		if(ifShootFlag[0]==0&&ifShootFlag[1]==0&&ifShootFlag[2]==0&&ifShootFlag[3]==0)
		{
			for(int i=0;i<4;i++)
			{
				ifShootFlag[i]=1;
			}
		}
		//出现 打过的桶的标志位 与 定点打过的桶的标志位 矛盾的情况  标志位全部重置
		if(ifShoot[0]!=1||ifShootFlag[0]!=1)
		{
			if(ifShoot[1]!=1||ifShootFlag[1]!=1)
			{
				if(ifShoot[2]!=1||ifShootFlag[2]!=1)
				{
					if(ifShoot[3]!=1||ifShootFlag[3]!=1)
					{
						for(int i=0;i<4;i++)
						{
							ifShoot[i]=1;
							ifShootFlag[i]=1;
						}				
					}
				}
			}
		}
		//对要锁定的桶进行扫描并且调节包胶轮转速
		switch(barrel)
		{
			case 0:
			{
				if(ifShoot[0]==1)
				{
					barrel=1;
				}
				else if(ifShoot[1]==1)
				{
					barrel=2;
				}
				else if(ifShoot[2]==1)
				{
					barrel=3;
				}
				else if(ifShoot[3]==1)
				{
					barrel=4;
				}
				break;
			}
			case 1:
			{
				//执行扫描操作
				LightPoint(distanceFirst);
				
				//定位系统给炮台一个桶的大致方向
				AimPoint(-2500,0);
				
				//控制包胶轮的转速
				shootVel=ShootBallVel(pointDistance);
				ShooterVelCtrl(shootVel);
				break;
			}
			case 2:
			{
				LightPoint(distanceSecond);
				AimPoint(-2300,4900);
				shootVel=ShootBallVel(pointDistance);
				ShooterVelCtrl(shootVel);
				break;
			}
			case 3:
			{
				LightPoint(distanceThird);
				AimPoint(2600,4800);
				shootVel=ShootBallVel(pointDistance);
				ShooterVelCtrl(shootVel);	
				break;				
			}
			case 4:
			{
				LightPoint(distanceFourth);
				AimPoint(2200,-200);
				shootVel=ShootBallVel(pointDistance);
				ShooterVelCtrl(shootVel);
				break;
			}
			case 5:
			{
				barrel=0;
				break;
			}
			default:
			{
				break;
			}
				
		}
	}
	//第五圈的停下来扫描
	else if(roundCnt==5)
	{
		//执行扫描操作
		AimBarrel();
		shootVel=ShootBallVel(pointDistance);
		ShooterVelCtrl(shootVel);
	}
	//分球和换桶
	BarrelChange();
	PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushBallpos);
}


/**
* @brief  控制分球和变换目标空中储藏室
* @param  vDiff：计算得到的包胶轮转速与实际包胶轮转速的差值的绝对值
          prepareOkFlag：激光在扫描时分球机构确认到球的标志位
		  ifShoot[]：记录打过的桶的数组
		  ifShootFlag[]：记录定点打过的桶的数组
		  pushBallpos：转盘的位置（脉冲）
		  Barrel：空中储藏室的编号
* @author ACTION
*/
void BarrelChange(void)
{
	vDiff=fabs(shootVel-ReadShooterVel());
	//已经瞄准
	if(certainFlag==1)
	{
		prepareTime=0;
	//分球机构：还没准备好
	  if(prepareOkFlag==0)
	  {
		if(pushFlag==1&&vDiff<1)
		{
			needBallFlag=1;
			pushBallpos+=32768/2;
			//对打过的桶记标记位
			if(roundCnt==5)
			 {
				if(clockFlg==-1)
				{
					ifShoot[deadZone-1]=0;
					ifShootFlag[deadZone-1]=0;
				}
				else if(clockFlg==1)
				{
					ifShoot[4-deadZone]=0;
					ifShootFlag[4-deadZone]=0;
				}
			 }
		     else if(roundCnt!=5)
		     {
				ifShoot[barrel-1]=0;
				ifShootFlag[barrel-1]=0;
		     }			
		}
		else if(pushFlag==2)
		{
			pushBallpos-=32768/2;
		}				
	  }
	  //分球机构：已经准备好了
	  else if(prepareOkFlag==1)
	  {
		  waitTime++;
		  //等待一小段时间而且确保包胶轮转速与计算转速偏差不超过2转/s
		  if(waitTime>=50&&vDiff<2)
		  {
			pushBallpos+=32768/2;
			  //对打过的桶记标记位
			  if(roundCnt==5)
			  {
				if(clockFlg==-1)
				{
					ifShoot[deadZone-1]=0;
					ifShootFlag[deadZone-1]=0;
				}
				else if(clockFlg==1)
				{
					ifShoot[4-deadZone]=0;
					ifShootFlag[4-deadZone]=0;
				}
			  }
			  else if(roundCnt!=5)
			  {
				ifShoot[barrel-1]=0;
				ifShootFlag[barrel-1]=0;	  
			  }
			needBallFlag=1;
			prepareOkFlag=0;
			waitTime=0;
		  }
	  }
	  //延迟一小段时间以等待球击出
	  if(needBallFlag==1)
	  {
		  waitCnt++;
		  if(waitCnt>=50)
		  {
		   changeBarrelFlag=1;
		   needBallFlag=0;
		   waitCnt=0;
		   needBallFlag=0;
		  }
	  }
	  //确定没有球，退出定点模式
	  if((sureNoBallFlag>=5&&roundCnt!=3&&roundCnt!=4)||(sureNoBallFlag>=3&&(roundCnt==3||roundCnt==4)))
	  {
		shoot_over=1;
		certainFlag=0;
		errorLockTime=0;
		sureNoBallFlag=0;
	  }
	}
	//还没瞄准的准备中
	else if(certainFlag==0)
	{
		if(pushFlag==1)
		{
			sureNoBallFlag=0;
			prepareOkFlag=1;
		}
		else if(pushFlag==2)
		{
			sureNoBallFlag=0;
			pushBallpos-=32768/2;
		}
	}
	//射完球改变射球的桶
	if(changeBarrelFlag==1)
	{
		errorFindFlag=0;
		findDirection=1;
		adcAngle=0;
		aimAngle=0;
		certainFlag=0;
		changeBarrelFlag=0;
		errorLockTime=0;
		//第五圈 投完走		
		if(roundCnt==5)
		{
			aimAngle=0;
			shoot_over=1;
		}
			//定点时换桶
		else if(roundCnt!=5)
		{
			//两个标志位出现矛盾就全部重置
			if(ifShoot[0]!=1||ifShootFlag[0]!=1)
			{
				if(ifShoot[1]!=1||ifShootFlag[1]!=1)
				{
					if(ifShoot[2]!=1||ifShootFlag[2]!=1)
					{
						if(ifShoot[3]!=1||ifShootFlag[3]!=1)
						{
							for(int i=0;i<4;i++)
							{
								ifShoot[i]=1;
								ifShootFlag[i]=1;
							}				
						}
					}
				}
			}
			//切换扫描与射球的桶
			if(ifShoot[0]==1&&ifShootFlag[0]==1)
			{
				barrel=1;
			}
			else if(ifShoot[1]==1&&ifShootFlag[1]==1)
			{
				barrel=2;
			}
			else if(ifShoot[2]==1&&ifShootFlag[2]==1)
			{
				barrel=3;
			}
			else if(ifShoot[3]==1&&ifShootFlag[3]==1)
			{
				barrel=4;
			}
		}
	}

}

/**
* @brief  走行第五圈 激光扫描寻找空中储藏室
* @param  beginAngle：炮台与车头方向的角度
		  aimAngle：激光改变的角度
          aimErrorTime：激光在扫描时的计时
		  aimErrorFlag：激光的扫描时间超出给定的限制时间的标志位
* @author ACTION
*/
void AimBarrel(void)
{	
	YawPosCtrl(beginAngle+aimAngle);
	if(distanceA<3500&&distanceB<3500)
	{
		certainFlag=1;
		aimErrorTime=0;
	}
	else if(distanceA>=3500&&distanceB<3500)
	{
		aimAngle+=0.3f;
		aimErrorTime=0;
	}
	else if(distanceA<3500&&distanceB>=3500)
	{
		aimAngle-=0.3f;
		aimErrorTime=0;
	}
	else if(distanceA>3500&&distanceB>3500)
	{
		//在正负largeAngle范围内寻找
		if(ReadYawPos()>LARGEST_ANGLE)
		{
			findDirection=1;
		}
		if(ReadYawPos()<-LARGEST_ANGLE)
		{
			findDirection=-1;
		}
		if(findDirection==1)	
		{
			aimAngle-=0.3f;	
		}
		else if(findDirection==-1)
		{
			aimAngle+=0.3f;
		}
		//一定时间内寻找不到目标就结束此模式
		aimErrorTime++;
		if(aimErrorTime>200)
		{
			aimErrorFlag=1;
			aimErrorTime=0;
		}
	}
	//如果出现扫描故障，退出扫描模式
	if(aimAngle>LARGEST_ANGLE)
	{
		aimAngle=0;
		aimErrorFlag=1;
	}
	else if(aimAngle<-LARGEST_ANGLE)
	{
		aimAngle=0;
		aimErrorFlag=1;
	}
}


/**
* @brief  按键 按下退出吐球模式
* @param  state：按键触发的次数
		  pmtCnt：prevent mistakenly touching 防误触时间
* @author ACTION
*/
int LetBallOut(void)
{
	uint8_t key_A= GPIO_ReadInputDataBit (GPIOB,GPIO_Pin_4);
	//由高电平（1）到低电平（0）的检测 或者一直是低电平（0）的状态
	if(key_A<lastKey_A||key_A+lastKey_A==0)  
	{
		changeFlag=0;
		lastState=state;
		pmtCnt=0;
	}
	//低电平（0）到由高电平（1）的检测 或者一直是高电平（1）的状态
	else                           
	{
		 changeFlag=1;
	}
	if(changeFlag==1)
	{
		pmtCnt++;
	}
	//防误触延时300ms，次数加1
    if (pmtCnt>=30&&lastState==state)	
	{
		state++;
	}	
	//记录上一次按键的电平		
	lastKey_A=key_A;
		
	//k初始值为0，第一种状态
	if(state%2==0)
	{	
		return 1;		
	}
	//第二种状态
	else
	{
		return 2;
	}
}

/**
* @brief  分球机构的摄像头判断球的颜色
* @param  Ballcolor[2]：摄像头返回的球的颜色
* @author ACTION
*/
int RecognizeBall(void)
{
	if(Ballcolor[2]==Need_ball_collor)
	{
		needBallTime++;
	}				
	if(Ballcolor[2]==No_need_ball_collor)
	{
		noNeedBallTime++;
	}
	else if(Ballcolor[2]==0)
	{
		noBallTime++;
	}
	if(needBallTime>75)
	{
		sureNoBallFlag=0;
		needBallTime=0;
		noNeedBallTime=0;
		noBallTime=0;
		return(1);
	}
	else if(noNeedBallTime>75)
	{
		sureNoBallFlag=0;
		needBallTime=0;
		noNeedBallTime=0;
		noBallTime=0;
		return(2);		
	}
	else if(noBallTime>150)
	{
		needBallTime=0;
		noNeedBallTime=0;
		noBallTime=0;
		//定点时对没球反推进行计数 累计一定次数结束定点模式
		if(roundCnt==8)
		{
			sureNoBallFlag+=1;
		}
		return(2);		
	}
	else 
	{
		return(0);
	}
	
}
