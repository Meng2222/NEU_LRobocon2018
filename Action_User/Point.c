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
int BeiShu1=0;
int BeiShu2=0;
extern int Sn;
extern float body_angle;
 float fake_last_angle2=0;
float real_send_angle;
float Last_bodyAngle=0;
int StableFlag=0;
float DistanceA,DistanceB;
float DistanceFirst,DistanceSecond,DistanceThird,DistanceFourth;
int PointDistance;
int Barrel=0;
float AdcAngle=0;
int DiffAB;
int LastDiffAB=0;
int DirDir=1;
float Shoot_v;
extern uint8_t Ballcolor[5];
int t=0;
int Find_Time=0;
extern int PushBallPosition;
int No_BallTime=0;
int WhiteBallFlag=0;
int TonextFlag=0;
int k=2,TIME=0,KEY=0,showchange=0,last=0;
int LeftPosition=0,RightPosition=0,LeftFlag=0,RightFlag=0;
float rollV;
int preNo_BallTime=0,prepareTime=0,prepareOkFlag=0,waitTime=0,anWaitTime,SureNo_BallFlag=0;
int shoot_over=0;
extern int if_shoot[];
int if_shootFlag[4]={1,1,1,1};
extern int roundCnt;
int turnAngle=0,findTime=0,findAngle=0,errorDistance=0;
float perTurnAngle=0;
int errorTime=0,errorLockTime=0;
int Barrelcnt,Snchoose;
int errorLockFlag=0;
extern int deadZone;
float AimAngle=0,diffAB=0;
float vDiff=0;
int aimErrorFlag=0,aimErrorTime=0;;
float beginAngle=0,largeAngle=0;
int needBallTime=0,noNeedBallTime=0,noBallTime=0;
int pushFlag=0;
float addAngle=0;
int addAngleFlag=0;
int lineAddFlag=0;
extern float go_real_send_angle;
int errorFind;
extern int normal_push;
void Aim_Point(float point_x,float point_y)
{
		
		float cartopoint_angle;//车指向固定点的角度//
		float getget_angle;//从bodyangle与y轴正方向的角度//
	    float begain_angle=0;//炮台的起始角度//       	
	    float fake_new_angle;
		body_angle=get_angle2(GetY()-point_y,-GetX()+point_x,(GetY()-point_y)/fabs((GetY()-point_y)),Sn);//得到固定点指向车的角度//
//	    if(fabs(make_angle_in_wide(Last_bodyAngle-body_angle,-180))>178)
//			body_angle=Last_bodyAngle;
//		else Last_bodyAngle=body_angle;
	   // real_send_angle=ReadYawPos();
		cartopoint_angle=make_angle_in_wide(body_angle+180,-180);//将角度限制在-180~180//	
		getget_angle=make_angle_in_wide(90+body_angle,-180);//将角度限制在-180~180//
	    fake_new_angle=make_angle_in_wide(begain_angle-getget_angle+GetAngle(),-180);
	    real_send_angle=real_send_angle+make_angle_in_wide(fake_new_angle-fake_last_angle2,-180);
	    fake_last_angle2=fake_new_angle;
		YawPosCtrl(real_send_angle+AdcAngle);
}

void LightPoint(int Distance)
{
	DiffAB=DistanceA-DistanceB;
	//改变参数
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
		//等激光稳定下来
		if((int)ReadYawPos()>=(int)(real_send_angle+AdcAngle-1)&&(int)ReadYawPos()<=(int)(real_send_angle+AdcAngle+1))
		{
			if(StableFlag==0)
			 {
				 errorLockTime=0;
				 //扫不到的故障处理
				 errorTime++;
				 if(errorTime>=250)
				 {	 				
					 if(LeftFlag==1&&RightFlag==0&&addAngleFlag==0)
					{
						findAngle+=addAngle;
						addAngleFlag=1;
					}
					else if(LeftFlag==0&&RightFlag==1&&addAngleFlag==0)
					{
						findAngle+=addAngle;
						addAngleFlag=1;
					}
				 }	
					if(errorTime>=500)
					{
						shoot_over=1;
						 errorTime=0;
					}
					if(ReadYawPos()>(real_send_angle+findAngle))
					{
						DirDir=1;
					}
					if(ReadYawPos()<(real_send_angle-findAngle))
					{
						DirDir=-1;
					}
					if(DirDir==1)	
					{
						//一开始自动往左边寻找
						AdcAngle-=perTurnAngle;		
					}
					else if(DirDir==-1)
					{
						AdcAngle+=perTurnAngle;
					}	
				if(DistanceA<Distance+errorDistance&&DistanceB<Distance+errorDistance)
				{
					Find_Time++;	
				}	
				else if(DistanceA>Distance+errorDistance&&DistanceB<Distance+errorDistance&&Find_Time>findTime)
				{
					LeftPosition=ReadYawPos();
					LeftFlag=1;
					AdcAngle+=turnAngle;
					DirDir=-1;
				}	
				else if(DistanceB>Distance+errorDistance&&DistanceA<Distance+errorDistance&&Find_Time>findTime)
				{
					RightPosition=ReadYawPos();
					RightFlag=1;
					AdcAngle-=turnAngle;
					DirDir=1;
				}
				if(RightFlag==1&&LeftFlag==1)
				{
					AdcAngle=(LeftPosition+RightPosition)/2-real_send_angle;
					RightFlag=0;
					LeftFlag=0;
					StableFlag=1;
					Find_Time=0;
				}

			}
			 if(StableFlag==1)
			 {
				 //扫到却一直没换桶
				 AimAngle=0;
				 errorTime=0;
				 errorLockTime++;
				 if(errorLockTime>=800)
				 {
					shoot_over=1;
					 StableFlag=0;
					 errorLockTime=0;
				 }
				if(DistanceA>Distance+errorDistance+100&&DistanceB>Distance+errorDistance+100)
				{
					StableFlag=0;
					errorFind+=1;
				}
				else if(DistanceA<Distance-errorDistance-100||DistanceB<Distance-errorDistance-100)
				{
					StableFlag=0;
					errorFind+=1;
				}
				if(addAngleFlag==1)
				{
					findAngle-=addAngle;
					addAngleFlag=0;
				}
				if(errorFind>2)
				{
					errorFind=0;
					shoot_over=1;
				}
			 }
		}
		
		
}

float ShootBall_v(float Distance)
{   
/*
	if(Distance<=5400)
	{
		return((sqrt(19600*pow(Distance,2)/((sqrt(3))*Distance-800)))/377*4.3f);
    }
	else return(50);
*/
	rollV=2.0f*(1E-10)*pow(Distance,3)-3.0f*(1E-6)*pow(Distance,2)+0.0255f*Distance+19.304f;
	if(roundCnt!=5)
	{	
	if(rollV<80&&rollV>=20)
	{		
		return(rollV-3.5f);
    }
	else if(rollV<110&&rollV>=80)
	{
		return(rollV-2.5f);
	}
	else if(rollV<20)
	{
		return(20);
	}
	else return(50);
	}
	else if(roundCnt==5)
	{
		rollV-=0.5f;
		if(rollV<80&&rollV>=20)
		{		
			return(rollV);
		}
		else if(rollV<20)
		{
			return(20);
		}
		else return(50);		
	}
}

void MovePoint(void)
{
	//定位系统得到4个点的距离
	DistanceFirst=get_d(-2400,0);
	DistanceSecond=get_d(-2400,4800);
	DistanceThird=get_d(2400,4800);
	DistanceFourth=get_d(2400,0);
	DistanceA=2.4541f*ReadLaserAValue()+82.744f;
	DistanceB=2.3839f*ReadLaserBValue()+413.37f;	
	PointDistance=(DistanceA+DistanceB)/2;
	diffAB=DistanceA-DistanceB;
	pushFlag=RecognizeBall();
	if(normal_push==0)
	{
		shoot_over=1;
	}
	//从左下角开始寻找
	if(roundCnt!=5)
	{
		//重置
		if(if_shoot[0]==0&&if_shoot[1]==0&&if_shoot[2]==0&&if_shoot[3]==0)
		{
			for(int i=0;i<4;i++)
			{
				if_shoot[i]=1;
			}
		}
		if(if_shootFlag[0]==0&&if_shootFlag[1]==0&&if_shootFlag[2]==0&&if_shootFlag[3]==0)
		{
			for(int i=0;i<4;i++)
			{
				if_shootFlag[i]=1;
			}
		}
		if(if_shoot[0]!=1||if_shootFlag[0]!=1)
		{
			if(if_shoot[1]!=1||if_shootFlag[1]!=1)
			{
				if(if_shoot[2]!=1||if_shootFlag[2]!=1)
				{
					if(if_shoot[3]!=1||if_shootFlag[3]!=1)
					{
						for(int i=0;i<4;i++)
						{
							if_shoot[i]=1;
							if_shootFlag[i]=1;
						}				
					}
				}
			}
		}
		if(Barrel==0)
		{	
			if(if_shoot[0]==1)
			{
				Barrel=1;
//				USART_OUT(UART4,(uint8_t*)"Barrel:1");
			}
			else if(if_shoot[1]==1)
			{
				Barrel=2;
//				USART_OUT(UART4,(uint8_t*)"Barrel:2");
			}
			else if(if_shoot[2]==1)
			{
				Barrel=3;
//				USART_OUT(UART4,(uint8_t*)"Barrel:3");
			}
			else if(if_shoot[3]==1)
			{
				Barrel=4;
//				USART_OUT(UART4,(uint8_t*)"Barrel:4");
			}
		}
		if(Barrel==1)
		{			
			LightPoint(DistanceFirst);
			Aim_Point(-2500,0);
			Shoot_v=ShootBall_v(PointDistance);
			ShooterVelCtrl(Shoot_v);
		//		USART_OUT(UART4,(uint8_t*)"Barrel1:DistanceSet:%d\tPointDistance:%d\r\n",(int)DistanceFirst,(int)PointDistance);
		//		USART_OUT(UART4,(uint8_t*)"Barrel1:rollV_%d\r\n",(int)Shoot_v);

		}
		else if(Barrel==2)
		{
			LightPoint(DistanceSecond);
			Aim_Point(-2300,4900);
			Shoot_v=ShootBall_v(PointDistance);
			ShooterVelCtrl(Shoot_v);
		//		USART_OUT(UART4,(uint8_t*)"Barrel2:DistanceSet:%d\tPointDistance:%d\r\n",(int)DistanceSecond,(int)PointDistance);
		//		USART_OUT(UART4,(uint8_t*)"Barrel2:rollV_%d\r\n",(int)Shoot_v);
		}
		else if(Barrel==3)
		{		
			LightPoint(DistanceThird);
			Aim_Point(2600,4800);
			Shoot_v=ShootBall_v(PointDistance);
			ShooterVelCtrl(Shoot_v);
		//		USART_OUT(UART4,(uint8_t*)"Barrel3:DistanceSet:%d\tPointDistance:%d\r\n",(int)DistanceThird,(int)PointDistance);
		//		USART_OUT(UART4,(uint8_t*)"Barrel3:rollV_%d\r\n",(int)Shoot_v);
		}
		else if(Barrel==4)
		{
			LightPoint(DistanceFourth);
			Aim_Point(2200,-200);
			Shoot_v=ShootBall_v(PointDistance);
			ShooterVelCtrl(Shoot_v);
		//		USART_OUT(UART4,(uint8_t*)"Barrel4:DistanceSet:%d\tPointDistance:%d\r\n",(int)DistanceFourth,(int)PointDistance);
		//		USART_OUT(UART4,(uint8_t*)"Barrel4:rollV_%d\r\n",(int)Shoot_v);			
		}
		else if(Barrel==5)
		{
			Barrel=0;
		}
	}
	else if(roundCnt==5)
	{
		AimBarrel();
		Shoot_v=ShootBall_v(PointDistance);
		ShooterVelCtrl(Shoot_v);
	}
	BarrelChange();
	PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,PushBallPosition);
//	USART_OUT(UART4,(uint8_t*)"errLock%d\terrTime%d\taimerr%d\taimerrFlag%d\t",errorLockTime,errorTime,aimErrorTime,aimErrorFlag);	
//	USART_OUT(UART4,(uint8_t*)"Barrel%d\tAdcAngle%d\tStableFlag%d\tshootv:%d\trealv%d",Barrel,(int)AdcAngle,StableFlag,(int)Shoot_v,(int)ReadShooterVel());
//	USART_OUT( UART4, (uint8_t*)"ReadLaserA:%d  %d\t",(int)ReadLaserAValue(),(int)ReadLaserBValue());
//	USART_OUT(UART4,(uint8_t*)"if_shoot:%d\t%d\t%d\t%d\t",if_shoot[0],if_shoot[1],if_shoot[2],if_shoot[3]);
//	USART_OUT(UART4,(uint8_t*)"if_shootFlag:%d\t%d\t%d\t%d\t",if_shootFlag[0],if_shootFlag[1],if_shootFlag[2],if_shootFlag[3]);
}
void BarrelChange(void)
{
	vDiff=fabs(Shoot_v-ReadShooterVel());
	//已经瞄准
	if(StableFlag==1)
	{
		preNo_BallTime=0;
		prepareTime=0;
	  //分球机构还没准备好
	  if(prepareOkFlag==0)
	  {
		if(pushFlag==1&&vDiff<1)
		{
			PushBallPosition+=32768/2;
			if(roundCnt==5)
			 {
				if(Sn==-1)
				{
					if_shoot[deadZone-1]=0;
					if_shootFlag[deadZone-1]=0;
				}
				else if(Sn==1)
				{
					if_shoot[4-deadZone]=0;
					if_shootFlag[4-deadZone]=0;
				}
			 }
			  else if(roundCnt!=5)
			  {
					if_shoot[Barrel-1]=0;
					if_shootFlag[Barrel-1]=0;
			  }
			WhiteBallFlag=1;
		}
		else if(pushFlag==2)
		{
			PushBallPosition-=32768/2;
		}				
	  }
	  //分球机构:已经准备好了
	  else if(prepareOkFlag==1)
	  {
		  anWaitTime++;
		  if(anWaitTime>=50&&vDiff<2)
		  {
			PushBallPosition+=32768/2;
			  if(roundCnt==5)
			  {
				if(Sn==-1)
				{
					if_shoot[deadZone-1]=0;
					if_shootFlag[deadZone-1]=0;
				}
				else if(Sn==1)
				{
					if_shoot[4-deadZone]=0;
					if_shootFlag[4-deadZone]=0;
				}
			  }
			  else if(roundCnt!=5)
			  {
				if_shoot[Barrel-1]=0;
				if_shootFlag[Barrel-1]=0;	  
			  }
			WhiteBallFlag=1;
			prepareOkFlag=0;
			anWaitTime=0;
//			USART_OUT(UART4,(uint8_t*)"PrepareOK!\r\n");
		  }
	  }
	  //延迟
	  if(WhiteBallFlag==1)
	  {
		  waitTime++;
		  if(waitTime>=50)
		  {
		   TonextFlag=1;
		   WhiteBallFlag=0;
		   waitTime=0;
		   WhiteBallFlag=0;
		  }
	  }
	  //确定没有球
	  if((SureNo_BallFlag>=5&&roundCnt!=3&&roundCnt!=4)||(SureNo_BallFlag>=3&&(roundCnt==3||roundCnt==4)))
	  {
		shoot_over=1;
		StableFlag=0;
		errorLockTime=0;
		SureNo_BallFlag=0;
	  }
	}
	//还没瞄准的准备中
	else if(StableFlag==0)
	{
		if(pushFlag==1)
		{
			SureNo_BallFlag=0;
			preNo_BallTime=0;
			prepareOkFlag=1;
		}
		else if(pushFlag==2)
		{
			SureNo_BallFlag=0;
			PushBallPosition-=32768/2;
			preNo_BallTime=0;
		}
	}
//	USART_OUT(UART4,(uint8_t*)"roundCnt:%d  ",roundCnt);
		//改变射球的桶
		if(TonextFlag==1)
		{
			errorFind=0;
			DirDir=1;
			AdcAngle=0;
			AimAngle=0;
			StableFlag=0;
			TonextFlag=0;
			errorLockTime=0;
			//第五圈 投完走		
			if(roundCnt==5)
			{
				AimAngle=0;
				shoot_over=1;
			/*
				for(int i=0;i<4;i++)
				{
					if_shootFlag[i]=1;
				}
			*/
			}
				//定点时换桶
				else if(roundCnt!=5)
				{
					if(if_shoot[0]!=1||if_shootFlag[0]!=1)
				{
					if(if_shoot[1]!=1||if_shootFlag[1]!=1)
					{
						if(if_shoot[2]!=1||if_shootFlag[2]!=1)
						{
							if(if_shoot[3]!=1||if_shootFlag[3]!=1)
							{
								for(int i=0;i<4;i++)
								{
									if_shoot[i]=1;
									if_shootFlag[i]=1;
								}				
							}
						}
					}
				}
					if(if_shoot[0]==1&&if_shootFlag[0]==1)
					{
						Barrel=1;
//						USART_OUT(UART4,(uint8_t*)"Barrel:1");
					}
					else if(if_shoot[1]==1&&if_shootFlag[1]==1)
					{
						Barrel=2;
//						USART_OUT(UART4,(uint8_t*)"Barrel:2");
					}
					else if(if_shoot[2]==1&&if_shootFlag[2]==1)
					{
						Barrel=3;
//						USART_OUT(UART4,(uint8_t*)"Barrel:3");
					}
					else if(if_shoot[3]==1&&if_shootFlag[3]==1)
					{
						Barrel=4;
//						USART_OUT(UART4,(uint8_t*)"Barrel:4");
					}
				}
//				USART_OUT(UART4,(uint8_t*)"if_shoot:%d\t%d\t%d\t%d\r\n",if_shoot[0],if_shoot[1],if_shoot[2],if_shoot[3]);
//				USART_OUT(UART4,(uint8_t*)"if_shootFlag:%d\t%d\t%d\t%d\r\n",if_shootFlag[0],if_shootFlag[1],if_shootFlag[2],if_shootFlag[3]);
		}

}

void AimBarrel(void)
{
		BeiShu1=real_send_angle/360;
	BeiShu2=real_send_angle-360*BeiShu1;
	
	if(real_send_angle>0)
	{
		if(BeiShu2>180)
			beginAngle=(BeiShu1+1)*360.0f;
		else 
			beginAngle=BeiShu1*360.0f;
	}
	if(real_send_angle<0)
	{
		if(BeiShu2<-180)
			beginAngle=(BeiShu1-1)*360.0f;
		else 
			beginAngle=BeiShu1*360.0f;
	}

	
	if(Sn==1)
	{
		//beginAngle=0;
		largeAngle=15;
	}
	else if(Sn==-1)
	{
		//beginAngle=0;
		largeAngle=15;
	}
	YawPosCtrl(beginAngle+AimAngle);
	if(DistanceA<3500&&DistanceB<3500)
	{
//		if(diffAB<100&&diffAB>-100)
//		{
//			StableFlag=1;
//		}
//		else if(diffAB<-100)
//		{
//			AimAngle+=0.2f;
//		}
//		else if(diffAB>100)
//		{
//			AimAngle-=0.2f;
//		}
		StableFlag=1;
		aimErrorTime=0;
	}
	else if(DistanceA>=3500&&DistanceB<3500)
	{
		AimAngle+=0.3f;
		aimErrorTime=0;
		lineAddFlag+=1;
	}
	else if(DistanceA<3500&&DistanceB>=3500)
	{
		AimAngle-=0.3f;
		aimErrorTime=0;
		lineAddFlag+=1;
	}
	else if(DistanceA>3500&&DistanceB>3500)
	{
		if(ReadYawPos()>largeAngle)
		{
			DirDir=1;
		}
		if(ReadYawPos()<-largeAngle)
		{
			DirDir=-1;
		}
		if(DirDir==1)	
		{
			AimAngle-=0.3f;		///一开始自动往左边寻找
		}
		else if(DirDir==-1)
		{
			AimAngle+=0.3f;
		}
		aimErrorTime++;
		if(aimErrorTime>200)
		{
			aimErrorFlag=1;
			aimErrorTime=0;
		}
	}
	if(AimAngle>largeAngle)
	{
		AimAngle=0;
		aimErrorFlag=1;
//	USART_OUT(UART4,(uint8_t*)"What?\r\n");

	}
	else if(AimAngle<-largeAngle)
	{
		AimAngle=0;
		aimErrorFlag=1;		
	}
//	USART_OUT(UART4,(uint8_t*)"AimAngle:%d\tDistace:%d\t%d\tdiff:%d",(int)AimAngle,(int)DistanceA,(int)DistanceB,(int)diffAB);
//	USART_OUT(UART4,(uint8_t*)"Aimpoint!\r\n");
}
//task.c中用	GPIO_Init_Pins(GPIOB,4,GPIO_Mode_IN); 进行按键初始化
//按下才能改变状态，松开不改变
int Let_BallOut(void)
{
	uint8_t key_A= GPIO_ReadInputDataBit (GPIOB,GPIO_Pin_4);
	if(key_A<last||key_A+last==0)  //由1到0的检测 或者一直是0的状态
	{
		KEY=0;showchange=k;TIME=0;
	}
	else                           //0到1的检测 或者一直是1的状态
	{
		 KEY=1;
	}
	if(KEY==1)
	{
		TIME++;
	}
	//延时300ms，次数记1
       if (TIME>=30&&showchange==k)	
	{
		k++;
	}	
		
	last=key_A;
		
	//k初始值为0
	if(k%2==0)
	{
//		USART_OUT( UART4, (uint8_t*)" Begin	k:%d	key%d\r\n",k,key_A);	
		return 1;		
	}
	//第二种状态
	if(k%2==1)
	{
//		USART_OUT( UART4, (uint8_t*)" We can go!	k:%d	key%d\r\n",k,key_A);	
		return 2;
	}
}

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
		SureNo_BallFlag=0;
		needBallTime=0;
		noNeedBallTime=0;
		noBallTime=0;
		return(1);
	}
	else if(noNeedBallTime>75)
	{
		SureNo_BallFlag=0;
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
		if(roundCnt==8)
		{
			SureNo_BallFlag+=1;
		}
		return(2);		
	}
	else 
	{
		return(0);
	}
	
}
