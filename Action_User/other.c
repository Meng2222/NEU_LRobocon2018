#include "other.h"
extern GunneryData gundata;
//====================================================================================
//                                     数据处理
//====================================================================================
float ReternMax(float Num1,float Num2)//返回最大值
{
	return ((Num1)>=(Num2)? (Num1):(Num2));
}
float ReturnMin(float Num3,float Num4)//返回最小值
{
	return ((Num3)<=(Num4)? (Num3):(Num4));
}
float ReturnAbsolute(float Num5)//返回绝对值
{
	return ((Num5)>=0? (Num5):(-Num5));
}
float ReturnLimitValue(float Num6, float NumMax, float NumMin)//返回限幅值
{
    return ((Num6)<(NumMin)?(NumMin):((Num6)>(NumMax)?(NumMax):(Num6)));
}
int ReturnJudgeResult(float Num7, float MaxNum, float MinNum)//返回判断值
{
	return ((Num7)<(MinNum)?(0):((Num7)>(MaxNum)?(0):(1)));
}
float Return_SymmetricRangeValue(float Num8,float Range1)//返回
{
	do
	{
	if((Num8)>(Range1))  {Num8-=(2*Range1);}
	if((Num8)<(-Range1)) {Num8+=(2*Range1);}
	}
	while( (Num8)<(-Range1) || (Num8)>(Range1) );
	return Num8;
}
float Return_NonSymmetricRangeValue(float Num9,float upper_bound,float lower_bound)
{
	do
	{
		if((Num9)>(upper_bound)){Num9-=(upper_bound-lower_bound);}
		if((Num9)<(lower_bound)){Num9+=(upper_bound-lower_bound);}
	}
	while( (Num9)>(upper_bound) || (Num9)<(lower_bound) );
	return Num9;
}
void GetFloat (float Num10, int places)
{
	if(Num10<0){USART_OUT(UART4,(uint8_t*)"%s","-");}
	if(Num10==0)  USART_OUT(UART4,(uint8_t*)"%d  ",0);
	else
	{
		Num10=ReturnAbsolute(Num10);
		USART_OUT(UART4,(uint8_t*)"%d%s",(int)Num10,".");
		while(Num10-1>=0){Num10--;}
		Num10*=(pow(10,places));
		USART_OUT(UART4,(uint8_t*)"%d  ",(int)Num10);
	}
}
//====================================================================================
//                                     激光触发
//====================================================================================
float Avalue;
float Bvalue;
void Laser_data(void)
{
	Avalue=2.5114*fort.laserAValueReceive+51.623;	
	Bvalue=2.4269*fort.laserBValueReceive+371.39;
	/*\
	\*/
}

void Laser_Trigger(void)
{
	//计算激光读数(mm)
	Laser_data();
}
//====================================================================================
//                                   多元走行
//====================================================================================

void Sweep (void)//走形++(输入方向、正方形标志位&参量、圆形标志位&参量)
{

}

int error_flag=0;  //错误标志位
int purpose_mode=0;//0为收球 1为投球 2为避障（利用避障机会投球）
int Sweep_mode=1;  //路径标志位：1增加   0减小
int Shape_mode=1;  //形状标志位1正方   0走圆

void Check_Path (void)//走行路径判断++
{
	if(error_flag==0)
	{
		if(purpose_mode==0)//收球走位(前10-15s)
		{

		}
		if(purpose_mode==1)//投球走位
		{

		}
	}
	if(error_flag==1)//故障处理
	{
		if(purpose_mode==2)
		{

		}
	}
}

void Check_Error(void)//error判断检测++
{

}

void Collision_Process(void)//碰撞处理++
{

}

//====================================================================================
//                          激光扫描（目标检测）【全方位】
//====================================================================================
extern FortType fort;
int t_Breakdown=0;          //定位系统异常计数
int ifBreakdownFlag=0;      //定位系统异常Flag
float d_angle=0;            //角度增量
float Set_FortInitialAngle1;//设定初始角度（未锁定）//对地
float Set_FortAngle1;       //设定航向（扫描）角度（未锁定）//对地
float FortAngle1;           //检测炮台当前角度（未锁定）//对地
int t_laserlock=0;          //激光锁定计数
int laserLock=0;            //激光扫描判断
float target_X;             //计算目标X坐标
float target_Y;             //计算目标Y坐标
float laserValue;           //激光返回值（计算目标坐标）
int TellFortToShoot=0;      //射球标志位（判断shoot()）

void Target_Lock(GunneryData *Gun,PID_Value *pos)
{
	//计算激光读数(mm)
	Laser_data();
	//激光夹角计算
	if(GetWZ()>=250) {t_Breakdown++;}
	if(GetWZ()<250)  {t_Breakdown=0;}
	if(t_Breakdown>=3){ifBreakdownFlag=1;}
	if(!ifBreakdownFlag)//定位系统正常
	{
		if(gundata.BucketNum==0){}
		if(gundata.BucketNum==1){}
		if(gundata.BucketNum==2){}
		if(gundata.BucketNum==3){}
	}
	if(ifBreakdownFlag)//定位系统异常
	{
		//未锁定目标（扫描逻辑）
		if(!laserLock)
		{
			d_angle++;
			switch(Gun->BucketNum)
			{
				case 0:
					if(pos->direction == ACW)Set_FortInitialAngle1=-180;
					if(pos->direction == CW)Set_FortInitialAngle1=-90;
					break;
				
				case 1:
					if(pos->direction == ACW)Set_FortInitialAngle1=-90;
					if(pos->direction == CW)Set_FortInitialAngle1=0;
					break;
				
				case 2:
					if(pos->direction == ACW)Set_FortInitialAngle1=0;
					if(pos->direction == CW)Set_FortInitialAngle1=90;
					break;
				
				case 3:
					if(pos->direction == ACW)Set_FortInitialAngle1=90;
					if(pos->direction == CW)Set_FortInitialAngle1=180;
					break;
			}
			if(GetWZ()<100)//角速度小于100
			{
				if(pos->direction == ACW)
				{
					if(d_angle<90)   {d_angle++;}
					if(d_angle==90)  {d_angle=0;}
				}
				if(pos->direction == CW)
				{
					if(d_angle>-90)  {d_angle--;}
					if(d_angle==-90) {d_angle=0;}
				}
			}
//			if(GetWZ()>100)//角速度大于100
//			{
//				
//			}
				Set_FortAngle1=Set_FortInitialAngle1+d_angle;
				Set_FortAngle1=Return_SymmetricRangeValue(Set_FortAngle1,180.f);
				
				//计算航向电机对车角度
				/*****************/
				//设定航向电机角度
				/*****************\
				 *
				 *
				 *
				 *
				\*****************/
			
				//检测目标（判断逻辑）
			{
				if(ReturnJudgeResult(Avalue,4800,1000)&&ReturnJudgeResult(Bvalue,4800,1000))
				{
					if(ReturnAbsolute(Avalue-Bvalue)<75){t_laserlock++;}
					else t_laserlock=0;
					if(t_laserlock==3)//记录当前炮台角度、车角度、两激光读数
					{
						FortAngle1=Return_SymmetricRangeValue(((-Return_SymmetricRangeValue(fort.yawPosReceive,180.f))+(pos->Angle)),180);
						laserValue=(Avalue+Bvalue)/2;
					}
					if(t_laserlock>=6){t_laserlock=0;laserLock=1;}
				}
			}
		}
		//已锁定目标（读取信息）
		if(laserLock)
		{
			//计算目标坐标
			target_X=Gun->Fort_X;
			target_Y=Gun->Fort_Y;
			
			//投球后laserlock归0
		}
	}
}

//====================================================================================
//                                     发球检测
//====================================================================================
extern PID_Value PID_A;
int ifPushFlag;
Record shooterVel;

void ShooterVel_Record(void)//记录数据
{
    shooterVel.p100=shooterVel.p90;
	shooterVel.p90=shooterVel.p80;
	shooterVel.p80=shooterVel.p70;
	shooterVel.p70=shooterVel.p60;
	shooterVel.p60=shooterVel.p50;
	shooterVel.p50=shooterVel.p40;
	shooterVel.p40=shooterVel.p30;
	shooterVel.p30=shooterVel.p20;
	shooterVel.p20=shooterVel.p10;
	shooterVel.p10=shooterVel.now;
	shooterVel.now=gundata.ShooterVelRec;
}

int ifShootFlag=0;
int ifPushFlag_remain=0;
int t_ifpushflag=0;
void Remain_ifPushFlag(void)//ifPushFlag标志持续250ms
{
	ifPushFlag=PID_A.fire_command;
	
	if(ifPushFlag==1){ifPushFlag_remain=1;}
	if(ifPushFlag_remain==1)
	{
		t_ifpushflag++;
		if(t_ifpushflag<=35){ifPushFlag=1;}
		if(t_ifpushflag>35) {ifPushFlag_remain=0;t_ifpushflag=0;}
	}
}

void Filter(void)//滤波函数
{}
void Data_Processing(void)//数据处理
{
//滤波
//积分
//微分
}

//射击次数
int Bucket_0_ShootNum=0;
int Bucket_1_ShootNum=0;
int Bucket_2_ShootNum=0;
int Bucket_3_ShootNum=0;
//击中次数
int Bucket_0_HitNum=0;
int Bucket_1_HitNum=0;
int Bucket_2_HitNum=0;
int Bucket_3_HitNum=0;

void Shoot_Judge(void)//发球检测
{
	if(ifPushFlag==1)
	{
		if(shooterVel.p100-shooterVel.p90>=6||shooterVel.p100-shooterVel.p80>=6)
		{
			if(shooterVel.p100-shooterVel.now<=6){ifShootFlag=1;}
			else{ifShootFlag=0;}
		}
		else{ifShootFlag=0;}
	}
	else{ifShootFlag=0;}
	
	if(ifShootFlag==1)
	{
		ifPushFlag_remain=0;
		t_ifpushflag=0;
		if(gundata.BucketNum==0){Bucket_0_ShootNum++;}
		if(gundata.BucketNum==1){Bucket_1_ShootNum++;}	
		if(gundata.BucketNum==2){Bucket_2_ShootNum++;}
		if(gundata.BucketNum==3){Bucket_3_ShootNum++;}	
		ifShootFlag=0;
	}
}

//====================================================================================
//                                  投球逻辑/adc（全场）
//====================================================================================
void Bucket_Choose (void)//锁定
{

}
//====================================================================================
//                                 视觉系统+分球机构
//====================================================================================
int ballCalour;// 0为白球;1为黑球
void JudgeBallColour()
{

}
//====================================================================================
//                                   新车激光拟合
//====================================================================================
extern FortType fort;

float laserAvaluereceive;
float laserBvaluereceive; 
float Xvaluereceive;
float Yvaluereceive; 
/*
void GetLaserData (void)
{
	laserAvaluereceive=fort.laserAValueReceive;
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%d%s","A","a","d","c",":",(int)laserAvaluereceive,".");
	while(laserAvaluereceive-1>=0)
	{laserAvaluereceive--;}
	laserAvaluereceive*=10000;
	USART_OUT(UART4,(uint8_t*)"%d  ",(int)laserAvaluereceive);	
	
	laserBvaluereceive=fort.laserBValueReceive;
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%d%s","B","a","d","c",":",(int)laserBvaluereceive,".");
	while(laserBvaluereceive-1>=0)
	{laserBvaluereceive--;}
	laserBvaluereceive*=10000;
	USART_OUT(UART4,(uint8_t*)"%d  ",(int)laserBvaluereceive);
}

void GetPositionValue(PID_Value *pid_out)//串口输出函数
{
	Xvaluereceive=pid_out->X;
	Yvaluereceive=pid_out->Y; 
	
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%d%s","p","o","s","X",":",(int)Xvaluereceive,".");
	while(Xvaluereceive-1>=0)
	{Xvaluereceive--;}
	Xvaluereceive*=10000;
	USART_OUT(UART4,(uint8_t*)"%d  ",(int)Xvaluereceive);
	
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%d%s","p","o","s","Y",":",(int)Yvaluereceive,".");
	while(Yvaluereceive-1>=0)
	{Yvaluereceive--;}
	Yvaluereceive*=10000;
	USART_OUT(UART4,(uint8_t*)"%d  ",(int)Yvaluereceive);
}
*/

void GetLaserData2 (void)
{
	laserAvaluereceive=fort.laserAValueReceive;
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","A","a","d","c",":");	
	GetFloat (laserAvaluereceive, 3);
	
	laserBvaluereceive=fort.laserBValueReceive;
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","B","a","d","c",":");
	GetFloat (laserBvaluereceive, 3);
}

void GetPositionValue2(PID_Value *pid_out)//串口输出函数
{
	Xvaluereceive=pid_out->X;
	Yvaluereceive=pid_out->Y; 
	
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","p","o","s","X",":");
	GetFloat (Xvaluereceive, 3);
	
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","p","o","s","Y",":");
	GetFloat (Yvaluereceive, 3);
}
