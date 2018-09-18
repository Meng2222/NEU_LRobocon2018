#include "other.h"
extern GunneryData Gundata;
//====================================================================================
//                                    数据处理
//====================================================================================
//以0为变量后缀地变量军事对车变量，在车坐标系（0~360）下计算
//以1为变量后缀的变量均是对地变量，在车坐标系（-180~+180）下计算
/*************************************************************************************
                                      数字运算
*************************************************************************************/
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

float Return_SymmetricRangeValue(float Num8,float Range1)//返回对称环设定值
{
	do
	{
	if((Num8)>(Range1))  {Num8-=(2*Range1);}
	if((Num8)<(-Range1)) {Num8+=(2*Range1);}
	}
	while( (Num8)<(-Range1) || (Num8)>(Range1) );
	return Num8;
}

float Return_NonSymmetricRangeValue(float Num9,float upper_bound,float lower_bound)//返回非对称环设定值
{
	do
	{
		if((Num9)>(upper_bound)){Num9-=(upper_bound-lower_bound);}
		if((Num9)<(lower_bound)){Num9+=(upper_bound-lower_bound);}
	}
	while( (Num9)>(upper_bound) || (Num9)<(lower_bound) );
	return Num9;
}

float SetToFort_AngleProcessing(float r_veh1,float r_fort0/*实测炮台角度*/,float R_fort0/*读取炮台角度*/,float setfortangle1)//新炮台角度处理（给定目标角度(车坐标系下)，设定炮台角度）
{
	return(Return_SymmetricRangeValue((Return_NonSymmetricRangeValue((r_veh1-setfortangle1),360,0)-Return_NonSymmetricRangeValue(r_fort0,360,0)),180)+R_fort0);
}

float FortToGround_AngleProcessing(float r_veh1,float r_fort0)//新炮台角度处理（给定目标角度(车坐标系下)，设定炮台角度）
{
	return(Return_SymmetricRangeValue((r_veh1+Return_SymmetricRangeValue(-Return_NonSymmetricRangeValue(r_fort0,360,0),180)),180));
}

void GetFloat (float Num10, int places)//蓝牙发送浮点数
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

float ReturnRadian(float ANG)
{
	return (Pi*ANG/180);
}

float ReturnAngle(float ANG)
{
	return (ANG*180/Pi);
}
/*************************************************************************************
                                      时间处理*
*************************************************************************************/
int t_Flag=0;
int t_cnt=0;
void Delay(int countTime)//延迟函数
{
	t_cnt=0;
	while(!t_Flag)//延迟标志位
	{
		if(t_cnt<countTime){t_cnt+=10;}//延迟计数（countTime ms）
		if(t_cnt>=countTime) {t_cnt=0;t_Flag=1;}
	}
	t_cnt=0;
	t_Flag=0;
}

int Timer(void)//计时函数
{
	if(t_Flag==0){t_cnt++;return 0;}
	if(t_Flag==1){t_cnt=0;return t_cnt;}
}
/*************************************************************************************
                                      回归方程
*************************************************************************************/
float ReturnVectorAngle1(float x1,float y1,float x2,float y2)//返回车坐标系下向量角度
{
	if((y2-y1)>0&&(x2-x1)<0) return Return_SymmetricRangeValue(((180*(atan((y2-y1)/(x2-x1)))/Pi)+180-90),180);
	else if((y2-y1)<0&&(x2-x1)<0) return Return_SymmetricRangeValue(((180*(atan((y2-y1)/(x2-x1)))/Pi)-180-90),180);
	else return Return_SymmetricRangeValue(((180*(atan((y2-y1)/(x2-x1)))/Pi)-90),180);
}

float ReturnSigma (float array[2][51],int choose_xy, int num)//求连续num个数之和
{
    float Sigma=0;
	for(int i=1 ; i<=num ; i++)
	{
		Sigma+=array[choose_xy][i];
	}
	return (Sigma);
}

float ReturnProductSum(float array[2][51] , int num)//求连续num个数x*y积之和
{
    float Product=0;
	for(int i=1 ; i<=num ; i++)
	{
		Product+=((array[0][i])*(array[1][i]));
	}
	return (Product);
}

float ReturnQuadraticSum(float array[2][51],int choose_xy, int num)//求连续num个数的平方之和
{
    float QuadraticSum=0;
	for(int i=1 ; i<=num ; i++)
	{
		QuadraticSum+=((array[choose_xy][i])*(array[choose_xy][i]));
	}
	return (QuadraticSum);
}

float ReturnAverage (float Array[2][51] , int Choose_xy , int Num)//求连续num个数平均值
{
    float Average=0;
	Average = (ReturnSigma(Array , Choose_xy , Num)/(Num));
	return (Average);
}

float Return_LinearRegressionEquation_k (float ARRAY[2][51], int NUM)//求回归方程斜率
{
	return (((ReturnProductSum(ARRAY , NUM))-(NUM*(ReturnAverage (ARRAY , 0 , NUM))*(ReturnAverage (ARRAY , 1 , NUM))))/((ReturnQuadraticSum(ARRAY , 0 , NUM))-(NUM*(ReturnAverage (ARRAY , 0 , NUM))*(ReturnAverage (ARRAY , 0 , NUM)))));
}

float Return_LinearRegressionEquation_b (float array[2][51], int num)//求回归方程截距
{
	return ((ReturnAverage (array , 1 , num))-((Return_LinearRegressionEquation_k (array, num))*(ReturnAverage (array , 0 , num))));
}

float Return_LinearRegressionEquation_angle (float array[2][51], int num)//求回归方程方向
{
	
}

void LinearRegressionEquation_3Coordinates(float x1,float y1,float x2,float y2,float x3,float y3)//三点回归方程
{
}
//====================================================================================
//                                     激光触发
//====================================================================================
float Avalue;
float Bvalue;
void Laser_data(void)
{
	Avalue=2.4479*fort.laserAValueReceive+71.215;	
	Bvalue=2.44885*fort.laserBValueReceive+57.925;
}

void Laser_Trigger(void)
{
	
	//计算激光读数(mm)
}
//====================================================================================
//                                   多元走行
//====================================================================================
void Sweep (void)//走形++(输入方向、正方形标志位&参量、圆形标志位&参量)
{}
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
{}
void Collision_Process(void)//碰撞处理++
{}
//====================================================================================
//                          激光扫描（目标检测）【全方位】
//====================================================================================
//	int t_PSBreakdown=0;          //定位系统异常计数
//	int ifPSBreakdownFlag=0;      //定位系统异常Flag

//	float FortAngle0;           //检测炮台当前角度（未锁定）//车
//	float FortAngle1;           //检测炮台当前角度（未锁定）//对地
//	float Set_FortInitialAngle1;//设定初始角度（未锁定）//对地
//	float d_angle=0;            //角度增量
//	float Set_FortAngle1;       //设定航向（扫描）角度（未锁定）//对地

//	int t_laserlock=0;          //激光锁定计数
//	int laserLock=0;            //目标锁定判断

//	float laserValue;           //激光返回值（计算目标坐标）
//	float target_X;             //计算目标X坐标
//	float target_Y;             //计算目标Y坐标

//	int TellFortToShoot=0;      //射球标志位（判断shoot()）

//void Target_Lock(GunneryData *Gun,PID_Value *pos)
//{
//	//计算激光读数(mm)
//	Laser_data();
//	/*激光扫描角计算*/
///*
//	//定位系统爆炸判断
//	if(ReturnAbsolute(GetWZ())>=500) {t_PSBreakdown++;}
//	if(ReturnAbsolute(GetWZ())<500)  {t_PSBreakdown=0;}
//	if(t_PSBreakdown>=5){ifPSBreakdownFlag=1;}校正定位系统后要归0
//*/
//	//未锁定目标（扫描逻辑）
//	if(!laserLock)
//	{
///*
//		//定位系统正常
//		if(!ifPSBreakdownFlag)
//		{
//			//激光角度不变，对正前方进行扫描
//			Set_FortAngle1=0;
//		}
//		//定位系统异常
//		if(ifPSBreakdownFlag)
//		{
//*/
//				d_angle++;
//				switch(Gun->BucketNum)
//				{
//					case 0:
//						if(pos->direction == ACW)Set_FortInitialAngle1=-180;
//						if(pos->direction == CW)Set_FortInitialAngle1=-90;
//						break;
//					
//					case 1:
//						if(pos->direction == ACW)Set_FortInitialAngle1=-90;
//						if(pos->direction == CW)Set_FortInitialAngle1=0;
//						break;
//					
//					case 2:
//						if(pos->direction == ACW)Set_FortInitialAngle1=0;
//						if(pos->direction == CW)Set_FortInitialAngle1=90;
//						break;
//					
//					case 3:
//						if(pos->direction == ACW)Set_FortInitialAngle1=90;
//						if(pos->direction == CW)Set_FortInitialAngle1=180;
//						break;
//				}
//				if(GetWZ()<100)//角速度小于100
//				{
//					if(pos->direction == ACW)
//					{
//						if(d_angle<90)   {d_angle++;}
//						if(d_angle==90)  {d_angle=0;}
//					}
//					if(pos->direction == CW)
//					{
//						if(d_angle>-90)  {d_angle--;}
//						if(d_angle==-90) {d_angle=0;}
//					}
//				}
//				if(GetWZ()>100)//角速度大于100
//				{
//					//角度补偿
//					d_angle=0;
//				}
//				Set_FortAngle1=Set_FortInitialAngle1+d_angle;
//				Set_FortAngle1=Return_SymmetricRangeValue(Set_FortAngle1,180.f);
//					
//					//计算航向电机对车角度
//					/*****************/
//					//设定航向电机角度
//					/*****************\
//					 *
//					 *
//					 *
//					 *
//					\*****************/
///*
//		}
//*/
//		//检测目标（判断逻辑）
//		{
//			if(ReturnJudgeResult(Avalue,4800,1000)&&ReturnJudgeResult(Bvalue,4800,1000))
//			{
//				if(ReturnAbsolute(Avalue-Bvalue)<75){t_laserlock++;}
//				else t_laserlock=0;
//				if(t_laserlock==3)//记录当前炮台角度、车角度、两激光读数
//				{
//					FortAngle1=Return_SymmetricRangeValue(((-Return_SymmetricRangeValue(fort.yawPosReceive,180.f))+(pos->Angle)),180);//炮台对地角度
//					laserValue=(Avalue+Bvalue)/2;
//				}
//				if(t_laserlock>=6){t_laserlock=0;laserLock=1;}
//			}
//		}
//	}
//	//已锁定目标（读取信息）
//	if(laserLock)
//	{
//		//计算目标坐标
//		target_X=Gun->Fort_X;
//		target_Y=Gun->Fort_Y;
//		
//		//投球后laserlock归0
//	}
//}
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
	shooterVel.now=Gundata.ShooterVelRec;
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

//void Filter(void)//滤波函数
//{}
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
		if(Gundata.BucketNum==0){Bucket_0_ShootNum++;}
		if(Gundata.BucketNum==1){Bucket_1_ShootNum++;}	
		if(Gundata.BucketNum==2){Bucket_2_ShootNum++;}
		if(Gundata.BucketNum==3){Bucket_3_ShootNum++;}	
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
float laserAvaluereceive;
float laserBvaluereceive; 
float Xvaluereceive;
float Yvaluereceive; 
void GetLaserData (void)
{
	laserAvaluereceive=fort.laserAValueReceive;
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","A","a","d","c",":");	
	GetFloat (laserAvaluereceive, 3);
	
	laserBvaluereceive=fort.laserBValueReceive;
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","B","a","d","c",":");
	GetFloat (laserBvaluereceive, 3);
}

void GetPositionValue(PID_Value *pid_out)//串口输出函数
{
	Xvaluereceive=pid_out->X;
	Yvaluereceive=pid_out->Y-2400; 
	
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","p","o","s","X",":");
	GetFloat (Xvaluereceive, 3);
	
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","p","o","s","Y",":");
	GetFloat (Yvaluereceive, 3);
}

//====================================================================================
//                                  角度锁定（ok）
//====================================================================================
/*
void SetFortAngle(PID_Value *pos,float set_angle)//
{
	Set_FortAngle1=SetToFort_AngleProcessing(pos->Angle,fort.yawPosReceive,set_angle);
	
	r_fortAngle1=Return_SymmetricRangeValue((pos->Angle+Return_SymmetricRangeValue(-Return_NonSymmetricRangeValue(fort.yawPosReceive,360,0),180)),180);
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","F","A","n","g",":");
	GetFloat (r_fortAngle1, 3);
}
*/
//====================================================================================
//                                  雷达实验
//====================================================================================
float setangle=0;           //

float r_fortAngle1;         //
float r_fortAngle0;         //
float r_standardFortAngle;  //
float Set_FortAngle1;       //设定航向（扫描）角度【对地】（-180，180）

float A_targetx_now=0;      //
float A_targety_now=0;      //
float B_targetx_now=0;      //
float B_targety_now=0;      //
float A_targetx_past=0;     //
float A_targety_past=0;     //
float B_targetx_past=0;     //
float B_targety_past=0;     //
int   ifkeepFlag;           //

float A_targetx;            //
float A_targety;            //
float B_targetx;            //
float B_targety;            //

float post_fortAngle=-60;   //
float FortPost_Flag=1;      //
float t_post=0;             //

float CorrectiveAngle=0;    //

int   abandonFlag=0;        //

float APositionRecord[2][4]={{0},{0}};
float BPositionRecord[2][4]={{0},{0}};
float ATargetPositionRecord[2][51]={{0},{0}};
float BTargetPositionRecord[2][51]={{0},{0}};
//int ai,aj,bi,bj,ap,aq,bp,bq;

float Fort_X;
float Fort_Y;
float A_Laser_X;
float A_Laser_Y;
float B_Laser_X;
float B_Laser_Y;
void DataProcessing (PID_Value *p)
{
		//计算炮台角度
		r_fortAngle0=fort.yawPosReceive+Compensation_Angle;

		//计算炮台坐标
		Fort_X = p->X - (10.5f) * sin(p->Angle * Pi / 180.0);
		Fort_Y = p->Y + (10.5f) * cos(p->Angle * Pi / 180.0);	
		//计算激光坐标
		r_fortAngle1=FortToGround_AngleProcessing(p->Angle,r_fortAngle0);
		A_Laser_X=Fort_X-40.f*sin(Return_SymmetricRangeValue((r_fortAngle1+90.f),180)*Pi/180)-23.32*sin(Return_SymmetricRangeValue(r_fortAngle1,180)*Pi/180.f);
		A_Laser_Y=Fort_Y+40.f*cos(Return_SymmetricRangeValue((r_fortAngle1+90.f),180)*Pi/180)+23.32*cos(Return_SymmetricRangeValue(r_fortAngle1,180)*Pi/180.f);
		B_Laser_X=Fort_X-40.f*sin(Return_SymmetricRangeValue((r_fortAngle1-90.f),180)*Pi/180)-23.32*sin(Return_SymmetricRangeValue(r_fortAngle1,180)*Pi/180.f);
		B_Laser_Y=Fort_Y+40.f*cos(Return_SymmetricRangeValue((r_fortAngle1-90.f),180)*Pi/180)+23.32*cos(Return_SymmetricRangeValue(r_fortAngle1,180)*Pi/180.f);
}

void SetFortAngle(PID_Value *pos,float set_angle)//
{
		DataProcessing (pos);
		Set_FortAngle1=SetToFort_AngleProcessing(pos->Angle,r_fortAngle0,fort.yawPosReceive,set_angle);
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","P","A","n","g",":");
			GetFloat (pos->Angle, 3);
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","F","A","n","g",":");
			GetFloat (fort.yawPosReceive, 3);
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","R","A","n","g",":");
			GetFloat (r_fortAngle0, 3);
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","s","A","n","g",":");
			GetFloat (set_angle, 3);
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","S","A","n","g",":");
			GetFloat (Set_FortAngle1, 3);

		r_fortAngle1=FortToGround_AngleProcessing(pos->Angle,r_fortAngle0);
//		r_fortAngle1=Return_SymmetricRangeValue((pos->Angle+Return_SymmetricRangeValue(-Return_NonSymmetricRangeValue(fort.yawPosReceive,360,0),180)),180);
		USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","F","A","n","g",":");
		GetFloat (r_fortAngle1, 3);

}
int ifAFirstOFOK=0;
int ifBFirstOFOK=0;
int cntNumA=0;
int cntNumB=0;
void FirstOrder_Filter(PID_Value *p)//滤波函数
{
	//过滤车身周围噪点
	if(sqrt((A_targetx-(A_Laser_X))*(A_targetx-(A_Laser_X))+(A_targety-(A_Laser_Y)+2400)*(A_targety-(A_Laser_Y)+2400))>=400)
	{
		//滤去超范围点
		if(sqrt(A_targetx*A_targetx+A_targety*A_targety)>2000&&sqrt(A_targetx*A_targetx+A_targety*A_targety)<4500)
		{
			A_targetx_past=A_targetx_now;
			A_targety_past=A_targety_now;
			A_targetx_now=A_targetx;
			A_targety_now=A_targety;
			//滤去孤独点
			if(sqrt((A_targetx_past-A_targetx_now)*(A_targetx_past-A_targetx_now)+(A_targety_past-A_targety_now)*(A_targety_past-A_targety_now))<=150||\
			   sqrt((A_targetx_past-APositionRecord[0][2])*(A_targetx_past-APositionRecord[0][2])+(A_targety_past-APositionRecord[1][2])*(A_targety_past-APositionRecord[1][2]))<=150)
			{
				APositionRecord[0][0]=A_targetx_past;
				APositionRecord[1][0]=A_targety_past;
				for(int ap=0 ; ap<=1 ; ap++)
				{
					for(int aq=3 ; aq>=1 ; aq--)
					{
						APositionRecord[ap][aq]=APositionRecord[ap][aq-1];
					}
				}
				ifAFirstOFOK=1;
				/*收数*/
				{
					/*
						USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","A","T","L","X",":");
						GetFloat (A_targetx_past, 3);
						USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","A","T","L","Y",":");
						GetFloat (A_targety_past, 3);
					*/
				}
			}
		}
	}
	
	//过滤车身周围噪点
	if(sqrt((B_targetx-(B_Laser_X))*(B_targetx-(B_Laser_X))+(B_targety-(B_Laser_Y)+2400)*(B_targety-(B_Laser_Y)+2400))>=400)
	{
		//滤去超范围点
		if(sqrt(B_targetx*B_targetx+B_targety*B_targety)>2000&&sqrt(B_targetx*B_targetx+B_targety*B_targety)<4500)
		{
			B_targetx_past=B_targetx_now;
			B_targety_past=B_targety_now;
			B_targetx_now=B_targetx;
			B_targety_now=B_targety;
			//滤去孤独点
			if(sqrt((B_targetx_past-B_targetx_now)*(B_targetx_past-B_targetx_now)+(B_targety_past-B_targety_now)*(B_targety_past-B_targety_now))<=150||\
			   sqrt((B_targetx_past-BPositionRecord[0][2])*(B_targetx_past-BPositionRecord[0][2])+(B_targety_past-BPositionRecord[1][2])*(B_targety_past-BPositionRecord[1][2]))<=150)
			{
				BPositionRecord[0][0]=B_targetx_past;
				BPositionRecord[1][0]=B_targety_past;
				for(int bp=0 ; bp<=1 ; bp++)
				{
					for(int bq=3 ; bq>=1 ; bq--)
					{
						BPositionRecord[bp][bq]=BPositionRecord[bp][bq-1];
					}
				}
				ifBFirstOFOK=1;
				/*收数*/
				{
					/*
						USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","B","T","L","X",":");
						GetFloat (B_targetx_past, 3);
						USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","B","T","L","Y",":");
						GetFloat (B_targety_past, 3);
					*/
				}
			}
		}
	}

}

void CheckLine(void)
{
	
}

void SecondOrder_Filter(void)//滤波函数
{
	if(ifAFirstOFOK==1)
	{
			/*二阶滤波*/
			if(sqrt((APositionRecord[0][2]-APositionRecord[0][1])*(APositionRecord[0][2]-APositionRecord[0][1])+(APositionRecord[1][2]-APositionRecord[1][1])*(APositionRecord[1][2]-APositionRecord[1][1]))<=150||\
			   sqrt((APositionRecord[0][3]-APositionRecord[0][2])*(APositionRecord[0][3]-APositionRecord[0][2])+(APositionRecord[1][3]-APositionRecord[1][2])*(APositionRecord[1][3]-APositionRecord[1][2]))<=150)
			{
					cntNumA++;
					ATargetPositionRecord[0][0]=APositionRecord[0][2];
					ATargetPositionRecord[1][0]=APositionRecord[1][2];
					for(int ai=0 ; ai<=1 ; ai++)
					{
						for(int aj=50; aj>=1 ; aj--)
						{
							ATargetPositionRecord[ai][aj]=ATargetPositionRecord[ai][aj-1];
						}
					}
					if(cntNumA>=2&&sqrt((ATargetPositionRecord[0][2]-ATargetPositionRecord[0][1])*(ATargetPositionRecord[0][2]-ATargetPositionRecord[0][1])+\
					  (ATargetPositionRecord[1][2]-ATargetPositionRecord[1][1])*(ATargetPositionRecord[1][2]-ATargetPositionRecord[1][1]))>200)
					{
						if(cntNumA>=6)
						/*__*\
					   |*|__|*|数组长度大于50会发0【待解决】
						\*__*/
						{
							/*解析数据*/
							{
								
							}
							/*收数*/
							{
									for(int i=cntNumA;i>=1;i--)
									{
									USART_OUT(UART4,(uint8_t*)"\r\n%s%s%s","A","X",":");
									GetFloat (ATargetPositionRecord[0][i], 3);
									USART_OUT(UART4,(uint8_t*)"%s%s%s","A","Y",":");
									GetFloat (ATargetPositionRecord[1][i], 3);
									}
							}
						}
						cntNumA=0;
						//ATargetPositionRecord数组清零
						memset(ATargetPositionRecord,0,sizeof(ATargetPositionRecord));
					}
			}
		ifAFirstOFOK=0;
	}
	if(ifBFirstOFOK==1)
	{
			/*二阶滤波*/
			if(sqrt((BPositionRecord[0][2]-BPositionRecord[0][1])*(BPositionRecord[0][2]-BPositionRecord[0][1])+(BPositionRecord[1][2]-BPositionRecord[1][1])*(BPositionRecord[1][2]-BPositionRecord[1][1]))<=150||\
			   sqrt((BPositionRecord[0][3]-BPositionRecord[0][2])*(BPositionRecord[0][3]-BPositionRecord[0][2])+(BPositionRecord[1][3]-BPositionRecord[1][2])*(BPositionRecord[1][3]-BPositionRecord[1][2]))<=150)
			{
					cntNumB++;
					BTargetPositionRecord[0][0]=BPositionRecord[0][2];
					BTargetPositionRecord[1][0]=BPositionRecord[1][2];
					for(int bi=0 ; bi<=1 ; bi++)
					{
						for(int bj=50; bj>=1 ; bj--)
						{
							BTargetPositionRecord[bi][bj]=BTargetPositionRecord[bi][bj-1];
						}
					}
					if(cntNumB>=2&&sqrt((BTargetPositionRecord[0][2]-BTargetPositionRecord[0][1])*(BTargetPositionRecord[0][2]-BTargetPositionRecord[0][1])+\
					  (BTargetPositionRecord[1][2]-BTargetPositionRecord[1][1])*(BTargetPositionRecord[1][2]-BTargetPositionRecord[1][1]))>200)
					{
						if(cntNumB>=6)
						/*__*\
					   |*|__|*|数组长度大于50会发0【待解决】
						\*__*/
						{
							/*解析数据*/
							{
								
							}
							/*收数*/
							{
									for(int i=cntNumB;i>=1;i--)
									{
									USART_OUT(UART4,(uint8_t*)"\r\n%s%s%s","B","X",":");
									GetFloat (BTargetPositionRecord[0][i], 3);
									USART_OUT(UART4,(uint8_t*)"%s%s%s","B","Y",":");
									GetFloat (BTargetPositionRecord[1][i], 3);
									}
							}
						}
						cntNumB=0;
						//ATargetPositionRecord数组清零
						memset(BTargetPositionRecord,0,sizeof(BTargetPositionRecord));
					}
			}
		ifBFirstOFOK=0;
	}
}

void Power_On_Self_Test(PID_Value *pos)//【加电自检】
{
	DataProcessing (pos);
	t_post++;
	Laser_data();
	/*炮台——航向电机——加电自检*/
		if(t_post<=240){post_fortAngle+=0.5;}
		if(t_post>240) {post_fortAngle=0;}
		Set_FortAngle1=SetToFort_AngleProcessing(pos->Angle,r_fortAngle0,fort.yawPosReceive,post_fortAngle);//
					USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","S","A","n","g",":");
					GetFloat (Set_FortAngle1, 3);
		
		{
			/*计算目标坐标*/
//			r_fortAngle1=Return_SymmetricRangeValue((pos->Angle+Return_SymmetricRangeValue(-Return_NonSymmetricRangeValue(fort.yawPosReceive,360,0),180)),180);
			r_fortAngle1=FortToGround_AngleProcessing(pos->Angle,r_fortAngle0);/**///炮台对地角度（车坐标系）
			r_standardFortAngle=Return_SymmetricRangeValue((r_fortAngle1+90.f),180.f);   //炮台对地角度（标准坐标系）
			A_targetx=Avalue*(cos(Pi*r_standardFortAngle/180.f))+A_Laser_X;
			A_targety=Avalue*(sin(Pi*r_standardFortAngle/180.f))+A_Laser_Y-2400.f;
			B_targetx=Bvalue*(cos(Pi*r_standardFortAngle/180.f))+B_Laser_X;
			B_targety=Bvalue*(sin(Pi*r_standardFortAngle/180.f))+B_Laser_Y-2400.f;
					USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%s","A","T","a","r","X",":");
					GetFloat (A_targetx, 3);
					USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%s","A","T","a","r","Y",":");
					GetFloat (A_targety, 3);
					USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%s","B","T","a","r","X",":");
					GetFloat (B_targetx, 3);
					USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%s","B","T","a","r","Y",":");
					GetFloat (B_targety, 3);
		}
		FirstOrder_Filter(pos);
	/*炮台——航向电机——角度初始化【0度校正】*/
		
	/*炮台——射球电机——加电自检*/
		
	/*滚轮——分球电机——加电自检*/
		
	/*辊子——左右电机——加电自检*/
		
	/*POST加电自检Finish*/
}

void RadarCorrection(PID_Value *pos)//雷达校正系统
{
	DataProcessing (pos);
	//航向电机—连续扫描指令
	Laser_data();
	setangle+=0.5;
	if(setangle>360){setangle-=360;}
	Set_FortAngle1=SetToFort_AngleProcessing(pos->Angle,r_fortAngle0,fort.yawPosReceive,setangle);//
//			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","S","A","n","g",":");
//			GetFloat (Set_FortAngle1, 3);
	//计算角度
//	r_fortAngle1=Return_SymmetricRangeValue((pos->Angle+Return_SymmetricRangeValue(-Return_NonSymmetricRangeValue(fort.yawPosReceive,360,0),180)),180);
	r_fortAngle1=FortToGround_AngleProcessing(pos->Angle,r_fortAngle0);/**///炮台对地角度（车坐标系）
			//返回角度
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","P","A","n","g",":");
			GetFloat (pos->Angle, 3);
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s","F","A","n","g",":");
			GetFloat (r_fortAngle1, 3);
	//计算坐标
	r_standardFortAngle=Return_SymmetricRangeValue((r_fortAngle1+90.f),180.f);//炮台对地角度（标准坐标系）
	A_targetx=Avalue*(cos(Pi*r_standardFortAngle/180.f))+A_Laser_X;/////////|————改为两激光坐标
	A_targety=Avalue*(sin(Pi*r_standardFortAngle/180.f))+A_Laser_Y-2400.f;////|————改为两激光坐标
	B_targetx=Bvalue*(cos(Pi*r_standardFortAngle/180.f))+B_Laser_X;/////////|————改为两激光坐标
	B_targety=Bvalue*(sin(Pi*r_standardFortAngle/180.f))+B_Laser_Y-2400.f;////|————改为两激光坐标
			//返回坐标
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%s","A","T","a","r","X",":");
			GetFloat (A_targetx, 3);
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%s","A","T","a","r","Y",":");
			GetFloat (A_targety, 3);
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%s","B","T","a","r","X",":");
			GetFloat (B_targetx, 3);
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%s","B","T","a","r","Y",":");
			GetFloat (B_targety, 3);
	//数据分析
	FirstOrder_Filter(pos);
	SecondOrder_Filter();
}

int possibility=0;
void Check_Target(void)
{
	Laser_data();	
}














void Entertainment(void)
{
/*
USART_OUT(UART4,(uint8_t*)"%s\r\n","		  ______           ___________   _______________   _____________     ___________      ____     ___");
USART_OUT(UART4,(uint8_t*)"%s\r\n","		 /      \         /           \ /               \ /             \   /           \    /    \   /   \");
USART_OUT(UART4,(uint8_t*)"%s\r\n","		/   /\   \       /    ________/ \_____     _____/ \____     ____/  /    _____    \  |      \  |    |");
USART_OUT(UART4,(uint8_t*)"%s\r\n","	   /   /  \   \     |    /                |   |            |   |      |    /     \    | |       \ |    |");
USART_OUT(UART4,(uint8_t*)"%s\r\n","	  /   /____\   \    |   |                 |   |            |   |      |   |       |   | |    \   \|    |");
USART_OUT(UART4,(uint8_t*)"%s\r\n","	 /   ________   \   |   |                 |   |            |   |      |   |       |   | |    |\   \    |");
USART_OUT(UART4,(uint8_t*)"%s\r\n","	/   /        \   \  |    \________        |   |        ____|   |____  |    \_____/    | |    | \       |");
USART_OUT(UART4,(uint8_t*)"%s\r\n","   /   /          \   \  \            \       |   |       /             \  \             /  |    |  \      |");
USART_OUT(UART4,(uint8_t*)"%s\r\n","  |___/            \___|  \___________/        \_/        \_____________/   \___________/    \___/   \____/");
*/
}

/*
		  ______          ___________   _______________   _____________     ___________      ____     ___
		 /      \        /           \ /               \ /             \   /           \    /    \   /   \  
		/   /\   \      /    ________/ \_____     _____/ \____     ____/  /    _____    \  |      \  |    |
	   /   /  \   \    |    /                |   |            |   |      |    /     \    | |       \ |    |  
	  /   /____\   \   |   |                 |   |            |   |      |   |       |   | |    \   \|    |
	 /   ________   \  |   |                 |   |            |   |      |   |       |   | |    |\   \    |
	|   /        \   \ |    \________        |   |        ____|   |____  |    \_____/    | |    | \       | 
    |   |        |   |  \            \       |   |       /             \  \             /  |    |  \      |
    |___|        |___|   \___________/        \_/        \_____________/   \___________/    \___/   \____/  
*/
/*
		  ______           ___________   _______________   _____________     ___________      ____     ___
		 /      \         /           \ /               \ /             \   /           \    /    \   /   \  
		/   /\   \       /    ________/ \_____     _____/ \____     ____/  /    _____    \  |      \  |    |
	   /   /  \   \     |    /                |   |            |   |      |    /     \    | |       \ |    |  
	  /   /____\   \    |   |                 |   |            |   |      |   |       |   | |    \   \|    |
	 /   ________   \   |   |                 |   |            |   |      |   |       |   | |    |\   \    |
	/   /        \   \  |    \________        |   |        ____|   |____  |    \_____/    | |    | \       | 
   /   /          \   \  \            \       |   |       /             \  \             /  |    |  \      |
  |___/            \___|  \___________/        \_/        \_____________/   \___________/    \___/   \____/  
*/
