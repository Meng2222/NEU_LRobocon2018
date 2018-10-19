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
#include "PushBall.h" 
#define push_mode (2)

//上一个圈的圈数
extern int lastRoundcnt;

//前一个发出去的分球电机的位置
extern int beforePushpos;

//上一个发出去的分球电机的位置
extern int lastPushpos;

//是否推出需要的球 0为未推出 1为推出
extern int ifNeedshoot;

extern int stable_flag;

//分球电机的位置接受数组
int pushBallpos;

extern union push_p push_position;

extern int trouble_flag;

//减少或增加补偿角 -1为减少 1为增加
float addReduceangle;

//计算出一条直线的角度 右手坐标系 ax+by+c=0； 
float CountAngle(float a,float b);

//计算出的补偿角
float adAngle;

//摄像机的接受数组
extern uint8_t Ballcolor[5];

//车到目标点的距离
float carToDointD;

//车与场地中心的连线的夹角 方向为中心指向车
int carToCenterAng;

//上一个计算得到的炮台假角度 以y轴正方向为0度 顺时针到180度 逆时针到-180度
float fakeLastAng=0;

//炮台新假角度与旧假角度的差值
float fakeNewAng;

//是否进入射球区域 1为进入 0为未进入
int pushBallFlag=0;

//顺逆时针方向 1为逆时针 -1为顺时针 
extern int Sn;

//车的走行圈数
extern int round_cnt;

//逆时针的位置数组
int nPoint[4][2];

//顺时针的位置数组
int sPoint[4][2];

//炮台编号 从左下开始 逆时针 依次为1 2 3 4
int pointNum;

//记录该桶是否射过 1为未射过 0为射过
int ifShoot[4]={1,1,1,1};

//运动时炮台应该转到的真实的角度
float goRealAng=0;

/**
* @brief  得到炮台转轮速度
* @param  none
* @author ACTION
*/
float GetRollV(void)
{   	
	if(carToDointD<=4000)
	{
		return((sqrt(19600*pow(carToDointD,2)/((sqrt(3))*carToDointD-800)))/377*3.55);//左下右上\\3.45
    }
	else return(50);	
}

/**
* @brief  将场地分成射球区域与非射球区域并且让炮台指向目标点
* @param  none
* @author ACTION
*/
void GetSendAngle(void)
{  
   
	//将位置数组赋值
	//逆时针//
	//1号//
	nPoint[0][0]=-2200;
	nPoint[0][1]=100;
	//2号//
	nPoint[1][0]=-2200;
	nPoint[1][1]=4500;
	//3号//
	nPoint[2][0]=2200;
	nPoint[2][1]=4500;
	//4号//
	nPoint[3][0]=2200;
	nPoint[3][1]=250;
	//顺时针//
	//1号//
	sPoint[0][0]=-2200;
	sPoint[0][1]=100;
	//2号//
	sPoint[1][0]=-2200;
	sPoint[1][1]=4500;
	//3号//
	sPoint[2][0]=2200;
	sPoint[2][1]=4500;
	//4号//
	sPoint[3][0]=2200;
	sPoint[3][1]=100;
	
	//计算出中心点指向车的角度
	carToCenterAng=CountAngle(GetY()-2300,-GetX());
	//规划射球区域与非射球区域
	//顺时针
	if(Sn==-1)	//顺时针
	{	
		//一号桶区域为 -125~-35
		if(carToCenterAng<=-35&&carToCenterAng>=-125) 
		{		
			//左下
		    pointNum=1;
			
			//射球区域为-120~-55
			if(carToCenterAng<=-55&&carToCenterAng>=-120)
				pushBallFlag=1;
			else pushBallFlag=0;
	
		}else if((carToCenterAng<=180&&carToCenterAng>=145)||(carToCenterAng>=-180&&carToCenterAng<=-125))
		{	
			//左上
			pointNum=2;
			
			//射球区域为-145~-180 150~180
			if((carToCenterAng<=-145)||(carToCenterAng<=180&&carToCenterAng>=150))
			pushBallFlag=1;
			else pushBallFlag=0;
		
		}else if(carToCenterAng<=145&&carToCenterAng>=55)
		{   
			//右上
			pointNum=3;
			
			//射球区域为60~125
		    if(carToCenterAng<=125&&carToCenterAng>=60)
			pushBallFlag=1;
			else pushBallFlag=0;
			
		}else if(carToCenterAng<=55&&carToCenterAng>=-35)
		{	
			//右下
		    pointNum=4;
			
			//射球区域为-30~30
            if(carToCenterAng<=35&&carToCenterAng>=-30)
			 pushBallFlag=1;
			else pushBallFlag=0;
		}

	}else 		//逆时针
	{
		if((carToCenterAng<=-145&&carToCenterAng>=-180)||(carToCenterAng>125&&carToCenterAng<=180))
		{
			//左下
			pointNum=1;	
			
			//射球区域为-150~-180 145~180
			if((carToCenterAng>=145)||(carToCenterAng>=-180&&carToCenterAng<=-150))
				pushBallFlag=1;
			else pushBallFlag=0;
			
		}else if(carToCenterAng<=125&&carToCenterAng>35)
		{ 
			//左上
			pointNum=2;
			
			//射球区域为55~-120
	        if(carToCenterAng>=55&&carToCenterAng<=120)
				pushBallFlag=1;
			else pushBallFlag=0;
		}else if(carToCenterAng<=35&&carToCenterAng>-55)
		{ 
			//右上
			pointNum=3;	
			
			//射球区域为-35~-30
			if(carToCenterAng>=-35&&carToCenterAng<=30)
				pushBallFlag=1;		
			else pushBallFlag=0;
		}else if(carToCenterAng<=-55&&carToCenterAng>-145)
		{
			//右下
			pointNum=4;
			
			//射球区域为-125~-60
            if(carToCenterAng>=-125&&carToCenterAng<=-60)
				pushBallFlag=1;
			else pushBallFlag=0;
		}
		
	}

	if(Sn==1)
	{
		//逆时针
		//计算出车与桶的距离
		carToDointD=GetDis(nPoint[pointNum-1][0],nPoint[pointNum-1][1]);
		
		//计算出炮台包胶轮的转速
		ShooterVelCtrl(GetRollV());
		
		//让炮台转向目标桶
		move_gun(nPoint[pointNum-1][0],nPoint[pointNum-1][1]);
	}else 
	{
		//顺时针
		//计算出车与桶的距离
		carToDointD=GetDis(sPoint[pointNum-1][0],sPoint[pointNum-1][1]);
		
		//计算出炮台包胶轮的转速
		ShooterVelCtrl(GetRollV());  
		
		//让炮台转向目标桶
		move_gun(sPoint[pointNum-1][0],sPoint[pointNum-1][1]);
	}
 

	if(Sn==1)
	{
		//逆时针时 从第一圈切换到第二个圈时 四号桶不射
		if(round_cnt==2&&pointNum==4&&lastRoundcnt==1)
			pushBallFlag=0;
    }else
	{
		//顺时针时 从第一圈切换到第二个圈时 一号桶不射
		if(round_cnt==2&&pointNum==1&&lastRoundcnt==1)
			pushBallFlag=0;
	}

	//如果可以边走边投
	if(stable_flag)
	{
		//如果在射球区域
		if(pushBallFlag) 
		{	
			//如果有需要的球被推出
			if(ifNeedshoot)
			{	
				//将ifShoot中该桶的数置0
				ifShoot[pointNum-1]=0;
				//将ififNeedshoot置0
				ifNeedshoot=0;
			}
			//将pushBallFlag赋值 判断所在区域的桶是否射过球
            pushBallFlag=ifShoot[pointNum-1];			
		}
					
    }
	
	//如果四个桶全部射过球 则将数组全部置为1 重新开始
	if(ifShoot[0]==0&&ifShoot[1]==0&&ifShoot[2]==0&&ifShoot[3]==0&&round_cnt!=5)
	{
		for( int i=0;i<4;i++)
			ifShoot[i]=1;
	}
	
}




/**
* @brief  得到车与四个点的距离
* @param  x：目标点的x坐标
* @param  y：目标点的y坐标
* @author ACTION
*/
float GetDis(float x,float y)
{
	return(sqrt(pow (GetX()-x,2)+pow(GetY()-y,2)));
}
 
/**
* @brief  得到车与四个点的角度与车的速度角度的插值
* @param  angle：车指向桶的角度
* @author ACTION
*/
float GetDifferAngle(float angle)
{
	//车指向桶的角度与车行进方向角度的差值
	float nowdiffer_angle=0;
	
	//计算出车车行进方向的角度
	float car_run_v=make_angle_in_wide(90-atan2(GetSpeedX(),GetSpeedY())/pi*180,-180);
	
	//得到两个角度的差值
	nowdiffer_angle=make_angle_in_wide(angle-car_run_v,-180);
	
	//如果差值>0 则减去补偿角 差值<0 则加上补偿角
	if(nowdiffer_angle>0)
		addReduceangle=-1 ;
	else if(nowdiffer_angle<0)
		addReduceangle=1;
	
	return(nowdiffer_angle);
}
/**
* @brief  计算出车的速度与跟桶的距离得到的补偿角
* @param  differ_angle：车指向桶的角度与车行进方向角度的差值
* @param  x：车与桶之间的距离
* @author ACTION
*/
float GetCompensateAng(float differ_angle,float x)
{
	//计算出车的行进速度
	float Add_V=sqrt(pow(GetSpeedX(),2)+pow(GetSpeedX(),2));
	
	float Pi=3.1415926;
	//计算出小球从距离为x的点射到桶内所需要的时间 运用自由落体公式
	float t=sqrt(2*(sqrt(3)*x-800)/9800);
	
	//计算出小球射入桶内的水平速度
	float x_v=x/t;
	
	//用余弦公式计算出运动时小球射出时需要的的水平速度
	float need_v=sqrt(pow(Add_V,2)+pow(x_v,2)-2*x_v*Add_V*cos(fabs(differ_angle)/180*Pi));
	
	//用余弦公式计算出运动时炮台应该转到的角度与车指向桶的角度的夹角的cos值
	float cos_x=(pow(need_v,2)+pow(x_v,2)-pow(Add_V,2))/(2*x_v*need_v);
	
	//转化为角度后输出
	return(acos(cos_x)/Pi*180);
}
/**
* @brief  将输入角度限幅
* @param  angle：需要限幅的角度
* @author ACTION
* @note   如果需要将角度限制在0~360度 输入point_angle=0  如果需要将角度限制在-180~180度 输入point_angle=-180
*/
float make_angle_in_wide(float angle,float point_angle)
{
	if(angle>360+point_angle)
		angle-=360.0f;
	else if(angle<=0+point_angle)
		angle+=360.0f;
	return(angle);
}
/**
* @brief  计算并得到炮台需要运动到的角
* @param  point_x：目标点x坐标
* @param  point_y：目标点y坐标
* @author ACTION
*/
void move_gun(float point_x,float point_y)
{
	//目标点指向车的角度 
	float pointCarAng=CountAngle(GetY()-point_y,-GetX()+point_x);
	
	//车与目标点的距离
	float carPointDistance=GetDis(point_x,point_y);
	
	//车指向固定点的角度//
	float carPointAng=make_angle_in_wide(pointCarAng+180,-180);
	
	//将目标点指向车的角度转化为以y轴正方向为0度的角度	
	float yPointCarAng=0;
    
    //计算出的假角度 以y轴正方向为0度 真假角度只为处理优劣弧 让炮台每次都已最短的路径转到需要的角度 需要在goRealAng的基础上加上	fakeNewAng-fakeLastAng的值
	float fakeNewAng=0;
	
	//补偿需要乘上的比例系数
	float compensateBili=0;		
	
	float fakeDifferAng;
	//计算出需要补偿的角度
	adAngle=GetCompensateAng(GetDifferAngle(carPointAng),carPointDistance);	
	
	//在不同的圈上对比例系数进行赋值
	if(Sn==1)
	{
		if(round_cnt==2)
			compensateBili=0.1;
		else if(round_cnt==3||round_cnt==4)
			compensateBili=0.05;
		else compensateBili=0.05;
    }else if(Sn==-1)
	{
		if(round_cnt==2)
			compensateBili=0.1;
		else if(round_cnt==3||round_cnt==4)
			compensateBili=0.05;
		else compensateBili=0.05;
	}
	//在目标指向车的角度上加上补偿角 再将角度限幅
	pointCarAng=make_angle_in_wide(pointCarAng+addReduceangle*compensateBili*adAngle,-180);
	
	//将目标点指向车的角度转化为以y轴正方向为0度的角度
	yPointCarAng=make_angle_in_wide(90+pointCarAng,-180);
	
	//计算出新的假角度 并且赋值 以y轴正方向为0度 逆时针到180 顺势针到-180 因为炮台顺时针为增加 逆时针为减小 所以要取负
	fakeNewAng=make_angle_in_wide(-yPointCarAng+GetAngle(),-180);
	
	//计算出新假角度与旧角度之间的差值 再限幅
	fakeDifferAng=make_angle_in_wide(fakeNewAng-fakeLastAng,-180);
	
	//在goRealAng的基础上加上差值来解决优劣弧问题
	goRealAng=goRealAng+fakeDifferAng;//弥补误差//
	
	//对旧假角度进行赋值
	fakeLastAng=fakeNewAng;
	
	//发送角度	
	YawPosCtrl(goRealAng);
}

//是否检测分球电机在目标位置的500范围内
int ifAddPushTime=0;

//分球电机在目标区域外的时间
int outPositionTime=0;

//是否正常推球 1则执行正常推球程序 0则执行分球电机故障处理程序
int normalPush=1;

//分球电机上下摇摆时间
int shaveTime=0;

extern int No_StableTime;

extern int Out_AreaTime;

//故障处理的模式 0 1 2 3 决定判断时间
int shave_mode=0;

//卡球的故障处理
/**
* @brief  卡球的故障处理
* @param  none
		  
* @author ACTION
*/
void PushBallErrorDeal(void)
{
	//如果未进入故障处理
	if(!ifAddPushTime)
	{
		//比较电机返回的实时位置与电机需要转到的目标值是否在500范围内
		if(abs(pushBallpos-push_position.push_pos[1])>500)
			outPositionTime++;
		else if(abs(pushBallpos-push_position.push_pos[1])<500)
		{
			outPositionTime=0;
			normalPush=1;
			shave_mode=0;
		}		
    }
	//当区域外的时间超过设定值
	if(outPositionTime==(int)200/(shave_mode+1))
	{
		
		shave_mode++;
		//不去判断是否在区域内
		ifAddPushTime=1;
		//将设定值变为距离电机当前位置最近的适合位置
		pushBallpos=(push_position.push_pos[1]/16384)*16384;
		//将上一个值赋值
		lastPushpos=pushBallpos;
		normalPush=0;
	}
	//如果进入故障处理
	if(ifAddPushTime)
	{
		shaveTime++;
	}
	if(shaveTime==50)
		pushBallpos+=5461;
	else if(shaveTime==100)
	{
		pushBallpos-=5461;
	}else if(shaveTime==150)
	{
		pushBallpos+=5461;
	}else if(shaveTime==200)
	{
		pushBallpos-=5461;
	}
	else if(shaveTime==250)
	{
		ifAddPushTime=0;
		outPositionTime=0;
		shaveTime=0;
	}
	//识别模式切换
    if(shave_mode>=3)
		shave_mode=3;
}
