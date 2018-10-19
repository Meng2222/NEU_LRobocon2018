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
#include "Walk.h" 
#include "Point.h" 
#include "PushBall.h" 
#define adcStandard1 150
#define adcStandard2 300
#define PAI 3.14159
#define BOTTOM_LIMIT 180
//走圆的PID参数
float rate=0.05;
float KP=220; 
//(最大偏离区域)
float buff=800;

//计圈数相关参数
int record=0;
int record1=0;
float lastX=0;
float lastY=0;
float lastAng=0;
float lastX1=0;
float lastY1=0;
float lastAng1=0;
int last_round_cnt;
int R_switch=0;
int r_switch=0;
//
int deadZone=0;
int go_and_shoot=0;
uint8_t R_switch1=0;
uint8_t r_switch1=0;
int roundCnt=0;
uint8_t changeComing=0;

//换半径的判决线
int change_angle=0;

//(可变半径)
float radiusA=400;

//(半径增减的标志位)
int decFlg=0;

//判断是否启动走投
int stableFlg=0;

//改变半径的时候为防止突变带来的不稳定而设置的缓冲，分别是半径的缓冲计数单位和速度计数单位
float rbuffCnt=0;
float vbuffCnt=0;

//判断是否启动定点射球
int pointErrDeal=0;


//循环走形的次数及标志位，-1代表定点启动模式，1
int againCnt=0;

//定位系统
pos_t action;
/*
左右轮脉冲值
*/
float Rr=0;
float Ll=0;
		
extern float go_real_send_angle;
extern float run_r;
extern float run_R;
extern float run_V;
extern int clockFlg;
extern int shoot_over;

//没球标志位
extern int8_t no_ball;
int troubleFlg=0;
int8_t speedAdd=1;

//换半径的提前量
int changeBuff=0;
float body_angle=0;	

//在第一圈和第二圈被撞时的	车与圆心的连线的角度（用来判断是否换半径）
float errposAng=0;

//上一时刻车与圆心的连线的角度（用来判断是否换半径）
float earlyAng=0;

//现在时刻车与圆心的连线的角度（用来判断是否换半径）
float nowAng=0;

//故障处理记数
int troubleCnt=0;

//开始循环走形标志位
int runAgain=0;
extern int SureNo_BallFlag;

//扫边的时候到达特定区域的标志位
int zoneArrive=0;
extern int aimErrorFlag;
/**
* @name 	    systerm_change
* @brief  		转换坐标系：（所有走形都基于该坐标系以下称走形坐标系）
				坐标：向前为-x，向右为 + y；
				角度：初始位置时机器人正前方为0度，顺时针为正，逆时针为负
				
* @param        
* @retval
*/
void systerm_change(void)
{
	action.angle=-GetAngle();
    action.x=-GetY();
    action.y=GetX();
	
}




/**
* @name 	    AdcFlag
* @brief  		用激光选择顺时针还是逆时针
* @param        ReadLaserAValue()和ReadLaserBValue()
* @retval
*/
int AdcFlag(void)//启动ADC，第一次挡设置顺逆时针，第二次挡设置启动半径
{
		//右边激光值和左边激光值
		static int adc_num1,adc_num2,AdcFLAG=0;
		
		//激光被挡住的时间的计数
		static int adc_cnt=0;
	do
	{
			//航向转90度，防止被对面激光触发
			YawPosCtrl(90);
		
			//给包胶轮预热
			ShooterVelCtrl(50);
		
			//读取两个激光的值
			adc_num1=ReadLaserAValue();
			adc_num2=ReadLaserBValue();
	
	//在激光被挡住一段时间后在进行判断，因为激光被挡住时不是突变的，所以给一个时间延迟让激光的值稳定下来再进行判断
	do
{
     	adc_num1=ReadLaserAValue();
		adc_num2=ReadLaserBValue();  
		if(adc_num1 < 500||adc_num2 < 500)
		adc_cnt++ ;	
		else adc_cnt=0;

}while(adc_cnt < 500); 
	
	//当左激光被挡则设置为顺时针	
	if(adc_num1>adcStandard1&&adc_num1 < adcStandard2&&adc_num2>adcStandard2)
	{
		 AdcFLAG=-1;
		clockFlg=-1;
	}
	
	//当右激光被挡则设置为逆时针
	if(adc_num2>adcStandard1&&adc_num2 < adcStandard2&&adc_num1>adcStandard2)
	{
			AdcFLAG=1;
			clockFlg=1;
	}
		
		
	
	}while(AdcFLAG == 0);		
	
	return AdcFLAG;
}


/**
* @name 	    start_mode
* @brief  		用按键判断启动模式
* @param        GPIO_ReadInputDataBit (GPIOE,GPIO_Pin_1)
* @retval
*/
void start_mode(void)
  {
		//加一个按键来增加一种启动模式
		uint8_t key_B= GPIO_ReadInputDataBit (GPIOE,GPIO_Pin_1);
	  
		//如果检测到按键被按下则从直接从第六圈走形开始跑（扫一圈半直接进入第八圈定点）
		if(key_B == 1)
		{
			roundCnt=6;
			runAgain=-1;
		}
		
		//如果检测到没检测到按键按下则进入正常模式从一圈走行开始跑
		if(key_B == 0)
			roundCnt=0;
		
}


/**
* @name 	   	exchange
* @brief  		将给定的速度（m/s）转换成脉冲每秒
* @param        v-设定的速度(m/s)
* @retval       pulse-给电机的脉冲量（脉冲/s）
*/
int exchange(float v)//
{
	int pulse=0;
	
	//电机转一圈的脉冲数为32768
	pulse=(int)(v * 32768 / (PAI * WHEEL_DIAMETER * 0.01f));
	return pulse;
}
/**
* @name 	   	err
* @brief  		计算偏差量，将目前机器人到设定直线的距离偏差转化成角度偏差（距离PID转化成角度PID）
* @param        distance-目前机器人到设定直线的距离偏差
				traAng-当前机器人的角度（定位系统返回的角度）
				buff-最大偏离距离，超过这个距离就垂直走回
				rate-距离偏差转化成补偿角度的比例参数
* @retval       d_angle-通过距离偏差计算出的实际应该转向的角度
*/
float err(float distance,float traAng)//距离偏差转化辅助角度函数
{
		float d_angle;
		//
		//在正常坐标系下，上为y正，右为x正
		//在直线设定直线下方
		if(distance < 0.f)
		{
			
			//车头方向（走形坐标系）在0~90时
			if(traAng < 90.f && traAng >= 0.f)
			{
				if(distance >= -buff)
				d_angle = traAng  +  fabs(distance * rate);
				if(distance < -buff)
				d_angle = traAng + 90.f;
			}
			//90~180度
			if((traAng > 90.f)&&(traAng <= 180.f))
			{
				if(distance >= -buff)
				d_angle = traAng-fabs(distance * rate);
			    if(distance <- buff)
				d_angle = traAng - 90.f;
			}			
			
			//在0~-90度
			if(traAng < 0.f && traAng > -90.f)
			{
				if(distance >= -buff)
				d_angle = traAng  +  fabs(distance * rate);
			    if(distance < -buff)
				d_angle = traAng  +  90.f;
			}
			
			//在-90~-180度下方
			if((traAng < -90.f) && (traAng > -180.f))
			{
				if(distance >= -buff)
				{
					d_angle = traAng-fabs(distance * rate);
				
				//越过-180边界
				if(d_angle < -180.f)
					d_angle = d_angle  +  360.f;
			    }
				if(distance < -buff)
					d_angle = traAng - 90.f  +  360.f;
		    }
			
			//边界情况
			//90度的位置的右边
			if(traAng == 90.f)
			{
				if(distance >= -buff)
				d_angle = 90.f - fabs(distance * rate);
				if(distance < -buff)
				d_angle = 0.f;
			}
			
			//-90度的方向的右边
			if(traAng == -90.f)
			{
				if(distance >= -buff)
				d_angle = - 90.f  +  fabs(distance * rate);
				if(distance < -buff)
				d_angle = 0.f;	
			}

		}
			if(distance > 0.f)
			{
				
				//0~90度
				if(traAng < 90.f && traAng >= 0.f)
				{
					if(distance <= buff)
				     d_angle = traAng - (distance * rate);
				    if(distance > buff)
					 d_angle = traAng - 90.f;
				}
				
				//90~180度
				if(traAng > 90.f && traAng <= 180.f)
				{
					if(distance < buff)
					{	
					d_angle = traAng  +  distance * rate;
					if(d_angle > buff)
					
					//计算的结果会超出180度
					d_angle = d_angle - 360.f;
				    }
					
					if(distance > buff)
					d_angle = traAng  +  90.f - 360.f;
				}
				
				//-90~0度
				if(traAng < 0 && traAng > -90.f)
				{
					if(distance <= buff)
					d_angle = traAng - distance * rate;
				    if(distance > buff)
						d_angle = traAng- 90.f;
				}
				
				//-90~-180
				if(traAng > -180.f && traAng < -90.f)
				{
					if(distance <= buff)
					d_angle = traAng  +  distance * rate;
				    if(distance > buff)
						d_angle = traAng  +  90.f;
				}
				
				//边界情况
				if(traAng == 90.f)
				{
					
					//90度左边
					if(distance < buff)
					{
						d_angle = 90.f  +  (distance * rate);
					}
					if(distance > buff)
					{
						d_angle = 180.f;
					}
				}
				
				//-90左边
				if(traAng == -90.f)
				{		if(distance <= buff)
						d_angle = -90.f - distance * rate;
			            if(distance > buff)
						d_angle = 180.f;
				}	
			}	
			
			if(distance == 0.f)
				d_angle = traAng;
			return d_angle;
			
		}	
				

/**
* @name 	   	rrun_to
* @brief  		使机器人在行进过程中调整朝向特定角度（扫死角圈）
* @param        beta-设定的角度
				Vm—行进的基础速度
* @retval       
*/
void rrun_to(float beta,float Vm)
{
	
	//角度偏差量
	float b_err = 0.f;
	
	//角度偏差量转化出来的两轮电机脉冲
	float b_Uk = 0.f;
	float straight_KP = 90.f;
	
	/*
	右轮基础速度应该给的脉冲量
	左轮基础速度应该给的脉冲量
	*/
	 Rr = exchange(Vm);
	 Ll = exchange(Vm);
	
	//PID算出来的脉冲量需要一个限幅防止机器人两轮差速过大发生自旋
	float Max = 0.f;
	extern int deadZone;
	
//如果不是在走向第一个死角
if(deadZone != -1)
{
	buff = 1000;
	rate = 0.005f;
	straight_KP=60;
}

//如果在走向第一个死角时刚好处于由圆形走形换成正方形走形的交界点，所以需要给一个大的提前量，使机器人平稳的切到正方形的第一条边
if(runAgain == 1 && deadZone == 1)
{
	buff = 2600;
	rate = 0.001f;
	straight_KP = 80;
}
	
	//当前角度与设定值的偏差
	b_err = beta - action.angle;
	
	//偏差超过180，优弧转劣弧
	if(b_err > 180)
		b_err = b_err - 360;
	if(b_err < -180)
		b_err = b_err  +  360;
	
	//P调节
	b_Uk = b_err*straight_KP;
	 Max = Vm * 5000;
	
	//限幅，不至于自旋
	if(b_Uk > Max)
		b_Uk = Max;
	if(b_Uk < -Max)
		b_Uk = -Max;
	
	//计算实际应该给多少脉冲
	Rr = (Rr - b_Uk * 32768 / 4096);
	Ll = -(Ll  +  b_Uk * 32768 / 4096);	
	
	//限速，不管怎样两轮速度不得超过2米每秒
	if(Rr >= exchange(2))
		Rr = exchange(2);
	if(Ll <= -exchange(2))
		Ll = -exchange(2);
	
	//把脉冲给到电机上
	VelCrl(CAN1, 02,Rr);
	VelCrl(CAN1, 01,Ll);
	
	//把这两个参数恢复，不影响其他圆的走形
	rate = 0.08f;
	buff = 800;
}
	
		
		
		

/**
* @name 	   	BoundaryShoot
* @brief  		判定机器人在扫边过程中是否能进行定点投球
* @param        whichline-扫的是哪条边
				
* @retval      point_shootflag-是否定点标志位
*/
int BoundaryShoot(int whichline)
{
	extern int if_shoot[4];
	extern uint8_t Ballcolor[5];
	int8_t carStop=0;
	int8_t point_shootflag=0;
switch (clockFlg)
{
	//如果为逆时针
	case 1: 
		switch (whichline)
		{
			//在扫地一条边时
			case 1:
				
				//正常坐标系下（前 + y右 + x），如果y轴速度小于设定值并且面前这个桶需要打并且机器人行驶到规定区域内，一旦定点激光扫不到超过一定时间就取消定点扫描
				if(fabs(GetSpeedY())<=BOTTOM_LIMIT&&if_shoot[3]==1&&action.y>-200&&action.y < 1000&&(aimErrorFlag == 0))//&&!(action.angle<-150||action.angle>150)
				{
					if(Ballcolor[2]==Need_ball_collor)
						point_shootflag=1;
				}
				break;
			
			case 2:
				
				//正常坐标系下（前 + y右 + x），如果x轴速度小于设定值并且面前这个桶需要打并且机器人行驶到规定区域内，一旦定点激光扫不到超过一定时间就取消定点扫描
				if(fabs(GetSpeedX())<=BOTTOM_LIMIT&&if_shoot[2]==1&&action.x<-2000&&action.x>-3500&&(Ballcolor[2]==Need_ball_collor)&&(aimErrorFlag == 0))//&&!carStop&&fabs(action.angle-90)<30)
				point_shootflag=1;
				break;
			
			case 3:
				
				//正常坐标系下（前 + y右 + x），如果y轴速度小于设定值并且面前这个桶需要打并且机器人行驶到规定区域内，一旦定点激光扫不到超过一定时间就取消定点扫描
				if(fabs(GetSpeedY())<=BOTTOM_LIMIT&&if_shoot[1]==1&&action.y < 500&&action.y>-1200&&(Ballcolor[2]==Need_ball_collor)&&(aimErrorFlag == 0))//&&!carStop&&fabs(action.angle)<30)
				point_shootflag=1;
				break;
			
			case 4:
				
				//正常坐标系下（前 + y右 + x），如果x轴速度小于设定值并且面前这个桶需要打并且机器人行驶到规定区域内，一旦定点激光扫不到超过一定时间就取消定点扫描
				if(fabs(GetSpeedX())<=BOTTOM_LIMIT&&if_shoot[0]==1&&action.x>-2800&&action.x<-1200&&(Ballcolor[2]==Need_ball_collor)&&(aimErrorFlag == 0))//&&!carStop&&fabs(action.angle + 90)<30)
				point_shootflag=1;
				break;
		}
		break;

	case -1:
		switch(whichline)
		{
			case 1:
				
				//正常坐标系下（前 + y右 + x），如果y轴速度小于设定值并且面前这个桶需要打并且机器人行驶到规定区域内，一旦定点激光扫不到超过一定时间就取消定点扫描
				if(fabs(GetSpeedY())<BOTTOM_LIMIT&&if_shoot[0]==1&&action.y < 500&&action.y>-1500&&(Ballcolor[2]==Need_ball_collor)&&(aimErrorFlag == 0)&&!carStop&&!(action.angle<-150||action.angle>150))
					point_shootflag=1;
				break;
			case 2:
				
				//正常坐标系下（前 + y右 + x），如果x轴速度小于设定值并且面前这个桶需要打并且机器人行驶到规定区域内，一旦定点激光扫不到超过一定时间就取消定点扫描
				if(fabs(GetSpeedX())<BOTTOM_LIMIT&&if_shoot[1]==1&&action.x<-1500&&action.x>-3500&&(Ballcolor[2]==Need_ball_collor)&&(aimErrorFlag == 0)&&!carStop&&fabs(action.angle + 90)<30)
				point_shootflag=1;
				break;
			case 3:
				
				//正常坐标系下（前 + y右 + x），如果y轴速度小于设定值并且面前这个桶需要打并且机器人行驶到规定区域内，一旦定点激光扫不到超过一定时间就取消定点扫描
				if(fabs(GetSpeedY())<BOTTOM_LIMIT&&if_shoot[2]==1&&action.y>-1000&&action.y < 1200&&(Ballcolor[2]==Need_ball_collor)&&(aimErrorFlag == 0)&&!carStop&&fabs(action.angle)<30)
				point_shootflag=1;
				break;
			case 4:
				
				//正常坐标系下（前 + y右 + x），如果x轴速度小于设定值并且面前这个桶需要打并且机器人行驶到规定区域内，一旦定点激光扫不到超过一定时间就取消定点扫描
				if(fabs(GetSpeedX())<BOTTOM_LIMIT&&if_shoot[3]==1&&action.x>-1500&&action.x>1200&&(Ballcolor[2]==Need_ball_collor)&&(aimErrorFlag == 0)&&!carStop&&fabs(action.angle-90)<30)
				point_shootflag=1;
				break;		
		}
		break;
	}
	
	//一旦有任何取消定点模式的情况发生，不进行定点
	if(shoot_over == 1)
	{
		point_shootflag=0;
		pointErrDeal=0;
	}
	return point_shootflag;
}


/**
* @name 	   	line
* @brief  		机器人扫边和死角的正方形走形
* @param        pointx-死角的x坐标（走形坐标系）
				pointy-死角的y坐标（走形坐标系）
				zoneOpt-死角的编号逆时针时从右下角开始为1，按逆时针的方向1,2,3,4；顺时针从左下角开始为1
* @retval      point_shootflag-是否定点标志位
*/
void line(float piontx,float pionty,int zoneOpt)
{
	
	//距离的偏差量
	float zoneErrx = 0;
	
	//根据偏差经过P调节给出的脉冲值
	float errValuex = 0;
	
	//p调节参数
	float zoneKpx = 80;
	
	//角度偏差量
	float zoneAngErr = 0;
	
	//用距离偏差算出来的应该朝向的角度
	float runAng = 0;
	
	//将角度偏差转化成两轮的脉冲值
	float zone_angle_value = 0;
	
	//P调节值
	float zone_angle_kp = 800;
	
	//在死角时机器人应该转向的角度
	float escape_ang = 0;
	
	//停下来定点用激光扫描的时间
	extern int aimErrorTime;
	switch(clockFlg)
	{
		case 1:
			switch (zoneOpt)
			{
				case 1:
				escape_ang = 0;
			if(zoneArrive == 0)
				{
					//如果启动了定点模式就不执行以下语句
					if(pointErrDeal == 0)
					{		
						//如果机器人姿态调整到位（角度，距离），就进行距离PID冲向目标点
						if(fabs(piontx-action.x)<100&&(fabs(action.angle-90)<=5))
							{
								zoneErrx=action.y-pionty;
								errValuex=zoneErrx*zoneKpx;
								//限幅
								if(errValuex>exchange(1.5))
									errValuex=exchange(1.5);
								VelCrl(CAN1, 02,-errValuex);
								VelCrl(CAN1, 01,errValuex);	
							}else
								{
									//靠向目标直线
									runAng=err(piontx-action.x,90);
									rrun_to(runAng,1.3);
								}
					}
					if(BoundaryShoot(1)==1)
						pointErrDeal=1;
			
				}
				//到达目标死角区域，开始转向
			if(fabs(action.y-pionty)<350)
				zoneArrive=1;
			if(zoneArrive == 1)
			{
				zoneAngErr=escape_ang-action.angle;
				if(zoneAngErr>180)
					zoneAngErr=zoneAngErr-360;
				if(zoneAngErr<-180)
					zoneAngErr=zoneAngErr + 360;
				zone_angle_value=zoneAngErr*zone_angle_kp;
				VelCrl(CAN1, 02,0);
				VelCrl(CAN1, 01,-zone_angle_value);
			
			}
		
			//机器人姿态达到设定标准后进行下一个动作
			if(fabs(action.angle)<=30&&zoneArrive == 1)
				{
					//相关标志位和参数清零
					deadZone++;
					zoneArrive=0;
					aimErrorFlag=0;
					aimErrorTime=0;
				}
					break;
			case 2:
				escape_ang=-90;
			if(zoneArrive == 0)
				{
					if(pointErrDeal == 0)
						{
						if(pionty-action.y < 100&&(fabs(action.angle)<=5))
							{
								zoneErrx=piontx-action.x;
								errValuex=zoneErrx*zoneKpx;
								if(errValuex>exchange(1.5))
									errValuex=exchange(1.5);
								VelCrl(CAN1, 02,-errValuex);
								VelCrl(CAN1, 01,errValuex);	
							}else
								{
									runAng=err(action.y-pionty,0);
									rrun_to(runAng,1.5);
								}
						}
					if(BoundaryShoot(2)==1)
						pointErrDeal=1;
				}
			if(fabs(action.x-piontx)<350)
				zoneArrive=1;
			if(zoneArrive == 1)
		{
			zoneAngErr=escape_ang-action.angle;
			if(zoneAngErr>180)
				zoneAngErr=zoneAngErr-360;
			if(zoneAngErr<-180)
				zoneAngErr=zoneAngErr + 360;
			zone_angle_value=zoneAngErr*zone_angle_kp;
			VelCrl(CAN1, 02,0);
			VelCrl(CAN1, 01,-zone_angle_value);
		}
		
		if(fabs(action.angle + 90)<=30&&zoneArrive == 1)
		{
			deadZone++;
			zoneArrive=0;
			aimErrorFlag=0;
			 aimErrorTime=0;
		}break;
				case 3:
					escape_ang=180;
				if(zoneArrive == 0)
				{
					if(pointErrDeal == 0)
						{
							if(fabs(piontx-action.x)<100&&(fabs(action.angle + 90)<=5))
								{
									zoneErrx=pionty-action.y;
									errValuex=zoneErrx*zoneKpx;
									if(errValuex>exchange(1.5))
										errValuex=exchange(1.5);
									VelCrl(CAN1, 02,-errValuex);//右
									VelCrl(CAN1, 01,errValuex);//左
								}else
									{
										runAng=err(piontx-action.x,-90);
										rrun_to(runAng,1.5);
									}
						}
							if(BoundaryShoot(3)==1)
								pointErrDeal=1;
			
				}
				if(fabs(action.y-pionty)<350)
					zoneArrive=1;
				if(zoneArrive == 1)
				{
					zoneAngErr=escape_ang-action.angle;
					if(zoneAngErr>180)
						zoneAngErr=zoneAngErr-360;
					if(zoneAngErr<-180)
						zoneAngErr=zoneAngErr + 360;
					zone_angle_value=zoneAngErr*zone_angle_kp;//负值
					VelCrl(CAN1, 02,0);
					VelCrl(CAN1, 01,-zone_angle_value);
				}
		
		
				if(((action.angle>=-179&&action.angle<=-150)||(action.angle <= 180&&action.angle>150))&&zoneArrive == 1)
				{
					deadZone++;
					zoneArrive=0;
					aimErrorFlag=0;
					aimErrorTime=0;
				}
					break;
				case 4:
					escape_ang=90;
					if(zoneArrive == 0)
						{
							if(pointErrDeal == 0)
								{
									if(fabs(piontx-action.x)<100&&((action.angle<=-179&&action.angle>-175)||(action.angle < 180&&action.angle>175)))
										{
											zoneErrx=action.x-piontx;
											errValuex=zoneErrx*zoneKpx;
											if(errValuex>exchange(1.5))
												errValuex=exchange(1.5);
											VelCrl(CAN1, 02,-errValuex);//右
											VelCrl(CAN1, 01,errValuex);//左  
										}else
											{
												runAng=err(action.y-pionty,180);
												rrun_to(runAng,1.5);
											}
								}
							if(BoundaryShoot(4)==1)
							pointErrDeal=1;
			
						}
					if(fabs(action.x-piontx)<350)
						zoneArrive=1;
					if(zoneArrive == 1)
						{
							zoneAngErr=escape_ang-action.angle;
							if(zoneAngErr>180)
								zoneAngErr=zoneAngErr-360;
							if(zoneAngErr<-180)
								zoneAngErr=zoneAngErr + 360;
							zone_angle_value=zoneAngErr*zone_angle_kp;//负值
							VelCrl(CAN1, 02,0);
							VelCrl(CAN1, 01,-zone_angle_value);
						}
					if(fabs(action.angle-90)<30&&zoneArrive == 1)
					{
						deadZone=1;
						zoneArrive=0;
						aimErrorFlag=0;
						aimErrorTime=0;
					}break;
			
	} break;
		case -1:
			switch (zoneOpt)
			{
				case 1:
				escape_ang=0;
				if(zoneArrive == 0)
					{
						if(pointErrDeal == 0)
							{
								if(fabs(piontx-action.x)<150&&(fabs(action.angle + 90)<=5))
								{
									zoneErrx=pionty-action.y;
									errValuex=zoneErrx*zoneKpx;
									if(errValuex>exchange(1.5))
										errValuex=exchange(1.5);
									VelCrl(CAN1, 02,-errValuex);
									VelCrl(CAN1, 01,errValuex);
								}else
									{
										runAng=err(piontx-action.x,-90);
										rrun_to(runAng,1.5);
									}
					}		
						if(BoundaryShoot(1)==1)
								pointErrDeal=1;
						
						
					}
				if(fabs(action.y-pionty)<350)
					zoneArrive=1;
				if(zoneArrive == 1)
				{
					zoneAngErr=escape_ang-action.angle;
					if(zoneAngErr>180)
						zoneAngErr=zoneAngErr-360;
					if(zoneAngErr<-180)
						zoneAngErr=zoneAngErr + 360;
					zone_angle_value=zoneAngErr*zone_angle_kp;//正值
					if(zone_angle_value < 18000)
						zone_angle_value=18000;
					VelCrl(CAN1, 02,-zone_angle_value);
					VelCrl(CAN1, 01,0);
				}
				
				if(fabs(action.angle)<=45&&zoneArrive == 1)
				{
					deadZone ++;
					zoneArrive=0;
					aimErrorFlag=0;
					 aimErrorTime=0;
				}break;
				case 2:
							escape_ang=90;
				if(zoneArrive == 0)
					{
						if(pointErrDeal == 0)
							{	
								if(action.y-pionty < 150&&(fabs(action.angle)<=2))
									{
										zoneErrx=piontx-action.x;
										errValuex=zoneErrx*zoneKpx;
										if(errValuex>exchange(1.5))
											errValuex=exchange(1.5);
										VelCrl(CAN1, 02,-errValuex);
										VelCrl(CAN1, 01,errValuex);
									}else
										{
											runAng=err(action.y-pionty,0);						
											rrun_to(runAng,1.5);
										}
							}
						if(BoundaryShoot(2)==1)
							pointErrDeal=1;
						
						
					}
					if(fabs(action.x-piontx)<350)
						zoneArrive=1;
				if(zoneArrive == 1)
					{
						zoneAngErr=escape_ang-action.angle;
						if(zoneAngErr>180)
							zoneAngErr=zoneAngErr-360;
						if(zoneAngErr<-180)
							zoneAngErr=zoneAngErr + 360;
						zone_angle_value=zoneAngErr*zone_angle_kp;//正值
						if(zone_angle_value < 18000)
							zone_angle_value=18000;
						VelCrl(CAN1, 02,-zone_angle_value);
						VelCrl(CAN1, 01,0);
					}
				if(fabs(90-action.angle)<=45&&zoneArrive == 1)
					{
						deadZone++ ;
						zoneArrive=0;
						aimErrorFlag=0;
						aimErrorTime=0;
					}
					break;
				case 3:
						escape_ang=180;
					if(zoneArrive == 0)
						{
							if(pointErrDeal == 0)
							{
								if(fabs(piontx-action.x)<100&&(fabs(action.angle-90)<=2))
								{
									zoneErrx=action.y-pionty;
									errValuex=zoneErrx*zoneKpx;
									if(errValuex>exchange(1.5))
										errValuex=exchange(1.5);
									VelCrl(CAN1, 02,-errValuex);
									VelCrl(CAN1, 01,errValuex);
								}else
								{
									runAng=err(piontx-action.x,90);
									rrun_to(runAng,1.5);
								}
								if(BoundaryShoot(3)==1)
									pointErrDeal=1;
							}		
						
						}
					if(fabs(action.y-pionty)<350)
						zoneArrive=1;
					if(zoneArrive == 1)
					{
						zoneAngErr=escape_ang-action.angle;//正值
						if(zoneAngErr>180)
							zoneAngErr=zoneAngErr-360;
						if(zoneAngErr<-180)
							zoneAngErr=zoneAngErr + 360;
						zone_angle_value=zoneAngErr*zone_angle_kp;//正值
						if(zone_angle_value < 18000)
							zone_angle_value=18000;
						VelCrl(CAN1, 02,-zone_angle_value);
						VelCrl(CAN1, 01,0);
					}
									
					if(((action.angle>=-179&&action.angle<-135)||(action.angle < 180&&action.angle>135))&&zoneArrive == 1)
					{
						deadZone++;
						zoneArrive=0;
						aimErrorFlag=0;
						 aimErrorTime=0;
					}
					break;
				case 4:
					escape_ang=-90;
					if(zoneArrive == 0)
					{
						if(pointErrDeal == 0)
						{
							if(fabs(piontx-action.x)<100&&((action.angle<=-179&&action.angle>-175)||(action.angle < 180&&action.angle>175)))
								{
									zoneErrx=action.x-piontx;
									errValuex=zoneErrx*zoneKpx;
									if(errValuex>exchange(1.5))
										errValuex=exchange(1.5);
									VelCrl(CAN1, 02,-errValuex);
									VelCrl(CAN1, 01,errValuex);
								}else
								{
									runAng=err(action.y-pionty,180);
									rrun_to(runAng,1.5);
								}
							if(BoundaryShoot(4)==1)
								pointErrDeal=1;
						}
					}
						if(fabs(action.x-piontx)<350)
							zoneArrive=1;
					
				
					
					if(zoneArrive == 1)
					{
						zoneAngErr=escape_ang-action.angle;
						if(zoneAngErr>180)
							zoneAngErr=zoneAngErr-360;
						if(zoneAngErr<-180)
							zoneAngErr=zoneAngErr + 360;
						zone_angle_value=zoneAngErr*zone_angle_kp;
						if(zone_angle_value < 18000)
							zone_angle_value=18000;
						VelCrl(CAN1, 02,-zone_angle_value);
						VelCrl(CAN1, 01,0);
					}
					
					
					if(fabs(action.angle + 90)<=45&&zoneArrive == 1)
					{
						deadZone++;
						zoneArrive=0;
						aimErrorFlag=0;
						aimErrorTime=0;
					}
			
					break;
			}
			break;
	
	}
}


/**
* @name 	   	runTo
* @brief  		使机器人在行进过程中调整朝向特定角度（圆圈）
* @param        beta-设定的角度
				Vm—行进的基础速度
* @retval       
*/
void runTo(float beta,float Vm)//由当前角度pao向beta
{
	float B_err=0;
	float B_Uk=0;
	
	//米每秒转化成脉冲每秒
	float R= exchange(Vm);
	float L= exchange(Vm);
	float max=0;
	float D_value=0;
	float Deta_err=0;
	float last_err=0;
	
	//当前角度与设定值的偏差
	B_err=beta-action.angle;
	Deta_err=B_err-last_err;
	D_value=Deta_err*20;
	
	//偏差超过180，优弧转劣弧
	if(B_err>180)
		B_err=B_err-360;
	if(B_err<-180)
		B_err=B_err + 360;
	B_Uk=B_err*KP;
	 max=Vm*5000;
	//限幅，不至于反转
	if(B_Uk>max)
		B_Uk=max;
	if(B_Uk<-max)
		B_Uk=-max;
	R=(R-B_Uk*32768/4096);
	L=-(L + B_Uk*32768/4096);
	if(pointErrDeal == 0)
	{
		VelCrl(CAN1, 02,R);
		VelCrl(CAN1, 01,L);
	}
	if(pointErrDeal == 1)
	{
		VelCrl(CAN1, 02,0);
		VelCrl(CAN1, 01,0);
		MovePoint();
	}
	if(roundCnt == 5)//&&(largest_cnt%3 == 2)
	{
	if(clockFlg == 1)
	{	
		if(deadZone == 1)
			line(-300,2300,1);
		  if(deadZone == 2)
			line(-4600,2100,2);
		  if(deadZone == 3)
			line(-4450,-2250,3);
		  if(deadZone == 4)
			line(-200,-2100,4);
		  if(deadZone == 5)
			deadZone=1;
	  }
	if(clockFlg==-1)
	{
		  if(deadZone == 4)
			line(-300,2000,4);
		  if(deadZone == 3)
			line(-4450,2200,3);
		  if(deadZone == 2)
			line(-4500,-2100,2);
		  if(deadZone == 1)
			line(-300,-2200,1);
		  if(deadZone == 5)
			deadZone=1;
	}
	}else deadZone=0;
}
		
		
		
		

/**
* @name 	   	angGen
* @brief  		输出给定直线的方向角  //         ax + by + c=0,y=(-a/b)*x + c0;k=-a/b=(action.y-y)/(action.x-x)----a=action.y-y,  ,b=-(action.x-x)
* @param       a b-直线参数ax+by=0;
正常坐标系下n=1为斜率不为零朝向上，-1为斜率不为零朝下，2为斜率为零且为x轴正向，-2为斜率为零且x轴福相
* @retval       输出给定直线的方向角
*/
float angGen(float a,float b,int n)
{	
	//直线给定角度
	float ANGLE=0;
	if(n == 1)//朝上
	{
		if(b == 0)
			ANGLE=90;
		if(b!=0)
		{
			if(-a/b>0)
				
			//范围在90~180
			ANGLE=180-(atan(-a/b))*180/PAI;
			if(-a/b < 0)
				
			//范围在0~90度	
			ANGLE=-atan(-a/b)*180/PAI;			
		}
		
	}
	//朝下
	if(n == -1)
	{
		if(b == 0)
			ANGLE=-90;
		if(b != 0)
		{
			if(-a/b > 0)
			
			//范围在0~-90度
			ANGLE=-atan(-a/b) * 180 / PAI;
			if(-a/b < 0)
				
			//范围在-90~-180度
			ANGLE = -180-(atan(-a / b)) * 180 / PAI;
		}
	}
	
	//Y=0，x轴正向
	if(n == 2)
	{
		ANGLE=180;
	}
	
	//y=0，x轴负向，初始方向
	if(n==-2)
	{
		ANGLE=0;
	}
	
	return ANGLE;
}



/**
* @name 	   	a_genforRadius
* @brief  		算车到圆心的角度（0~180,-180~0）
* @param       a b-直线参数ax+by=0;
正常坐标系下n=1为斜率不为零朝向上，-1为斜率不为零朝下，2为斜率为零且为x轴正向，-2为斜率为零且x轴福相
* @retval       输出给定直线的方向角
*/
float a_genforRadius(float a,float b,int n)
{	
	if(action.y>0)
		n=1;
	if(action.y < 0)
		n=-1;
	if(action.y == 0&&action.x>-2000)
		n=2;
	if(action.y == 0&&action.x<-2000)
		n=-2;
	float ANGLE=0;//直线给定角度
	if(n == 1)
	{
		if(b == 0)
			ANGLE=90;
		if(b!=0)
		{
			if(-a/b>0)//K>0且向上atan>0
			ANGLE=180-(atan(-a/b))*180/PAI;//范围在90~180
			if(-a/b < 0)//atan < 0
			ANGLE=-atan(-a/b)*180/PAI;//范围在0~90度				
		}
		
	}
	if(n==-1)
	{
		if(b == 0)
			ANGLE=-90;
		if(b!=0)
		{
			if(-a/b>0)
			ANGLE=-atan(-a/b)*180/PAI;//范围在0~-90度
			if(-a/b < 0)
			ANGLE=-180-(atan(-a/b))*180/PAI;//范围在-90~-180度
		}
	}
	if(n == 2)//Y=0，x轴正向
	{
		ANGLE=180;
	}
	if(n==-2)//y=0，x轴负向，初始方向
	{
		ANGLE=0;
	}
	
	return ANGLE;
}









/**
* @name 	   Walkline
* @brief  		 圆形走形入口函数
* @param       	corex：设定圆心x坐标，
				corey: 设定圆心y坐标
				radium：设定圆的半径
				V_loop: 设定绕圆半径
				clockFlg：顺逆时针
* @retval
*/
void Walkline( float corex,float corey,float Radium,float V_loop,int clockFlg )//正常走行,输入圆心坐标，半径，速度，方向
{
	//限速
	if(V_loop <= 1.5)
		V_loop=1.5;
	if(V_loop>3.5)
		V_loop=3.5;
	float mid=0;
    float index=1;
	corex=mid;
	corex=-corey;
	corey=mid;
	
	//距离圆心的偏差
	float distance_err=0;
	
	//距离圆心的距离
	float DISTANCE=0;
	float tangent=0;
	float fake_tan=0;
	DISTANCE=sqrt((action.x-corex)*(action.x-corex) + (action.y-corey)*(action.y-corey));
	
	//偏离圆周的距离
	distance_err=DISTANCE-Radium;
	if (fabs(distance_err)>500)
		index=0.02;
	if(fabs(distance_err)<50&&fabs(distance_err)>100)
		index=(300-fabs(distance_err))/400;
	if(distance_err < 100)
		index=1;
	
	//逆时针
	if(clockFlg == 1)
	{
		if(action.y>corey)
		
		//输出给定圆的切线方向
		tangent=angGen(corey-action.y,action.x-corex,1)-90;
		if(action.y < corey)
		{
			//输出给定直线的方向角
			tangent=angGen(corey-action.y,action.x-corex,-1)-90;
		    
			//超过-180之后
			if(tangent<-180)
			tangent=360 + tangent;
		}
		if(action.y == corey&&action.x==(corex + Radium))
			tangent=90;
		if(action.y == corey&&action.x==(corex-Radium))
			tangent=-90;
		
		if(action.y>corey)
			
		//距离转化角度函数
		fake_tan=err(distance_err,tangent);
		if(action.y < corey)
		
		//距离转化角度函数
		fake_tan=err(-distance_err,tangent);
		
		//特殊情况给出角度
		if(action.y == corey&&action.x>corex)
		{
			if(distance_err>0)
			{
				//90度右边
				if(distance_err < buff)
				fake_tan=90-fabs(distance_err*rate);
				if(distance_err>buff)
				fake_tan=0;	
			}
			
			//90度左边
			if(distance_err < 0)
			{	
				
				//90度左边
				if(fabs(distance_err)<buff)
				fake_tan=90 + fabs(distance_err*rate);
				
				//90度左边
				if(fabs(distance_err)>buff)
				fake_tan=180;
			}	
		}
		if(action.y == corey&&action.x < corex)
		{
			if(distance_err>0)
			{
				
				//-90度左边
				if(distance_err < buff)
				fake_tan=-90-fabs(distance_err*rate);
				if(distance_err>buff)
				fake_tan=180;
			}
			if(distance_err < 0)
			{
				
				//90度左边
				if(fabs(distance_err)<buff)
				fake_tan=-90 + fabs(distance_err*rate);
				if(fabs(distance_err)>buff)
				fake_tan=0;
			}
		}
		
		//由当前角度转向beta
		runTo(fake_tan,V_loop);
		
	}
	
	
	
	
	
	
	
	if(clockFlg==-1)
	{
		if(action.y>corey)
		{
			
			//输出给定圆的切线方向
			tangent=angGen(corey-action.y,action.x-corex,1) + 90;
			if(tangent>180)
			tangent=tangent-360;
		}
		if(action.y < corey)
		{
			
			//输出给定圆的切线方向
			tangent=angGen(corey-action.y,action.x-corex,-1) + 90;
		}
		if(action.y == corey&&action.x==(corex + Radium))
			tangent=-90;
		if(action.y == corey&&action.x==(corex-Radium))
			tangent=90;
		if(action.y>corey)
			
		//距离转化角度函数
		fake_tan=err(distance_err,tangent);
		if(action.y < corey)
			
		//距离转化角度函数
		fake_tan=err(-distance_err,tangent);
		
		//特殊情况
		if(action.y == corey&&action.x>corex)
		{
			if(distance_err>0)
			{
				
				//90度右边
				if(distance_err < buff)
				fake_tan=-90 + fabs(distance_err*rate);
				if(distance_err>buff)
				fake_tan=0;	
			}
			if(distance_err < 0)
			{
				//90度左边
				if(fabs(distance_err)<buff)
				fake_tan=-90-fabs(distance_err*rate);
				
				//90度左边
				if(fabs(distance_err)>buff)
				fake_tan=180;
			}	
		}
		
		
		if(action.y == corey&&action.x < corex)
		{
			if(distance_err>0)
			{
				//-90度左边
				if(distance_err < buff)
				fake_tan=90 + fabs(distance_err*rate);
				if(distance_err>buff)
				fake_tan=180;
			}
			if(distance_err < 0)
			{
				//90度左边
				if(fabs(distance_err)<buff)
				fake_tan=90-fabs(distance_err*rate);
				if(fabs(distance_err)>buff)
				fake_tan=0;
			}
		}
		
		//由当前角度转向beta
		runTo(fake_tan,V_loop);

	}
	
}




/**
* @name 	   		R_buff
* @brief  		设置缓冲，缓解改变半径时的轨道突变导致的机器人的不稳定
				比如我要从半径为1500骤降到900，则需要设置的参数为-1（减），要降的幅度是6（一共600）,每次降2
* @param       	buffType: 决定是半径激增的缓冲还是半径骤减的缓冲的标志位
				buffDegree: 需要改变的总量（要乘以100）
				perVal:决定每一个运行周改变多少
				standard_R：最后要达到的半径
* @retval
*/
float R_buff(int buffType,int buffDegree,int perVal,float standardR)
{  
	rbuffCnt += perVal;
	       if(rbuffCnt >= buffDegree * 100)
	       {
			   rbuffCnt=buffDegree * 100;
			   
		   }
    run_r = standardR - buffType * buffDegree * 100 + buffType * rbuffCnt;  
return run_r;
}



/**
* @name 	   		V_buff
* @brief  		设置缓冲，缓解改变速度时的突变导致的机器人的不稳定
				比如我要从半径为2米每秒骤降到1米每秒，则需要设置的参数为-1（减），要降的幅度是10（一共10*0.1）,每次降0.00025
* @param       	buffType: 决定是速度激增的缓冲还是速度骤减的缓冲的标志位
				buffDegree: 需要改变的总量（要乘以0.1）
				perVal:决定每一个运行周改变多少
				standard_R：最后要达到的速度
* @retval
*/
float V_buff(float buffType,float buffDegree,float perVal,float standard_V)
{
		vbuffCnt += perVal;
	       if(vbuffCnt >= buffDegree * 0.1)
	       {
			   vbuffCnt = buffDegree * 0.1;
		   }
    run_V = standard_V - buffType * buffDegree * 0.1 + buffType * vbuffCnt;  
return run_V;
}



/**
* @name 	   		roundSetting
* @brief  			给一圈设置相关参数
* @param       	 	radiusNum: 需要设置的圈数
* @retval			给出半径
*/
float roundSetting(int radiusNum)
{
		float ra_set=0;
	if(radiusNum == 1000)
	{
		ra_set=1000;
		KP=250;
		buff=800;
	}
	if(radiusNum == 900)
	{
		  buff=800;
		  ra_set=900;
		  stableFlg=0;
		  KP=180;
	}
	if(radiusNum == 1200)
	{
			buff=800;
			ra_set=12000;
			stableFlg=0;
			KP=240;
	}
	if(radiusNum == 1900)
	{
		ra_set=2000;
		stableFlg=0;
		KP=120;
	}
    if(radiusNum == 600)
	{
		ra_set=600;
		stableFlg=0;
		KP=200;
	}
	if(radiusNum == 1500)
	{
		if(againCnt < 3)
		stableFlg=1;
		else stableFlg=0;
	    KP=110;
	}
		return ra_set;
	
}



float get_angle2(float a,float b,int n,int round)//n指向,round顺逆时针，ax + by + c=0；
{ 	
	if(a == 0)
	{
		if(b < 0)
			n=1;
		else if(b>0)
			n=-1;
	}
	else if(a>0)
		n=1;
	else if(a < 0)
		n=-1;
	
	if(b!=0)
	{ if(n == 1)
		{ if(-a/b>0)
		  {   
			  return(atan2(-a/b,1)/3.14159*180);
		  }
		  else if(-a/b < 0)
		  {    
			  return(atan2(a/b,-1)/3.14159*180);
		  }
		}else
		{
			if(-a/b>0)
			{
		    return(atan2(a/b,-1)/3.14159*180);
				
			}
		  else if(-a/b < 0)
		  {   
			 return(atan2(-a/b,1)/3.14159*180);
		  }
		}
	}
	else if(b == 0)
	{
		if(round==-1)//顺指针
		{   if(n == 1)
		   	{
				
				return(90);
			}
			else 
			{
				return(-90);
		       
				
			}
		}
        else 
		{   
			if(n == 1)
		   {
			
				return(90);
			  
			}
			
			else 
			{
				return(-90);
		      
			}       			
		}
	} 
     
	if(a == 0)
	{
		if(n == 1)
		{  
			return(0);
		
		}
        else 
		{   
			return(-180);
            			
		}
		
	}	
}





/**
* @name 	   		get_positive_angle
* @brief  			得到车到圆心的角度（0~360度）
* @param       	 	a1 b1-直线参数ax+by=0;
正常坐标系下n=1为斜率不为零朝向上，-1为斜率不为零朝下，2为斜率为零且为x轴正向，-2为斜率为零且x轴福相
* @retval			给出角度
*/
float get_positive_angle(float a1,float b1,int n1)
{
	if(a_genforRadius(a1,b1,n1 )<0)
		return (a_genforRadius( a1,b1,n1) + 360);
	if(a_genforRadius( a1,b1,n1)>=0)
		return a_genforRadius( a1,b1,n1);
}

int8_t no_round5=0;

/**
* @name 	   		Radius
* @brief  			切换半径·
* @param	
* @retval			给出半径
*/
int Radius(void)
{
	extern int errRecord;
	
	//在第一圈或者第二圈的时候如果进行了避障处理（调换时针），将该位置记下为下一次换半径的位置
	  if(troubleCnt>0&&(roundCnt == 1||roundCnt == 2||roundCnt == 6))
	  {
		  change_angle=errposAng;
	  }
	  

	  else if(runAgain!=-1)
		{
			
			//在正常走形模式下，经过扫边圈（runAgain==1）后，在此地方换半径，进入定点模式
			if(runAgain == 1&&roundCnt == 7)
				change_angle = 180 + clockFlg * 10;
			
			//正常换半径的位置 （机器人在我方出发区角度为180）
			else change_angle = 180 + clockFlg * 60;
		}		
		
		//如果启动模式为定点的话，在半圈的地方换半径
	  else if(runAgain==-1)
	  {		if(clockFlg == 1)
			change_angle=0 + clockFlg*20;
		  if(clockFlg==-1)
			  change_angle=360 + clockFlg*20;
	  }
	  
	  //得到当前机器人到圆心的角度
	nowAng=get_positive_angle(action.y,-(action.x + 2350),clockFlg);//记这次的与圆心的角度//a=action.y-y,  ,b=-(action.x-x)
	
	//防止get_positive_angle得到的数发生突变
	  if(fabs(nowAng-earlyAng)>100)
	{
	  earlyAng=nowAng;
	}
	
	//进入换轨和轨道设置
	if((troubleFlg == 1&&(roundCnt == 1 || roundCnt == 2||roundCnt == 6)) || (roundCnt == 0) || (earlyAng >= change_angle&&nowAng < change_angle && clockFlg == 1) || (earlyAng <= change_angle && nowAng > change_angle && clockFlg == -1))//换到
{
		R_switch = 1;  
		if(R_switch > r_switch || roundCnt == 0 || troubleFlg == 1)  
{
	      troubleFlg = 0;
	last_round_cnt = roundCnt;
		  roundCnt ++;
		  rbuffCnt = 0;
	      vbuffCnt = 0;
switch (roundCnt)
{
	case 1:
		
		//第1轨道的相关参数设置
		buff = 1200;
		radiusA = 300;
		stableFlg = 0;
		KP = 220;
		break;
	
	case 2:
		
		//第2轨道的相关参数设置
		stableFlg=1;			  
		if(clockFlg == 1)
			go_real_send_angle +=6;
		else if(clockFlg==-1)
			go_real_send_angle-=10;	
		if(runAgain == 1)
			stableFlg=0;
		 else   
			 stableFlg=1;
		buff=900;
		radiusA=roundSetting(1000);
		break;
		 
	case 3:
		
		//第3轨道的相关参数设置
		 if(clockFlg == 1)
			  {
				go_real_send_angle -= 10.f;
			  }
			  else if(clockFlg == -1)
					go_real_send_angle += 13.f;
			  
			 if(runAgain == 1)
			 {
				roundCnt = 4;
			 }
			  radiusA = 1500;
			 	 if(runAgain == 1)
						stableFlg = 0;
				 else   stableFlg = 1;
			
			   buff = 800;
			  KP=125;
			 troubleCnt=0;
		break;
				 
	case 4:
		//第4轨道的相关参数设置
		go_real_send_angle -= clockFlg * 2.f;
		radiusA = 1500;
		stableFlg = 1;
	
		//只在跑第一轮走形的时候进行边走边投
		if(againCnt == 1 && runAgain == 1)
			stableFlg=0;
		if(againCnt != 1 && runAgain == 1)
			stableFlg = 0;
		if(runAgain == 0)
			stableFlg = 1;
		buff=800;
		KP=125;
		break;
		
	case 5:
		
		//只有到第二轮走形才会走第五轨道（扫边和死角）
		if(runAgain == 1)
		{
			errRecord=0;
			stableFlg=0;
			radiusA=roundSetting(1900);
			deadZone=1;
			shoot_over=0;
		}
		if(runAgain == 0)
			  {
				  roundCnt=7;
			  }
		//在第五圈经历过三次避障（定位系统不准了），就不再走第五轨道
		if(no_round5 == 1)
			roundCnt = 6;
				  
		break;
	case 6:
		
		//第6轨道的相关参数设置
		if(runAgain == 0||runAgain == -1)
		{
			radiusA = roundSetting(900);
			deadZone = 1;
			shoot_over = 0;
		}
		if(runAgain == 1)
		{
			roundCnt = 7;
		}

	
		break;
	case 7:
		
		//第6轨道的相关参数设置
		radiusA = roundSetting(600);
		SureNo_BallFlag = 0;
		break;
	case 8:
		//第8轨道的相关参数设置
		pointErrDeal = 1;
		againCnt++ ;
		break;
	
	default:
		break;
} 
		  
}
}	  else R_switch=0;
	 r_switch =R_switch;
		
if(troubleFlg == 0)
{
	//设置换轨的缓冲
	switch (roundCnt)
	{
		
		//刚启动的时入圆的速度缓冲
		case 1:	V_buff(1,3,0.005,1.5);
			break;
		
		case 2:
			
			//从第一轨道上升到第二轨道，半径经过2秒的缓冲从500扩大到1000
			radiusA=R_buff(1,4,2,1000);
		
			//边走边投保持1米5每秒
			if(stableFlg == 1)
				V_buff(1,1,0.005,1.5);
			
			//不边走边投保持1米每秒
			if(stableFlg == 0)
				{
					V_buff(1,5,0.005,1);
				}
			break;
			
		case 3:
			
			//半径从1米在1秒3内增到1.5米
			if(stableFlg == 1)
				{
					radiusA=R_buff(1,5,3,1500);
				}
		
			//不边走边投情况下在2秒内速度从1.5米增加到2米每秒
			if(stableFlg == 0)
				{
					radiusA=R_buff(1,3,2,1500);
					V_buff(1,5,0.0025,2);
				}
			break;
		
		case 4:
			if(runAgain == 1&&againCnt == 1)
				{
					V_buff(1,5,0.0025,2);
					radiusA=R_buff(1,6,3,1500);
				}
			break;
		
		case 5:
			
			//扫边圈
			radiusA=R_buff(1,3,2,2000);
			if(no_round5 == 1)
				roundCnt=6;
			buff=2000;
			break;
		
		case 6:
			//回收圈
		//如果为定点模式，这一圈为启动圈，缓冲半径和速度比较大，让机器人更早的入圆
			if(runAgain==-1)
		{
			buff=1400;
			radiusA=R_buff(-1,6,5,300);
			V_buff(1,8,0.0025,2);
		}else
		
		//正常模式下速度和半径的缓冲
		{
			buff=1000;
			radiusA=R_buff(-1,5,4,900);
			V_buff(1,8,0.0025,2);
		}KP=250;
			break;
		case 7:
			if(runAgain==-1)
				{
					buff=1200;
					radiusA=R_buff(-1,7,6,200);
					V_buff(-1,3,0.004,1.3);
				}
				if(runAgain == 0)
				{
					radiusA=R_buff(-1,10,5,400);
				}
				if(runAgain == 1)//congwaiquanjinlai
				{
					radiusA=R_buff(-1,10,4,400);
					V_buff(-1,1,0.0025,1.2);
				}
				KP=250;
			break;
			default:
		//第八轨道为定点模式		
		if(pointErrDeal == 1)
		{
			pointErrDeal=1;
			roundCnt=8;
		}
		
		//定点结束退出定点模式，需要往后退一段距离为了给下次入圆留出空间
		if(pointErrDeal == 0)
		{
			Walkback(600);
			roundCnt=2;
			runAgain=1;
		}
		break;
		
	}

	
}

//记录上一次的与圆心的角度
 earlyAng=get_positive_angle(action.y,-(action.x + 2350),clockFlg);
    if(go_and_shoot >= 3)
		go_and_shoot=4;
return radiusA;
}

int examingtime=0;
extern float recover_buff;  
int8_t warm=0;
int icnt=0;

/**
* @name 	   		walk_turn_around
* @brief  			调换时针的避障处理
* @param			around_time-执行的时间
* @retval			
*/
void walk_turn_around(int around_time)
{
	if(around_time>0)
	{
		for(icnt=0;icnt < around_time;icnt++ )
		{
			
			//机器人逆时针绕轨
			if(clockFlg == 1)
			{	
				VelCrl(CAN1,0x01,exchange(1.5));
				VelCrl(CAN1,0x02,-exchange(0.8));
			}
		
			//机器人顺时针绕轨
			if(clockFlg==-1)
			{
				VelCrl(CAN1,0x01,exchange(0.5));
				VelCrl(CAN1,0x02,-exchange(1.5));
			}
		}
	}
	if(around_time < 0)
	{
		for(icnt=0;icnt<-around_time;icnt++ )
		{
			if(clockFlg == 1)
			{	
				VelCrl(CAN1,0x01,exchange(0.8));
				VelCrl(CAN1,0x02,-exchange(1.5));
			}
			if(clockFlg==-1)
			{
				VelCrl(CAN1,0x01,exchange(1.5));
				VelCrl(CAN1,0x02,-exchange(0.8));
			}
		}
	}
	clockFlg=-clockFlg;
}


/**
* @name 	   		 Walkback
* @brief  			向后退的避障处理
* @param			time-执行的时间
* @retval			
*/
void Walkback(float time)
{
	for(icnt=0;icnt < time;icnt++ )
	{
		VelCrl(CAN1,0x01,exchange(1));
		VelCrl(CAN1,0x02,-exchange(1));
	}
}

/**
* @name 	   		Walk_left_away
* @brief  			向右后方退的避障处理
* @param			time-执行的时间
* @retval			
*/
void Walk_left_away(int time)//故障处理
{	
	//如果不是扫边轨道
if(roundCnt != 5)
{
	for(icnt = 0;icnt <= time;icnt++ )
	{
    	VelCrl(CAN1,0x01,exchange(1.8));//左
	    VelCrl(CAN1,0x02,-exchange(0.5));	//右
	}
}

//如果不是扫边轨道
if(roundCnt == 5)
{
	for(icnt = 0;icnt <= time;icnt++ )
	{
    	VelCrl(CAN1,0x01,exchange(1.5));//左
	    VelCrl(CAN1,0x02,-exchange(0.1));	//右
	}
}
}

/**
* @name 	   		Walk_right_away
* @brief  			向右左方退的避障处理
* @param			time-执行的时间
* @retval			
*/
void Walk_right_away(int time)//故障处理
{   	
	if(roundCnt != 5)
{
	for(icnt = 0;icnt <= time;icnt++ )
	{
	VelCrl(CAN1,0x01,exchange(0.5));
	VelCrl(CAN1,0x02,-exchange(1.8));	
	}
}
	if(roundCnt == 5)
{
	for(icnt = 0;icnt <= time;icnt++ )
	{
	VelCrl(CAN1,0x01,exchange(0.1));
	VelCrl(CAN1,0x02,-exchange(1.8));	
	}
}
}






/**
* @name 	   		car_angle_gen
* @brief  			得出机器人实际运动速度的方向
* @param			xV-x轴速度
					yV-y轴速度
* @retval			gen_angle-机器人实际运动速度的方向
*/
float car_angle_gen(float xV,float yV)
{
	float gen_angle = 0;
	if(xV == 0)
	{
		if(yV > 0)
			gen_angle = 90.f;
		if(yV < 0)
			gen_angle = -90.f;
	}
	if(xV != 0)
	{
		if(yV > 0)
		{
			if(xV > 0)
			gen_angle = 90-atan(yV / xV) * 180.f / 3.14f;
			if(xV < 0)
			gen_angle = -90.f - atan(yV / xV) * 180.f / 3.14f;
		}
		if(yV < 0)
		{
			if(xV > 0)
			gen_angle = 90.f - atan(yV / xV) * 180.f / 3.14f;
			if(xV < 0)
			gen_angle = -90.f - atan(yV / xV) * 180.f / 3.14f;
		}
		if(yV == 0)
		{
			if(xV > 0)
			gen_angle = 0;
			if(xV < 0)
			gen_angle = 180;
		}
	}
	return gen_angle;
}


/**
* @name 	   		boomAccident
* @brief  			检测机器人姿态是否异常：机器人朝向与运动方向偏差过大
* @param			
* @retval		
*/
int boomAccident()
{
	static int accidentTime=0; 
		
	//实际运动方向与车头方向的偏差角
	float pratical_ang=0; 
	int8_t accidentFlag=0;
	pratical_ang=fabs(car_angle_gen(GetSpeedX(),GetSpeedY())-action.angle);
	if(pratical_ang>300)
		pratical_ang=360-pratical_ang;
	
	//如果偏差大于45度，开始计数
	if(pratical_ang>45)
		accidentTime++ ;
	else accidentTime=0;
	
	//异常时间达到临界，判断为异常情况
	if(accidentTime>250)
	{
		accidentFlag=1;
		accidentTime=0;
	}	
		return accidentFlag;
}
int err_kind=0;
int errRecord=0;
int nothingCanDo=0;


/**
* @name 	   		errdeal
* @brief  			处理机器人姿态和走形发生异常的情况
* @param			
* @retval		
*/
void errdeal(void)
{
		 err_kind = 0;
		static int errkind5 = 0;
		static int errkind7 = 0;
		static int stoptime = 0;
		float move_angle = 0;
		int8_t needtomove = 0;
		int errpos[3];//记录每次避障的位置
		int turntime = 600;
		static int qianZhi = 0;
		static int errtime = 0;
		static int8_t dead_err = 0;
		static int8_t nextzone = 0;
		static int8_t boomAccidentflag = 0;
		extern int zoneArrive;
		extern int errorLockFlag;
	
	//计算机器人的实际行进速度
	    float now_car_v = sqrt(pow(GetSpeedY(),2) + pow(GetSpeedX(),2)),car_d_to_center=sqrt(pow(GetX(),2) + pow(GetY()-2400,2));
		
	//如果机器人在非定点模式下速度减小到一定范围，开始计数
		if(((now_car_v <= 200)&&(clockFlg!=0)&&pointErrDeal == 0))//||(needtomove == 1)||(pointErrDeal == 1&&errorLockFlag == 1)||(pointErrDeal == 1&&now_car_v>500)
		{
			errtime++ ;
		}else errtime = 0;	
		boomAccidentflag=boomAccident();
		
		//机器人卡死5秒
		if(((now_car_v <= 20)&&(clockFlg != 0) && pointErrDeal == 0))//||(needtomove == 1)||(pointErrDeal == 1&&errorLockFlag == 1)||(pointErrDeal == 1&&now_car_v>500)
		{	
			stoptime++ ;
		}else stoptime=0;	
		
		//机器人行进速度减小到一定范围或者机器人的行进方向与机器人朝向不一样或者在一个位置卡死5秒以上时，判断为发生故障，进入故障处理
		if((errtime>50&&deadZone == 0)||(errtime>150&&roundCnt == 5&&pointErrDeal == 0) || stoptime > 500 || boomAccidentflag)	//||boomAccidentflag == 1||(needtomove == 1&&errtime>100)||(pointErrDeal == 1&&now_car_v>5&&errtime>150)
			{
				troubleFlg=1;
				
				//在第一和第二轨道发生故障
				if(roundCnt == 1||roundCnt == 2)
				troubleCnt++ ;
				if(roundCnt == 1)
					err_kind=1;
				if(roundCnt == 2)
					err_kind=2;

				//在第三第四轨道发生故障
				if(roundCnt == 3||roundCnt == 4)
					err_kind=3;
				
				//在第五轨道发生故障
				if(roundCnt == 5&&pointErrDeal == 0)
					err_kind=5;
				
				//在第六轨道发生故障
				if(roundCnt == 6)
					err_kind=6;
				
				//在第七轨道发生故障
				if(roundCnt == 7)
					err_kind=7;
				
				//在第八道发生故障
				if(roundCnt == 8)
				{
					roundCnt=8;
					troubleFlg=0;
				}
				
				//异常卡死
				if(stoptime>500)
				{
					pointErrDeal =1;
					roundCnt=8;
					nothingCanDo=1;
				}
					
				//第一圈和一二圈发生故障的处理
				if(err_kind == 1||err_kind == 2)
				{
					
					//记录此次避障处理前当前相对于圆心的位置,记为下次换轨道的位置
					errposAng=get_positive_angle(action.y,-(action.x + 2350),clockFlg);
					
					//掉头
					walk_turn_around(1000);
				}
				
				//第三第四轨道发生故障，缩到内圈启动定点模式
				if(err_kind == 3)
				{
					//调整机器人姿态
					if(clockFlg == 1)
						Walk_right_away(900);
					if(clockFlg==-1)
						Walk_left_away(900);
						Walkback(1000);
					//启动定点
					pointErrDeal=1;
				}
				
				//第五轨道发生故障
				if(err_kind == 5)
				{
					
					if(errkind5 >= 2)
					{
						no_round5=1;
					}
					
				//如果在该轨道发生故障超过两次
				if(no_round5)
				{				
					
					//退出该轨道，进入第六轨道
					if(clockFlg == 1)
						Walk_left_away(600);
					if(clockFlg==-1)
						Walk_right_away(600);
					Walkback(1000);
					roundCnt=6;
				}
				
				//如果在该轨道第一次或第二次发生故障
				if(no_round5 == 0)
				{
					errkind5++ ;
					if(clockFlg == 1)
					{
						if(deadZone == 1)
						{
							//撞到别的车了或者连续三次卡墙
							if(fabs(action.angle-90)<40||dead_err>2)
							{
								if(dead_err>2)
								dead_err=0;
								else Walk_left_away(turntime);
								nextzone=1;
							}
							
							//垂直撞在墙边							
							if((action.angle<=-150&&action.angle>=-179)||(action.angle >= 150&&action.angle <= 180))
							{
								Walk_left_away(turntime);
								dead_err++ ;
								zoneArrive=0;
							}
							
					
						}
						if(deadZone == 2)
						{
							//撞到别的车或者连续三次卡墙就奔下一个点
							if((fabs(action.angle)<40)||dead_err>2)
							{
								
								if(dead_err>2)
								dead_err=0;
								else Walk_left_away(turntime);
								nextzone=1;
							}
							
							//垂直撞在墙边		
							if(fabs(action.angle-90)<30)
							{
								Walk_left_away(turntime);
								dead_err++ ;
								zoneArrive=0;
							}
							
					
						}
						if(deadZone == 3)
						{
							//撞到别的车或者连续三次卡墙就奔下一个点
							if((fabs(action.angle + 90)<40)||dead_err>2)
							{
								if(dead_err>2)
								dead_err=0;
								else Walk_left_away(turntime);
								nextzone=1;
							}
							
							//垂直撞在墙边		
							if(fabs(action.angle)<40)
							{
								Walk_left_away(turntime);
								dead_err++ ;
								zoneArrive=0;
							}
						
						}
						if(deadZone == 4)
						{
							//撞到别的车或者连续三次卡墙就奔下一个点
							if(((action.angle<=-150&&action.angle>=-179)||(action.angle >= 150&&action.angle <= 180))||dead_err>2)
							{
								if(dead_err>2)
								dead_err=0;
								else Walk_left_away(turntime);
								nextzone=1;
							}
							
							//垂直撞在墙边	
							if(fabs(action.angle + 90)<40)
							{
								Walk_left_away(turntime);
								dead_err++ ;
								zoneArrive=0;
							}		
							
					
						}
						
					}
					if(clockFlg==-1)
					{
						if(deadZone == 1)
						{
							//撞到别的车了或者连续三次卡墙
							if(fabs(action.angle + 90)<40||dead_err>2)
							{
								if(dead_err>2)
								dead_err=0;
								else Walk_right_away(turntime);
								nextzone=1;
							}
							
							//垂直撞在墙边	
							if((action.angle<=-150&&action.angle>=-179)||(action.angle >= 150&&action.angle <= 180))
							{
								Walk_right_away(turntime);
								dead_err++ ;
								zoneArrive=0;
							}
						}
						if(deadZone == 2)
						{
							//撞到别的车或者连续三次卡墙就奔下一个点
							if((fabs(action.angle) < 40) || dead_err > 2)
							{
								
								if(dead_err > 2)
								dead_err = 0;
								else Walk_right_away(turntime);
								nextzone = 1;
							}
							
							//卡墙上了
							if(fabs(action.angle + 90) < 40)
							{
								Walk_right_away(turntime);
								dead_err ++ ;
								zoneArrive = 0;
							}
						}
						if(deadZone == 3)
						{
							//撞到别的车或者连续三次卡墙就奔下一个点
								if((fabs(action.angle-90) < 40) || dead_err > 2)
							{
								if(dead_err > 2)
								dead_err = 0;
								else Walk_right_away(turntime);
								nextzone = 1;
							}
							
							//卡墙上了
							if(fabs(action.angle) < 40)
							{
								Walk_right_away(turntime);
								dead_err ++ ;
								zoneArrive = 0;
							}
						}
						if(deadZone == 4)
						{
							//撞到别的车或者连续三次卡墙就奔下一个点
							if(((action.angle <= -150&&action.angle >= -179) || (action.angle >= 150 && action.angle <= 180)) || dead_err > 2)
							{
								if(dead_err > 2)
								dead_err = 0;
								else Walk_right_away(turntime);
								nextzone = 1;
							}
							//卡墙上了
							if(fabs(action.angle - 90) < 40)
							{
								Walk_right_away(turntime);
								dead_err ++ ;
								zoneArrive = 0;
							}		
							
					
						}
					}
					
					//清楚标志位
					if(nextzone == 1)
					{
						nextzone=0;
						deadZone++ ;
						zoneArrive=0;
						if(deadZone >= 5)
						deadZone=1;
					}
				}					
				}
				
				//第六轨道故障处理：掉头
				if(err_kind == 6)
				{
					errposAng=get_positive_angle(action.y,-(action.x + 2350),clockFlg);
					walk_turn_around(-1000);
				}
				
				//第七轨道故障处理：进入定点模式
				if(err_kind == 7)
				{	
					errkind7++ ;
					if(errkind7 >= 2)
					{
					if(clockFlg == 1)
					Walk_left_away(800);
					if(clockFlg==-1)
					Walk_right_away(800);
						roundCnt=8;
						pointErrDeal=1;
					troubleFlg=0;
					}else walk_turn_around(-1000);
				}
				errtime=0;
				err_kind=0; 
			}
		dead_err=0;
			}			