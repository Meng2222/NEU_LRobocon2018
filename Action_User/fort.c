/**
  ******************************************************************************
  * @file     
  * @author  Dantemiwa
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @attention
  *  
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/

#include "fort.h"
#include "stm32f4xx_usart.h"
#include "string.h"
#include "timer.h"
#include "stm32f4xx_it.h"
#include "math.h"
#include "elmo.h"
#include "usart.h"
#include "moveBase.h"

//对应的收发串口
#define USARTX UART5


FortType fort;

//给推球电机的脉冲
int32_t pushPulse=0;
int bufferI = 0;
char buffer[20] = {0};



/**
* @brief 炮台航向控制
* @param  ang:转台航向角度，范围为0~360度
* @retval none
* @attention none
*/
void YawPosCtrl(float ang)
{
		fort.usartTransmitData.dataFloat = ang;
		USART_SendData(USARTX,'Y');
		USART_SendData(USARTX,'A');
		USART_SendData(USARTX,fort.usartTransmitData.data8[0]);
		USART_SendData(USARTX,fort.usartTransmitData.data8[1]);
		USART_SendData(USARTX,fort.usartTransmitData.data8[2]);
		USART_SendData(USARTX,fort.usartTransmitData.data8[3]);
		USART_SendData(USARTX,'\r');
		USART_SendData(USARTX,'\n');
		fort.usartTransmitData.data32 = 0;
		
}

/**
* @brief 发射电机转速控制
* @param  rps:发射电机速度，单位转每秒
* @retval none
* @attention none
*/

void ShooterVelCtrl(float rps)
{
		fort.usartTransmitData.dataFloat = rps;
		USART_SendData(USARTX,'S');
		USART_SendData(USARTX,'H');
		USART_SendData(USARTX,fort.usartTransmitData.data8[0]);
		USART_SendData(USARTX,fort.usartTransmitData.data8[1]);
		USART_SendData(USARTX,fort.usartTransmitData.data8[2]);
		USART_SendData(USARTX,fort.usartTransmitData.data8[3]);
		USART_SendData(USARTX,'\r');
		USART_SendData(USARTX,'\n');
		fort.usartTransmitData.data32 = 0;
}


void bufferInit()
{
	for(int i = 0; i < 20; i++)
	{
		buffer[i] = 0;
	}
	bufferI = 0;
}

/**
* @brief 接收炮台返回的数据
* @param data：串口每次中断接收到的一字节数据
* @retval none
* @attention 该函数请插入到对应的串口中断中
							注意清除标志位
*/
void GetValueFromFort(uint8_t data)
{	
	buffer[bufferI] = data;
	bufferI++;
	if(bufferI >= 20)
	{
		bufferInit();
	}
	if(buffer[bufferI - 2] == '\r' && buffer[bufferI - 1] == '\n')
	{ 
		if(bufferI > 2 &&  strncmp(buffer,"PO",2) == 0)//接收航向位置
		{
				for(int i = 0; i < 4; i++)
					fort.usartReceiveData.data8[i] = buffer[i + 2];
				fort.yawPosReceive = fort.usartReceiveData.dataFloat;
		}
		else if(bufferI > 2 &&  strncmp(buffer,"VE",2) == 0)//接收发射电机转速
		{
				for(int i = 0; i < 4; i++)
					fort.usartReceiveData.data8[i] = buffer[i + 2];
				fort.shooterVelReceive = fort.usartReceiveData.dataFloat;
		}
		else if(bufferI > 2 && strncmp(buffer,"LA",2) == 0)//接收A激光的ADC值
		{
			for(int i = 0; i < 4; i++)
					fort.usartReceiveData.data8[i] = buffer[i + 2];
				fort.laserAValueReceive = fort.usartReceiveData.dataFloat;
			
		}
		else if(bufferI > 2 && strncmp(buffer,"LB",2) == 0)//接收B激光的ADC值
		{
			for(int i = 0; i < 4; i++)
					fort.usartReceiveData.data8[i] = buffer[i + 2];
				fort.laserBValueReceive = fort.usartReceiveData.dataFloat;
		}
		bufferInit();
	}
}

char BUFF[8]={0};

struct comend Cmd;
/**
* @brief 电脑串口发命令给主控
* @param  data:串口数据
* @retval none
* @attention none
*/
void UARTCmd(uint8_t data)
{
	static uint8_t buffI=0;
	BUFF[buffI] = data;
	buffI++;
	if(BUFF[buffI - 2] == '\r' && BUFF[buffI - 1] == '\n')
	{ 
		if(buffI > 2 &&  strncmp(BUFF,"PO",2) == 0)//接收航向位置
		{
			for(int i = 0; i < 4; i++)
				Cmd.uv4Data.data8[i] = BUFF[i + 2];
			Cmd.turn=(float)((Cmd.uv4Data.data8[0]-0x30)*100+(Cmd.uv4Data.data8[1]-0x30)*10+(Cmd.uv4Data.data8[2]-0x30)+(Cmd.uv4Data.data8[3]-0x30)*0.1);
		}
		else if(buffI > 2 &&  strncmp(BUFF,"VE",2) == 0)//接收发射电机转速
		{
			for(int i = 0; i < 4; i++)
				Cmd.uv4Data.data8[i] = BUFF[i + 2];
			Cmd.shoot=(float)((Cmd.uv4Data.data8[0]-0x30)*100+(Cmd.uv4Data.data8[1]-0x30)*10+(Cmd.uv4Data.data8[2]-0x30)+(Cmd.uv4Data.data8[3]-0x30)*0.1);
		}
		for(int i = 0; i < 8; i++)
		{
			BUFF[i] = 0;
		}
			buffI = 0;
	}
}

//记录四个框推出球情况，为1 该框推出球，为0 该框未推出球，为2该框激光未扫描到
uint8_t shootReady[4]={0,0,0,0};
//控制航向电机转的角度
static float shootTurnAngle=0;
//外圈停下投球标志位
uint8_t stopFlg=0;
//该投桶的标志位
uint8_t shootFlag=0;
//摄像头识别是否为所要颜色的球
uint8_t isBallRight=0;	
//没有所要球标志位，用于定点投球长时间没有正确的球，为1表示没有正确的球，机器人出去收球
uint8_t noRightBall=0;
//四个桶有一个未扫描到，计算得出距离，为1，使用计算距离发射
uint8_t noScanFlg=0;
//车走形所在轨道投球的桶
uint8_t shootFlagOne;
//定点投球四个桶的距离，四个桶都投过后，先投距离最远的桶
float laserDistance[4]={0};
extern uint8_t step;
//故障判断标志位 主要用于车卡在某一位置，即errFlg=3;
extern uint8_t errFlg;

extern float judgeSpeed;
extern uint8_t flagOne;
extern usartValue uv4;
/**
* @brief 炮台发射
* @param flg：车走行的标志位
* @param pushTime：切换轨道后推球计数时间
* @retval none
* @attention 
*/

//四个桶坐标
static float bucketPosX[4]={BUCKET_ONE_X,BUCKET_TWO_X,BUCKET_THR_X,BUCKET_FOR_X};
static float bucketPosY[4]={BUCKET_ONE_Y,BUCKET_TWO_Y,BUCKET_THR_Y,BUCKET_FOR_Y};

//摩擦轮转速
static float shootSpeed=0;
void Shoot(uint8_t flg)
{
	static float shootAngleLast=0,getAngleLast=0;
	
	//errOne上一次炮台相对车角度与这次炮台相对车角度的差，errTwo上一次定位系统角度与这次定位系统角度的差，用于计算炮台角度
	float errOne=0,errTwo=0;
	float shootX=GetPosX();
	float shootY=GetPosY();
	float getAngle=GetAngle();
	
	//射击距离
	float shootDistance=0;
	float shootAngle=0;
	
	//激光测得距离和定位系统算得的参考距离
	float laserdistance=0,judgeDistance=0; 
	
	BallColorRecognition();
	
	if(flg == 0 || flg == 1 || flg == 2)
	{
		//完全卡住
		if(errFlg >= 3)
		{
			CarStuck();
		}
		
		//正常发射
		else
		{
			NormalShootOne();

				
		}
	}

	if(flg == 3 || flg == 4 || flg == 5)
	{
		//完全卡住
		if(errFlg >= 3)
		{
			CarStuck();
		}
		
		//正常发射
		else
		{
			NormalShootOne();
		}			
	}
	
	//枪定位桶角度
	if(shootFlag == 2 || shootFlag == 3)
	{
		shootAngle=90-(atan((shootY-bucketPosY[shootFlag])/(shootX-bucketPosX[shootFlag]))*180/PI);
	}
	
	else if(shootFlag == 0 || shootFlag == 1)
	{
		shootAngle=-(atan((shootY-bucketPosY[shootFlag])/(shootX-bucketPosX[shootFlag]))*180/PI)-90;
	}
	errOne=shootAngle-shootAngleLast;
	errTwo=getAngle-getAngleLast;
	
	//优弧劣弧处理，转角始终小于180
	if(errOne > 180 && errTwo > 180)
		shootTurnAngle+=(errOne+errTwo-720);
	
	else if(errOne < -180 && errTwo < -180)
		shootTurnAngle+=(errOne+errTwo+720);
	
	else if(errOne > 180 || errTwo > 180)
		shootTurnAngle+=(errOne+errTwo-360);
	
	else if(errOne < -180 || errTwo < -180)
		shootTurnAngle+=(errOne+errTwo+360);
	
	else
		shootTurnAngle+=(errOne+errTwo);
	
		getAngleLast=getAngle;
		shootAngleLast=shootAngle;
	
	//完全卡住发射
	if(errFlg >= 3 || stopFlg == 1)
	{
		if(noScanFlg == 0)
		{
			if((fort.laserAValueReceive*LASER_SCALE_A+LASER_INTERCEPT_A) > (fort.laserBValueReceive*LASER_SCALE_B+LASER_INTERCEPT_B))
				laserdistance=(fort.laserAValueReceive*LASER_SCALE_A+LASER_INTERCEPT_A)-254;
			else
				laserdistance=(fort.laserBValueReceive*LASER_SCALE_B+LASER_INTERCEPT_B)-254;
			
			judgeDistance=sqrt(((shootY-bucketPosY[shootFlag])*(shootY-bucketPosY[shootFlag]))+((shootX-bucketPosX[shootFlag])*(shootX-bucketPosX[shootFlag])))-65;
			
			//激光计算距离和定位系统算出的距离差在800以内用激光测得距离
			if(fabs(judgeDistance-laserdistance) < 800)
			{
				shootDistance=laserdistance;
				if(shootDistance < 3333)
					shootSpeed=(SHOOT_KP*shootDistance)+SHOOT_INTERCEPT;
				else if(shootDistance >= 3333)
					shootSpeed=(0.01f*shootDistance)+50.5f;
				
			}
			else 
			{
				shootDistance=judgeDistance;
				shootSpeed=(SHOOT_KP*shootDistance)+SHOOT_INTERCEPT;
			}
			
			//边上停下打球
			if(stopFlg == 1)
				ShooterVelCtrl(shootSpeed);
			else
				ShooterVelCtrl(shootSpeed);
			
			//记录每个桶定点打的距离
			laserDistance[shootFlag]=shootDistance;
		}
		else;
	}
	//正常发射
	else
	{
		shootDistance=sqrt(((shootY-bucketPosY[shootFlag])*(shootY-bucketPosY[shootFlag]))+((shootX-bucketPosX[shootFlag])*(shootX-bucketPosX[shootFlag])))-65;
		
		//顺时针控制航向角
		if(flg == 0 || flg == 1 || flg == 2)
		{
			
			//正常中圈打球
			if(stopFlg == 0)
			{
				if(flagOne < 6 && flagOne > 1)
					YawPosCtrl(shootTurnAngle-(4*0.000001f*shootDistance*shootDistance-0.0386f*shootDistance+96));
				else if(flagOne >= 6 && flagOne < 11)
					YawPosCtrl(shootTurnAngle-judgeSpeed*0.007f);
			}
			else
				YawPosCtrl(shootTurnAngle);
			
		}
		
		//逆时针控制航向角
		else if(flg == 3 || flg == 4 || flg == 5)
		{
			//正常中圈打球
			if(stopFlg == 0)
			{
				if(flagOne < 6 && flagOne > 1)
					YawPosCtrl(shootTurnAngle+(4*0.000001f*shootDistance*shootDistance-0.0386f*shootDistance+100));
				else if(flagOne >= 6 && flagOne < 11)
					YawPosCtrl(shootTurnAngle+judgeSpeed*0.004f);
			}
			else
				YawPosCtrl(shootTurnAngle);
			
		}
		
		//顺时针走内圈，中圈，外圈
		if(flg == 0)
		{
			if(stopFlg == 0)
			{
				//内圈打球
				if(flagOne < 6)
				{
					shootSpeed=(0.0137f*shootDistance)+22;
					if(shootSpeed > 85)
						shootSpeed=85;
				}
				
				//外圈打球
				else if(flagOne <= 10)
				{
					shootSpeed=(SHOOT_KP*shootDistance)+SHOOT_INTERCEPT-judgeSpeed*0.011f;
					if(shootSpeed > 85)
						shootSpeed=85;
				}

			}
		}
		
		//顺时针走中圈，外圈
		else if(flg == 1)
		{   
			if(stopFlg == 0)
			{
				//内圈打球
				if(flagOne < 6)
				{
					shootSpeed=(0.0137f*shootDistance)+22;
					if(shootSpeed > 85)
						shootSpeed=85;
				}
				
				//外圈打球
				else if(flagOne <= 10)
				{
					shootSpeed=(SHOOT_KP*shootDistance)+SHOOT_INTERCEPT-judgeSpeed*0.011f;
					if(shootSpeed > 85)
						shootSpeed=85;
				}

			}

		}
		
		//逆时针走内圈，中圈，外圈
		else if(flg == 3)
		{
			if(stopFlg == 0)
			{
				//内圈打球
				if(flagOne < 6)
				{
					shootSpeed=(0.0137f*shootDistance)+21;
					if(shootSpeed > 85)
						shootSpeed=85;
					
				}
				
				//外圈打球
				else if(flagOne <= 10)
				{
					shootSpeed=(SHOOT_KP*shootDistance)+SHOOT_INTERCEPT-judgeSpeed*0.011f;
					if(shootSpeed > 85)
						shootSpeed=85;
				}


			}
		}
		
		//逆时针走中圈，外圈
		else if(flg == 4)
		{
			if(stopFlg == 0)
			{   
				//内圈打球
				if(flagOne < 6)
				{
					shootSpeed=(0.0137f*shootDistance)+21.0f;
					if(shootSpeed > 85)
						shootSpeed=85;
				}
				
				//外圈打球
				else if(flagOne <= 10)
				{
					shootSpeed=(SHOOT_KP*shootDistance)+SHOOT_INTERCEPT-judgeSpeed*0.011f;
					if(shootSpeed > 85)
						shootSpeed=85;
				}

			}
		}
		
		
		ShooterVelCtrl(shootSpeed);
	}
	
	
	//串口发数赋值

	uv4.shootFlg=shootFlag;
	uv4.distance=shootDistance;
	uv4.shootSp=shootSpeed;
	uv4.shootangle=shootTurnAngle;
	uv4.ball=isBallRight;
	uv4.ready[0]=shootReady[0];
	uv4.ready[1]=shootReady[1];
	uv4.ready[2]=shootReady[2];
	uv4.ready[3]=shootReady[3];
	
	
}

/**
* @brief 车卡住投球和定点投球
* @param none
* @retval none
* @attention 
*/

float X[4]={-2454,-2454,2454,2454},Y[4]={-119,4789,4789,-119};
float laserShootAngle[4]={0},laserShootDistance[4]={0};

//扫描到桶的标志位
uint8_t laserShootFlg=0;

//发射桶改变标志位
static uint8_t changeFlg=0;
void CarStuck(void)
{
	float cosAngle=0,d2=0;;
	static uint16_t carStuckCnt=0;
	
	//被撞动处理
	if(GetSpeeedX() > 100 || GetSpeeedY() > 100)
	{
		for(int i=0;i<4;i++)
			laserShootAngle[i]=0;
		for(int i=0;i<4;i++)
			laserShootDistance[i]=0;
		laserShootFlg=0;
		changeFlg=1;
		noScanFlg=0;
	}
	
	//条件满足，推球发射
	if(isBallRight == 1 && laserShootFlg == 1 && changeFlg == 1 && fabs(fort.shooterVelReceive-shootSpeed) < 1.3)
	{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);	
			shootReady[shootFlag] = 1;
			laserShootFlg=0;
			noScanFlg=0;
			changeFlg=0;
			USART_OUT(UART4, "shoot\r\n");		

	}
	
	//发射方位改变
	else if(changeFlg == 0)
	{
		
		if(carStuckCnt > 100)		
		{
			//左下桶没打过
			if(shootReady[0] == 0)
			{ 
				shootFlag=0;
				carStuckCnt=0;
				changeFlg=1;
			}
			
			//左上桶没打过
			else if(shootReady[1] == 0)
			{
				shootFlag=1;
				carStuckCnt=0;
				changeFlg=1;
			}
			
			//右上桶没打过
			else if(shootReady[2] == 0)
			{
				shootFlag=2;
				carStuckCnt=0;
				changeFlg=1;
			}
			
			//右下桶没打过
			else if(shootReady[3] == 0)
			{
				shootFlag=3;
				carStuckCnt=0;
				changeFlg=1;
			}
			
			//未扫描到计算出该桶的位置和距离，用后两个连续的桶算出该桶
			else if(shootReady[0]== 2 && laserShootDistance[1] != 0 && laserShootDistance[2] != 0)
			{
				shootFlag=0;
				cosAngle=fabs(laserShootDistance[2]*sin(fabs(laserShootAngle[2]-laserShootAngle[1])*PI/180)/4800);
				d2=laserShootDistance[1]*laserShootDistance[1]+4800*4800-2*4800*laserShootDistance[1]*cosAngle;
				laserShootDistance[0]=sqrt(d2);
				shootSpeed=(SHOOT_KP*laserShootDistance[0])+SHOOT_INTERCEPT+1;
				laserShootAngle[0]=laserShootAngle[1]-acos((laserShootDistance[1]*laserShootDistance[1]+d2-4800*4800)/(2*laserShootDistance[1]*laserShootDistance[0]))*180/PI;
				ShooterVelCtrl(shootSpeed);
				YawPosCtrl(laserShootAngle[0]);
				noScanFlg=1;
				if(fabs(fort.shooterVelReceive-shootSpeed) < 2.1 && fabs(fort.yawPosReceive-laserShootAngle[0]) < 2.1)
				{
					laserShootFlg=1;
					carStuckCnt=0;
					changeFlg=1;
				}						
				
			}
			else if(shootReady[1]== 2 && laserShootDistance[2] != 0 && laserShootDistance[3] != 0)//2,3
			{
				shootFlag=1;
				cosAngle=fabs(laserShootDistance[3]*sin(fabs(laserShootAngle[3]-laserShootAngle[2])*PI/180)/4800);
				d2=laserShootDistance[2]*laserShootDistance[2]+4800*4800-2*4800*laserShootDistance[2]*cosAngle;
				laserShootDistance[1]=sqrt(d2);
				shootSpeed=(SHOOT_KP*laserShootDistance[1])+SHOOT_INTERCEPT+1;
				laserShootAngle[1]=laserShootAngle[2]-acos((laserShootDistance[2]*laserShootDistance[2]+d2-4800*4800)/(2*laserShootDistance[2]*laserShootDistance[1]))*180/PI;
				ShooterVelCtrl(shootSpeed);
				YawPosCtrl(laserShootAngle[1]);
				noScanFlg=1;
				if(fabs(fort.shooterVelReceive-shootSpeed) < 2.1 && fabs(fort.yawPosReceive-laserShootAngle[1]) < 2.1)
				{
					laserShootFlg=1;
					carStuckCnt=0;
					changeFlg=1;
				}
			}
			else if(shootReady[2]== 2 && laserShootDistance[3] != 0 && laserShootDistance[0] != 0)//3,0
			{
				shootFlag=2;
				cosAngle=fabs(laserShootDistance[0]*sin(fabs(laserShootAngle[3]-laserShootAngle[0])*PI/180)/4800);
				d2=laserShootDistance[3]*laserShootDistance[3]+4800*4800-2*4800*laserShootDistance[3]*cosAngle;
				laserShootDistance[2]=sqrt(d2);
				shootSpeed=(SHOOT_KP*laserShootDistance[2])+SHOOT_INTERCEPT+1;
				laserShootAngle[2]=laserShootAngle[3]-acos((laserShootDistance[3]*laserShootDistance[3]+d2-4800*4800)/(2*laserShootDistance[3]*laserShootDistance[2]))*180/PI;
				ShooterVelCtrl(shootSpeed);
				YawPosCtrl(laserShootAngle[2]);
				noScanFlg=1;
				if(fabs(fort.shooterVelReceive-shootSpeed) < 2.1 && fabs(fort.yawPosReceive-laserShootAngle[2]) < 2.1)
				{
					laserShootFlg=1;
					carStuckCnt=0;
					changeFlg=1;
				}
			}
			else if(shootReady[3]== 2 && laserShootDistance[0] != 0 && laserShootDistance[1] != 0)//0,1
			{
				shootFlag=3;
				cosAngle=fabs(laserShootDistance[1]*sin(fabs(laserShootAngle[0]-laserShootAngle[1])*PI/180)/4800);
				d2=laserShootDistance[0]*laserShootDistance[0]+4800*4800-2*4800*laserShootDistance[0]*cosAngle;
				laserShootDistance[3]=sqrt(d2);
				shootSpeed=(SHOOT_KP*laserShootDistance[3])+SHOOT_INTERCEPT+1;
				laserShootAngle[3]=laserShootAngle[0]-acos((laserShootDistance[0]*laserShootDistance[0]+d2-4800*4800)/(2*laserShootDistance[0]*laserShootDistance[3]))*180/PI;
				ShooterVelCtrl(shootSpeed);
				YawPosCtrl(laserShootAngle[3]);
				noScanFlg=1;
				if(fabs(fort.shooterVelReceive-shootSpeed) < 2.1 && fabs(fort.yawPosReceive-laserShootAngle[3]) < 2.1)
				{
					laserShootFlg=1;
					carStuckCnt=0;
					changeFlg=1;
				}
				
			}
			
			//没有计算数据，先跑个半圆扫描，如果还没有，就其他3个桶再打一次，得到计算数据
			else if(shootReady[shootFlag] == 2)
			{
				shootReady[0]=0;
				shootReady[1]=0;
				shootReady[2]=0;
				shootReady[3]=0;
				noScanFlg=0;				
				flagOne=16;
				carStuckCnt=0;
				changeFlg=1;
			}
			
			//都打完后，刷新一次，先打最远点
			else
			{
				if(laserDistance[shootFlag] <= laserDistance[0])
					shootFlag=0;
				if(laserDistance[shootFlag] <= laserDistance[1])
					shootFlag=0;
				if(laserDistance[shootFlag] <= laserDistance[2])
					shootFlag=2;
				if(laserDistance[shootFlag] <= laserDistance[3])
					shootFlag=3;
				shootReady[0]=0;
				shootReady[1]=0;
				shootReady[2]=0;
				shootReady[3]=0; 
				noScanFlg=0;
				carStuckCnt=0;
				changeFlg=1;
			}
		}
		else 
			carStuckCnt++;
			
	}
	
	
	else if(laserShootFlg == 0 && changeFlg == 1)
	{
		//没找到桶，在中间扫描，没在中间跑到中间
		if(GetPosX() > -1500 && GetPosX() < 1500 && GetPosY() > 900 && GetPosY() < 3900)
			Laser_Aim();
		else
			noRightBall=1;
	}
}

/**
* @brief 正常发射
* @param getPushTime：推球时间
* @retval none
* @attention 
*/
static uint16_t shootCnt1=0;

uint8_t laserStopShootFlg=0;
void NormalShootOne(void)
{
	float D=0;
	static uint16_t shootCnt2=0;

	float speed=sqrt(GetSpeeedX()*GetSpeeedX()+GetSpeeedY()*GetSpeeedY());
	static uint8_t flgLast=0;
	if(flgLast != shootFlagOne)
	{	
		shootCnt2=0;
		shootCnt1=0;
		flgLast=shootFlagOne;
		laserStopShootFlg=0;
	}
	
	//外圈
	if(flagOne >= 6 && flagOne <= 10)
	{
		//打0号桶
		if(shootFlagOne == 0)
		{
			shootFlag=0;
			
			//没打过球
			if(shootReady[0] == 0 && step == 1)
			{
				shootCnt1++;
				
				//被撞处理
				if(GetSpeeedX() > 100 || GetSpeeedY() > 100)
				{
					laserStopShootFlg=0;
				}
				
				//有球后外圈定点打，没球不停下
				if(isBallRight == 1 && shootCnt1 < 800)
				{
					stopFlg=1;
				}
				else
				{
					stopFlg=0;
				}
				D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
				if(speed <= SPEED_TWO && isBallRight == 1)
				{
					//扫描到桶且转速和枪的角度达到所给值
					if(laserStopShootFlg == 1 && fabs(fort.shooterVelReceive-shootSpeed) < 2.1 && fabs(fort.yawPosReceive-shootTurnAngle) < 3)
					{
						// 推球	
						pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
						PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);	
						shootReady[0] = 1;
						laserStopShootFlg=0;
						USART_OUT(UART4, "shoot\r\n");		

					}
					else if(laserStopShootFlg == 0)
					{
						Laser_Aim_Two();
					}
							
				}
		
			}
			
			//外圈定点打完球后stopFlg置0
			else if(shootReady[0] == 1)
			{
				shootCnt2++;
				if(shootCnt2 > DELAY_TIME)
				{
					stopFlg=0;
					shootCnt2=0;
				}
			}
		}
		
		//打1号桶
		else if(shootFlagOne == 1)
		{
			shootFlag=1;
			
			//没打过球
			if(shootReady[1] == 0 && step == 1)
			{
				shootCnt1++;
				if(GetSpeeedX() > 100 || GetSpeeedY() > 100)
				{
					laserStopShootFlg=0;
				}
				if(isBallRight == 1 && shootCnt1 < 800)
				{
					stopFlg=1;
				}
				else
				{
					stopFlg=0;
				}
				D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
				
				//有要打的球且静止
				if(speed <= SPEED_TWO && isBallRight == 1)
				{
					//扫描到桶且转速和枪的角度达到所给值
					if(laserStopShootFlg == 1 && fabs(fort.shooterVelReceive-shootSpeed) < 2.1 && fabs(fort.yawPosReceive-shootTurnAngle) < 3)
					{
						// 推球	
						pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
						PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);	
						shootReady[1] = 1;
						laserStopShootFlg=0;
						USART_OUT(UART4, "shoot\r\n");		

					}
					else if(laserStopShootFlg == 0)
					{
						Laser_Aim_Two();
					}
							
				}
			}
			
			//打完球
			else if(shootReady[1] == 1)
			{
				shootCnt2++;
				if(shootCnt2 > DELAY_TIME)
				{
					stopFlg=0;
					shootCnt2=0;
				}
			}	
		}
		
		//打2号桶
		else if(shootFlagOne == 2)
		{
			shootFlag=2;
			
			//没打过球
			if(shootReady[2] == 0 && step == 1)
			{
				shootCnt1++;
				if(GetSpeeedX() > 100 || GetSpeeedY() > 100)
				{
					laserStopShootFlg=0;
				}
				if(isBallRight == 1 && shootCnt1 < 800)
				{
					stopFlg=1;
				}
				else
				{
					stopFlg=0;
				}
				D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
				
				//有要打的球且静止
				if(speed <= SPEED_TWO && isBallRight == 1)
				{
					//扫描到桶且转速和枪的角度达到所给值
					if(laserStopShootFlg == 1 && fabs(fort.shooterVelReceive-shootSpeed) < 2.1  && fabs(fort.yawPosReceive-shootTurnAngle) < 3)
					{
						// 推球	
						pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
						PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);	
						shootReady[2] = 1;
						laserStopShootFlg=0;
						USART_OUT(UART4, "shoot\r\n");		

					}
					else if(laserStopShootFlg == 0)
					{
						Laser_Aim_Two();
					}
							
				}
			}
			
			//打完球
			else if(shootReady[2] == 1)
			{
				shootCnt2++;
				if(shootCnt2 > DELAY_TIME)
				{
					stopFlg=0;
					shootCnt2=0;
				}
			}
		}
		
		//打3号桶
		else if(shootFlagOne == 3)
		{
			shootFlag=3;
			
			//没打过球
			if(shootReady[3] == 0 && step == 1)
			{
				shootCnt1++;
				if(GetSpeeedX() > 100 || GetSpeeedY() > 100)
				{
					laserStopShootFlg=0;
				}
				if(isBallRight == 1 && shootCnt1 < 800)
				{
					stopFlg=1;
				}
				else
				{
					stopFlg=0;
				}
				D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
				
				//有要打的球且静止
				if(speed <= SPEED_TWO && isBallRight == 1)
				{
					//扫描到桶且转速和枪的角度达到所给值
					if(laserStopShootFlg == 1 && fabs(fort.shooterVelReceive-shootSpeed) < 2.1 && fabs(fort.yawPosReceive-shootTurnAngle) < 3)
					{
						// 推球	
						pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
						PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);	
						shootReady[3] = 1;
						laserStopShootFlg=0;
						USART_OUT(UART4, "shoot\r\n");		

					}
					else if(laserStopShootFlg == 0)
					{
						Laser_Aim_Two();
					}
							
				}
			}
			
			//打完球
			else if(shootReady[3] == 1)
			{
				shootCnt2++;
				if(shootCnt2 > DELAY_TIME)
				{
					stopFlg=0;
					shootCnt2=0;
				}
			}	
		}
		else;
	}
	
	//里圈
	else if(flagOne < 6 && flagOne > 2)
	{
		//打右上桶
		if(shootFlagOne == 2)
		{
			shootFlag=2;
			
			//速度达到1300以上，有要打的球，转速和枪的角度达到所给值
			if(judgeSpeed > SPEED_ONE && isBallRight == 1 && shootReady[2] == 0 && fabs(fort.shooterVelReceive-shootSpeed) < SHOOT_ERR_2  && fabs(fort.yawPosReceive-shootTurnAngle) < 3)
			{
				// 推球	
				pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
				PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
				shootReady[2]=1;
				USART_OUT(UART4, "shoot\r\n");
			}
		}
		
		//打右下桶
		else if(shootFlagOne == 3)
		{
			shootFlag=3;
			
			//速度达到1300以上，有要打的球，转速和枪的角度达到所给值
			if(judgeSpeed > SPEED_ONE && isBallRight == 1 && shootReady[3] == 0 && fabs(fort.shooterVelReceive-shootSpeed) < SHOOT_ERR_2 && fabs(fort.yawPosReceive-shootTurnAngle) < 3)
			{
				// 推球	
				pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
				PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
				shootReady[3]=1;
				USART_OUT(UART4, "shoot\r\n");
			}
		}
		
		//打左下桶
		else if(shootFlagOne == 0)
		{
			shootFlag=0;
			
			//速度达到1300以上，有要打的球，转速和枪的角度达到所给值
			if(judgeSpeed > SPEED_ONE && isBallRight == 1 && shootReady[0] == 0 && fabs(fort.shooterVelReceive-shootSpeed) < SHOOT_ERR_2  && fabs(fort.yawPosReceive-shootTurnAngle) < 3)
			{
				// 推球	
				pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
				PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
				shootReady[0]=1;
				USART_OUT(UART4, "shoot\r\n");
			}
		}
		
		//打左上桶
		else if(shootFlagOne == 1)
		{
			shootFlag=1;

			//速度达到1300以上，有要打的球，转速和枪的角度达到所给值
			if(judgeSpeed > SPEED_ONE && isBallRight == 1 && shootReady[1] == 0 && fabs(fort.shooterVelReceive-shootSpeed) < SHOOT_ERR_2  && fabs(fort.yawPosReceive-shootTurnAngle) < 3)
			{
				// 推球	
				pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
				PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
				shootReady[1]=1;
				USART_OUT(UART4, "shoot\r\n");
			}
				
	
		}
		
	}
	
}


/**
* @brief 定点激光扫描
* @param none
* @retval none
* @attention 
*/

typedef enum{
	reachAngle,scan,scanSomething,judge
}laserScan;
void Laser_Aim(void)
{
	static int stage=0,stableflag=0,sureFlag=0,laserCnt=0;
	float yawangle=0,Width=0;
	static float addangle=-30,lastpulseA=0,lastpulseB=0,laser_A=0,laser_B=0,Agl_A=0,Agl_B=0,MaxLaservalue=0,add=0.2; 
	float laserAvalue=0,laserBvalue=0,bucketdistance=0;
	static uint8_t flg=0;
	bucketdistance=sqrt(pow((GetPosX()-X[shootFlag]),2)+pow((GetPosY()-Y[shootFlag]),2));

	
	laserAvalue=fort.laserAValueReceive*LASER_SCALE_A+LASER_INTERCEPT_A;
	laserBvalue=fort.laserBValueReceive*LASER_SCALE_B+LASER_INTERCEPT_B;

	
	switch(stage)
	{
		case reachAngle:
			laserCnt++;
			YawPosCtrl(shootTurnAngle+addangle);
			if(laserCnt>100)
			{
				laserCnt=0;
				stage++;
			}
			break;
			
		//扫描到突变后，算得物体宽度
		case scan:
			addangle=addangle+add;
			yawangle=shootTurnAngle+addangle;
			YawPosCtrl(yawangle);
			
			//右激光先突变减小，扫到挡板左侧
			if((lastpulseB-fort.laserBValueReceive)>200&&laserAvalue>bucketdistance-500&&laserAvalue<bucketdistance+500)
			{
				stableflag=1;
				laser_A=laserAvalue;
				Agl_A=fort.yawPosReceive;
			}
			if(stableflag)
			{
				if(fabs(lastpulseA-fort.laserAValueReceive)<50)
				{
					flg=1;
					laser_A=laserAvalue;
					stableflag=0;
				}
			}
			
			//扫到左侧后，右激光先突变增大，扫到右侧
			if((lastpulseA-fort.laserAValueReceive)<-200&&laserBvalue>bucketdistance-500&&laserBvalue<bucketdistance+500&&flg==1)
			{
				flg=0;
				laser_B=laserBvalue;
				Agl_B=fort.yawPosReceive;
			}
			lastpulseA=fort.laserAValueReceive;
			lastpulseB=fort.laserBValueReceive;
			
			if(addangle>39.5f)
			{
				stage=3;
			}
			
			if((!stableflag)&&laser_A&&laser_B)
				Width=sqrt(laser_A*laser_A+laser_B*laser_B-2*laser_A*laser_B*cos((Agl_B-Agl_A)*PI/180.0f));
			if(Width>350.0f&&Width<800.0f)
				stage=2;
			break;
		
		//物体宽度满足后，激光打在中间
		case scanSomething:
			laserCnt++;
			YawPosCtrl((Agl_A+Agl_B)/2-1);
			if(laserCnt==40)
			{
				laserCnt=0;
				if(laserAvalue>laserBvalue)
					MaxLaservalue=laserAvalue;
				else
					MaxLaservalue=laserBvalue;
				
				//中间比两边远则是挡板，否则回退6度继续扫描
				if(MaxLaservalue>laser_A&&MaxLaservalue>laser_B && fabs(MaxLaservalue-bucketdistance) < 400.0f)
				{
					stage++;
					sureFlag=1;
				}
				else
				{
					addangle-=6.0f;
					stage=1;
					Width=0;
					laser_A=0;
					laser_B=0;
				}
			}
			break;
		//判断是否扫描到挡板，是则打球，否则扫描角度增大，速度减慢
		case judge:
			laserCnt++;
			addangle=-30.0f;
			laser_A=0;
			laser_B=0;
			lastpulseA=0;
			lastpulseB=0;
			flg=0;
			//没扫到
			if(!sureFlag)
			{
				laserCnt=0;
				stage=0;
				changeFlg=0;
				shootReady[shootFlag]=2;
				Agl_A=0;
				Agl_B=0;
				MaxLaservalue=0;
				add=0.1f;
				addangle=-50;
			}
			//扫到
			else
			{

				stage=0;
				sureFlag=0;
				laserShootAngle[shootFlag]=(Agl_A+Agl_B)/2-1;
				laserShootDistance[shootFlag]=MaxLaservalue;
				laserShootFlg=1;
				Agl_A=0;
				Agl_B=0;
				MaxLaservalue=0;
			}
		
			break;
		default: stage=3; break;
	}
}






/**
* @brief 走投激光扫描
* @param none
* @retval none
* @attention 
*/
void Laser_Aim_Two(void)
{
	static int stage2=0,stableflag2=0,sureFlag2=0,laserCnt2=0;
	float yawangle2=0,Width2=0;
	static float addangle2=-15.0f,lastpulseA2=0,lastpulseB2=0,laser_A2=0,laser_B2=0,Agl_A2=0,Agl_B2=0; 
	float laserAvalue2=0,laserBvalue2=0,bucketdistance2=0;
	static uint8_t flg2=0;
	bucketdistance2=sqrt(pow((GetPosX()-X[shootFlag]),2)+pow((GetPosY()-Y[shootFlag]),2));
	
	laserAvalue2=fort.laserAValueReceive*2.48f+24.8f;
	laserBvalue2=fort.laserBValueReceive*2.48f+24.8f;
	
	switch(stage2)
	{
		case 0:
			laserCnt2++;
			YawPosCtrl(shootTurnAngle+addangle2);
			if(laserCnt2>100)
			{
				laserCnt2=0;
				stage2=1;
			}
			break;
			
		//扫到东西之后，判断物体宽度，宽度在范围内则扫到挡板，在30度范围内没扫到则继续走
		case 1:
			laserCnt2=0;
			addangle2=addangle2+0.2f;
			yawangle2=shootTurnAngle+addangle2;
			YawPosCtrl(yawangle2);
			
			//右激光先突变减小，扫到挡板左侧
			if((lastpulseB2-fort.laserBValueReceive)>200&&laserAvalue2>bucketdistance2-400&&laserAvalue2<bucketdistance2+400)
			{
				stableflag2=1;
				laser_A2=laserAvalue2;
				Agl_A2=fort.yawPosReceive;
			}
			if(stableflag2)
			{
				if(fabs(lastpulseA2-fort.laserAValueReceive)<50)
				{
					flg2=1;
					laser_A2=laserAvalue2;
					stableflag2=0;
				}
			}
			
			//扫到左侧后，右激光先突变增大，扫到右侧
			if((lastpulseA2-fort.laserAValueReceive)<-200.0f&&laserBvalue2>bucketdistance2-500.0f&&laserBvalue2<bucketdistance2+500.0f&&flg2==1)
			{
				flg2=0;
				laser_B2=laserBvalue2;
				Agl_B2=fort.yawPosReceive;
			}
			lastpulseA2=fort.laserAValueReceive;
			lastpulseB2=fort.laserBValueReceive;
			
			if(addangle2>17)
			{
				stage2=2;
			}
			
			//物体宽度满足后，表示为挡板
			if((!stableflag2)&&laser_A2&&laser_B2)
			{
				Width2=sqrt(laser_A2*laser_A2+laser_B2*laser_B2-2*laser_A2*laser_B2*cos((Agl_B2-Agl_A2)*PI/180.0f));
				if(Width2>300.0f&&Width2<700.0f)
					sureFlag2=1;
				else
					sureFlag2=0;
				stage2=2;
			}
			break;
		case 2:
			laserCnt2++;
			addangle2=-15;
			laser_A2=0;
			laser_B2=0;
			lastpulseA2=0;
			lastpulseB2=0;
			//没扫到
			if(!sureFlag2)
			{
				shootCnt1=805;
				Agl_A2=0;
				Agl_B2=0;
				laserCnt2=0;
				stage2=0;
				sureFlag2=0;
				stopFlg=0;
			}
			//扫到
			else
			{
				YawPosCtrl((Agl_A2+Agl_B2)/2-1);
				if(laserCnt2 > 40)
				{
					laserCnt2=0;
					stage2=0;
					sureFlag2=0;
					Agl_A2=0;
					Agl_B2=0;
					laserStopShootFlg=1;
				}
			}
			break;
		default: stage2=2;break;
	}
}



extern uint8_t rightBall;
extern uint8_t wrongBall;
extern int32_t pushPos;
extern uint8_t ballColor;

/**
* @brief 车卡球处理
* @param none
* @retval none
* @attention 
*/
void BallStuck(void)
{
	static uint16_t ballStuckCnt=0;
	ballStuckCnt++;
	pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
	PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
	
}

/**
* @brief 球颜色识别
* @param none
* @retval none
* @attention 
*/
void BallColorRecognition(void)
{
	static uint16_t stuckCnt=0;
	static uint16_t noRightBallCnt=0;
	static uint16_t pushStableCnt=0;
	static uint32_t longTimeNoRightBallCnt=0;
	
	//等待推球电机稳定
	if(pushStableCnt > 100)
	{
		ReadActualPos(CAN2, PUSH_BALL_ID);
		
		//推球在进球位置
		if(pushPos > pushPulse-600 && pushPos < pushPulse+800)
		{
			stuckCnt=0;
			
			//要的球等待发射
			if(ballColor == rightBall)
			{
				noRightBallCnt=0;
				pushStableCnt=0;
				longTimeNoRightBallCnt=0;
				isBallRight= 1;
			}
			
			//不要的球和没有球0.75s推一次
			else
			{
				longTimeNoRightBallCnt++;
				noRightBallCnt++;
				if(noRightBallCnt > 150)
				{
					if(ballColor == NO_BALL || ballColor == wrongBall)
						pushPulse+=(-OTHER_COUNTS_PER_ROUND/2);
					PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
					noRightBallCnt=0;
					pushStableCnt=0;
				}
				isBallRight= 0;
			}
		}
		
		//卡球处理
		else
		{
			longTimeNoRightBallCnt++;
			noRightBallCnt=0;
				if(stuckCnt > 250)
				{
					BallStuck();
					stuckCnt=0;
					pushStableCnt=0;
					
				}
			stuckCnt++;
			isBallRight= 0;
		}
	}
	else
	{
		noRightBallCnt=0;
		pushStableCnt++;
	}
	
	//长时间没有要的球
	if(longTimeNoRightBallCnt > 1000)	
	{
		longTimeNoRightBallCnt=0;
		noRightBall=1;
		laserShootFlg=0;
	}
	
		
}





