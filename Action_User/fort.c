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


int32_t pushPulse=0;
int bufferI = 0;
char buffer[20] = {0};



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
	float shootSp;
}usartValue;


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
					Cmd.turn=(BUFF[2]-0x30)*100+(BUFF[3]-0x30)*10+(BUFF[4]-0x30)*1+(BUFF[5]-0x30)*0.1;
		}
		else if(buffI > 2 &&  strncmp(BUFF,"VE",2) == 0)//接收发射电机转速
		{
				for(int i = 0; i < 4; i++)
					Cmd.shoot=(BUFF[2]-0x30)*10+(BUFF[3]-0x30)*1+(BUFF[4]-0x30)*0.1+(BUFF[5]-0x30)*0.01;
		}
		for(int i = 0; i < 8; i++)
		{
			BUFF[i] = 0;
		}
			bufferI = 0;
	}
}

//发射的桶
uint8_t shootReady[4]={0,0,0,0};
uint8_t shootReadyflg=0;
uint8_t backshoot=0;
uint8_t shootFlag=0;	
static uint8_t isBallRight=0;	
//车走形所在轨道
uint8_t shootFlagOne;

//故障判断标志位 主要用于车卡在某一位置，即errFlg=4;
extern uint8_t errFlg;

/**
* @brief 炮台发射
* @param flg：车走行的标志位
* @param pushTime：切换轨道后推球计数时间
* @retval none
* @attention 
*/
static float bucketPosX[4]={BUCKET_ONE_X,BUCKET_TWO_X,BUCKET_THR_X,BUCKET_FOR_X};
static float bucketPosY[4]={BUCKET_ONE_Y,BUCKET_TWO_Y,BUCKET_THR_Y,BUCKET_FOR_Y};	

void Shoot(uint8_t flg)
{
	
	static uint8_t shootFlagLast=0;
	static float shootAngleLast=0;
	static float getAngleLast=0;
	static float shootTurnAngle=0;
	static uint8_t i=0;
	static float shootSpeed=0;
	static uint8_t judgeFlg=1;
	
	float errOne=0;
	float errTwo=0;
	float shootX=GetPosX();
	float shootY=GetPosY();
	float getAngle=GetAngle();
	float shootDistance=0;
	float shootAngle=0;
	float laserdistance=0; 
	
	isBallRight=BallColorRecognition();
	
	//判断车所在区域
	if(flg == 0)
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

	if(flg == 1)
	{
		//完全卡住
		if(errFlg >= 3)
		{
			CarStuck();
		}
		
		//正常发射
		else
		{
			NormalShootTwo();
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
	
	
	
	//区域改变，计数清0
	if(shootFlagLast != shootFlagOne)
	{	
		shootFlagLast=shootFlagOne;
		judgeFlg=1;
		if(i< 3)
		{
			i++;
		}
		else
			i=0;
	}
	
	//完全卡住发射
	if(errFlg >= 3)
	{
		YawPosCtrl(shootTurnAngle);
		
		laserdistance=(((fort.laserAValueReceive+fort.laserBValueReceive)/2)*LASER_SCALE)-200;
	
		shootDistance=sqrt(((shootY-bucketPosY[shootFlag])*(shootY-bucketPosY[shootFlag]))+((shootX-bucketPosX[shootFlag])*(shootX-bucketPosX[shootFlag])));
		if((shootDistance+100) > laserdistance && (shootDistance-100) < laserdistance)
			shootDistance=laserdistance;
		else;
		shootSpeed=(SHOOOT_KP*shootDistance)+36.5;
		ShooterVelCtrl(shootSpeed);
		
	}
	
	
	//正常发射
	else
	{
		
		if(flg == 0)
		{
			if(shootFlagOne < 8)
				YawPosCtrl(shootTurnAngle-33);
			else
				YawPosCtrl(shootTurnAngle-14);
		}
		else
		{
			if(shootFlagOne < 8)
				YawPosCtrl(shootTurnAngle+2);
			else
				YawPosCtrl(shootTurnAngle+2);
		}
		
		shootDistance=sqrt(((shootY-bucketPosY[shootFlag])*(shootY-bucketPosY[shootFlag]))+((shootX-bucketPosX[shootFlag])*(shootX-bucketPosX[shootFlag])));
		
		float speed=sqrt(GetSpeeedX()*GetSpeeedX()+GetSpeeedY()*GetSpeeedY());
		if(backshoot == 0)
		{
			if(shootFlagOne < 8)
				shootSpeed=(SHOOOT_KP*shootDistance)+32-speed*0.009;
			else
				shootSpeed=(SHOOOT_KP*shootDistance)+28-speed*0.0095;
		}
		else if(backshoot == 1)
		{
			shootSpeed=(SHOOOT_KP*shootDistance)+28+speed*0.0095;
		}
		
		ShooterVelCtrl(shootSpeed);
	}
	
	
	
	
	//检测是否打出球
//	if(judgeFlg == 1)
//	{
//		if(fort.shooterVelReceive < (shootSpeed-5))
//		{
//			shootReady[i]=1;
//			judgeFlg=0;
//		}
//		else
//		{
//			shootReady[i]=0;
//		}
//	}
//	USART_OUT(UART4, " %d\t", (int)shootSpeed);
//	USART_OUT(UART4, " %d\t", (int)fort.shooterVelReceive);
//	
	USART_OUT(UART4, " %d\t", (int)shootReady[0]);
	USART_OUT(UART4, " %d\t", (int)shootReady[1]);
	USART_OUT(UART4, " %d\t", (int)shootReady[2]);
	USART_OUT(UART4, " %d\t", (int)shootReady[3]);
}

/**
* @brief 车卡住处理
* @param none
* @retval none
* @attention 
*/
void CarStuck(void)
{
	static uint8_t shootFlag2=0;
	static uint16_t shootCnt=0;
	
	if(isBallRight == 1)
	{
		shootCnt++;
		if(shootCnt == 100)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);			
		}
		else if(shootCnt > 200)
		{
			shootCnt=0;
				
			if(shootReady[0] == 0 && shootReady[1] == 0 && shootReady[2] == 0 && shootReady[3] == 0)
			{
				if(shootFlag2 < 3)
				{
					shootFlag=shootFlag2+1;
					shootFlag2++;
				}
				else if(shootFlag2 < 5)
				{
					shootFlag=5-shootFlag2;
					shootFlag2++;
				}
				else
				{
					shootFlag2=0;
					shootFlag=0;
				}
			}
			else if(shootReady[0] == 0)
			{
				shootFlag=0;
				shootFlag2=shootFlag;
				shootReady[0] = 1;
			}
			else if(shootReady[1] == 0)
			{
				shootFlag=1;
				shootFlag2=shootFlag;
				shootReady[1] = 1;
			}
			else if(shootReady[2] == 0)
			{
				shootFlag=2;
				shootFlag2=shootFlag;
				shootReady[2] = 1;
			}
			else if(shootReady[3] == 0)
			{
				shootFlag=3;
				shootFlag2=shootFlag;
				shootReady[3] = 1;
			}
			else
			{
				shootReady[0] = 0;
				shootReady[1] = 0;
				shootReady[2] = 0;
				shootReady[3] = 0;
			}
		}
		
	}
}

/**
* @brief //正常发射1，车顺时针转时的正常发射
* @param getPushTime：推球时间
* @retval none
* @attention 
*/
extern uint8_t step;
void NormalShootOne(void)
{
	float D=0;
	if((shootFlagOne == 7 || shootFlagOne == 11) && shootReady[0] == 0)
	{
		shootFlag=0;
		backshoot=0;
		D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
		if(/*fabs(GetSpeeedX())>= 1600 && */isBallRight == 1 && D > 2000 && D < 3500)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			shootReady[0]=1;
		}
		
	}
	else if((shootFlagOne == 8 || shootFlagOne == 12) && shootReady[1] == 0)
	{
		shootFlag=1;
		backshoot=0;
		D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
		if(/*fabs(GetSpeeedY()) >= 1600 &&*/ (shootFlagOne == 12|| shootFlagOne == 8) &&isBallRight == 1 && D > 2000 && D < 3500)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			shootReady[1]=1;
		}
		
	}
	else if((shootFlagOne == 5 || shootFlagOne == 9) && shootReady[2] == 0)
	{
		shootFlag=2;
		backshoot=0;
		D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
		if(/*fabs(GetSpeeedX())>= 1600 && */shootFlagOne == 9 && isBallRight == 1 && D > 2000 && D < 3500)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			shootReady[2]=1;
		}
		else if(shootFlagOne == 5 &&  fabs(GetSpeeedX()) >= 1500 && isBallRight == 1 && D > 2000)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			shootReady[2]=1;
		}
	}
	else if((shootFlagOne == 6 || shootFlagOne == 10) && shootReady[3] == 0 )
	{
		shootFlag=3;
		backshoot=0;
		D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
		if(/*fabs(GetSpeeedY()) >= 1600 &&*/ shootFlagOne == 10 && isBallRight == 1 && D > 2000 && D < 3500)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			shootReady[3]=1;
		}
		else if(shootFlagOne == 6 && fabs(GetSpeeedY()) >= 1500 && isBallRight == 1 && D > 2000)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			shootReady[3]=1;
		}
	}
	
	
	
	else if((shootFlagOne == 8 || shootFlagOne == 12) && shootReady[0] == 0 && shootReady[1] == 1)
	{
		shootFlag=0;
		backshoot=1;
		D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
		if(/*fabs(GetSpeeedX())>= 1600 && */isBallRight == 1 && D > 2000 && D < 3500)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			shootReady[0]=1;
		}
	}
	else if(shootFlagOne == 9 && shootReady[1] == 0 && shootReady[2] == 1)
	{
		shootFlag=1;
		backshoot=1;
		D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
		if(/*fabs(GetSpeeedY()) >= 1600 &&*/isBallRight == 1 && D > 2000 && D < 3500)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			shootReady[1]=1;
		}
	}
	else if(shootFlagOne == 10 && shootReady[2] == 0 && shootReady[3] == 1)
	{
		shootFlag=2;
		backshoot=1;
		if(/*fabs(GetSpeeedY()) >= 1600 &&*/isBallRight == 1 && D > 2000 && D < 3500)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			shootReady[2]=1;
		}
	}
	else if(shootFlagOne == 11 && shootReady[3] == 0 && shootReady[0] == 1)
	{
		shootFlag=3;
		backshoot=1;
		D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
		if(/*fabs(GetSpeeedX())>= 1600 && */isBallRight == 1 && D > 2000 && D < 3500)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			shootReady[3]=1;
		}
	}
}

/**
* @brief //正常发射2，车逆时针转时的正常发射
* @param getPushTime：推球时间
* @retval none
* @attention 
*/

void NormalShootTwo(void)
{
	float D=0;
	
	if(shootFlagOne == 6 || shootFlagOne == 10)
	{ 
		shootFlag=0;
		D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
		if(shootFlagOne == 10 && fabs(GetSpeeedY()) >= 1400 && isBallRight == 1 && D > 2400 )
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
		}
		else if(shootFlagOne == 6 && fabs(GetSpeeedY()) >= 1100 && isBallRight == 1 && D > 2000)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
		}
		
	}
	else if(shootFlagOne == 5 || shootFlagOne == 9)
	{ 
		shootFlag=1;
		D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
		if(shootFlagOne == 9 && fabs(GetSpeeedX()) >= 1400 && isBallRight == 1 && D > 2400)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
		}
		else if(shootFlagOne == 5 && fabs(GetSpeeedX()) >= 1100 && isBallRight == 1 && D > 2000)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
		}
	}
	else if(shootFlagOne == 2 || shootFlagOne == 8 || shootFlagOne == 12)
	{
		
		shootFlag=2;
		D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
		if((shootFlagOne == 12|| shootFlagOne == 8) && fabs(GetSpeeedY()) >= 1500 && isBallRight == 1 && D > 2400)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
		}
		else if(shootFlagOne == 4 && fabs(GetSpeeedY()) >= 1100 && isBallRight == 1 && D > 2400)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
		}
	}
	else if(shootFlagOne == 7 || shootFlagOne == 11)
	{
		shootFlag=3;
		D=sqrt(((GetPosY()-bucketPosY[shootFlag])*(GetPosY()-bucketPosY[shootFlag]))+((GetPosX()-bucketPosX[shootFlag])*(GetPosX()-bucketPosX[shootFlag])));
		if(fabs(GetSpeeedX()) >= 1400 && isBallRight == 1 && D > 2400)
		{
			// 推球	
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
		}
	}
}


/**
* @brief 球颜色识别
* @param none
* @retval none
* @attention 
*/
extern int32_t pushPos;
extern uint8_t ballColor;
uint8_t BallColorRecognition(void)
{
	static uint16_t ballCnt1=0;
	static uint16_t ballCnt2=0;
	ReadActualPos(CAN2, PUSH_BALL_ID);
	if(pushPos > pushPulse-600 && pushPos < pushPulse+300)
	{
		ballCnt2++;
		if(ballColor == RIGHT_BALL)
		{
			ballCnt2=0;
			return 1;
		}
		else if(ballColor == WRONG_BALL && ballCnt2 > 70)
		{
			pushPulse+=(-OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			ballCnt2=0;
			return 0;
			
		}
		else
		{
			if(ballCnt2 == 200)
			{
				pushPulse+=(-OTHER_COUNTS_PER_ROUND/2);
				PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
				
				
			}
			else if(ballCnt2 > 400)
			{
				pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
				PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
				ballCnt2=0;
			}
			return 0;
		}
	}
	else
	{
		
		ballCnt2=0;
		if(ballCnt1 > 200 && ballColor == WRONG_BALL)
		{
			pushPulse+=(-OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			ballCnt1=0;
			
		}
		else if(ballCnt1 > 200 && ballColor == RIGHT_BALL)
		{
			pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
			PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			ballCnt1=0;
		}
		else
		{
			if(ballCnt1 == 200)
			{
				pushPulse+=(OTHER_COUNTS_PER_ROUND/2);
				PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
			}
			else if(ballCnt1 > 400)
			{
				pushPulse+=(-OTHER_COUNTS_PER_ROUND/2);
				PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,pushPulse);
				ballCnt1=0;
			}
		}
		ballCnt1++;
		return 0;
	}
		
		
}

