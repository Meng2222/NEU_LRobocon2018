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



int bufferI = 0;
char buffer[20] = {0};

//未发射的桶
uint8_t notShoot[2]={0};

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

static uint16_t shootCnt=0;
static uint8_t shootFlag=0;	
static uint8_t shootFlag2=0;

extern uint8_t flagOne;
extern uint8_t errFlg;

void Shoot(uint8_t flg,uint16_t pushTime)
{
	static float bucketPosX[4]={BUCKET_ONE_X,BUCKET_TWO_X,BUCKET_THR_X,BUCKET_FOR_X};
	static float bucketPosY[4]={BUCKET_ONE_Y,BUCKET_TWO_Y,BUCKET_THR_Y,BUCKET_FOR_Y};	
	
	static uint8_t shootFlagLast=0;

	static uint16_t judgeCnt=0;

	
	static uint8_t i=0;
	static float shootSpeed=0;
	static uint8_t judgeFlg=0;
		
	float shootX=GetPosX();
	float shootY=GetPosY();
	float getAngle=GetAngle();
	float shootDistance=0;
	float shootAngle=0;
	float shootTurnAngle=0;

	shootCnt++;
	//判断是否发出球
	if( shootCnt == pushTime)
	{
		judgeFlg=1;
	}
	else;
	
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
			NormalShootOne(pushTime);
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
			NormalShootTwo(pushTime);
		}			
	}
	

	//枪定位桶角度
	if(shootFlag == 2 || shootFlag == 3)
	{
		shootAngle=260-(atan((shootY-bucketPosY[shootFlag])/(shootX-bucketPosX[shootFlag]))*180/PI);
		shootTurnAngle=getAngle+shootAngle;
		if(shootTurnAngle > 350 && shootTurnAngle <= 360)
		{
			shootTurnAngle=350;
		}
		else if(shootTurnAngle > 360) 
		{
			shootTurnAngle=shootTurnAngle-360;
		}
		else if(shootTurnAngle < 0 && shootTurnAngle >= -10)
		{
			shootTurnAngle=0;
		}
	}
	else if(shootFlag == 0 || shootFlag == 1)
	
	{
		shootAngle=80-(atan((shootY-bucketPosY[shootFlag])/(shootX-bucketPosX[shootFlag]))*180/PI);
		shootTurnAngle=getAngle+shootAngle;
		if(shootTurnAngle > 350 && shootTurnAngle <= 360)
		{
			shootTurnAngle=350;
		}
		else if(shootTurnAngle < -10)
		{
			shootTurnAngle=shootTurnAngle+360;
		}
		else if(shootTurnAngle < 0 && shootTurnAngle >= -10)
		{
			shootTurnAngle=0;
		}
	}
	
	//区域改变，计数清0
	if(shootFlagLast != shootFlag)
	{
		if(errFlg >= 3)
			YawPosCtrl(shootAngle);	
		
		shootCnt=0;
		judgeCnt=0;
		shootFlagLast=shootFlag;
		i=!i;
	}
	
	//完全卡住发射
	if(errFlg >= 3)
	{
		YawPosCtrl(shootTurnAngle);
		shootDistance=sqrt(((shootY-bucketPosY[shootFlag])*(shootY-bucketPosY[shootFlag]))+((shootX-bucketPosX[shootFlag])*(shootX-bucketPosX[shootFlag])));
		shootSpeed=(SHOOOT_KP*shootDistance)+35.7;
		ShooterVelCtrl(shootSpeed);
	}
	
	
	//正常发射
	else
	{
		if(flg == 0)
		{
			if(flagOne < 8)
				YawPosCtrl(shootTurnAngle-2);
			else
				YawPosCtrl(shootTurnAngle);
		}
		else
		{
			if(flagOne < 8)
				YawPosCtrl(shootTurnAngle+4);
			else
				YawPosCtrl(shootTurnAngle+6);
		}
		
		shootDistance=sqrt(((shootY-bucketPosY[shootFlag])*(shootY-bucketPosY[shootFlag]))+((shootX-bucketPosX[shootFlag])*(shootX-bucketPosX[shootFlag])));
		
		//激光校准
		LaserCalibration(shootDistance);
		
		if(shootDistance < 4000 && shootDistance > 2300)
		{
			if(flagOne < 8)
				shootSpeed=(SHOOOT_KP*shootDistance)+18;
			else
				shootSpeed=(SHOOOT_KP*shootDistance)+19;
			
			ShooterVelCtrl(shootSpeed);
			usartValue.shootSp=shootSpeed;
		}
	}
	
	
	
	//检测是否打出球
	if(judgeFlg == 1)
	{
		if(judgeCnt < 100)
		{
			judgeCnt++;
			if(fort.shooterVelReceive < (shootSpeed-5));
			else 
			{
				notShoot[i]=shootFlag;
			}
		}
		else
		{
			judgeFlg=0;
			judgeCnt=0;
		}
	}
	
}

//车卡住处理
void CarStuck(void)
{
	if(shootCnt == 150)
	{
		PosCrl(CAN2, 7,ABSOLUTE_MODE,4500);
	}
	else if(shootCnt == 300)
	{
		PosCrl(CAN2, 7,ABSOLUTE_MODE,5);
	}
	else if(shootCnt > 450)
	{
		shootCnt=0;
		if(notShoot[0] == 0 && notShoot[1] == 0)
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
		else if(notShoot[0] > 0)
		{
			shootFlag=notShoot[0];
			notShoot[0]=0;
		}
		else if(notShoot[1] > 0)
		{
			shootFlag=notShoot[1];
			notShoot[1]=0;
		}
		
	}
}

//正常发射1
void NormalShootOne(uint16_t getPushTime)
{
	if(flagOne == 7 || flagOne == 11)
	{
		shootFlag=0;
		
		if(flagOne == 7)
			getPushTime=getPushTime-TIME_DIFF_1_ONE;
		else;
		
		if(shootCnt == getPushTime)
		{
			// 推球	
			PosCrl(CAN1, 0x06,ABSOLUTE_MODE,4500);
			shootCnt=0;
		}
		
	 }
	else if(flagOne == 4 || flagOne == 8 || flagOne == 12)
	{
		shootFlag=1;
		
		if(flagOne == 4)
			getPushTime=getPushTime-TIME_DIFF_1_TWO;
		else 
			getPushTime=getPushTime;
		
		if(shootCnt == getPushTime)
		{
			// 推球	
			PosCrl(CAN1, 0x06,ABSOLUTE_MODE,5);
			shootCnt=0;
		}
	}
	else if(flagOne == 5 || flagOne == 9)
	{
		shootFlag=2;
		
		if(flagOne == 5)
			getPushTime=getPushTime-TIME_DIFF_1_THR;
		else;
		
		if(shootCnt == getPushTime)
		{
			// 推球	
			PosCrl(CAN1, 0x06,ABSOLUTE_MODE,4500);
			shootCnt=0;
		}
	}
	else if(flagOne == 6 || flagOne == 10)
	{
		shootFlag=3;
		
		if(flagOne == 6)
			getPushTime=getPushTime-TIME_DIFF_1_FOR;
		
		if(shootCnt == getPushTime)
		{
			// 推球	
			PosCrl(CAN1, 0x06,ABSOLUTE_MODE,5);
			shootCnt=0;
		}
	}
}

//正常发射2
void NormalShootTwo(uint16_t getPushTime)
{
	if(flagOne == 6 || flagOne == 10)
	{ 
		shootFlag=0;
		if(flagOne == 6)
			getPushTime=getPushTime-TIME_DIFF_2_ONE;
		else;
		
		if(shootCnt == getPushTime)
		{
			// 推球	
			PosCrl(CAN1, 0x06,ABSOLUTE_MODE,4500);
			shootCnt=0;
		}
		
	}
	else if(flagOne == 5 || flagOne == 9)
	{ 
		shootFlag=1;
		if(flagOne == 5)
			getPushTime=getPushTime-TIME_DIFF_2_TWO;
		else;
		
		if(shootCnt == getPushTime)
		{
			// 推球	
			PosCrl(CAN1, 0x06,ABSOLUTE_MODE,5);
			shootCnt=0;
		}
	}
	else if(flagOne == 4 || flagOne == 8 || flagOne == 12)
	{
		shootFlag=2;
		
		if(flagOne == 4)
			getPushTime=getPushTime-TIME_DIFF_2_THR;
		else;
		
		if(shootCnt == getPushTime)
		{
			// 推球	
			PosCrl(CAN1, 0x06,ABSOLUTE_MODE,4500);
			shootCnt=0;
		}
	}
	else if(flagOne == 7 || flagOne == 11)
	{
		shootFlag=3;
		
		if(flagOne == 7)
			getPushTime=getPushTime-TIME_DIFF_2_FOR;
		else;
		
		if(shootCnt == getPushTime)
		{
			// 推球	
			PosCrl(CAN1, 0x06,ABSOLUTE_MODE,5);
			shootCnt=0;
		}
	}
}

//激光校准
float posXAdd=0;
float posYAdd=0;
void LaserCalibration(float getreferenceDistance)
{
	float laserdistance=0; 
	laserdistance=(((fort.laserAValueReceive+fort.laserBValueReceive)/2)*LASER_SCALE)-200;
	
	if(getreferenceDistance < 3800 && getreferenceDistance > 2300)
	{
		if((laserdistance+100) < getreferenceDistance || (laserdistance-100) > getreferenceDistance)
		{
			if(flagOne == 10 || flagOne == 12)
			{
				posXAdd=laserdistance-getreferenceDistance;
			}
			else if(flagOne == 9 || flagOne == 11)
			{
				posYAdd=laserdistance-getreferenceDistance;
			}
		}
	}
		

}

