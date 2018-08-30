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

extern uint8_t flagOne;
extern uint8_t flag;
void Shoot(uint8_t flg,uint16_t pushTime)
{
	static float bucketPosX[4]={BUCKET_ONE_X,BUCKET_TWO_X,BUCKET_THR_X,BUCKET_FOR_X};
	static float bucketPosY[4]={BUCKET_ONE_Y,BUCKET_TWO_Y,BUCKET_THR_Y,BUCKET_FOR_Y};	
	static uint8_t shootFlag=0;
	static uint8_t shootFlagLast=0;
	static uint16_t shootCnt=0;
	static uint8_t shootFlag2=0;
	
	float shootX=GetPosX();
	float shootY=GetPosY();
	float getAngle=GetAngle();
	float shootDistance=0;
	float shootSpeed=0;
	float shootAngle=0;
	float shootTurnAngle=0;

	shootCnt++;
	if(flg == 0)
	{
		if(flag >= 3)
		{
			shootCnt++;
			if(shootCnt == 150)
			{
				PosCrl(CAN1, 0x06,ABSOLUTE_MODE,4500);
			}
			else if(shootCnt == 300)
			{
				PosCrl(CAN1, 0x06,ABSOLUTE_MODE,5);
			}
			else if(shootCnt > 450)
			{
				shootCnt=0;
				
				if(shootFlag2 < 4)
				{
					shootFlag=shootFlag2;
					shootFlag2++;
				}
				else if(shootFlag2 < 8)
				{
					shootFlag=7-shootFlag2;
					shootFlag2++;
				}
				else
				{
					shootFlag2=0;
				}
			}
		}
		else
		{
			if(((flagOne <= 3) && (shootX <= 1400 && shootX >= -1400 && shootY < 3600 && shootY > 1000) && shootY < shootX) || (shootX > -1400  && shootY < 1000))
			{
				shootFlag=0;
				if(shootCnt == pushTime)
				{
					// 推球	
					PosCrl(CAN1, 0x06,ABSOLUTE_MODE,4500);
					shootCnt=0;
				}
				
			}
			else if(((flagOne <= 7 && flagOne >= 3) && (shootX <= 1400 && shootX >= -1400 && shootY < 3600 && shootY > 1000) && shootY < -shootX) || (shootX < -1400  && shootY < 3600))
			{
				shootFlag=1;
				if(shootCnt == pushTime)
				{
					// 推球	
					PosCrl(CAN1, 0x06,ABSOLUTE_MODE,5);
					shootCnt=0;
				}
			}
			else if(((flagOne <= 3) && (shootX <= 1400 && shootX >= -1400 && shootY < 3600 && shootY > 1000) && shootY > shootX) || (shootX < 1400  && shootY > 3600))
			{
				shootFlag=2;
				if(shootCnt == pushTime)
				{
					// 推球	
					PosCrl(CAN1, 0x06,ABSOLUTE_MODE,4500);
					shootCnt=0;
				}
			}
			else if(((flagOne <= 7 && flagOne >= 3) && (shootX <= 1400 && shootX >= -1400 && shootY < 3600 && shootY > 1000) && shootY > -shootX) || (shootX > 1400  && shootY > 1000))
			{
				shootFlag=3;
				if(shootCnt == pushTime)
				{
					// 推球	
					PosCrl(CAN1, 0x06,ABSOLUTE_MODE,5);
					shootCnt=0;
				}
			}
		}		
	}

	if(flg == 1)
	{
		if(flag >= 3)
		{
			shootCnt++;
			if(shootCnt == 150)
			{
				PosCrl(CAN1, 0x06,ABSOLUTE_MODE,4500);
			}
			else if(shootCnt == 300)
			{
				PosCrl(CAN1, 0x06,ABSOLUTE_MODE,5);
			}
			else if(shootCnt > 450)
			{
				shootCnt=0;
				if(shootFlag2 < 4)
				{
					shootFlag=shootFlag2;
					shootFlag2++;
				}
				else if(shootFlag2 < 8)
				{
					shootFlag=7-shootFlag2;
					shootFlag2++;
				}
				else
				{
					shootFlag2=0;
				}
			}
		}
		else
		{
			if(((flagOne <= 3) && (shootX <= 1500 && shootX >= -1500 && shootY < 3700 && shootY > 1000) && shootY < shootX) || (shootX < -1500  && shootY > 1000))
			{ 
				shootFlag=0;
				if(shootCnt == pushTime)
				{
					// 推球	
					PosCrl(CAN1, 0x06,ABSOLUTE_MODE,4500);
					shootCnt=0;
				}
				
			}
			else if(((flagOne <= 7 && flagOne >= 3) && (shootX <= 1500 && shootX >= -1500 && shootY < 3700 && shootY > 1000) && shootY < -shootX) || (shootX > -1500  && shootY > 3700))
			{
				shootFlag=1;
				if(shootCnt == pushTime)
				{
					// 推球	
					PosCrl(CAN1, 0x06,ABSOLUTE_MODE,5);
					shootCnt=0;
				}
			}
			else if(((flagOne <= 3) && (shootX <= 1500 && shootX >= -1500 && shootY < 3700 && shootY > 1000) && shootY > shootX) || (shootX > 1500  && shootY < 3700))
			{
				shootFlag=2;
				if(shootCnt == pushTime)
				{
					// 推球	
					PosCrl(CAN1, 0x06,ABSOLUTE_MODE,4500);
					shootCnt=0;
				}
			}
			else if(((flagOne <= 7 && flagOne >= 3) && (shootX <= 1500 && shootX >= -1500 && shootY < 3700 && shootY > 1000) && shootY > -shootX) || (shootX < 1500  && shootY < 1000))
			{
				shootFlag=3;
				if(shootCnt == pushTime)
				{
					// 推球	
					PosCrl(CAN1, 0x06,ABSOLUTE_MODE,5);
					shootCnt=0;
				}
			}
		}			
	}
	
	if(shootFlagLast != shootFlag)
	{
		shootCnt=0;
		shootFlagLast=shootFlag;
	}
		
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
	YawPosCtrl(shootTurnAngle-2);
	
	shootDistance=sqrt(((shootY-bucketPosY[shootFlag])*(shootY-bucketPosY[shootFlag]))+((shootX-bucketPosX[shootFlag])*(shootX-bucketPosX[shootFlag])));
	
	if(shootDistance < 4000 && shootDistance > 2300)
	{
		shootSpeed=(SHOOOT_KP*shootDistance)+24;
		
		ShooterVelCtrl(shootSpeed);
	}

}
