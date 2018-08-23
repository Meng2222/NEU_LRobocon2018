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


//对应的收发串口
#define USARTX UART5


FortType fort;



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
void ReadShooterVel(void)
{
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%d%s","Y","a","w","P",":",(int)fort.shooterVelReceive,".");
	while(fort.shooterVelReceive-1>=0)
	{fort.shooterVelReceive--;}
	fort.shooterVelReceive*=10000;
	USART_OUT(UART4,(uint8_t*)"%d  ",(int)fort.shooterVelReceive);	
}
void ReadYawPos(void)
{
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%d%s","Y","a","w","P",":",(int)fort.yawPosReceive,".");
	while(fort.yawPosReceive-1>=0)
	{fort.yawPosReceive--;}
	fort.yawPosReceive*=10000;
	USART_OUT(UART4,(uint8_t*)"%d  ",(int)fort.yawPosReceive);		
}
void ReadLaserAValue(void)
{
	fort.laserAValueReceive=2.4973*fort.laserAValueReceive+40;
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%d%s","A","a","d","c",":",(int)fort.laserAValueReceive,".");
	while(fort.laserAValueReceive-1>=0)
	{fort.laserAValueReceive--;}
	fort.laserAValueReceive*=10000;
	USART_OUT(UART4,(uint8_t*)"%d  ",(int)fort.laserAValueReceive);	
}
void ReadLaserBValue(void)
{
	fort.laserBValueReceive=2.4973*fort.laserBValueReceive+360;
	USART_OUT(UART4,(uint8_t*)"%s%s%s%s%s%d%s","B","a","d","c",":",(int)fort.laserBValueReceive,".");
	while(fort.laserBValueReceive-1>=0)
	{fort.laserBValueReceive--;}
	fort.laserBValueReceive*=10000;
	USART_OUT(UART4,(uint8_t*)"%d  ",(int)fort.laserBValueReceive);
}