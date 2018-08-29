/**
  ******************************************************************************
  * @file     
  * @author  lxy and qzj and xfr
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
#include "string.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_it.h"
#include  <ucos_ii.h>
#include "timer.h"
#include "pps.h"
#include "fort.h"

/*告诉定位系统准备开始积分*/
static uint8_t ppsTalkOk = 0;
/*定位系统准备完毕开始发数*/
static uint8_t ppsReady = 0;
static PosSend_t posture={0};
/*定义定位系统返回值结构体*/
static Pos_t ppsReturn={0.f};

//四号车定位系统串口接受中断函数，更新频率200Hz
void USART3_IRQHandler(void)
{
		static uint8_t ch;
		static uint8_t count=0;
		static uint8_t i=0;
		OS_CPU_SR  cpu_sr;
		OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
		OSIntNesting++;
		OS_EXIT_CRITICAL();

		if(USART_GetITStatus(USART3, USART_IT_RXNE)==SET)   
		{
			USART_ClearITPendingBit(USART3,USART_IT_RXNE);
			ch=USART_ReceiveData(USART3);
			switch(count)
			{
				case 0:
					if(ch==0x0d||ch=='O')
						count++;
					else
						count=0;
				break;

				case 1:
					if(ch==0x0a)
					{
						i=0;
						count++;
					}
					else if(ch=='K')
					{
						ppsTalkOk=1;
						count=0;
					}
				else if(ch==0x0d);
				else
					count=0;
				break;

				case 2:
					posture.data[i]=ch;
					i++;
					if(i>=GET_PPS_DATA_NUM)
					{
						i=0;
						count++;
					}
				break;

				case 3:
					if(ch==0x0a)
						count++;
					else
						count=0;
				break;

				case 4:
					if(ch==0x0d)
					{
						SetOpsReady(1);
						/*传入定位系统返回的值*/
						SetAngle(posture.value[0]);
						SetSpeedX(posture.value[1]);
						SetSpeedY(posture.value[2]);
						SetX(posture.value[3]);
						SetY(posture.value[4]);
						SetWZ(posture.value[5]);

						/*定义的全局结构体变量可以在这里赋值*/
						//						=posture.value[0];
						//						=posture.value[1];
						//						=posture.value[2];
						//						=posture.value[3];
						//						=posture.value[4];
						//						=posture.value[5];
					}
					count=0;
				break;
				default:
					count=0;
				break;		 
			}
		}
		else
		{
			USART_ClearITPendingBit(USART3, USART_IT_PE);
			USART_ClearITPendingBit(USART3, USART_IT_TXE);
			USART_ClearITPendingBit(USART3, USART_IT_TC);
			USART_ClearITPendingBit(USART3, USART_IT_ORE_RX);
			USART_ClearITPendingBit(USART3, USART_IT_IDLE);
			USART_ClearITPendingBit(USART3, USART_IT_LBD);
			USART_ClearITPendingBit(USART3, USART_IT_CTS);
			USART_ClearITPendingBit(USART3, USART_IT_ERR);
			USART_ClearITPendingBit(USART3, USART_IT_ORE_ER);
			USART_ClearITPendingBit(USART3, USART_IT_NE);
			USART_ClearITPendingBit(USART3, USART_IT_FE);
			USART_ReceiveData(USART3);
		}
		
		OSIntExit();
}
/*告诉定位系统开始准备*/
void TalkOpsToGetReady(void)
{
	uint8_t i = 0;
	uint8_t tdata[4];

  tdata[0]='A';
  tdata[1]='T';
  tdata[2]='\r';
  tdata[3]='\n';
	
	ppsTalkOk=0;
	while(!ppsTalkOk)
	{
		delay_ms(1);
		for(i=0;i<4;i++)
		 USART_SendData(USART3,tdata[i]);	
	}
}



/*一直等待定位系统初始化完成*/
void WaitOpsPrepare(void)
{
	  /*告诉定位系统准备*/
		TalkOpsToGetReady();
		/*等待定位系统准备完成*/
		while(!GetOpsReady()){};
}

void SetOpsReady(uint8_t flag)
{
	ppsReady = flag;
}
uint8_t GetOpsReady(void)
{
	return ppsReady;
}


void SetAngle(float setValue)
{
	ppsReturn.ppsAngle = setValue;
}

void SetX(float setValue)
{
	ppsReturn.ppsX = setValue;
}

void SetY(float setValue)
{
	ppsReturn.ppsY = setValue;
}

void SetSpeedX(float setValue)
{
	ppsReturn.ppsSpeedX = setValue;
}

void SetSpeedY(float setValue)
{
	ppsReturn.ppsSpeedY = setValue;
}

void SetWZ(float setValue)
{
	ppsReturn.ppsWZ = setValue;
}

extern float *error;
/*返回定位系统的角度*/
float GetAngle(void)
{
	return ppsReturn.ppsAngle;
}
/*返回定位系统的X值*/
float GetX(void)
{
	return (0-ppsReturn.ppsX+*error);
}
/*返回定位系统的Y值*/
float GetY(void)
{
	return (0-ppsReturn.ppsY);
}
/*返回定位系统的X轴的速度*/
float GetSpeedX(void)
{
	return (0-ppsReturn.ppsSpeedX);
}
/*返回定位系统的Y轴的速度*/
float GetSpeedY(void)
{
	return (0-ppsReturn.ppsSpeedY);
}
/*返回定位系统的Z轴角速度值*/
float GetWZ(void)
{
	return (ppsReturn.ppsWZ);
}



/*  ********************************************给定位系统发数矫正，矫正X，Y，angle***************************************************** */
void CorrectX(float value)
{
	uint8_t i = 0;
	uint8_t tdata[8];
  union{
		float   val;
		uint8_t data[4];
	}valSend;

  tdata[0]='A';
  tdata[1]='X';
  tdata[6]='\r';
  tdata[7]='\n';
	
	valSend.val=(float)value;
  memcpy(tdata+2,valSend.data,4);
	
	ppsTalkOk=0;
	while(!ppsTalkOk)
	{
		delay_ms(1);
		for(i=0;i<8;i++)
		 USART_SendData(USART3,tdata[i]);	
	}
}

void CorrectY(float value)
{
	uint8_t i=0;
	uint8_t tdata[8];
  union{
		float   val;
		uint8_t data[4];
	}valSend;

  tdata[0]='A';
  tdata[1]='Y';
  tdata[6]='\r';
  tdata[7]='\n';
	
	valSend.val=(float)value;
  memcpy(tdata+2,valSend.data,4);
	
	ppsTalkOk=0;
	while(!ppsTalkOk)
	{
		delay_ms(1);
		for(i=0;i<8;i++)
		 USART_SendData(USART3,tdata[i]);	
	}
}

void CorrectAngle(float value)
{
	uint8_t	i = 0;

	if(value>180.f)
		value=value-360.f;
	else if(value<-180.f)
		value=value+360.f;
	
	uint8_t tdata[8];
  union{
		float   val;
		uint8_t data[4];
	}valSend;

  tdata[0]='A';
  tdata[1]='A';
  tdata[6]='\r';
  tdata[7]='\n';
	
	valSend.val=(float)value;
  memcpy(tdata+2,valSend.data,4);
	
	ppsTalkOk=0;
	while(!ppsTalkOk)
	{
		delay_ms(1);
		for(i=0;i<8;i++)
		 USART_SendData(USART3,tdata[i]);	
	}
}


/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
float ABS(float thing)
{
	if(thing > 0) return thing;
	else return (0-thing);
}
float Compare(float a1,float b1)
{
	if(a1>b1) return 1.0f;
	else return -1.0f;
}
float constrain(float amt, float high, float low) 
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

void UART4_OUT(PID_Value *pid_out)
{
	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetX());
	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetY());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->Angle_Set);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->Angle);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetWZ());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->vel);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->V);
	USART_OUT(UART4,(uint8_t*)"%d	", (int)(sqrt(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY())));
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)ReadShooterVel());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetShooterVelCommand());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)ReadYawPos());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetYawPosCommand());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)ReadLaserAValue());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)ReadLaserBValue());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->Angle);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)Get_Adc_Average(15,10));
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)Get_Adc_Average(14,10));
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetShooterVelCommand());
	USART_SendData(UART4,'\r');
	USART_SendData(UART4,'\n');
}

command commandRecieveData;
int buffer2 = 0;
char buffer1[10] = {0};
void bufferInit1()
{
	for(int i = 0; i < 20; i++)
	{
		buffer1[i] = 0;
	}
	buffer2 = 0;
}

void GetValueFromPC(u8 data)
{
	buffer1[buffer2] = data;
	buffer2++;
	if(buffer2 >= 10)
	{
		bufferInit1();
	}
	if(buffer1[buffer2 - 2] == '\r' && buffer1[buffer2 - 1] == '\n')
	{
		if(buffer2 > 2 &&  strncmp(buffer1,"PO",2) == 0)//接收航向位置
		{
			commandRecieveData.yawPosReceive1 = (float)((int)(buffer1[2]-0x30))*0+((int)(buffer1[3]-0x30))*100+((int)(buffer1[4]-0x30))*10+((int)(buffer1[4]-0x30));
		}
		else if(buffer2 > 2 &&  strncmp(buffer1,"VE",2) == 0)//接收发射电机转速
		{
			commandRecieveData.shooterVelReceive1 = (float)((int)(buffer1[2]-0x30))*1000+((int)(buffer1[3]-0x30))*100+((int)(buffer1[4]-0x30))*10+((int)(buffer1[4]-0x30));
		}
		bufferInit1();
	}
}

float GetYawPosCommand(void)
{
	return ((commandRecieveData.yawPosReceive1)/10);
}

float GetShooterVelCommand(void)
{
	return ((commandRecieveData.shooterVelReceive1)/10);
}
