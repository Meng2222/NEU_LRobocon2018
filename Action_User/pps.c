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

/*告诉定位系统准备开始积分*/
static uint8_t ppsTalkOk = 0;
/*定位系统准备完毕开始发数*/
static uint8_t ppsReady = 0;
static PosSend_t posture={0};
/*定义定位系统返回值结构体*/
static Pos_t ppsReturn={0.f};


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
						SetAngle( posture.value[0]);
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


/*返回定位系统的角度*/
float GetAngle(void)
{
	return ppsReturn.ppsAngle;
}
/*返回定位系统的X值*/
float GetX(void)
{
	return ppsReturn.ppsX;
}
/*返回定位系统的Y值*/
float GetY(void)
{
	return ppsReturn.ppsY;
}
/*返回定位系统的X轴的速度*/
float GetSpeedX(void)
{
	return ppsReturn.ppsSpeedX;
}
/*返回定位系统的角度*/
float GetSpeedY(void)
{
	return ppsReturn.ppsSpeedY;
}
/*返回定位系统的Z轴角速度值*/
float GetWZ(void)
{
	return ppsReturn.ppsWZ;
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
