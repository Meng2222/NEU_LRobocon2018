/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include <ucos_ii.h>
#include "app_cfg.h"
#include <math.h>
#include "usart.h"
#include "timer.h"
#include "can.h"
#include "gpio.h"
#include "elmo.h"
#define car 4

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

void CAN1_RX0_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	uint8_t a[8];
	uint8_t len=8;
	CAN_RxMsg(CAN1,0,a,&len);
	CAN_ClearFlag(CAN1, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV1);
	OSIntExit();
}

/**
  * @brief  CAN2 receive FIFO0 interrupt request handler
  * @note
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	uint8_t a[8];
	uint8_t len=8;
	CAN_RxMsg(CAN2,0,a,&len);
	CAN_ClearFlag(CAN2, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN2, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN2, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN2, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV1);
	OSIntExit();
}

/*************定时器2******start************/
//每1ms调用一次

extern OS_EVENT *PeriodSem;

void TIM2_IRQHandler(void)
{
#define PERIOD_COUNTER 10

	//用来计数10次，产生10ms的定时器
	static uint8_t periodCounter = PERIOD_COUNTER;

	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{

		//实现10ms 发送1次信号量
		periodCounter--;
		if (periodCounter == 0)
		{
			OSSemPost(PeriodSem);
			periodCounter = PERIOD_COUNTER;
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	OSIntExit();
}
//OS_EVENT *PeriodSem;
////////////////////////定时器1发送消息定时中断////////////////////////////////
//void TIM1_UP_TIM10_IRQHandler(void)
//{
//	OS_CPU_SR cpu_sr;
//	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
//	OSIntNesting++;
//	OS_EXIT_CRITICAL();
//	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
//	{
//		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
//		OSSemPost (PeriodSem);
//	}
//	OSIntExit();
//}

void TIM8_UP_TIM13_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM8, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM5_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM3_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM4_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
	OSIntExit();
}

void UART4_IRQHandler(void)
{

	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{

		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	}
	OSIntExit();
}
/***************************试场调参数用蓝牙串口中断*****************************************************/
void USART1_IRQHandler(void)
{

	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	OSIntExit();
}

void USART2_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
	OSIntExit();
}

void USART6_IRQHandler(void) //更新频率200Hz
{
	static uint8_t ch;
	static union {
		uint8_t data[24];
		float ActVal[6];
	} posture;
	static uint8_t count = 0;
	static uint8_t i = 0;
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
		ch = USART_ReceiveData(USART6);
		switch (count)
		{
		case 0:
			if (ch == 0x0d)
				count++;
			else
				count = 0;
			break;

		case 1:
			if (ch == 0x0a)
			{
				i = 0;
				count++;
			}
			else if (ch == 0x0d)
				;
			else
				count = 0;
			break;

		case 2:
			posture.data[i] = ch;
			i++;
			if (i >= 24)
			{
				i = 0;
				count++;
			}
			break;

		case 3:
			if (ch == 0x0a)
				count++;
			else
				count = 0;
			break;

		case 4:
			if (ch == 0x0d)
			{

				posture.ActVal[0] = posture.ActVal[0];
				posture.ActVal[1] = posture.ActVal[1];
				posture.ActVal[2] = posture.ActVal[2];
				posture.ActVal[3] = posture.ActVal[3];
				posture.ActVal[4] = posture.ActVal[4];
				posture.ActVal[5] = posture.ActVal[5];
			}
			count = 0;
			break;

		default:
			count = 0;
			break;
		}
	}
	else
	{
		USART_ClearITPendingBit(USART6, USART_IT_PE);
		USART_ClearITPendingBit(USART6, USART_IT_TXE);
		USART_ClearITPendingBit(USART6, USART_IT_TC);
		USART_ClearITPendingBit(USART6, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART6, USART_IT_IDLE);
		USART_ClearITPendingBit(USART6, USART_IT_LBD);
		USART_ClearITPendingBit(USART6, USART_IT_CTS);
		USART_ClearITPendingBit(USART6, USART_IT_ERR);
		USART_ClearITPendingBit(USART6, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART6, USART_IT_NE);
		USART_ClearITPendingBit(USART6, USART_IT_FE);
		USART_ReceiveData(USART6);
	}
	OSIntExit();
}
//////////////////////////////////


float angle,posX,posY,avel;
extern int	isOKFlag;
void USART3_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	static uint8_t ch;  
		static union {   
	uint8_t data[24]; 
  float ActVal[6];  
	} posture;  
	static uint8_t count = 0;
  static uint8_t i = 0; 

	if(USART_GetITStatus(USART3,USART_IT_ORE_ER) ==SET)
  {   
	USART_ClearITPendingBit(USART3,USART_IT_ORE_ER); 
  USART_ReceiveData(USART3); 
	}    
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET) 
		{   USART_ClearITPendingBit(USART3, USART_IT_RXNE);  
			ch = USART_ReceiveData(USART3);  
			switch (count)  
				{   
					case 0:    
						if (ch == 0x0d)  count++;    
						else if(ch=='O') count=5; 
						else count = 0;break; 
 
					case 1:    
						if (ch == 0x0a)  
						{  
							i = 0;  
							count++;   
						} 
						else  count = 0; break; 
 
					case 2:    
						posture.data[i] = ch;    i++;
						if (i >= 24)
						{ 
							i = 0;
						  count++; 
						}
						break; 
					case 3:
						if (ch == 0x0a)count++;
   					else  count = 0;
					  break; 
 
					case 4:
						if (ch == 0x0d)
							{    
								angle =posture.ActVal[0] ;//角度 
								posture.ActVal[1] = posture.ActVal[1];
								posture.ActVal[2] = posture.ActVal[2];
								posX = posture.ActVal[3];//x
								posY = posture.ActVal[4];//y
								avel=posture.ActVal[5]; 
								SetXpos(posX);
                SetYpos(posY);
                SetAngle(angle);
							}
							count = 0;
							break;
					case 5:
						count = 0;
					  if(ch=='K') isOKFlag=1;
					    break;
					default:
						count = 0; 
						break;  
			}  }  
		else  
			{
   			USART_ClearITPendingBit(USART3, USART_IT_RXNE);
   			USART_ReceiveData(USART3);
			}       

	OSIntExit();
}
struct position
{
	float x;
	float y;
	float angle;
}pos_t;
void SetAngle(float val)
{
	pos_t.angle=val;
}

void SetXpos(float val)
{
	pos_t.x=val;
}

void SetYpos(float val)
{
	pos_t.y=val;
}
float GetXpos(void)
{
	return pos_t.x;
}

float GetYpos(void)
{
	return pos_t.y;
}

float GetAngle(void)
{
	return pos_t.angle;
}

/////////////////////////PID//////////////////////
float LastAngleErr=0,i=0;
int ChangeFlag=0;
extern int T;
float AnglePID(float Angle,float SetAngle)
{
	struct PID Ang;
	Ang.p=180;
	Ang.i=0;
	Ang.d=0;
	float err=0,pid=0,err1=0,err2=0;   ///////劣弧调节
	err1=Angle-SetAngle;
	if(err1>=0)
		err2=err1-360;
	else
		err2=360+err1;
	if(err1>180||err1<-180)
	{
		err=err2;
	}
	else
	{
		err=err1;
	}
	i+=err;
	pid=Ang.p*err+Ang.i*i+Ang.d*(err-LastAngleErr);
	LastAngleErr=err;
	return pid;
}

float xyPID(float distance)
{
	float distancepid;
	struct PID xy;
	xy.p=0.04;
	float errxy=0,pid=0,errangle=0,err2=0;
	int angle=0,xielv;
	
		distancepid=xy.p*distance;
	return distancepid;
}
extern float setangle;
int AngleChange(void)
{
	float Ang,x,y;
	#if Car==4
	Ang=GetAngle();
	x=GetXpos();
	y=GetYpos();
	#elif Car==1
	Ang=-GetAngle();
	y=-GetXpos();
	x=GetYpos();
	#endif

	int flag=1;
	if(y>=1750&&ChangeFlag==0&&setangle==0)
		ChangeFlag=1;
	else if(x<=-1750&&ChangeFlag==1&&setangle==90)
		ChangeFlag=2;
	else if(y<=250&&ChangeFlag==2&&(setangle==180||setangle==-180))
		ChangeFlag=3;
	else if(x>=-250&&ChangeFlag==3&&setangle==-90)
		ChangeFlag=0;
	else
		flag=0;
	return flag;
}



void UART5_IRQHandler(void)
{

	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
	{

		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
	}
	OSIntExit();
}

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
	while (1)
	{
	}
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{

	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{

	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{

	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}
