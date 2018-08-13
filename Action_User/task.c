#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "movebase.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "moveBase.h"
#define PAI 3.14
int car=4;
int Angle=0;
int X=0;
int Y=0;
static int isOKFlag=0;
void driveGyro(void)
{
	while(!isOKFlag){
		delay_ms(5);
		USART_SendData(USART3,'A');
		USART_SendData(USART3,'T');
		USART_SendData(USART3,'\r');
		USART_SendData(USART3,'\n');
	}
isOKFlag=0;
}
/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
int car_cnt=0;
static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
/*
==================================================================================
						自定义添加函数
*/
void  go_straight(float V,int ElmoNum1,int ElmoNum2 ,CAN_TypeDef* CANx);
void  go_round(float V,float R,int ElmoNum1,int ElmoNum2 ,CAN_TypeDef* CANx);
void straight(float setValue,float feedbackValue);
void turn(float setValue,float feedbackValue);
  	   typedef struct{
	float x;
	float y;
	float angle;
	}pos_t;
pos_t action;
/*
==================================================================================
*/
void App_Task()
{
	CPU_INT08U os_err;
	os_err = os_err; /*防止警告...*/

	/*创建信号量*/
	PeriodSem = OSSemCreate(0);

	/*创建任务*/
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, /*初始化任务*/
						  (void *)0,
						  (OS_STK *)&App_ConfigStk[Config_TASK_START_STK_SIZE - 1],
						  (INT8U)Config_TASK_START_PRIO);

	os_err = OSTaskCreate((void (*)(void *))WalkTask,
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  (INT8U)Walk_TASK_PRIO);
}

/*
   ===============================================================
   初始化任务
   ===============================================================
   */
static int opsflag=0;
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM_Init(TIM2, 999, 839, 0x00, 0x00);
	//CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	//驱动器初始化
	ElmoInit( CAN2);
	//速度环和位置环初始化
	//右轮
	VelLoopCfg(CAN2, 1, 100, 100);
	//PosLoopCfg(CAN2, 1, 100, 100,0);
	//左轮
	VelLoopCfg(CAN2, 2, 100, 100);
	//PosLoopCfg(CAN2, 2, 100, 100,0);
	//电机使能
	MotorOn(CAN2, 01);
	MotorOn(CAN2, 02);
	//定位系统串口初始化
     USART3_Init(115200);
	 //蓝牙串口
	 UART4_Init(921600); 
	 delay_s(5);
     if(car==4|car==3)
	 delay_s(10);
	 
	 if(car==1)
	 {
	 driveGyro();
	 while(!opsflag)
	 delay_s(5);

	 }
	 OSTaskSuspend(OS_PRIO_SELF);

}

int mission=1;

void WalkTask(void)
{
	extern float Uk_1,Uk_2;
	int uk_1;
	int uk_2;
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{

		OSSemPend(PeriodSem, 0, &os_err);
		if(car==1)
	{
			//任务切换		
		if(action.angle>=-2&&action.angle<=2&&(mission==12||mission==2))
			mission=1;//直行________Uk取反
		if(((action.angle>2&&action.angle<180)|(action.angle<-2&&action.angle>-179))&&mission==1)
			mission=2;//纠偏
		if(action.x>=-2050&&action.x<=-1950&&mission==1)
			mission=3;//转向
		
		
		if(action.angle>=88&&action.angle<=92&&(mission==3||mission==5))
			mission=4;//直行——————UK取反
		if(((action.angle>=-180&&action.angle<88)|(action.angle>92&&action.angle<=180)|(action.angle<=0&&action.angle>=-180))&&mission==4)
			mission=5;//纠偏
		if(action.y>=-2050&&action.y<=-1950&&mission==4)
			mission=6;//转向
		
		
		if(((action.angle>=178&&action.angle<=180)|(action.angle<=-178&&action.angle>=-179))&&(mission==6||mission==8))
		    mission=7;//直行
		if((action.angle<177&action.angle>-178)&&mission==7)
			mission=8;//纠偏
		if(action.x>=-50&&action.x<=50&&mission==7)
			mission=9;//转向
		
		
		if((action.angle<=-88&&action.angle>=-92)&&mission==9)
			mission=10;//直行
		if(((action.angle>-88&&action.angle<=180)|(action.angle>-92&&action.angle<=-180))&&(mission==10))
		    mission=11;//纠偏
		if(action.y>=-50&&action.y<=50&&mission==10)
			mission=12;//转向
		
       //第一阶段
		if(mission==1)//直行（目标：-2000，0）
		{
			straight(-2000,action.x);
			
		}
		if(mission==2)//纠偏维持角度为0
		{
			turn(0,action.angle);
			
		}
		if(mission==3)//转向到90
		{
			turn(90,action.angle);
			
		}
		
		
		
		//第二阶段
		if(mission==4)//直行（目标：-2000,-2000）
		{
			straight(-2000,action.y);
			
		}
		if(mission==5)//纠偏 维持角度为90
		{
			turn(90,action.angle);
			
		}
		if(mission==6)//转向到180
		{
			turn(180,action.angle);
		
		}
		
		//第三阶段
		if(mission==7)//直行（目标：0,2000）
		{
			straight(0,action.x);
		
		}
		if(mission==8)//纠偏 维持角度为180
		{
			turn(180,action.angle);
		
		}
		if(mission==9)//转向到-90
		{
			turn(-90,action.angle);
		
		}
		
		//第四阶段
		if(mission==10)//直行（目标：0,0）
		{
			straight(0,action.y);
		
		}
		if(mission==11)//纠偏 维持角度为-90
		{
			turn(-90,action.angle);
		
		}
		if(mission==12)//转向到0
		{
			turn(0,action.angle);
		
		}		
	//	cnt++;
//蓝牙发送
        
		Angle=(int)action.angle;
		X=(int)action.x;
		Y=(int)action.y;
	//	if(cnt==100)
		{	
		USART_OUT(UART4,(uint8_t*)"Angle=");
		USART_OUT( UART4, (uint8_t*)"%d ", Angle);
		USART_OUT(UART4,(uint8_t*)"X=");
		USART_OUT( UART4, (uint8_t*)"%d ", X);
		USART_OUT(UART4,(uint8_t*)"Y=");
		USART_OUT( UART4, (uint8_t*)"%d ", Y);
			
		USART_OUT(UART4,(uint8_t*)"mission=");
		USART_OUT( UART4, (uint8_t*)"%d ", mission);	
			
		uk_1=(int)Uk_1;
		USART_OUT(UART4,(uint8_t*)"angleUk=");
		USART_OUT( UART4, (uint8_t*)"%d ", uk_1);
		uk_2=(int)Uk_2;
		USART_OUT(UART4,(uint8_t*)"dietanceUk=");
		USART_OUT( UART4, (uint8_t*)"%d ", uk_2);
		USART_OUT(UART4,(uint8_t*)"\r\n");
//		cnt=0;	
		}
	}
      if(car==4)
  {
		if(action.angle>=-2&&action.angle<=2&&(mission==12||mission==2))
			mission=1;//直行
		if(((action.angle>2&&action.angle<180)||(action.angle<-2&&action.angle>-179))&&mission==1)
			mission=2;//纠偏
		if(action.y<=2020&&action.y>=1980&&mission==1)
			mission=3;//转向
		
		
		if(action.angle>=88&&action.angle<=98&&(mission==3||mission==5))
			mission=4;//直行
		if(((action.angle>=-179&&action.angle<88)||(action.angle>92&&action.angle<=180)||(action.angle<=0&&action.angle>=-180))&&mission==4)
			mission=5;//纠偏
		if(action.x<=-1980&&action.x>=-2020&&mission==4)
			mission=6;//转向
		
		
		if(((action.angle>=178&&action.angle<=180)||(action.angle<=-178&&action.angle>=-179))&&(mission==6||mission==8))
		    mission=7;//直行
		if((action.angle<178&action.angle>-178)&&mission==7)
			mission=8;//纠偏
		if(action.y>=-20&&action.y<=20&&mission==7)
			mission=9;//转向
		
		
		if((action.angle<=-88&&action.angle>=-98)&&(mission==9||mission==11))
			mission=10;//直行
		if(((action.angle>-88&&action.angle<=180)||(action.angle>-98&&action.angle<=-180))&&(mission==10))
		    mission=11;//纠偏
		if(action.x>=-20&&action.x<=20&&mission==10)
			mission=12;//转向

        //第一阶段
		if(mission==1)//直行（目标：0，2000）
		{
			straight(2000,action.y);
			
		}
		if(mission==2)//纠偏维持角度为0
		{
			turn(0,action.angle);
			
		}
		if(mission==3)//转向到90
		{
			turn(90,action.angle);
			
		}
		
		//第二阶段
		if(mission==4)//直行（目标：-2000,2000）
		{
			straight(-2000,action.x);
			
		}
		if(mission==5)//纠偏 维持角度为90
		{
			turn(90,action.angle);
			
		}
		if(mission==6)//转向到180
		{
			turn(180,action.angle);
		
		}
		
		//第三阶段
		if(mission==7)//直行（目标：-2000,0）
		{
			straight(0,action.y);
		
		}
		if(mission==8)//纠偏 维持角度为180
		{
			turn(180,action.angle);
		
		}
		if(mission==9)//转向到-90
		{
			turn(-90,action.angle);
		
		}
		
		//第四阶段
		if(mission==10)//直行（目标：0,0）
		{
			straight(0,action.x);
		
		}
		if(mission==11)//纠偏 维持角度为-90
		{
			turn(-90,action.angle);
		
		}
		if(mission==12)//转向到0
		{
			turn(0,action.angle);
		
		}		
//		cnt++;
//蓝牙发送
        
		Angle=(int)action.angle;
		X=(int)action.x;
		Y=(int)action.y;
	//	if(cnt==100)
		{	
		//USART_OUT(UART4,(uint8_t*)"Angle=");
		//USART_OUT( UART4, (uint8_t*)"%d ", Angle);
	//	USART_OUT(UART4,(uint8_t*)"X=");
		USART_OUT( UART4, (uint8_t*)"%d ", X);
	//	USART_OUT(UART4,(uint8_t*)"Y=");
		USART_OUT( UART4, (uint8_t*)"%d ", Y);
			
	//	USART_OUT(UART4,(uint8_t*)"mission=");
		//USART_OUT( UART4, (uint8_t*)"%d ", mission);	
			
	//	uk_1=(int)Uk_1;
		//USART_OUT(UART4,(uint8_t*)"angleUk=");
		//USART_OUT( UART4, (uint8_t*)"%d ", uk_1);
		//uk_2=(int)Uk_2;
		//USART_OUT(UART4,(uint8_t*)"dietanceUk=");
		//USART_OUT( UART4, (uint8_t*)"%d ", uk_2);
		USART_OUT(UART4,(uint8_t*)"\r\n");
	//	cnt=0;	
		}
	}
		if(car==3)
	{
			
			
			//if(car_cnt<=100&&car>=50)
		//	turn(0,action.angle);
			straight(2000,action.y);
		if(((action.angle>1&&action.angle<180)||(action.angle<-1&&action.angle>-179)))
			turn(0,action.angle);
//			car_cnt++;
//		if(car_cnt>0&&car_cnt<=100)
//			turn(-90,action.angle);
//		if(car_cnt>100&&car_cnt<=200)
//			turn(180,action.angle);
//		if(car_cnt>200&&car_cnt<=300)
//			turn(0,action.angle);
//		if(car==400)
//			car_cnt=0;
//--------------------------------------------------------------------------------------			
//			if(car_cnt>200&&car_cnt<=300)
//			turn(180,action.angle);
//			
//			if(car_cnt>300&&car_cnt<400)
//			turn(-90,action.angle);	
//			
//			if(car_cnt==400)
//			car_cnt=0;	
//			if(car_cnt%5==0)
          if(!(action.y<20&&action.y>-20))		  
		{	
		Angle=(int)action.angle;
		X=(int)action.x;
		Y=(int)action.y;
		USART_OUT(UART4,(uint8_t*)"Angle=");
		USART_OUT( UART4, (uint8_t*)"%d ", Angle);
		USART_OUT(UART4,(uint8_t*)"X=");
		USART_OUT( UART4, (uint8_t*)"%d ", X);
		USART_OUT(UART4,(uint8_t*)"Y=");
		USART_OUT( UART4, (uint8_t*)"%d ", Y);
		uk_1=(int)Uk_1;
		USART_OUT(UART4,(uint8_t*)"angleUk=");
		USART_OUT( UART4, (uint8_t*)"%d ", uk_1);
		uk_2=(int)Uk_2;
		USART_OUT(UART4,(uint8_t*)"dietanceUk=");
		USART_OUT( UART4, (uint8_t*)"%d ", uk_2);
		USART_OUT(UART4,(uint8_t*)"\r\n");					
	  }
	}

	
 }
}




void USART3_IRQHandler(void) //更新频率 200Hz 
{  
 static uint8_t ch; 

 static union 
{   
	uint8_t data[24];   
    float ActVal[6]; 
 } posture;  
    static uint8_t count = 0;  
    static uint8_t i = 0; 
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL(); 
 if(USART_GetITStatus(USART3,USART_IT_ORE_ER)==SET)  
{  
USART_ClearITPendingBit(USART3,USART_IT_ORE_ER); 
USART_ReceiveData(USART3); 
}  
if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)  
	{  
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);  
		ch = USART_ReceiveData(USART3);  
		switch (count) 
			{   
				case 0:  
			if (ch == 0x0d)    
				count++;  
			else if(ch=='O')   
				count=5;  
			else    
				count = 0; 
			break; 
 
  case 1:   
 if (ch == 0x0a)  
  {  
  i = 0;  
  count++; 
  } 
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
 
  case 3:    if (ch == 0x0a)     
	  count++;   
  else     count = 0;    break; 
 
  case 4:    if (ch == 0x0d)    
{   
 opsflag=1;
 action.angle =posture.ActVal[0] ;//角度 
 posture.ActVal[1] = posture.ActVal[1];     
posture.ActVal[2] = posture.ActVal[2];    
action.x = posture.ActVal[3];//x     
action.y = posture.ActVal[4];//y       
}  
  count = 0;   
  break;   
  case 5:    
  count = 0;  
  if(ch=='K')    
  isOKFlag=1;    
  break;      
  default:  
   count = 0; 
   break;   
  }  
  }  
  else 
 { 
  USART_ClearITPendingBit(USART3, USART_IT_RXNE);   
  USART_ReceiveData(USART3); 
  }
OSIntExit();  
  } 
