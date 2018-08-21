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
#include "adc.h"
#include "pps.h"
#include "MRKSteven.h"

extern float v1_record,v2_record;
extern float v1,v2;               //两轮速度（2左 || 1右）

extern Place positionf;           //浮点 positionf (X、Y、Angle)
       Place positioni;           //整型 positioni (X、Y、Angle)
extern CAngle CangleLock;
extern CLine ClineLock;
extern CSquare CsquareLock;
extern CCircle CcircleLock;

extern float G_Adc_A4;
extern float G_Adc_A5;
	
extern float Angle_p10;  //10ms前角度值
extern float Angle_p20;  //20ms前角度值
float d_Angle;           //角度微分（10ms）
extern float x_p10;
extern float x_p20;
extern float y_p10;
extern float y_p20;

extern int square_edg;
extern int left,right;

//===================================================================================================================================================
//===================================================================================================================================================
// 
//                           信号量定义
//
//===================================================================================================================================================
//===================================================================================================================================================
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];

void App_Task()
{
	CPU_INT08U os_err;
	os_err = os_err; //防止警告...

	//创建信号量
	PeriodSem = OSSemCreate(0);
	//创建任务
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, //初始化任务
						  (void *)0,
						  (OS_STK *)&App_ConfigStk[Config_TASK_START_STK_SIZE - 1],
						  (INT8U)Config_TASK_START_PRIO);

	os_err = OSTaskCreate((void (*)(void *))WalkTask,    //运动指令
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  (INT8U)Walk_TASK_PRIO);
    OSTaskSuspend(OS_PRIO_SELF);
}

//===================================================================================================================================================
//===================================================================================================================================================
// 
//                                                           初始化任务
//
//===================================================================================================================================================
//===================================================================================================================================================

void ConfigTask(void)        //初始化
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM_Init(TIM2,1000-1,84-1,1,3);	//产生10ms中断，抢占优先级为1，响应优先级为3

	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	
	VelLoopCfg(CAN2,1, 5000, 5000);				//驱动器速度环初始化
	VelLoopCfg(CAN2,2, 5000, 5000);
	
	ElmoInit(CAN2);								//驱动器初始化
	MotorOn(CAN2,1);							//电机使能（通电）
	MotorOn(CAN2,2);
	
	//TIM4_Pwm_Init (9999,83);//pwm初始化（10ms）
	TIM_Init(TIM2, 999, 83, 0X01, 0X03);//TIM2 1ms中断

	
	UART4_Init(921600);
	
//	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);//CAN1通信（将CAN1时钟赋给CAN2）
	
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);//CAN2通信
	
	//Elmo驱动器初始化
	ElmoInit(CAN2);
	
	//驱动器速度环初始化
	VelLoopCfg(CAN2,1,40960000,40960000);
	VelLoopCfg(CAN2,2,40960000,40960000);

//	//驱动器位置环初始化
//	PosLoopCfg(CAN2,1,2048000,2048000,1024);
//	PosLoopCfg(CAN2,2,2048000,2048000,1024);
	
	//电机使能（通电）
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	
	Adc_Init();
	
	USART3_Init(115200);
	
	delay_ms(2000);
	//一直等待定位系统初始化完成
	WaitOpsPrepare();
	
	OSTaskSuspend(OS_PRIO_SELF);
}


	/////////////////////////// 
	//          |-y/         //
	//         0|0/          //
	//          |/           //
	// 90       /       -90  //
	// ————————/|—————————— >//
	//        / |          -x//
	//       /  |            //
	//      /180|-180        //
	//     /    |            //
	///////////////////////////

	/////////////////////////// 
	//         0|0           //
	//     【3】|y           //
	//     |<———|————|【2】  //
	//     |    |    |       //
	//     |    +d_M |       //
	//     |   r|    |       //
	//【4】|————|———>|       //
	// 90       |【1】  -90  //
	// —————————|—————————— >//
	//          |          x //
	//          |            //
	//       180|-180        //
	//          |            //
	///////////////////////////
	
//===================================================================================================================================================
//===================================================================================================================================================
 
//                                                         运动模块

//===================================================================================================================================================
//===================================================================================================================================================
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem,0,&os_err);
	OSSemPend(PeriodSem, 0, &os_err);
	while (1)
	{
			G_Adc_A4=Get_Adc_Average(14,10);
			G_Adc_A5=Get_Adc_Average(15,10);
			
			///////////////////////////////////
			OSSemPend(PeriodSem, 0, &os_err);//
			///////////////////////////////////	
			
			Coordinate_Reverse();//坐标反转（一定要放在while开始，OSSemPend后一行）	
			Position_Record();   //坐标记录、实时位置发送
			
			v1=(int)10865*v;
			v2=(int)10865*v;
		
		if(Mode==0)//测试状态
	    {
			USART_OUT(UART4,(uint8_t*)"%s %s %s %s\r\n","T","E","S","T");//mode0
			VelCrl(CAN2,1,256);//右轮
			VelCrl(CAN2,2,256);//左轮
		}
		
		if(Mode==1)                       //Mode1 直行（r=0）或圆周运动 前进/后退
		{Move_Basic();}
		
		if(Mode==2)                       //Mode2 直行（r=0）||多边形运动（带自动校正）（r为多边形边长；angle为多边形邻边角度） 
		{
			USART_OUT(UART4,(uint8_t*)"%s%s%s%s%d\r\n","M","o","d","e",2);//mode2
			if(r==0) //直行or正方形（附自动校正）
			{Angle_Lock4(CangleLock.m_angle_Target);}
			if(r!=0)//正方形//4车
			{Square_Movement();}
		}
		
		if(Mode==3)//蛇皮走位（直线闭环）
		{Line_Lock4(ClineLock.m_line_angle, ClineLock.m_line_yintercept, ClineLock.m_line_xintercept);}

		
		if(Mode==4)//正方形闭环
		{Square_Lock1(CsquareLock.m_square_middle , CsquareLock.m_square_halfedges);}
		
		if(Mode==5)
		{Circle_Lock1();}
		
		if(Mode==6)//正方形扫荡+ADC激光爆炸
		{
			if(left==0&&right==0)
			{
			Adc();//adc收到数据处理、反馈
            Adc_Check();//检测左右
			}
			if(left==1||right==1)
			{
//				//判断是否被卡住
//				Check_Error();

//				//如果被卡住
//				if()
//				{
//				Collision_Processing();
//				}
//				  //死亡旋转
//				//如果没有被卡住
				{
					if(right==1)                             //右
					{
					{Square_Sweep_Right1(2200 , square_edg);}
					USART_OUT(UART4,(uint8_t*)"%d  ",(int)square_edg);	
					}
					else if(left==1)	                     //左
					{
					{Square_Sweep_Left1(2200 , square_edg);}
					USART_OUT(UART4,(uint8_t*)"%d  ",(int)square_edg);	
					}
			    }
		    }
			USART_OUT(UART4,(uint8_t*)"\r\n");	//换行（独列）
		}
		
//		OSSemPend(PeriodSem, 0, &os_err);
	}

}	
