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
#include "fort.h"
#include "MRKSteven.h"

extern float v1_record,v2_record;
extern float v1,v2;               //两轮速度（2左 || 1右）

extern Place positionf;           //浮点 positionf (X、Y、Angle)
extern Place positioni;           //整型 positioni (X、Y、Angle)
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
extern int square_break;

// 发射航向角转换函数 由度转换为脉冲
// yawAngle为角度，范围180到-180之间，初始位置为0度。


static int opsflag=0;
typedef union
{
    //这个32位整型数是给电机发送的速度（脉冲/s）
    int32_t Int32 ;
    //通过串口发送数据每次只能发8位
    uint8_t Uint8[4];
}num_t;
//定义联合体
num_t u_Num;
void SendUint8(void)
{
    u_Num.Int32 = 1000;
    //起始位
    USART_SendData(USART1, 'A');
    //通过串口1发数
    USART_SendData(USART1, u_Num.Uint8[0]);
    USART_SendData(USART1, u_Num.Uint8[1]);
    USART_SendData(USART1, u_Num.Uint8[2]);
    USART_SendData(USART1, u_Num.Uint8[3]);
    //终止位
    USART_SendData(USART1, 'J');
}
//===================================================================================================================================================
//===================================================================================================================================================
// 
//                           信号量定义
//
//===================================================================================================================================================
//===================================================================================================================================================

/*
===============================================================
						信号量定义
===============================================================
*/

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
	
//	TIM_Init(TIM2,1000-1,84-1,1,3);	//产生10ms中断，抢占优先级为1，响应优先级为3

//	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
//	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
//	
//	VelLoopCfg(CAN2,1, 5000, 5000);				//驱动器速度环初始化
//	VelLoopCfg(CAN2,2, 5000, 5000);
//	
//	ElmoInit(CAN2);								//驱动器初始化
//	MotorOn(CAN2,1);							//电机使能（通电）
//	MotorOn(CAN2,2);
	
	//TIM4_Pwm_Init (9999,83);//pwm初始化（10ms）
	
	TIM_Init(TIM2, 999, 83, 0X01, 0X03);//TIM2 1ms中断	
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);//CAN1通信（将CAN1时钟赋给CAN2）
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);//CAN2通信
	
	if(veh==0)
	{
		USART1_Init(921600);
		//驱动器速度环初始化
		VelLoopCfg(CAN2,5,40960000,40960000);
		VelLoopCfg(CAN2,6,40960000,40960000);
		//Elmo驱动器初始化
		ElmoInit_0(CAN2);
//		ElmoInit_0(CAN1);//////////////////////////////////////////////////
			//电机使能（通电）
		MotorOn(CAN2, 5);
		MotorOn(CAN2, 6);
    }
	if(veh==1)
	{
		UART4_Init(921600);
		//驱动器速度环初始化
		VelLoopCfg(CAN2,1,40960000,40960000);
		VelLoopCfg(CAN2,2,40960000,40960000);
		//Elmo驱动器初始化
		ElmoInit(CAN1);	
		ElmoInit(CAN2);	
		//电机使能（通电）
		MotorOn(CAN2, 1);
		MotorOn(CAN2, 2);
			//Adc初始化
		Adc_Init();
			
		UART5_Init(921600);	

		/*棍子收球机*/		
		// 配置速度环
		VelLoopCfg(CAN1, 8, 50000, 50000); 
		
		/*推球电机*/
		// 配置位置环
		PosLoopCfg(CAN1, PUSH_BALL_ID, 50000,50000,20000);
		
		/*航向电机*/
		// 配置位置环
//		PosLoopCfg(CAN1, GUN_YAW_ID, 50000,50000,20000); 
		 
		MotorOn(CAN1,6);  
//	MotorOn(CAN1,7); 
		MotorOn(CAN1,8);
	}

	TIM_Init(TIM2, 99, 839, 1, 0);
	/*一直等待定位系统初始化完成*/
	BEEP_ON;
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
extern FortType fort;
int t_push=0;
extern FortType fort;

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem,0,&os_err);
	OSSemPend(PeriodSem, 0, &os_err);
	while (1)
	{
		    if(veh==1)
			{	
			G_Adc_A4=Get_Adc_Average(14,10);
			G_Adc_A5=Get_Adc_Average(15,10);
			}
			///////////////////////////////////
			OSSemPend(PeriodSem, 0, &os_err);//
			///////////////////////////////////
			
			Coordinate_Reverse();//坐标反转（一定要放在while开始，OSSemPend后一行）	
			Position_Record();   //坐标记录、实时位置发送

			if(veh==1)
			{
				/*棍子收球机*/
				// 控制电机的转速，脉冲。
				VelCrl(CAN1,COLLECT_BALL_ID,60*4096);
				
				//控制发射枪电机转速// 
				ShooterVelCtrl(66);

				/*推球电机*/
				t_push++;
				if(t_push==300)
				{
					// 推球
					PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);
				}
				else if(t_push==600)
				{
					// 复位
					PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);
					t_push=0;
				}

				ReadShooterVel();
				ReadYawPos();
				ReadLaserAValue();
				ReadLaserBValue();
				
			v1=(int)10865*v;
			v2=(int)10865*v;
			}
			if(veh==0)
			{
			v1=(NEW_CAR_COUNTS_PER_ROUND*1000*v*REDUCTION_RATIO)/(pi*WHEEL_DIAMETER);//后轮
//			if(r!=0){v2=(v*TURN_AROUND_WHEEL_TO_BACK_WHEEL*NEW_CAR_COUNTS_PER_ROUND*REDUCTION_RATIO)/(r*pi*TURN_AROUND_WHEEL_DIAMETER);}//前轮
//			if(r==0){v2=0;}
			v2=0;
			}	
		
		if(Mode==0)//测试状态
	    {
			USART_OUT(UART4,(uint8_t*)"%s %s %s %s\r\n","T","E","S","T");//mode0
			if(veh==0)
			{
			USART_OUT(USART1,(uint8_t*)"%s %s %s %s\r\n","T","E","S","T");//mode0
			Move_0(v1,v2);
			}
		}
		
		if(Mode==1)                       //Mode1 直行（r=0）或圆周运动 前进/后退
		{Move_Basic();}
		
		if(Mode==2)                       //Mode2 直行（r=0）||多边形运动（带自动校正）（r为多边形边长；angle为多边形邻边角度） 
		{
			USART_OUT(USART1,(uint8_t*)"%s%s%s%s%d\r\n","M","o","d","e",2);//mode2
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

				Collision_Processing();

//				  //死亡旋转
//				//如果没有被卡住
				if(square_break==0)
				{
					if(right==1)                             //右
					{
					{Square_Sweep_Right1(2200 , square_edg);}
					USART_OUT(UART4,(uint8_t*)"%d  ",(int)square_edg);	
					}
					else if(left==1)	                     //左
					{
					{Square_Sweep_Left1(2200 , square_edg);}
					USART_OUT(UART4,(uint8_t*)"%s%s%s%s%d  ","e","d","g",":",(int)square_edg);	
					}
			    }
		    }
			USART_OUT(UART4,(uint8_t*)"\r\n");	//换行（独列）
		}
		if(veh==1){USART_OUT(UART4,(uint8_t*)"\r\n");}	//换行（独列）
		if(veh==0){USART_OUT(USART1,(uint8_t*)"\r\n");}
//		OSSemPend(PeriodSem, 0, &os_err);
	}

}	
