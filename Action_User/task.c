#include "PID.h"
#include "pps.h"
#include "fort.h"
#define COLLECT_BALL_ID (8)
/*
===============================================================
					                        	信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
OS_EVENT *adc_msg;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
/*
===============================================================
                                                APP_Task
===============================================================
*/
void App_Task(void)
{
	CPU_INT08U os_err;
	os_err = os_err; /*防止警告...*/

	/*创建信号量*/
	PeriodSem = OSSemCreate(0);
	adc_msg = OSMboxCreate(NULL); 

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
PID_Value *PID_x = NULL;
PID_Value PID_A;
Err *Error_x;
Err Error_A;
float *error = NULL;
void ConfigTask(void)
{
	PID_x = &PID_A;
	Error_x = &Error_A;
	Error_A.Err_X = 0;
	Error_A.Err_Y = 0;
	Error_A.flag = 0;
	Error_A.timeCnt = 0;
	Error_A.distance = 0;
	Error_A.err_distance = 100;
	int ADC_Left = 0;
	int ADC_Right = 0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                  //系统中断优先级分组2
	TIM_Init(TIM2,999,83,0,0);                                       //时钟2初始化，1ms周期
	Adc_Init();                                                      //adc_init
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);                //can1初始化
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);                //can2初始化
	ElmoInit(CAN2);                                                  //驱动器初始化
	ElmoInit(CAN1);                                                  //驱动器初始化
	VelLoopCfg(CAN2,2,8000000,8000000);                              //左电机速度环初始化
	VelLoopCfg(CAN2,1,8000000,8000000);                              //右电机速度环初始化
	VelLoopCfg(CAN1, 8, 40000000, 40000000);                               //收球电机
	PosLoopCfg(CAN1, PUSH_BALL_ID, 5000000,5000000,200000);
	MotorOn(CAN2,1);                                                 //右电机使能
	MotorOn(CAN2,2);                                                 //左电机使能
	MotorOn(CAN1,8); 
	MotorOn(CAN1,6); 
	USART3_Init(115200);                                             //串口3初始化，定位系统用
	UART4_Init(921600);                                              //串口4初始化，与上位机通信用
	UART5_Init(921600);
	USART1_Init(921600);
	TIM_Delayms(TIM4,2000);                                          //延时2s，给定位系统准备时间
	WaitOpsPrepare();                                                //等待定位系统准备完成
	PID_Init(PID_x);                                                 //PID参数初始化
	VelCrl(CAN1,COLLECT_BALL_ID,400*4096);
	YawPosCtrl(170);
	
	static float error1 = 0;                                         //用激光传感器矫正定位系统偏移
	//error1 = ((Get_Adc_Average(15,100) - Get_Adc_Average(14,100))/2)*0.922854f;
	error1 = 0;
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL();                                             /*互斥访问*/
	error = &error1;
	OS_EXIT_CRITICAL();
	
	while(1)
	{
		ADC_Left = Get_Adc_Average(15,10);                          //左ADC
		ADC_Right = Get_Adc_Average(14,10);                         //右ADC
		if(ADC_Left<100)
		{
			OSMboxPost(adc_msg,(void *)Right);                     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			OSTaskSuspend(OS_PRIO_SELF);                             //挂起初始化函数
		}
		else if(ADC_Right<100)
		{
			OSMboxPost(adc_msg,(void *)Right);
			OSTaskSuspend(OS_PRIO_SELF);                             //挂起初始化函数
		}
//		else USART_OUT(UART4,(uint8_t*)"%s","wait for adc data\r\n");
	}
}

/*
===============================================================
                                  WalkTask      初始化后执行
===============================================================
*/
extern GunneryData gundata;
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;                                                 //防报错
	static u32 direction = 0;
	if(direction == 0) direction = (u32)OSMboxPend(adc_msg,0,&os_err);
	OSSemSet(PeriodSem, 0, &os_err);                 	 //信号量归零
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);                            //等信号量，10ms一次
		
		GetData(PID_x);
		ErrorDisposal(PID_x,Error_x);
		PID_Competition(PID_x,direction,Error_x);

		gundata.BucketNum = PID_A.target_Num;
					switch(gundata.BucketNum)
			{
				case 0:
					gundata.Yaw_Angle_Offset = 4.0;        //航向角补偿 
					gundata.Shooter_Vel_Offset = -3.0;      //射球转速补偿	
					break;
				case 1:
					gundata.Yaw_Angle_Offset = 4.0;        //航向角补偿 
					gundata.Shooter_Vel_Offset = -3.0;      //射球转速补偿	
					break;
				case 2:
					gundata.Yaw_Angle_Offset = 4.0;        //航向角补偿 
					gundata.Shooter_Vel_Offset = -3.0;      //射球转速补偿	
					break;
				case 3:
					gundata.Yaw_Angle_Offset = 4.0;        //航向角补偿 
					gundata.Shooter_Vel_Offset = -3.0;      //射球转速补偿	
					break;
			}
		gundata.Distance_Accuracy = 10.0;    //距离精度
		GunneryData_Operation(&gundata,PID_x);
		YawPosCtrl(gundata.YawPosTarActAngle);
		ShooterVelCtrl(gundata.ShooterVelSet);
		if(gundata.Angle_Deviation < 20.0) shoot();
//		shoot();
		//     上位机控制炮台航向，射速函数
//		YawPosCtrl(GetYawPosCommand());
//		ShooterVelCtrl(GetShooterVelCommand());
//		UART4_OUT(PID_x)
//		USART_OUT(UART4,(uint8_t*)"%d	", (int)gundata.YawPos);
//		USART_OUT(UART4,(uint8_t*)"%d	", (int)gundata.YawPosTarActAngle);
//		USART_OUT(UART4,(uint8_t*)"%d	", (int)gundata.ShooterVel);
//		USART_OUT(UART4,(uint8_t*)"%d	", (int)gundata.ShooterVelSet);
//		USART_SendData(UART4,'\r');
//		USART_SendData(UART4,'\n');
	}
}

