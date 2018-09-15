#include "PID.h"
#include "pps.h"
#include "fort.h"
#include "other.h"
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

	/*创建信号量**/
	PeriodSem = OSSemCreate(0);
	adc_msg = OSMboxCreate(NULL); 

	/*创建任务*/
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, /*初始化任务**/
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
extern FortType fort;
extern GunneryData Gundata;
extern Command CmdRecData;
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
	int Laser_Left = 0;
	int Laser_Right = 0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                  //系统中断优先级分组2
	TIM_Init(TIM2,999,83,0,0);                                       //时钟2初始化，1ms周期
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);                //can1初始化
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);                //can2初始化
	TIM_Delayms(TIM4,2000);                                          //延时2s，给定位系统准备时间
	ElmoInit(CAN2);                                                  //驱动器初始化
	ElmoInit(CAN1);                                                  //驱动器初始化
	VelLoopCfg(CAN1,2,16384000,16384000);                              //左电机速度环初始化
	VelLoopCfg(CAN1,1,16384000,16384000);                              //右电机速度环初始化
	VelLoopCfg(CAN2,5, 1638400, 1638400);                               //收球电机
	VelLoopCfg(CAN2,6, 1638400, 1638400);                               //收球电机
	PosLoopCfg(CAN2,7, 16384000,16384000,20000000);
	MotorOn(CAN1,1);                                                 //右电机使能
	MotorOn(CAN1,2);                                                 //左电机使能
	MotorOn(CAN2,5); 
	MotorOn(CAN2,6); 
	MotorOn(CAN2,7); 
	USART3_Init(115200);                                             //串口3初始化，定位系统用
	UART4_Init(921600);                                              //串口4初始化，与上位机通信用
	UART5_Init(921600);
	USART1_Init(921600);
	WaitOpsPrepare();                                                //等待定位系统准备完成
	PID_Init(PID_x);                                                 //PID参数初始化
	VelCrl(CAN2,5,60*32768);
	VelCrl(CAN2,6,0-60*32768);
	YawPosCtrl(0);
	
//	static float error1 = 0;                                         //用激光传感器矫正定位系统偏移
//	//error1 = ((Get_Adc_Average(15,100) - Get_Adc_Average(14,100))/2)*0.922854f;
//	error1 = 0;
//	OS_CPU_SR cpu_sr;
//	OS_ENTER_CRITICAL();                                             /*互斥访问*/
//	error = &error1;
//	OS_EXIT_CRITICAL();

	CmdRecData.TarBucketNum_cmd = 0;
	CmdRecData.FireFlag_cmd = 1;
	CmdRecData.MoveFlag_cmd = 1;
	CmdRecData.YawZeroOffset_cmd = 0;
	CmdRecData.YawPosSet_cmd = 0;
	CmdRecData.ShooterVelSet_cmd = 0;
	
	//距离精度
	Gundata.Distance_Accuracy = 10.0;
	Gundata.Yaw_Zero_Offset = -16.0;
	
	//设定各桶编号及坐标
	Gundata.Bucket_X[0] = 2200.0;       Gundata.Bucket_Y[0] = 200.0;
	Gundata.Bucket_X[1] = 2200.0;       Gundata.Bucket_Y[1] = 4600.0;
	Gundata.Bucket_X[2] = -2200.0;      Gundata.Bucket_Y[2] = 4600.0;
	Gundata.Bucket_X[3] = -2200.0;      Gundata.Bucket_Y[3] = 200.0;

	memset(Gundata.Yaw_Angle_Offset, 0, 8);
	memset(Gundata.Shooter_Vel_Offset, 0, 8);	
	while(1)
	{
		Laser_Right = fort.laserBValueReceive;                         //左ADC
		Laser_Left = fort.laserAValueReceive;                          //右ADC
		if(Laser_Left<100)
		{
			OSMboxPost(adc_msg,(void *)Left);                     
			OSTaskSuspend(OS_PRIO_SELF);                             //挂起初始化函数
		}
		else if(Laser_Right<100)
		{
			OSMboxPost(adc_msg,(void *)Right);
			OSTaskSuspend(OS_PRIO_SELF);                             //挂起初始化函数
		}
	}
}

/*
===============================================================
                                  WalkTask      初始化后执行
===============================================================
*/

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;                                                 //防报错
	static u32 direction = 0;
	int cntSendTime = 0;
	if(direction == 0) direction = (u32)OSMboxPend(adc_msg,0,&os_err);
	OSSemSet(PeriodSem, 0, &os_err);                 	             //信号量归零
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);                            //等信号量，10ms一次
		
//		ReadActualVel(CAN1,1);
//		ReadActualVel(CAN1,2);
		ReadActualPos(CAN2,7);
		
		GetData(PID_x);
//		ErrorDisposal(PID_x,Error_x);
//		PID_Competition(PID_x,direction,Error_x);
//		GO(PID_x);
	
		Gundata.BucketNum = PID_A.target_Num;      //设置目标桶号
		GunneryData_Operation(&Gundata, PID_x);    //计算射击诸元
		YawPosCtrl(Gundata.YawPosAngleSet);        //设置航向角
		ShooterVelCtrl(Gundata.ShooterVelSet);     //设置射球转速

		//以5 * 10ms为间隔发送数据
		cntSendTime++;
		cntSendTime = cntSendTime % 5;
		if(cntSendTime == 1)
		{
			USART_OUT(UART4, (uint8_t*)"YawSet=%d,YawRac=%d,ShoSet=%d,ShoRec=%d,Yaw0=%d,Yaw1=%d,Yaw2=%d,Yaw3=%d,Sho0=%d,Sho1=%d,Sho2=%d,Sho3=%d\r\n",\
			(int)Gundata.YawPosAngleSet, (int)Gundata.YawPosAngleRec, (int)Gundata.ShooterVelSet, (int)Gundata.ShooterVelRec,\
			(int)Gundata.Yaw_Angle_Offset[0], (int)Gundata.Yaw_Angle_Offset[1], (int)Gundata.Yaw_Angle_Offset[2], (int)Gundata.Yaw_Angle_Offset[3],\
			(int)Gundata.Shooter_Vel_Offset[0], (int)Gundata.Shooter_Vel_Offset[1], (int)Gundata.Shooter_Vel_Offset[2], (int)Gundata.Shooter_Vel_Offset[3]);
		}	

		if(fabs(Gundata.ShooterVelRec - Gundata.ShooterVelSet) < 5.0 && Gundata.ShooterVelSet < 75.0 && CmdRecData.FireFlag_cmd == 1)PID_A.fire_command = 1;
		shoot(PID_x);
		
//		UART4_OUT(PID_x);
	}
}
