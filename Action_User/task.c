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
int shootDebug = 0;
int pidDebug = 1;
int fortDebug = 0;
int radarDebug = 0; /*雷达*/
int rollerDebug = 0;/*辊子*/
int ballcommand = 0;
PID_Value *PID_x = NULL;
PID_Value PID_A;
Err *Error_x = NULL;
Err Error_A;
int target[4] = {0};
float *error = NULL;
extern float Set_FortAngle1;
extern FortType fort;
extern GunneryData Gundata;
extern Command CmdRecData;
extern ScanData Scan;
extern int Entrepot[2];
void ConfigTask(void)
{
	PID_x = &PID_A;
	Error_x = &Error_A;
	Error_A.Err_X = 0,Error_A.Err_Y = 0,Error_A.flag = 0,Error_A.timeCnt = 0,Error_A.distance = 0,Error_A.err_distance = 100;
	int Laser_Left = 0;
	int Laser_Right = 0;
	KeyInit2();
	KeyInit0();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                  //系统中断优先级分组2
	TIM_Init(TIM2,999,83,0,0);                                       //时钟2初始化，1ms周期
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);                //can1初始化
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);                //can2初始化
	TIM_Delayms(TIM4,2000);                                          //延时2s，给定位系统准备时间
	ElmoInit(CAN2);                                                  //驱动器初始化
	ElmoInit(CAN1);                                                  //驱动器初始化
	VelLoopCfg(CAN1,2,1638400,1638400);                              //左电机速度环初始化
	VelLoopCfg(CAN1,1,1638400,1638400);                              //右电机速度环初始化
	VelLoopCfg(CAN2,5, 16384000, 16384000);                               //收球电机
	VelLoopCfg(CAN2,6, 16384000, 16384000);                               //收球电机
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
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0) == 1) ballcommand = BLACK_BALL;
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4) == 0)
	{
		WaitOpsPrepare();                                                //等待定位系统准备完成
		PID_Init(PID_x);                                                 //PID参数初始化
		CmdRecData.TarBucketNum_cmd = 0;
		CmdRecData.FireFlag_cmd = 1;
		CmdRecData.MoveFlag_cmd = 1;
		CmdRecData.YawZeroOffset_cmd = 0;
		CmdRecData.YawPosSet_cmd = 0;
		CmdRecData.ShooterVelSet_cmd = 0;
		VelCrl(CAN2, 5, 70 * 32768);
		VelCrl(CAN2, 6, 0-70 * 32768);
		
		//距离精度
		Gundata.Distance_Accuracy = 10.0;
		Gundata.Yaw_Zero_Offset = 2.5f;
		
		//设定各桶编号及坐标
		Gundata.Bucket_X[0] =  2200.0;      Gundata.Bucket_Y[0] =  200.0;
		Gundata.Bucket_X[1] =  2200.0;      Gundata.Bucket_Y[1] = 4600.0;
		Gundata.Bucket_X[2] = -2200.0;      Gundata.Bucket_Y[2] = 4600.0;
		Gundata.Bucket_X[3] = -2200.0;      Gundata.Bucket_Y[3] =  200.0;
		
		Gundata.Yaw_Angle_Offset[0] =  0.0f;  Gundata.Shooter_Vel_Offset[0] =  1.0f;
		Gundata.Yaw_Angle_Offset[1] =  0.0f;  Gundata.Shooter_Vel_Offset[1] =  1.0f;
		Gundata.Yaw_Angle_Offset[2] =  0.0f;  Gundata.Shooter_Vel_Offset[2] =  1.0f;
		Gundata.Yaw_Angle_Offset[3] =  0.0f;  Gundata.Shooter_Vel_Offset[3] =  1.0f;
		
		Gundata.Yaw_Angle_Offset[4] = -2.5f;  Gundata.Shooter_Vel_Offset[4] =  0.0f;
		Gundata.Yaw_Angle_Offset[5] = -2.5f;  Gundata.Shooter_Vel_Offset[5] =  0.0f;
		Gundata.Yaw_Angle_Offset[6] = -2.5f;  Gundata.Shooter_Vel_Offset[6] =  0.0f;
		Gundata.Yaw_Angle_Offset[7] = -2.5f;  Gundata.Shooter_Vel_Offset[7] =  0.0f;
		
	//	memset(Gundata.Yaw_Angle_Offset, 0, 8);
	//	memset(Gundata.Shooter_Vel_Offset, 0, 8);

		//初始化扫描参数
		Scan.ScanStatus = 0;
		Scan.BucketNum = 0;
		Scan.FirePermitFlag = 0;
		Scan.DelayFlag = 0;
		Scan.CntDelayTime = 0;
		Scan.GetBorderLeftFlag = 0;
		Scan.GetBorderRightFlag = 0;
		Scan.ScanPermitFlag = 0;
		Scan.Yaw_Zero_Offset = 2.5f;
		Scan.YawPosAngle_Offset = -2.5f;
		Scan.Shooter_Vel_Offset = 4.0f;
		
		//设定各挡板边缘坐标值
		Scan.Bucket_Border_X[0] =  2000.0;       Scan.Bucket_Border_Y[0] =   -54.0;
		Scan.Bucket_Border_X[1] =  2454.0;       Scan.Bucket_Border_Y[1] =   400.0;
		Scan.Bucket_Border_X[2] =  2454.0;       Scan.Bucket_Border_Y[2] =  4400.0;
		Scan.Bucket_Border_X[3] =  2000.0;       Scan.Bucket_Border_Y[3] =  4854.0;
		Scan.Bucket_Border_X[4] = -2000.0;       Scan.Bucket_Border_Y[4] =  4854.0;
		Scan.Bucket_Border_X[5] = -2454.0;       Scan.Bucket_Border_Y[5] =  4400.0;
		Scan.Bucket_Border_X[6] = -2454.0;       Scan.Bucket_Border_Y[6] =   400.0;
		Scan.Bucket_Border_X[7] = -2000.0;       Scan.Bucket_Border_Y[7] =   -54.0;	
		
		while(1)
		{
			Laser_Right = fort.laserBValueReceive;                       //左ADC
			Laser_Left = fort.laserAValueReceive;                        //右ADC
			if(20 < Laser_Left && Laser_Left < 100)
			{
				OSMboxPost(adc_msg,(void *)Left);                     
				OSTaskSuspend(OS_PRIO_SELF);                             //挂起初始化函数
			}
			else if(20 < Laser_Right && Laser_Right < 100)
			{
				OSMboxPost(adc_msg,(void *)Right);
				OSTaskSuspend(OS_PRIO_SELF);                             //挂起初始化函数
			}
		}
	}
	else
	{
		VelCrl(CAN2,5,0-70*32768);
		VelCrl(CAN2,6,70*32768);
		while(1){};
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
	os_err = os_err;																		//防报错
	static u32 direction = 0;
	int cntSendTime = 0;
	if(direction == 0) direction = (u32)OSMboxPend(adc_msg,0,&os_err);
	OSSemSet(PeriodSem, 0, &os_err);														//信号量归零
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);													//等信号量，10ms一次
		ReadActualPos(CAN2,7);																//读取分球电机位置
		ReadActualVel(CAN2,5);																//读取收球辊子转速
		ReadActualVel(CAN2,6);																//读取收球辊子转速
		
		GetData(PID_x);																		//读取定位系统信息
		
		WatchDog(PID_x);																	//球数看门狗
//		ErrorDisposal(PID_x,Error_x);														//错误检测
		PID_Priority(PID_x,direction,Error_x,target);										//走形计算函数
		PriorityControl(PID_x,Error_x,target);												//走形优先级管理
		GO(PID_x);																			//电机控制
		
		GetData(PID_x);																		//读取定位系统信息
		
		
		Error_x->errCnt = 1;
		
		
		if(PID_x->V != 0 && Error_x->errCnt == 0){											/*未避障模式*/
		Gundata.BucketNum = PID_A.target_Num;												//设置目标桶号
		GunneryData_Operation(&Gundata, PID_x);												//计算射击诸元
		YawPosCtrl(Gundata.YawPosAngleSetAct);												//设置航向角
		ShooterVelCtrl(Gundata.ShooterVelSetAct);}											//设置射球转速
		else if(PID_x->V == 0){																/*扫描模式*/
		YawPosCtrl(Scan.YawPosAngleSet);
		ShooterVelCtrl(Scan.ShooterVelSet);}
		else
		{
			YawPosCtrl(0);
			ShooterVelCtrl(50);
		}
		
//=================================================================================================================
		//激光
//		RadarCorrection(PID_x);			/*雷达扫描*/
//		YawPosCtrl(Set_FortAngle1);		//设定航向角
//		ShooterVelCtrl(25);				//设定射球转速
		//收球检测
		CntGolf();
		//发球检测
//=================================================================================================================

		if(fortDebug == 1)
		{
			cntSendTime++;
			cntSendTime = cntSendTime % 10;
			if(cntSendTime == 0)
			{
				//Scan参数
				USART_OUT(UART4, (uint8_t*)"PosX=%d	PosY=%d	PosAng=%d	ScanSta=%d	BucketNum=%d	ScanPer=%d i=%d	GetLeft=%d	GetRight=%d	StartAng=%d	EndAng=%d	YawSet=%d	delay=%d	cntdelay=%d	tar0=%d	tar1=%d	tar2=%d	tar3=%d\r\n",\
				(int)PID_A.X, 					(int)PID_A.Y, 					(int)PID_A.Angle,\
				(int)Scan.ScanStatus,			(int)Scan.BucketNum,     		(int)Scan.ScanPermitFlag, (int)Scan.i,\
				(int)Scan.GetBorderLeftFlag, 	(int)Scan.GetBorderRightFlag,	(int)Scan.ScanAngleStart,		(int)Scan.ScanAngleEnd, (int)Scan.YawPosAngleSet,\
				(int)Scan.DelayFlag,			(int)Scan.CntDelayTime, 		(int)target[0], 		(int)target[1], (int)target[2], (int)target[3]);
			}
		}
		
		if(PID_x->V != 0 && Error_x->errCnt == 0)
		{
			if(fabs(Gundata.YawPosAngleRec - Gundata.YawPosAngleSet) < 3.0f && fabs(Gundata.ShooterVelRec - Gundata.ShooterVelSet) < 3.0f &&\
				    Gundata.ShooterVelSet < 85.0f && target[PID_x->target_Num] == 0)PID_A.fire_command = 1;
			else PID_A.fire_command = 0;
		}
		else if(PID_x->V == 0)
		{
			if(CmdRecData.FireFlag_cmd == 1 && Scan.FirePermitFlag == 1) PID_A.fire_command = 1;
			else PID_A.fire_command = 0;
		}
		if(pidDebug) UART4_OUT(PID_x,Error_x);
		shoot(PID_x,target,shootDebug);
		OSSemSet(PeriodSem, 0, &os_err);
	}
}
