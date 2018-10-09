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
//GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1);
int shootDebug = 0;
int pidDebug = 0;
int fortDebug = 1;
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
extern CalibrationData Cal;
void ConfigTask(void)
{
	PID_x = &PID_A;
	Error_x = &Error_A;
	Error_A.Err_X = 0,Error_A.Err_Y = 0,Error_A.flag = 0,Error_A.timeCnt = 0,Error_A.distance = 0,Error_A.err_distance = 100;
	KeyInit2();
	KeyInit0();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);					//系统中断优先级分组2
	TIM_Init(TIM2,999,83,0,0);										//时钟2初始化，1ms周期
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);				//can1初始化
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);				//can2初始化
	TIM_Delayms(TIM4,2000);											//延时2s，给定位系统准备时间
	ElmoInit(CAN2);													//驱动器初始化
	ElmoInit(CAN1);													//驱动器初始化
	VelLoopCfg(CAN1, 2, 16384000, 32768000);						//左电机速度环初始化
	VelLoopCfg(CAN1, 1, 16384000, 32768000);						//右电机速度环初始化
	VelLoopCfg(CAN2, 5, 16384000, 16384000);						//收球电机
	VelLoopCfg(CAN2, 6, 16384000, 16384000);						//收球电机
	PosLoopCfg(CAN2, 7, 16384000, 16384000, 20000000);
	MotorOn(CAN1,1);												//右电机使能
	MotorOn(CAN1,2);												//左电机使能
	MotorOn(CAN2,5); 
	MotorOn(CAN2,6);
	MotorOn(CAN2,7);
	USART3_Init(115200);											//串口3初始化，定位系统用
	UART4_Init(921600);												//串口4初始化，与上位机通信用
	UART5_Init(921600);
	USART1_Init(921600);
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0) == 1) ballcommand = BLACK_BALL;
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4) == 0)
	{
		WaitOpsPrepare();											//等待定位系统准备完成
		PID_Init(PID_x);											//PID参数初始化
		CmdRecData.TarBucketNum_cmd = 0;
		CmdRecData.FireFlag_cmd = 1;
		CmdRecData.MoveFlag_cmd = 1;
		CmdRecData.YawZeroOffset_cmd = 0;
		CmdRecData.YawPosSet_cmd = 0;
		CmdRecData.ShooterVelSet_cmd = 0;
		VelCrl(CAN2, 5, 70 * 32768);
		VelCrl(CAN2, 6, 0-70 * 32768);
		YawPosCtrl(20);
		
		Gundata.Dist_Error_Accuracy = 10.0;
		Gundata.YawAngle_Zero_Offset = -0.6f;
		Gundata.MovingShootFlag = 0;
		
		//设定各桶编号及坐标
		Gundata.Pos_Bucket_X[0] =  2200.0;	Gundata.Pos_Bucket_Y[0] =  200.0;
		Gundata.Pos_Bucket_X[1] =  2200.0;	Gundata.Pos_Bucket_Y[1] = 4600.0;
		Gundata.Pos_Bucket_X[2] = -2200.0;	Gundata.Pos_Bucket_Y[2] = 4600.0;
		Gundata.Pos_Bucket_X[3] = -2200.0;	Gundata.Pos_Bucket_Y[3] =  200.0;
		
		Gundata.YawAngle_Offset[0] =  0.0f;	Gundata.ShooterVel_Offset[0] =  0.0f;
		Gundata.YawAngle_Offset[1] =  0.0f;	Gundata.ShooterVel_Offset[1] =  0.0f;
		Gundata.YawAngle_Offset[2] =  0.0f;	Gundata.ShooterVel_Offset[2] =  0.0f;
		Gundata.YawAngle_Offset[3] =  0.0f;	Gundata.ShooterVel_Offset[3] =  0.0f;
		
		Gundata.YawAngle_Offset[4] =  0.0f;	Gundata.ShooterVel_Offset[4] =  0.0f;
		Gundata.YawAngle_Offset[5] =  0.0f;	Gundata.ShooterVel_Offset[5] =  0.0f;
		Gundata.YawAngle_Offset[6] =  0.0f;	Gundata.ShooterVel_Offset[6] =  0.0f;
		Gundata.YawAngle_Offset[7] =  0.0f;	Gundata.ShooterVel_Offset[7] =  0.0f;
		
//		memset(Gundata.Yaw_Angle_Offset, 0, 8);
//		memset(Gundata.Shooter_Vel_Offset, 0, 8);

		//初始化扫描参数
		Scan.BucketNum = 0;
		Scan.Bubble_Mode = 0;
		Scan.DelayFlag = 0;
		Scan.CntDelayTime = 0;
	
		Scan.FirePermitFlag = 0;
		Scan.ScanPermitFlag = 0;
	
		Scan.GetLeftFlag = 0;
		Scan.GetRightFlag = 0;
		
		Scan.ScanStatus = 0;
		Scan.ScanShootFlag = 0;
		Scan.SetTimeFlag = 1;
		Scan.SetFireFlag = 1;
		
		Scan.YawAngle_Zero_Offset = 0.0f;
		Scan.YawAngle_Offset = -2.0f;
		Scan.ShooterVel_Offset = 0.0f;
		Scan.ScanVel = 0.03f;			
		
		//设定各挡板边缘坐标值
		Scan.Pos_Border_X[0] =  2000.0;       Scan.Pos_Border_Y[0] =   -54.0;
		Scan.Pos_Border_X[1] =  2454.0;       Scan.Pos_Border_Y[1] =   400.0;
		Scan.Pos_Border_X[2] =  2454.0;       Scan.Pos_Border_Y[2] =  4400.0;
		Scan.Pos_Border_X[3] =  2000.0;       Scan.Pos_Border_Y[3] =  4854.0;
		Scan.Pos_Border_X[4] = -2000.0;       Scan.Pos_Border_Y[4] =  4854.0;
		Scan.Pos_Border_X[5] = -2454.0;       Scan.Pos_Border_Y[5] =  4400.0;
		Scan.Pos_Border_X[6] = -2454.0;       Scan.Pos_Border_Y[6] =   400.0;
		Scan.Pos_Border_X[7] = -2000.0;       Scan.Pos_Border_Y[7] =   -54.0;
		
		Scan.DistChange_L = 0;
		Scan.DistChange_R = 0;
		Scan.PosOK_L = 0;
		Scan.PosOK_R = 0;
		Scan.DistOK_L = 0;
		Scan.DistOK_R = 0;
		Scan.AngleOK = 0;
		
		Scan.ScanMaxFlag = 1;

		int startFlag = 1;
		int cntSendTime = 0, cntClearTime = 0, cntLeftTrigger = 0, cntRightTrigger = 0;
		int Laser_Left, Laser_Right;
		while(startFlag)
		{
			delay_ms(10);
			Laser_Left = fort.laserAValueReceive;
			Laser_Right = fort.laserBValueReceive;
			
			//每0.5s发送一次触发状态
			cntSendTime++;
			cntSendTime = cntSendTime % 50;
			if(cntSendTime == 1)
			{
				USART_OUT(UART4, (uint8_t*)"Stand by: Left=%d	cntLeft=%d	Right=%d	cntRight=%d	cntClear=%d\r\n",\
				(int)Laser_Left, (int)cntLeftTrigger, (int)Laser_Right, (int)cntRightTrigger, (int)cntClearTime);
			}
	
			//判断激光器探测距离并计数
			if(10 < Laser_Left && Laser_Left < 100)
			{
				cntLeftTrigger++;
			}		
			else if(10 < Laser_Right && Laser_Right < 100)
			{
				cntRightTrigger++;
			}
			else
			{
				cntClearTime++;
			}

			//连续触发0.2s则触发成功
			if(cntLeftTrigger > 20)
			{
				OSMboxPost(adc_msg,(void *)Left);
				USART_OUT(UART4, (uint8_t*)"Go Clockwise\r\n");					
				OSTaskSuspend(OS_PRIO_SELF);
			}
			else if(cntRightTrigger > 20)
			{
				OSMboxPost(adc_msg,(void *)Right);
				USART_OUT(UART4, (uint8_t*)"Go Anti-Clockwise\r\n");
				OSTaskSuspend(OS_PRIO_SELF);
			}
			else if(cntClearTime > 20)
			{
				cntClearTime = 0;
				cntLeftTrigger = 0;
				cntRightTrigger = 0;
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
	os_err = os_err;										//防报错
	static u32 direction = 0;
	int cntSendTime = 0;
	if(direction == 0) direction = (u32)OSMboxPend(adc_msg,0,&os_err);
	OSSemSet(PeriodSem, 0, &os_err);						//信号量归零
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);					//等信号量，10ms一次
		ReadActualPos(CAN2,7);								//读取分球电机位置
		ReadActualVel(CAN2,5);
		ReadActualVel(CAN2,6);
		
		GetData(PID_x);										//读取定位系统信息
		PriorityControl(PID_x,Error_x,target);
		WatchDog(PID_x);
		PID_Priority(PID_x,direction,Error_x,target);		//走形计算函数
		ErrorDisposal(PID_x,Error_x);						//错误检测
		GO(PID_x);											//电机控制
		
		GetData(PID_x);										//读取定位系统信息
		if(PID_x->V != 0 && Error_x->errCnt == 0)
		{
			Gundata.MovingShootFlag = 1;
			Scan.ScanShootFlag = 0;
			Gundata.BucketNum = PID_A.target_Num;			//设置目标桶号
			GunneryData_Operation(&Gundata, PID_x);			//计算射击诸元
			YawPosCtrl(Gundata.YawAngle_SetAct);			//设置航向角
			ShooterVelCtrl(Gundata.ShooterVel_SetAct);		//设置射球转速
		}
		else if(PID_x->V == 0)
		{
			Gundata.MovingShootFlag = 0;
			Scan.ScanShootFlag = 1;

			//扫描时间计时，6s时强制将ScanStatus置为0
			if(Scan.DelayFlag)
			{
				Scan.CntDelayTime++;
				if(Scan.CntDelayTime > 600)
				{
					Scan.DelayFlag = 0;
					Scan.CntDelayTime = 0;
					Scan.CntFireDelayTime = 0;
					
					Scan.GetLeftFlag = 0;
					Scan.GetRightFlag = 0;
	
					Scan.ScanPermitFlag = 0;
					Scan.FirePermitFlag = 0;					
					Scan.ScanStatus = 0;
					Scan.SetTimeFlag = 1;
					Scan.SetFireFlag = 1;
					Scan.ScanVel = 0.03f;
					
					Scan.DistChange_L = 0;
					Scan.DistChange_R = 0;
					Scan.PosOK_L = 0;
					Scan.PosOK_R = 0;
					Scan.DistOK_L = 0;
					Scan.DistOK_R = 0;
					Scan.AngleOK = 0;
					
					Scan.ScanMaxFlag = 1;
				}
			}
			YawPosCtrl(Scan.YawAngle_Set);
			ShooterVelCtrl(Scan.ShooterVel_Set);
			
			if(Scan.GetBucketFlag == 1)
			{
				Scan.GetBucketFlag = 0;
				Calibration_Operation(&Cal, &Scan, &Gundata, PID_x);
			}
		}
		else
		{
			YawPosCtrl(0);
			ShooterVelCtrl(50);
			PID_A.fire_command = 0;
		}

		if(fortDebug == 1)
		{
			cntSendTime++;
			cntSendTime = cntSendTime % 10;
			if(cntSendTime == 0)
			{
				//比赛收数
				USART_OUT(UART4, (uint8_t*)"X=%d	Y=%d	Ang=%d	SpeX=%d	SpeY=%d	WZ=%d	LaserA=%d	LaserB=%d	food=%d	hungry=%d	stop=%d	FireCmd=%d	FireReq=%d	ScanSta=%d	BucNum=%d	ScanPer=%d	SetTime=%d	SetFire=%d	GetLeft=%d	GetRight=%d	StartAng=%d	EndAng=%d	YawSet=%d	delay=%d	cntdelay=%d	Tar0=%d	Tar1=%d	Tar2=%d	Tar3=%d\r\n",\
				(int)PID_A.X,			(int)PID_A.Y,				(int)PID_A.Angle,			(int)PID_A.X_Speed,			(int)PID_A.Y_Speed,			(int)GetWZ(),
				(int)fort.laserAValueReceive,						(int)fort.laserBValueReceive,\
				(int)PID_A.food,		(int)PID_A.dogHungry,		(int)PID_A.stop,			(int)PID_A.fire_command,	(int)PID_A.fire_request,\
				(int)Scan.ScanStatus,	(int)Scan.BucketNum,		(int)Scan.ScanPermitFlag, 	(int)Scan.SetTimeFlag,		(int)Scan.SetFireFlag,\
				(int)Scan.GetLeftFlag,	(int)Scan.GetRightFlag,		(int)Scan.ScanAngle_Start,	(int)Scan.ScanAngle_End,	(int)Scan.YawAngle_Set,\
				(int)Scan.DelayFlag,	(int)Scan.CntDelayTime,\
				(int)target[0],			(int)target[1],				(int)target[2], 			(int)target[3]);

//				//Scan参数
//				USART_OUT(UART4, (uint8_t*)"X=%d	Y=%d	Ang=%d	ScanSta=%d	BucNum=%d	ScanPer=%d	FirePer=%d	SetTime=%d	SetFire=%d	GetLeft=%d	GetRight=%d	ScanMax=%d	Del=%d	cntDel=%d	cntFireDel=%d	StartAng=%d	TarNum=%d	ProBLX=%d	ProBLY=%d	ProBRX=%d	ProBRY=%d	DisCL=%d	DisCR=%d	PPSL=%d	PPSR=%d	DisL=%d	DisR=%d	Ang=%d	MaxDis=%d	MaxX=%d	MaxY=%d	MTLDis=%d	MTLAng=%d	MTRDis=%d	MTRAng=%d	LTRAng=%d	tar0=%d tar1=%d tar2=%d tar3=%d\r\n",\
//				(int)PID_A.X,			(int)PID_A.Y,				(int)PID_A.Angle,\
//				(int)Scan.ScanStatus,	(int)Scan.BucketNum,		(int)Scan.ScanPermitFlag, 	(int)Scan.FirePermitFlag,	(int)Scan.SetTimeFlag,		(int)Scan.SetFireFlag,\
//				(int)Scan.GetLeftFlag,	(int)Scan.GetRightFlag,		(int)Scan.ScanMaxFlag,		(int)Scan.DelayFlag,		(int)Scan.CntDelayTime,		(int)Scan.CntFireDelayTime,	(int)Scan.ScanAngle_Start,	(int)PID_A.target_Num,\
//				(int)Scan.Pro_Border_Left_X,	(int)Scan.Pro_Border_Left_Y,	(int)Scan.Pro_Border_Right_X,	(int)Scan.Pro_Border_Right_Y,\
//				(int)Scan.DistChange_L,	(int)Scan.DistChange_R,		(int)Scan.PosOK_L,			(int)Scan.PosOK_R,\
//				(int)Scan.DistOK_L, 	(int)Scan.DistOK_R,			(int)Scan.AngleOK,\
//				(int)Scan.Pro_Max_Dist, (int)Scan.Pro_Max_X, 		(int)Scan.Pro_Max_Y,\
//				(int)Scan.Max_To_Left_Dist, (int)Scan.Max_To_Left_Angle, (int)Scan.Max_To_Right_Dist, (int)Scan.Max_To_Right_Angle, (int)Scan.Left_To_Right_Angle,\
//				(int)target[0],			(int)target[1],				(int)target[2],				(int)target[3]);
//				
		
//				//Cal参数
//				USART_OUT(UART4, (uint8_t*)"X=%d	Y=%d	Ang=%d	ScanSta=%d	BucketNum=%d	GetLeft=%d	GetRight=%d	StartAng=%d	EndAng=%d	YawSet=%d	DelFlag=%d	cntDelTime=%d	ProBLX=%d	ProBLY=%d	ProBRX=%d	ProBRY=%d	toLAng=%d	toLDis=%d	toRAng=%d	toRDis=%d	ActX=%d	ActY=%d	ActAng=%d	TheAng=%d	CalBLX=%d	CalBLY=%d	CalBRX=%d	CalBRY=%d\r\n",\
//				(int)PID_A.X,					(int)PID_A.Y,					(int)PID_A.Angle,\
//				(int)Scan.ScanStatus,			(int)Scan.BucketNum,			(int)Scan.GetLeftFlag,			(int)Scan.GetRightFlag,\
//				(int)Scan.ScanAngle_Start,		(int)Scan.ScanAngle_End,		(int)Scan.YawAngle_Set,			(int)Scan.DelayFlag,		(int)Scan.CntDelayTime,\
//				(int)Scan.Pro_Border_Left_X,	(int)Scan.Pro_Border_Left_Y,	(int)Scan.Pro_Border_Right_X,	(int)Scan.Pro_Border_Right_Y,\
//				(int)Cal.OToLeft_Angle,			(int)Cal.OToLeft_Dist,			(int)Cal.OToRight_Angle,		(int)Cal.OToRight_Dist,\
//				(int)Cal.LToR_Act_Dist_X,		(int)Cal.LToR_Act_Dist_Y,		(int)Cal.LToR_Act_Angle,		(int)Cal.LToR_The_Angle,\
//				(int)Cal.Pos_Border_Left_X,		(int)Cal.Pos_Border_Left_Y,		(int)Cal.Pos_Border_Right_X,	(int)Cal.Pos_Border_Right_Y);
			}
		}
		
		if(PID_x->V != 0 && Error_x->errCnt == 0)
		{
			if(fabs(Gundata.YawAngle_Rec - Gundata.YawAngle_Set) < 3.0f && fabs(Gundata.ShooterVel_Rec - Gundata.ShooterVel_Set) < 4.0f &&\
				    Gundata.ShooterVel_Set < 85.0f && target[PID_x->target_Num] < 2 && GetWZ() < 100.0f)PID_A.fire_command = 0;
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
