#include "PID.h"
extern union u8andfloat                                              //引用定位系统数据
{   
	uint8_t data[24];
	float ActVal[6];
}posture;
/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

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
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                  //系统中断优先级分组2
	TIM_Init(TIM2,999,83,0,0);                                       //时钟2初始化，1ms周期
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);                //can1初始化
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);                //can2初始化
	
	ElmoInit(CAN2);                                                  //驱动器初始化
	VelLoopCfg(CAN2,2,80000000,80000000);                            //左电机速度环初始化
	VelLoopCfg(CAN2,1,80000000,80000000);                            //右电机速度环初始化
	MotorOn(CAN2,1);                                                 //右电机使能
	MotorOn(CAN2,2);                                                 //左电机使能
	USART3_Init(115200);
	UART4_Init(921600);
	#if CarNumber == 4
	TIM_Delayms(TIM4,15000);
	#elif CarNumber == 1
	TIM_Delayms(TIM4,5000);
	driveGyro();
	#endif
//	MotorOff(CAN2,2);                                                //左电机失能
//	MotorOff(CAN2,1);                                                //右电机失能
	
	OSTaskSuspend(OS_PRIO_SELF);                                     //挂起初始化函数
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
	OSSemSet(PeriodSem, 0, &os_err);                            	 //信号量归零
	int cnt = 0;                                                     //计数用
	int lasttime = 0;                                                //计数用
	int time = 0;                                                    //计数用
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);                            //等信号量，10ms一次
//		PID_Line(-500,0,-500,-4000,1000);                         //走线函数
		PID_Sauare(1000);
		cnt++;
		if(cnt>799) cnt = 0;
		time = cnt/2;                                               //100ms周期发状态，只有x，y坐标
		if(lasttime != time)
		{
			USART_OUT(UART4,(uint8_t*)"%d	", (int)posture.ActVal[3]);
			USART_OUT(UART4,(uint8_t*)"%d	", (int)posture.ActVal[4]);
			
			USART_SendData(UART4,'\r');
			USART_SendData(UART4,'\n');
			lasttime = time;
		}
	}
}
