
#include "PID.h"
extern union u8andfloat                                              //引用定位系统数据
{   
	uint8_t data[24];
	float ActVal[6];
}posture;

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
#define PI (3.1415)                               //圆周率                 3.1415
#define One_Meter_Per_Second (10865.0)            //车轮一米每秒的设定值   4096*(1000/120π)
#define BaseVelocity (0.5 * One_Meter_Per_Second) //基础速度               0.5m/s
#define CarOne 1                                  //一号车编号             1
#define CarFour 4                                 //四号车编号             4
#define Side_Length (2000)                        //方形边长               2m


#define Pulse2mm COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi)

/*
一个脉冲是4096/(120*Pi)
定义输入速度mm/s和半径mm
*/
float ratio1,ratio2;
void vel_radious(float vel,float radious)
{
	ratio1=(radious+WHEEL_TREAD/2)/radious;
	ratio2=(radious-WHEEL_TREAD/2)/radious;
	VelCrl(CAN2,1,ratio1*vel*Pulse2mm);
	VelCrl(CAN2,2,-ratio2*vel*Pulse2mm);
}



/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
OS_EVENT *CPUUsageSem;
OS_EVENT *adc_msg;


extern Pos_t pos;

//目标坐标信息结构体，x为横坐标，y为纵坐标，angle为与0°的相对角度
typedef struct{ 
	float x;
	float y;
	float angle;
}Target;
Target tar;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];

static OS_STK RunTaskStk[RUN_TASK_STK_SIZE];

static int CarNum = CarOne;

int isOKFlag = 0;    //定位系统初始化完毕标志位
uint8_t opsFlag = 0; //收到定位系统数据标志位

//一直以5ms间隔发“AT/r/n”给定位系统，等待定位系统回数 OK定位系统初始化结束
int IsSendOK(void)
{
	return isOKFlag;
}
void SetOKFlagZero(void)
{
	isOKFlag = 0;
}
void driveGyro(void)
{
	while(!IsSendOK())
	{
		delay_ms(5);
		USART_SendData(USART3, 'A');
		USART_SendData(USART3, 'T');
		USART_SendData(USART3, '\r');
		USART_SendData(USART3, '\n');
	}
	SetOKFlagZero();
}

/**
* @brief  PID控制参数数据结构体
* @author 陈昕炜
* @note   定义一个PID_Value类型，此时PID_Value为一个数据类型标识符，数据类型为结构体
*/
typedef struct{
	float setValue;      //系统待调节量的设定值 
    float feedbackValue; //系统待调节量的反馈值，就是传感器实际测量的值    
     
    float Kp;            //比例系数
    float Ki;            //积分系数
    float Kd;            //微分系数
    
    
    float error;         //当前偏差
    float error_1;       //前一步的偏差
	float error_sum;     //偏差累计
	float error_def;     //当前偏差与上一偏差的差值
    
    float output;        //PID控制器的输出
    float out_max;       //输出上限
    float out_min;       //输出下限
}PID_Value;

PID_Value PID_Angle_;

/**
* @brief  PID控制器
* @param  *p 要进行计算输出的PID控制数据结构体指针
* @author 陈昕炜
* @note   用于PID中
*/
float PID_operation(PID_Value *p)
{
	p->Kp = 0.02;
	p->Ki = 0;
	p->Kd = 0.001;
	
	
	p->error = p->setValue - p->feedbackValue;
	p->error_sum = p->error_sum + p->error;
	p->error_def = p->error - p->error_1;
	p->error_1 = p->error;
	p->output = p->Kp * p->error + p->Ki * p->error_sum + p->Kd * p->error_def;
	return p->output;
}

/**
* @brief  浮点数限幅
* @param  amt：需要进行限幅的数
* @param  high：输出上限
* @param  low：输出下限
* @author 陈昕炜
* @note   constrain ->约束，限制
*/
float constrain_float(float amt, float high, float low) 
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
/**
* @brief  转速差实现转弯函数
* @param  speed：两车轮圆心连线中点的移动速度 单位米
* @param  radius：两车轮圆心连线中点的转弯半径 单位米
* @param  mode：转弯模式 1为右转弯 2为左转弯 3为向前直走
* @param  slowwheel：内侧轮的设定值
* @param  fastwheel：外侧轮的设定值
* @author 陈昕炜
* @note   用于WalkTask中
*/
void Go(float velocity, float radius, char mode)
{
	float slowwheel, fastwheel;
	//设定值 = 速度 * ( 转弯半径 - 车轮距离的一半）/ 转弯半径 * 车轮一米每秒的设定值
	slowwheel = velocity * (1000 * radius - WHEEL_TREAD / 2) / (1000 * radius) * One_Meter_Per_Second;
    fastwheel = velocity * (1000 * radius + WHEEL_TREAD / 2) / (1000 * radius) * One_Meter_Per_Second;
	if(mode == 1)//右转弯
	{
		VelCrl(CAN2, 1, (int)slowwheel);
		VelCrl(CAN2, 2, (int)(fastwheel * -1));
	}
	if(mode == 2)//左转弯
	{
		VelCrl(CAN2, 1, (int)fastwheel);
		VelCrl(CAN2, 2, (int)(slowwheel * -1));
	}
	if(mode == 3)//直走
	{
		VelCrl(CAN2, 1, (int)(velocity * One_Meter_Per_Second));
		VelCrl(CAN2, 2, (int)(velocity * One_Meter_Per_Second * -1));
	}
}


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
	CPUUsageSem = OSSemCreate(0);
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
//	os_err = OSTaskCreate((void (*)(void *))RunTask,
//						  (void *)0,
//						  (OS_STK *)&RunTaskStk[RUN_TASK_STK_SIZE-1],
//						  (INT8U)RUN_TASK_PRIO);	  

//						  (INT8U)Walk_TASK_PRIO);
//	OSTaskSuspend(OS_PRIO_SELF);

}



/*
===============================================================
                          初始化任务
===============================================================
*/
//float error = 0;
//void ConfigTask(void)
//{
//	int ADC_Left = 0;
//	int ADC_Right = 0;
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                  //系统中断优先级分组2
//	TIM_Init(TIM2,9999,83,0,0);                                        //时钟2初始化，1ms周期
//	Adc_Init();
//	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);                //can1初始化
//	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);                //can2初始化	
//	ElmoInit(CAN2);                                                  //驱动器初始化
//	VelLoopCfg(CAN2,2,80000000,80000000);                            //左电机速度环初始化
//	VelLoopCfg(CAN2,1,80000000,80000000);                            //右电机速度环初始化
//	MotorOn(CAN2,1);                                                 //右电机使能
//	MotorOn(CAN2,2);                                                 //左电机使能
//	USART3_Init(115200);
//	UART4_Init(921600);
//	#if CarNumber == 4
//	TIM_Delayms(TIM4,15000);
//	#elif CarNumber == 1
//	TIM_Delayms(TIM4,5000);
//	driveGyro();
//	#endif
//	float error1 = 0;
//	error1 = ((Get_Adc_Average(15,100) - Get_Adc_Average(14,100))/2)*0.922854;
//	OS_CPU_SR cpu_sr;
//	OS_ENTER_CRITICAL();                                         /*互斥访问*/
//	error = error1;
//	OS_EXIT_CRITICAL();
//	while(1)
//	{
//		ADC_Left = Get_Adc_Average(15,100);
//		ADC_Right = Get_Adc_Average(14,100);
//		if(ADC_Left<100)
//		{
//			OSMboxPost(adc_msg,(void *)3);
//			OSTaskSuspend(OS_PRIO_SELF);                                     //挂起初始化函数
//		}
//		else if(ADC_Right<100)
//		{
//			OSMboxPost(adc_msg,(void *)2);
//			OSTaskSuspend(OS_PRIO_SELF);                                     //挂起初始化函数
//		}
//		else USART_OUT(UART4,(uint8_t*)"%s","wait for adc data\r\n");		
//	}
////	MotorOff(CAN2,2);                                                //左电机失能
////	MotorOff(CAN2,1);                                                //右电机失能	
//}

/*
===============================================================
                   WalkTask      初始化后执行
===============================================================
*/
//int cnt = 0;                                                     //计数用
//void WalkTask(void)
//{
//	CPU_INT08U os_err;
//	os_err = os_err;                                                 //防报错
//	OSSemSet(PeriodSem, 0, &os_err);                 	 //信号量归零
//	int lasttime = 0;                                                //计数用
//	int time = 0;                                                    //计数用
//	while (1)
//	{
//		OSSemPend(PeriodSem, 0, &os_err);                            //等信号量，10ms一次
//		static u32 direction = 0;
//		if(direction == 0) direction = (u32)OSMboxPend(adc_msg,0,&os_err);
////		PID_Line(-500,0,-500,1000,500);                         //走线函数
////		PID_Square(1000);
////		PID_Round(0,2000,1000,1000,direction);
////		PID_Coordinate_following(500);
//		OS_CPU_SR cpu_sr;
//		OS_ENTER_CRITICAL();                                         /*互斥访问*/
//		PID_RUN(500,direction);
//		OS_EXIT_CRITICAL();
////	    PID_Square_x(1000,500,direction);
////		OS_CPU_SR cpu_sr;
////		OS_ENTER_CRITICAL(); /*互斥访问*/
//	    cnt++; 
//		if(cnt>799) cnt = 0;
//		time = cnt/1;                                               //10ms周期发状态，只有x，y坐标 
////		OS_EXIT_CRITICAL();
//		if(lasttime != time)
//		{
//			USART_OUT(UART4,(uint8_t*)"%d	", (int)posture.ActVal[3]);
//			USART_OUT(UART4,(uint8_t*)"%d	", (int)posture.ActVal[4]);
//			USART_OUT(UART4,(uint8_t*)"%d	", (int)Get_Adc_Average(15,10));
//			USART_OUT(UART4,(uint8_t*)"%d	", (int)Get_Adc_Average(14,10));
//			USART_SendData(UART4,'\r');
//			USART_SendData(UART4,'\n');
//			lasttime = time;
//		}
//	}
//}

//void RunTask(void)
//{
//	CPU_INT08U os_err;
//	os_err = os_err;                                                 //防报错
//	OS_CPU_SR cpu_sr;
//	OS_ENTER_CRITICAL(); /*互斥访问*/
//	int last_pdate = cnt;
//	OS_EXIT_CRITICAL();
//	int pdate_now = last_pdate;
//	static int cnt1 = 100;
//	while(1)
//	{
//		OSSemPend(CPUUsageSem,0,&os_err);		
//		cnt1--;
//		OS_CPU_SR cpu_sr;
//		OS_ENTER_CRITICAL();                                          /*互斥访问*/
//		pdate_now = cnt;
//		OS_EXIT_CRITICAL();
//		if(pdate_now != last_pdate)
//		{
//			USART_OUT(UART4,(uint8_t*)"CPU load is: %d",cnt1);
//			USART_SendData(UART4,'%');
//			USART_SendData(UART4,'\r');
//			USART_SendData(UART4,'\n');
//			cnt1 = 100;
//			last_pdate = pdate_now;
//		}
//		OSSemSet(CPUUsageSem, 0, &os_err);                            	 //信号量归零               
//	}
//}
void ConfigTask(void)
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
	
	
	TIM_Init(TIM2, 999, 83, 0, 0);
	USART3_Init(115200);
	UART4_Init(921600);
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);
	
	ElmoInit(CAN2);
	VelLoopCfg(CAN2, 1, 2333333, 2333333);
	VelLoopCfg(CAN2, 2, 2333333, 2333333);
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	//一号车定位系统初始化
	if(CarNum == CarOne)
	{
		delay_s(2);
		driveGyro();
		while(!opsFlag);
	}
	//四号车定位系统初始化
	if(CarNum == CarFour)
	{
		delay_s(10);
	}
	OSTaskSuspend(OS_PRIO_SELF);
	
}

int cntTurn = 0, cntSendTime = 0;
char switchNextModeFlag = 1, adjustFlag = 0, turnFlag = 0;
float adjustVelocity, baseVelocity;

void WalkTask(void)
{
	int cntSendTime;
	CPU_INT08U os_err;
	os_err = os_err;
	tar.angle = pos.angle;

	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{

		OSSemPend(PeriodSem, 0, &os_err);

		
		//以50 * 10ms为间隔发送数据
		cntSendTime++;
		cntSendTime = cntSendTime % 50;
		if(cntSendTime == 1)
		{
            USART_OUT(UART4, (uint8_t*)"x=%d,y=%d,ang=%d,tx=%d,ty=%d,ta=%d,cT=%d,tF=%d\r\n", (int)pos.x, (int)pos.y, (int)pos.angle,(int)tar.x, (int)tar.y, (int)tar.angle, (int)cntTurn, (int)turnFlag);
		}
		
        //到达目标地点后，turnFlag = 1开始转弯
		if(cntTurn == 0)
		{
            if(tar.x > pos.x){turnFlag = 1;}
		}
		if(cntTurn == 1)
		{	
			if(tar.y > pos.y){turnFlag = 1;}
		}
		if(cntTurn == 2)
		{	
			if(tar.x < pos.x){turnFlag = 1;}
		}
		if(cntTurn == 3)
		{
			if(tar.y < pos.y){turnFlag = 1;}
		}
		
		//设置转弯目标角度
		if(turnFlag == 1)
		{
			
			switch (cntTurn)
		    {
		    case 0:
			    tar.angle = -90;
			    break;
		    case 1:
			    tar.angle = -180;			    
			    break;
		    case 2:
			    tar.angle = 90;			
			    break;
		    case 3:
			    tar.angle = 0;					
			    break;
		    }
		}
		
		//转弯时速度值完全由PID输出，直线行驶时加上基础速度
		if(turnFlag == 1)
		{
			baseVelocity = 0;
		}
		if(turnFlag == 0)
		{
			baseVelocity = BaseVelocity;
		}
		
        //根据当前角度正负调整目标角度，使转弯圆弧始终为劣弧
		if(tar.angle == 180 || tar.angle == -180)
		{
			if(pos.angle > 0)
			{
				tar.angle = 180;
			}
			if(pos.angle < 0)
			{
				tar.angle = -180;
			}
		}
		
	    
		PID_Angle_.setValue = tar.angle;
        PID_Angle_.feedbackValue = pos.angle;		
		PID_Angle_.error = PID_Angle_.setValue - PID_Angle_.feedbackValue;

//		adjustVelocity = PID_operation(&PID_Angle) * One_Meter_Per_Second;//逆时针旋转时，四号车adjustVelocity为负数
		//adjustVelocity = constrain_float(adjustVelocity, PID_Angle.out_max, PID_Angle.out_min);
		
		VelCrl(CAN2, 1, (int)(baseVelocity - adjustVelocity));//右轮
		VelCrl(CAN2, 2, (int)((baseVelocity + adjustVelocity) * -1));//左轮

		//角度误差在允许范围内时停止转弯turnFlag = 0，设置下一个目标的位置信息switchNextModeFlag = 1，转弯计数加1
		int Angle_Error_Range = 1;
		if((fabs(tar.angle - pos.angle) < Angle_Error_Range) && (turnFlag == 1))
		{
			turnFlag = 0;
			switchNextModeFlag = 1;
			cntTurn++;
		}
		cntTurn = cntTurn % 4;
		//设置下一个目标的位置信息
		if(switchNextModeFlag == 1)
		{
			switchNextModeFlag = 0;
			switch (cntTurn)
		    {
		    case 0:
				tar.x = pos.x - Side_Length;
			    tar.y = pos.y;
			    tar.angle = 0;
			    break;
		    case 1:
				tar.x = pos.x;
			    tar.y = pos.y - Side_Length;
			    tar.angle = -90;			    
			    break;
		    case 2:
				tar.x = pos.x + Side_Length;
			    tar.y = pos.y;
			    tar.angle = -180;			
			    break;
		    case 3:
				tar.x = pos.x;
			    tar.y = pos.y + Side_Length;
			    tar.angle = 90;					
			    break;
		    }
		}
		OSSemSet(PeriodSem, 0, &os_err);

		vel_radious(500.0,500.0);			//半径为0.5m，速度为0.5m/s

	}
}
