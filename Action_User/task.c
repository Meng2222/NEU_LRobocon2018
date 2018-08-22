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
#include "pps.h"
#include "moveBase.h"
#include "math.h"
#define PAI 3.14
// 宏定义棍子收球电机ID
#define COLLECT_BALL_ID (8)
// 宏定义推球电机ID
#define PUSH_BALL_ID (6)
// 宏定义送弹机构送弹时电机应该到达位置：单位位脉冲
#define PUSH_POSITION (4500)
// 宏定义送弹机构收回时电机位置
#define PUSH_RESET_POSITION (5)
/// 宏定义发射机构航向电机ID
#define GUN_YAW_ID (7)
// 电机旋转一周的脉冲数
#define COUNT_PER_ROUND (4096.0f)
// 宏定义每度对应脉冲数
#define COUNT_PER_DEGREE  (COUNT_PER_ROUND/360.0f)
// 宏定义航向角减速比
#define YAW_REDUCTION_RATIO (4.0f)


int car=44;
int Car=1;  
int Angle=0;     //蓝牙发数的整型转换
int X=0;         //蓝牙发数的整型转换
int Y=0;
int max=8000;//最大偏差
//调车参数///////////////
float Ierr=0;//圆形
float I_value=0;//圆形
float KI=-0.12;//圆形
float rate=0.09;// 正方形  1m/s距离转化角度的比例为（0.06）,,1.5m/s为0.04,           0.15
   //正方形  提前量：duty=650(1m/s)，   1.5m/s duty为900，
float KP=450; //Kp给300(1m/s)，，，kp给450（1.5m/s）
float V0=0.5;//车的基础速度(m/s)
float buff=500;//(最大偏离区域)
//////////////////////////////////////////////////////
	//x=-400
	float Aa=-1;
	float Bb=0;
	float Cc=-400;
	float Nn=1;

//蓝牙发数的整型转换
static int isOKFlag=0;
static PosSend_t posture={0};

float out_angle=0;//距离环换算的角度
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
u16 Get_Adc(u8 ch);
u16 Get_Adc_Average(u8 ch,u8 times);//ADC磨块
void  Adc_Init(void);
void  go_straight(float V,int ElmoNum1,int ElmoNum2 ,CAN_TypeDef* CANx);
void  go_round(float V,float R,int ElmoNum1,int ElmoNum2 ,CAN_TypeDef* CANx);
void straight(float setValue,float feedbackValue);
void turn(float setValue,float feedbackValue,float direc);//原地转
float err(float distance,float ANGLE);//距离转化角度函数
float a_gen(float a,float b,int n);//角度生成函数（输入直线参数，给出角度）
void run_to(float BETA,float Vm);//跑向特定方向BETA
int line(float aa, float bb, float cc, float nn,float VL);//车在干扰下跑向任意直线·，注：aa<0,当aa=0时，bb=0；nn=1控制向X轴上方走，n=-1向x轴下方，n=-2向x轴正向走，n=2向x轴负向走
void loop(float corex,float corey,float Radium,float V_loop,int SN);//顺时针转
void square(float length,float square_corex,float square_corey,int square_sn,float Vsq,float duty);//闭环正方形
int ADC_judge();//ADC判断朝哪个方向
int accident_check();//检测障碍
float w_to_paulse(float w);//角速度转化脉冲rad/s(新车)
float new_meters_paulse(float new_meters);//新车的速度m/s转化脉冲（新车）
float meters(float V1)//米每秒转化成脉冲每秒

{
	float Va;
	Va=4096*V1/(PAI*0.12);
	return Va;
} 
// 将角度转换为脉冲
float YawTransform(float yawAngle)
{
	return (yawAngle * YAW_REDUCTION_RATIO * COUNT_PER_DEGREE);
}
//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
void YawAngleCtr(float yawAngle)
{
	PosCrl(CAN1, GUN_YAW_ID,ABSOLUTE_MODE, YawTransform(yawAngle));
}

  	   typedef struct{
	float x;
	float y;
	float angle;
	}pos_t;
pos_t action;


   #define x1_pos 0
   #define y1_pos 0
   #define x2_pos 2000
   #define y2_pos 2000
	//float aa=0;     //（输入aa的值应该a<0）
    //float bb=0;        //(如果a=0，b应该也>0)
    //float cc=0;
	int cycle_flag=0;//cycle_flag为全局变量，两个函数都能改变(autoPID)
	int change=1;//change为全局变量，两个函数都能改变(auto_PID)
	float Ra=2000;//(可变半径)
	int add_flag=-1;//(半径增减的标志位)
	int start_flag=0;
	int sn=0;//如果sn为1则逆时针，sn为-1则顺时针
	float last_x=0;
	float last_y=0;
	float last_angle=0;
	//后退的程序变量

	int back_flag=0;
	int mark_flag=0;
	float back_angel=0;//被卡住的时刻的角度
	float move_angel=0;//需要转到的角度
	float back_sn=1;//后退的时候转的方向
	float back_time=0;//卡住的时间
	float back_accident=0;
	int accident_flag=1;//意外判断标识符
	int lets_go=1;//(正常走)
	int choose=0;
	uint8_t square_flag=0;
	int record=0;
	int Last_angle=0;
	int Last_x=0;
	int Last_y=0;
	int R_switch=0;
	int r_switch=0;
    int push=0;//推球机构的运行周期控制
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
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	TIM_Init(TIM2, 999, 839, 0x00, 0x00);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	//Adc_Init();
	//驱动器初始化
	//ElmoInit( CAN2);
	ElmoInit( CAN2);
	//速度环和位置环初始化
	//右轮
	//VelLoopCfg(CAN2, 1, 50000, 10000);
	//后轮
	VelLoopCfg(CAN2, 5, 10000000, 1000000);
	//PosLoopCfg(CAN2, 1, 100, 100,0);
	//左轮
	//VelLoopCfg(CAN2, 2, 50000, 10000);
	//前轮
	VelLoopCfg(CAN2, 6, 10000000, 1000000);
	//PosLoopCfg(CAN2, 2, 100, 50000,10000);
	// 配置速度环
   // VelLoopCfg(CAN1, 8, 50000, 50000);
    // 控制电机的转速，脉冲。
    //VelCrl(CAN1,COLLECT_BALL_ID,60*4096); 
	// 配置位置环
    //PosLoopCfg(CAN1, PUSH_BALL_ID, 50000,50000,20000);
   
	//航向电机   
   //PosLoopCfg(CAN1, GUN_YAW_ID, 50000,50000,20000);

	
	
	//电机使能
//	MotorOn(CAN2, 01);旧车
//	MotorOn(CAN2, 02);旧车
	//	MotorOn(CAN1, 07);炮台机构
	//	MotorOn(CAN1, 06);炮台机构
	//	MotorOn(CAN1, 8);炮台机构
	MotorOn(CAN2, 05);//新车
	MotorOn(CAN2, 06);//新车
	//定位系统串口初始化
	USART3_Init(115200);
	//航向电机串口初始化
	//USART1_Init(115200);
	/*一直等待定位系统初始化完成*/
	delay_s(2);
	WaitOpsPrepare();
	 //蓝牙调试串口  旧车4，新车1
	 USART1_Init(921600); 
	 //给电机发数
	 // SendUint8();

//     if(Car==4)
	// delay_s(10);
    



//	 while(ADC_judge()==0);
//	 if(ADC_judge()==1)//左边被挡住
//		 sn=1;
//	  if(ADC_judge()==-1)//右边被挡住
//		 sn=-1;
	 OSTaskSuspend(OS_PRIO_SELF);


}



int mission=1;

void WalkTask(void)
{
	extern float Uk_1,Uk_2;
	int uk_1;
	int uk_2;
	float err_1=0;
	  float Aa=0;
      float Bb=0;
      float Cc=0;
      float Nn=0;

	  
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{

		OSSemPend(PeriodSem, 0, &os_err);

		

	
 
	if(car==40)//离散点走直线
	{
		int a_flag=0;
		int d_flag=0;
		int ar_flag=0;
		//把直线离散成点

	float angle=0;
	float k=0;
	float b=0;
	float x_pos[100]={0};
	float y_pos[100]={0};
    int pos_cnt=0;//算点
	int pos_num=0;//走点
//    float distance;	
//	float d_value=0;
//	float d_err=0;
//	float d_Uk=0;
//	float alpha=0;
//	float a_value=0;
//	float a_Uk=0;
	//设定直线的角度
	if(x1_pos!=x2_pos)//除掉x=a的所有线
	{
		k=(y1_pos-y2_pos)/(x1_pos-x2_pos);
		b=y1_pos-k*(x1_pos);
		angle=atan(k);
	}
	if(x1_pos==x2_pos)//x=a的直线
	{
		angle=90;
		b=x1_pos;
		
	}
	
	//确定初始位置
	if(k>=0)
    x_pos[0]=-2000;
	if(k<0)
	x_pos[0]=2000;
	
	if(y1_pos==y2_pos)//x=a的直线
	x_pos[0]=x1_pos;
	//标记每个点坐标
	for(pos_cnt=1;pos_cnt<100;pos_cnt++)
	{
		x_pos[pos_cnt]=x_pos[pos_cnt]+50;
		y_pos[pos_cnt]=x_pos[pos_cnt]*k+b;	
	}
		
	
		
		
		if(mission==1)//走到起点
		{
			d_flag=go_to(0,0,x_pos[0],y_pos[0]);
			if(d_flag==1)
			{
				mission=2;
				d_flag=0;
			}
	    }
		if(mission==2)//到直线上了，往直线方向调整角度
		{
			a_flag=adjust_angle(angle);
		    if(a_flag==1)
			{
				mission=3;
			    a_flag=0;
			}
		}
		if(mission==3)//开始走直线
		{   
			ar_flag=patrolline(x_pos[pos_num],y_pos[pos_num],2000);//输入要去的点的坐标(和巡线速度V)
			if(ar_flag==1)//如果到达那个点就继续走下一个点
			pos_num++;
		}
		
		if(action.x-x_pos[pos_num]>400||action.x-x_pos[pos_num]<-400||action.y-y_pos[pos_num]>400||action.y-y_pos[pos_num]<-400)//检测跑偏
				mission=4;
		
		if(mission==4)
		{
			go_to(action.x,action.y,x_pos[pos_num+4],y_pos[pos_num]);
			if((action.x-x_pos[pos_num+4]<20)&&(action.y-y_pos[pos_num]<20))
				mission=3;
		}
	USART_OUT(UART4,(uint8_t*)"mission");
	USART_OUT( UART4, (uint8_t*)"%d ", mission);
    USART_OUT(UART4,(uint8_t*)"x=");
	USART_OUT( UART4, (uint8_t*)"%d ", action.x);
	 USART_OUT(UART4,(uint8_t*)"y=");
	USART_OUT( UART4, (uint8_t*)"%d ", action.y);
	USART_OUT(UART4,(uint8_t*)"angle=");
	USART_OUT( UART4, (uint8_t*)"%d ", action.angle);
    USART_OUT(UART4,(uint8_t*)"pos_x=");
	USART_OUT( UART4, (uint8_t*)"%d ",x_pos[pos_num]);		
	USART_OUT(UART4,(uint8_t*)"pos_y=");
	USART_OUT( UART4, (uint8_t*)"%d ",y_pos[pos_num]);		
	USART_OUT(UART4,(uint8_t*)"\r\n");	
		
		
		
  }
 
  
 
  
  if(car==44)
{
	  if( sn==0)
{	  
	if(record==50)
	  {
		  push++;
//		  if(accident_check()==0)
//		    lets_go=1;
//		  else lets_go=0;
//		    last_angle=action.angle;
//            last_x=action.x;
//            last_y=action.y;
//		    record=0;
//		  Last_angle=(int)last_angle;
//		  Last_x=(int)last_x;
//		  Last_y=(int)last_y;
//		  USART_OUT( UART4, (uint8_t*)"%d ", Last_angle);
//		  USART_OUT( UART4, (uint8_t*)"%d ", Last_x);
//		  USART_OUT( UART4, (uint8_t*)"%d ", Last_y);
//		  USART_OUT(UART4,(uint8_t*)"\r\n");
	  }//
    if(lets_go==1)
    {
//	  if(action.y>0&&last_y<0&&action.x>-2400)
//	  {
//		R_switch=1;  
//	    if(R_switch!=r_switch)  
//  { 
//	   if(add_flag==-1)
//	   Ra=Ra-200;
//	   if(add_flag==1)
//	   Ra=Ra+200;
//      
//	 
//	  if(Ra<=800)	  
//	   {
//		   choose++;
//		   if(choose%2==1)//正方形和圆形的切换
//		   {
//			   square_flag=2;
//			   add_flag=0;
//			    USART_OUT( UART4, (uint8_t*)"%d ", choose);
//		   }
//			   if(choose%2==0)
//		   {
//			   add_flag=1;
//			   square_flag=0;
//			    USART_OUT( UART4, (uint8_t*)"%d ", choose);
//			   choose=0;
//		   }
//	   }
//	   if(Ra>=2000)
//	   {
//	     choose++;
//		   if(choose%2==1)
//		   {
//			   square_flag=2;    //1为小正方形，2为大正方形
//			    USART_OUT( UART4, (uint8_t*)"%d ", choose);
//			   add_flag=0;
//		   }
//		   if(choose%2==0)
//		   {
//			  add_flag=-1;
//			  square_flag=0;
//			    USART_OUT( UART4, (uint8_t*)"%d ", choose);
//			  choose=0;
//		   }
//	   }
//   }  r_switch=R_switch;
//	  }	  else R_switch=0;
//	 r_switch =R_switch;
//	  if(square_flag==0)
//	  {
//		  if((fabs(action.y)<2050&&fabs(action.y)>1950&&action.x>-2350&&action.x<2450)||(action.x>-4450&&action.x<-4350&&fabs(action.y)<50))//最大的圈会冲一下
//	  {
//	   VelCrl(CAN2, 01,18000);
//	   VelCrl(CAN2, 02,-18000); 
//	  }
//	  else  loop(-2400,0,Ra,Ra/1000,sn);//逆时针转为1,顺时针为-1
//      }
//	  if(square_flag==1)
//	square(1200,-2400,0,1,1,0);//走到半径最小值开始走边长1200的正方形，（边长，x，y顺逆）
//	  if(square_flag==2)
//	square(3600,-2400,0,1,2,800);//走到半径最大开始走边长4000的正方形	  
	  X=(int)action.x;
	  Y=(int)action.y;
      Angle=(int)action.angle;	
//     int rr=(int)Ra;
//	  
//	  USART_OUT( UART4, (uint8_t*)"%d ", rr);
	  USART_OUT( USART1, (uint8_t*)"%d ", X);
	  USART_OUT( USART1, (uint8_t*)"%d ", Y);
	  USART_OUT( USART1, (uint8_t*)"%d ", Angle);
//	  USART_OUT( UART4, (uint8_t*)"%d ", square_flag);
//	 
//      USART_OUT( UART4, (uint8_t*)"%d ", record);
//	  
	  USART_OUT(USART1,(uint8_t*)"\r\n");
//	  
//	  //撞墙判断程序
//	  record++;
//    YawAngleCtr(90.0f); 													旧车炮台系统
//	// 推球
//	if(push/2==1)                                                           车炮台系统
//    PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);				车炮台系统
//	if(push/2==0)															车炮台系统
//    // 复位   															车炮台系统
//	PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);			车炮台系统
//	if(push==10000)															车炮台系统
//		push=0;	

	   VelCrl(CAN2, 05,new_meters_paulse(1));//驱动轮V
	   VelCrl(CAN2, 06,w_to_paulse(2));//转向轮W

    }//accident_check=0无故障运行
}//sn!=0//启动
	

  }//car=44	
	
	}//while
}//task

int cnt_flag=0;
float last_amount=0;
float last_P0=0;
float last_cycle_num=0;


void accident_turn(int turn_dire)//单轮转90度,-1为（逆时针），1为顺时针
{
float turn_Setvalue=0;
float turn_err=0;
int turn_time=0;
uint8_t turn_over=0;//是否转到指定角度标志位
 if(turn_dire==-1)//向右转
 {
	turn_Setvalue=action.angle-90;
    if(turn_Setvalue<-180)
    turn_Setvalue=turn_Setvalue+360;//超过-180
	do
	{
	turn_time++;
	if(turn_time==50)
	{
		turn_err=turn_Setvalue-action.angle;//偏差值为负
	    VelCrl(CAN2, 01,0);
		VelCrl(CAN2, 02,-turn_err*100); //右轮不转，左轮反	
	turn_time=0;
	}
     if(fabs(fabs(action.angle)-fabs(action.angle))<10)
		 turn_over=1;
	}while(turn_over==1);//转到指定角度
 }
 if(turn_dire==1)
 {
	 turn_Setvalue=action.angle+90;
	 if(turn_Setvalue>180)
     turn_Setvalue=turn_Setvalue-360;//超过-180
	 do
	{
		turn_time++;
		if(turn_time==50)
		{
			turn_err=turn_Setvalue-action.angle;//偏差值为正
			VelCrl(CAN2, 01,-turn_err*100);
		    VelCrl(CAN2, 02,0); //右轮不转，左轮反	
			turn_time=0;
		}
		  if(fabs(fabs(action.angle)-fabs(action.angle))<10)
		  turn_over=1;
	 }
	
	 while(turn_over==1);
 }
}
int accident_check()
{
	float check_angel=0;
	float last_check_angel=0;
	uint8_t accident_conner=0;//意外判断--卡在角落
	uint8_t accident_back=0;//意外判断--被撞退
    uint8_t accident_ahead=0;	//意外判断--遇到阻力
	uint8_t accident_frame=0;//意外判断--碰到边框
	uint8_t accident_clear=0;//意外是否清除
//	check_angel=atan2(action.y-0,action.x-(-2400))*180/PAI;
//	last_check_angel=atan2(last_y-0,last_x-(-2400))*180/PAI;
	
	
	if(fabs(action.x-last_x)<100&&fabs(action.y-last_y)<100)//被卡住
	{
		accident_back=1;
//			accident_clear=1;
//		if(action.x<-4300&&action.y>1900&action.y<2900)//卡在右上角
//		{
//			if(action.angle==0)//头朝上
//			accident_conner=1;//
//			if(action.angle==90)//头朝右
//			accident_conner=2;
//		}
//		if(action.x<-4300&&action.y<-1900)//卡z在左上角
//	{	
//		if(action.angle==0)
//			accident_conner=3;//头朝上
//		if(action.angle==-90)
//		   accident_conner=4;	//头朝左
//	}
//	   if(action.x>-500&&action.y<-1900)//卡在左下角
//	   {
//		   if(action.angle==180&&action.angle==-179)//头朝下
//			   accident_conner=5;
//		   if(action.angle==-90)//头朝左
//			   accident_conner=6;
//	   }
//	   if(action.x<-500&&action.y>1900)//卡在右下角
//		   if(action.angle==180&&action.angle==-179)//头朝下
//			   accident_conner=7;
//		   if(action.angle==90)//头朝右
//			   accident_conner=8;
//		if(action.x<=-500&&action.x>=-4300&&action.y>1900)  //卡在右边框 
//			 accident_frame=1;
//		if(action.y<=-1900&&action.y>=1900&&action.x<-4300)//卡在上边框
//		     accident_frame=2;
//		if(action.x<=-500&&action.x>=-4300&&action.y<-1900)//卡在左边框
//			accident_frame=3;
//		if(action.y<=-1900&&action.y>=1900&&action.x>-500)//卡在下边框
//			accident_frame=4;
	}
	//可能需要知道顺时针还是逆时针
	{
		if(accident_back==1)//被撞退后退，去下一个圆
		{
			for(back_time=0;back_time<1000;back_time++)
			{
				VelCrl(CAN2, 01,-10000);//倒车· 
	            VelCrl(CAN2, 02,10000); //倒车
			}
			if(add_flag==-1)
				Ra=Ra-200;
			if(add_flag==1)
				Ra=Ra+200;
			if(Ra>=2200)
			{	
			   Ra=1800;
			   add_flag=-1;
			}
			if(Ra<=600)
			{
			   Ra=800;
				add_flag=1;
			}
			accident_back=0;//故障解除	
		}
		if(accident_ahead==1)//遇到阻力，向前加速
		{
			for(back_time=0;back_time<100;back_time++)
			{
			if(Ra<=1600)
			loop(-2400,0,Ra,(Ra/1000)+0.5,sn);
		     }
			accident_ahead=0;//故障解除
		}
		if(accident_conner!=0)
		{
			for(back_time=0;back_time<50;back_time++)
				{
					VelCrl(CAN2, 01,-10000);//倒车· 
					VelCrl(CAN2, 02,10000); //倒车
				}
			if(accident_conner==1||accident_conner==4||accident_conner==5||accident_conner==8)
			accident_turn(-1);
			if(accident_conner==2||accident_conner==3||accident_conner==6||accident_conner==7)
			accident_turn(1);
			accident_conner=0;//故障解除
		}
		if(accident_frame==1)
		{
		while(accident_frame!=0)
		{
			back_time++;
			if(back_time==50)
			{
				turn(0,action.angle,1);//原地单轮转到固定角度
				back_time=0;
			}
			if(fabs(action.angle)<5)
				accident_frame=0;
		}
		}
		
		
		if(accident_frame==2)
		{
		while(accident_frame!=0)
		{
			back_time++;
			if(back_time==50)
			{
				turn(-90,action.angle,1);//原地单轮转到固定角度
				back_time=0;
			}
			if(fabs(action.angle+90)<5)
				accident_frame=0;
		}
		}
		
		
		if(accident_frame==4)
		{
		while(accident_frame!=0)
		{
			back_time++;
			if(back_time==50)
			{
				turn(90,action.angle,1);//原地单轮转到固定角度
				back_time=0;
			}
			if(fabs(action.angle-90)<5)
				accident_frame=0;
		}
		}
		
		if(accident_frame==3)
		{
		while(accident_frame!=0)
		{
			back_time++;
			if(back_time==50)
			{
				turn(180,action.angle,1);//原地单轮转到固定角度
				back_time=0;
			}
			if(((action.angle>0)&&(180-action.angle)<10)||((action.angle<0)&&(action.angle+180)<10))
				accident_frame=0;
		}
		}
		
	}
	
	if(accident_back==0)
	   accident_clear=0;
	
	USART_OUT( UART4, (uint8_t*)"%d ", accident_back);
	USART_OUT( UART4, (uint8_t*)"%d ", accident_conner);
	USART_OUT( UART4, (uint8_t*)"%d ", accident_frame);
	USART_OUT( UART4, (uint8_t*)"%d ", accident_clear);
	return accident_clear;
}



int ADC_judge()
{
	float Analog_L=0;
	float Analog_R=0;
	float judge_L=10000;
	float judge_R=10000;
	Analog_L=Get_Adc_Average(15,10);	
   // judge_L=(100000.0f/4096.0f)*Analog_L;
	Analog_R=Get_Adc_Average(14,10);	
    //judge_R=(100000.0f/4096.0f)*Analog_R;
//	if(Analog_R<200)
//		start_flag=-1;
	if(Analog_L<500)
		start_flag=1;
	int Analog_l=0;
		Analog_l=(int)Analog_L;
	int Analog_r=0;
        Analog_r=(int)Analog_R;
	USART_OUT(UART4,(uint8_t*)"%d ",Analog_l);
	USART_OUT(UART4,(uint8_t*)"%d ",Analog_r);
	USART_OUT(UART4,(uint8_t*)"%d ",start_flag);
	return start_flag;
	
}
float auto_PID(float P0)//输入值是当前圈数和是否走完一圈标号
{
	float amount_x=0;
	float amount_d=0;
	float amount=0;
	float round_s1=0;
    float round_s2=0;
	float round_R=0;
	float round_core=0;
	float per=0;//为了方便最好把这个放上边
	if(change==2&&action.y>1000)
		cnt_flag=1;
	if(change==3&&action.x>-1000)
		cnt_flag=0;//不在记录偏差的时候
	if(cnt_flag==1)
	{
	if(action.y>round_s1&&action.x<round_s2)//y在圆弧的方形区域,记录偏差用弧形比较
		amount_d=amount_d+fabs((action.x-round_s2)*(action.x-round_s2)+(action.y-round_s1)*(action.y-round_s1)-round_R);
	if(action.y>1000&&action.x<round_s2&&action.y<round_s1)//y在1000到圆弧下边缘并且X在圆弧做边缘左边的区域，记录偏差用x=-2000比较
		amount_x=amount_x+fabs(action.x-2000);
    }
	
	if(cnt_flag==0&&change==1)//出了纪录偏差的范围，检查偏差值并给出KP
	{
		amount=amount_d+amount_x;
		if(last_amount>amount)//如果本次偏差小于上次偏差，就把上次偏差(P0)覆盖，作为下一次的比较值，记录本次的KP作为比较值
		{
			last_amount=amount;
			last_P0=P0;//     last_P0用串口显示出来
		}
		if(cycle_flag==1)//清除标志位，当车跑完一圈如果偏差值没有小于之前的的话那就试下一个P0
			{
				cycle_flag=0;
				if(last_amount<=amount)
				  P0=P0+per;	
			}
	}
	return P0;
}











void square(float length,float square_corex,float square_corey,int square_sn,float Vsq,float duty)
{

	if(square_sn==1)//逆时针
{
if((action.y>(square_corey+0.5*length-duty))&&change==1)//到达转换该y=1/2 length的地方
{//y=coreyy+0.5 length
	Aa=0;
	Bb=1;
	Cc=-square_corey-(0.5)*length;
	Nn=-2;
	change=2;
}	

if((action.x<square_corex-0.5*length+duty)&&change==2)
{//x=corex-0.5*lenth
	Aa=-1;
	Bb=0;
	Cc=square_corex-0.5*length;
	Nn=-1;
    change=3;
}
if((action.y<square_corey-0.5*length+duty)&&change==3)
{//y=corey-0.5*lenth
	Aa=0;
	Bb=1;
	Cc=-square_corey+0.5*length;
	Nn=2;
    change=4;
}
if((action.x>square_corex+0.5*length-duty)&&change==4)
{//x=corex+0.5lenth
	Aa=-1;
	Bb=0;
	Cc=square_corex+0.5*length;
	Nn=1;
    change=1;
}
}
//执行
if(Vsq>=1.5)
	Vsq=1;
if(Vsq<=1)
	Vsq=1;
line(Aa, Bb, Cc, Nn,Vsq);
 USART_OUT( UART4, (uint8_t*)"%d ", change);
   }   



float round_gen(float vp,float BUff)//未完成
{
	float Round_core=0;
	
	
	
	
	
	
	
	return Round_core;
}







float a_gen(float a,float b,int n)//输出给定直线的方向角
{
	float ANGLE=0;//直线给定角度
	if(n==1)
	{
		if(b==0)
			ANGLE=90;
		if(b!=0)
		{
			if(-a/b>0)//K>0且向上atan>0
			ANGLE=180-(atan(-a/b))*180/PAI;//范围在90~180
			if(-a/b<0)//atan<0
			ANGLE=-atan(-a/b)*180/PAI;//范围在0~90度				
		}
		
	}
	if(n==-1)
	{
		if(b==0)
			ANGLE=-90;
		if(b!=0)
		{
			if(-a/b>0)
			ANGLE=-atan(-a/b)*180/PAI;//范围在0~-90度
			if(-a/b<0)
			ANGLE=-180-(atan(-a/b))*180/PAI;//范围在-90~-180度
		}
	}
	if(n==2)//Y=0，x轴正向
	{
		ANGLE=180;
	}
	if(n==-2)//y=0，x轴负向，初始方向
	{
		ANGLE=0;
	}
	
	return ANGLE;
}



float err(float distance,float ANGLE)//距离偏差转化辅助角度函数
	{
		float d_angle;//距离转化的角度
		
		if(distance<0)
		{
	
			if(ANGLE<90&&ANGLE>=0)//在0~90度方向下方
			{
				if(distance>=-buff)//buff为最大偏离区域，超过这个区域就垂直走回
				d_angle=ANGLE+fabs(distance*rate);//
				if(distance<-buff)
				d_angle=ANGLE+90;
			}
			if((ANGLE>90)&&(ANGLE<=180))//在90~180度下方
			{
				if(distance>=-buff)
				d_angle=ANGLE-fabs(distance*rate);
			    if(distance<-buff)
				d_angle=ANGLE-90;
			}			
			if(ANGLE<0&&ANGLE>-90)//在0~-90度下方
			{
				if(distance>=-buff)
				d_angle=ANGLE+fabs(distance*rate);
			    if(distance<-buff)
				d_angle=ANGLE+90;
			}
			if((ANGLE<-90)&&(ANGLE>-180))//在-90~-180度下方
			{
				if(distance>=-buff)
				{
					d_angle=ANGLE-fabs(distance*rate);
				if(d_angle<-180)//越过-180边界，
					d_angle=d_angle+360;
			    }
				if(distance<-buff)
					d_angle=ANGLE-90+360;
		    }
			//边界情况
			if(ANGLE==90)//90度的位置的右边边
			{
				if(distance>=-buff)
				d_angle=90-fabs(distance*rate);
				if(distance<-buff)
				d_angle=0;
			}
			if(ANGLE==-90)//-90度的方向的右边
			{
				if(distance>=-buff)
				d_angle=-90+fabs(distance*rate);
				if(distance<-buff)
				d_angle=0;	
			}

		}
			if(distance>0)
			{
				
				if(ANGLE<90&&ANGLE>=0)//0~90度上方
				{
					if(distance<=buff)
				     d_angle=ANGLE-(distance*rate);
				    if(distance>buff)
					 d_angle=ANGLE-90;
				}
				if(ANGLE>90&&ANGLE<=180)//90~180度上方
				{
					if(distance<buff)
					{	
					d_angle=ANGLE+distance*rate;
					if(d_angle>buff)//越过180度界限
					d_angle=d_angle-360;
				    }
					
					if(distance>buff)
					d_angle=ANGLE+90-360;
				}
				if(ANGLE<0&&ANGLE>-90)//-90~0度上方
				{
					if(distance<=buff)
					d_angle=ANGLE-distance*rate;
				    if(distance>buff)
						d_angle=ANGLE-90;
				}
				if(ANGLE>-180&&ANGLE<-90)//-·90~-180上方
				{
					if(distance<=buff)
					d_angle=ANGLE+distance*rate;
				    if(distance>buff)
						d_angle=ANGLE+90;
				}
				//边界情况
				if(ANGLE==90)
				{
					if(distance<buff)//90度左边
					d_angle=90+(distance*rate);
					if(distance>buff)
					d_angle=180;
				}
				if(ANGLE==-90)//-90左边
				{		if(distance<=buff)
						d_angle=-90-distance*rate;
			            if(distance>buff)
						d_angle=180;
				}	
			}	
			
			if(distance==0)
				d_angle=ANGLE;
			return d_angle;
			
		}	
		

void run_to(float BETA,float Vm)//由当前角度pao向BETA
{
	float B_err=0;
	float B_Uk=0;
	float R= meters(Vm);//米每秒转化成脉冲每秒
	float L= meters(Vm);
	int  RR=0;
	int  LL=0;
	B_err=BETA-action.angle;//当前角度与设定值的偏差
	if(B_err>180)//偏差超过180，优弧转劣弧
		B_err=B_err-360;
	if(B_err<-180)
		B_err=B_err+360;
	//I设置限幅
	if(I_value>2500)
		I_value=2500;
	if(I_value<-2500)
		I_value=-2500;
	B_Uk=B_err*KP+I_value;
	
	
	if(B_Uk>max)//限幅，不至于反转
		B_Uk=max;
	if(B_Uk<-max)
		B_Uk=-max;
	R=R-B_Uk;
	L=-(L+B_Uk);
	VelCrl(CAN2, 01,R);
	VelCrl(CAN2, 02,L);
	RR=(int)R;
	LL=(int)L;
	int i_value=(int)I_value;
	 USART_OUT(UART4,(uint8_t*)"danger");
	USART_OUT( UART4, (uint8_t*)"%d ", LL);
	USART_OUT( UART4, (uint8_t*)"%d ", RR);
	USART_OUT( UART4, (uint8_t*)"%d ", i_value);
}
	
	
	
	
	
	
	
int line(float aa, float bb, float cc, float nn,float VL)
{
	   float d=0;
	   float aSetvalue=0;//距离转化角度函数返回的角度，存在一个地方
	   float target=0;
	   int Target=0;    //蓝牙发数的整型转换
	   int ASetvalue=0; //蓝牙发数的整型转换
	   int D=0;         //蓝牙发数的整型转换
	   
	   d=(aa*action.x+bb*action.y+cc)/sqrt(aa*aa+bb*bb);//(d<0在直线下方,a为负)
	   target=a_gen(aa,bb,nn);//得出直线设定角
	   aSetvalue=err(d,target);//根据距离转化出的角度
	   //转向这个角度
       run_to(aSetvalue,VL);
	
	   X=(int)action.x;
	   Y=(int)action.y;
	   Angle=(int)action.angle;
	   Target=(int)target;
	   ASetvalue=(int)aSetvalue;
	   D=(int)d;
	   
}
















void loop(float corex,float corey,float Radium,float V_loop,int SN)//闭环转圆
{
	float distance_err=0;//距离圆心的偏差
	float DISTANCE=0;//距离圆心的距离
	float tangent=0;
	float fake_tan=0;
	DISTANCE=sqrt((action.x-corex)*(action.x-corex)+(action.y-corey)*(action.y-corey));
	distance_err=DISTANCE-Radium;//偏离圆周的距离
	Ierr=Ierr+distance_err;
	I_value=Ierr*KI;
	if(V_loop<0.8)
		V_loop=0.8;//速度不小于1米每秒
	if(V_loop>=1.5)
		V_loop=1.5;
	if(SN==1)//逆时针
	{
		if(action.y>corey)
		tangent=a_gen(corey-action.y,action.x-corex,1)-90;//输出给定圆的切线方向
		if(action.y<corey)
		{

			tangent=a_gen(corey-action.y,action.x-corex,-1)-90;//输出给定直线的方向角
		    if(tangent<-180)//超过-180之后
			tangent=360+tangent;
		}
		if(action.y==corey&&action.x==(corex+Radium))
			tangent=90;
		if(action.y==corey&&action.x==(corex-Radium))
			tangent=-90;
		
		if(action.y>corey)
		fake_tan=err(distance_err,tangent);//距离转化角度函数
		if(action.y<corey)
		fake_tan=err(-distance_err,tangent);//距离转化角度函数
		//特殊情况给出角度
		if(action.y==corey&&action.x>corex)
		{
			if(distance_err>0)
			{
				if(distance_err<buff)//90度右边
				fake_tan=90-fabs(distance_err*rate);
				if(distance_err>buff)
				fake_tan=0;	
			}
			if(distance_err<0)
			{	if(fabs(distance_err)<buff)//90度左边
				fake_tan=90+fabs(distance_err*rate);
				if(fabs(distance_err)>buff)//90度左边
				fake_tan=180;
			}	
		}
		if(action.y==corey&&action.x<corex)
		{
			if(distance_err>0)
			{
				if(distance_err<buff)//-90度左边
				fake_tan=-90-fabs(distance_err*rate);
				if(distance_err>buff)
				fake_tan=180;
			}
			if(distance_err<0)
			{
				if(fabs(distance_err)<buff)//90度左边
				fake_tan=-90+fabs(distance_err*rate);
				if(fabs(distance_err)>buff)
				fake_tan=0;
			}
		}
		run_to(fake_tan,V_loop);//由当前角度转向BETA
		
	}
	if(SN==-1)
	{
		if(action.y>corey)
		{
			tangent=a_gen(corey-action.y,action.x-corex,1)+90;//输出给定圆的切线方向
			if(tangent>180)
			tangent=tangent-360;
		}
		if(action.y<corex)
		{
			tangent=a_gen(corey-action.y,action.x-corex,-1)+90;//输出给定圆的切线方向
		}
		if(action.y==corey&&action.x==(corex+Radium))
			tangent=-90;
		if(action.y==corey&&action.x==(corex-Radium))
			tangent=90;
		if(action.y>corey)
		fake_tan=err(distance_err,tangent);//距离转化角度函数
		if(action.y<corey)
		fake_tan=err(-distance_err,tangent);//距离转化角度函数
		
		if(action.y==corey&&action.x>corex)
		{
			if(distance_err>0)
			{
				if(distance_err<buff)//90度右边
				fake_tan=-90+fabs(distance_err*rate);
				if(distance_err>buff)
				fake_tan=0;	
			}
			if(distance_err<0)
			{	if(fabs(distance_err)<buff)//90度左边
				fake_tan=-90-fabs(distance_err*rate);
				if(fabs(distance_err)>buff)//90度左边
				fake_tan=180;
			}	
		}
		
		
		if(action.y==corey&&action.x<corex)
		{
			if(distance_err>0)
			{
				if(distance_err<buff)//-90度左边
				fake_tan=90+fabs(distance_err*rate);
				if(distance_err>buff)
				fake_tan=180;
			}
			if(distance_err<0)
			{
				if(fabs(distance_err)<buff)//90度左边
				fake_tan=90-fabs(distance_err*rate);
				if(fabs(distance_err)>buff)
				fake_tan=0;
			}
		}
		run_to(fake_tan,V_loop);//由当前角度转向BETA

	}
	
    
}












void USART3_IRQHandler(void)
{
		static uint8_t ch;
		static uint8_t count=0;
		static uint8_t i=0;
	    extern uint8_t ppsTalkOk;
		OS_CPU_SR  cpu_sr;
		OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
		OSIntNesting++;
		OS_EXIT_CRITICAL();

		if(USART_GetITStatus(USART3, USART_IT_RXNE)==SET)   
		{
			USART_ClearITPendingBit(USART3,USART_IT_RXNE);
			ch=USART_ReceiveData(USART3);
			switch(count)
			{
				case 0:
					if(ch==0x0d||ch=='O')
						count++;
					else
						count=0;
				break;

				case 1:
					if(ch==0x0a)
					{
						i=0;
						count++;
					}
					else if(ch=='K')
					{
						ppsTalkOk=1;
						count=0;
					}
				else if(ch==0x0d);
				else
					count=0;
				break;

				case 2:
					posture.data[i]=ch;
					i++;
					if(i>=GET_PPS_DATA_NUM)
					{
						i=0;
						count++;
					}
				break;

				case 3:
					if(ch==0x0a)
						count++;
					else
						count=0;
				break;

				case 4:
					if(ch==0x0d)
					{
						SetOpsReady(1);
						/*传入定位系统返回的值*/
						SetAngle( posture.value[0]);
						SetSpeedX(posture.value[1]);
						SetSpeedY(posture.value[2]);
						SetX(posture.value[3]);
						SetY(posture.value[4]);
						SetWZ(posture.value[5]);
						
                        
						/*定义的全局结构体变量可以在这里赋值*/
												action.angle=-posture.value[0];
					    //					    action.x=posture.value[1];
						//						action.y=posture.value[2];
												action.y=-posture.value[3];
												action.x=posture.value[4];//y
						//						=posture.value[5];
						action.x=action.x+OPS_TO_BACK_WHEEL*(cos(action.angle*PAI/180.0)-1);
                        action.y=action.y-OPS_TO_BACK_WHEEL *sin(action.angle*PAI/180.0);						
					} 
					count=0;
				break;

				default:
					count=0;
				break;		 
			}
		}
		else
		{
			USART_ClearITPendingBit(USART3, USART_IT_PE);
			USART_ClearITPendingBit(USART3, USART_IT_TXE);
			USART_ClearITPendingBit(USART3, USART_IT_TC);
			USART_ClearITPendingBit(USART3, USART_IT_ORE_RX);
			USART_ClearITPendingBit(USART3, USART_IT_IDLE);
			USART_ClearITPendingBit(USART3, USART_IT_LBD);
			USART_ClearITPendingBit(USART3, USART_IT_CTS);
			USART_ClearITPendingBit(USART3, USART_IT_ERR);
			USART_ClearITPendingBit(USART3, USART_IT_ORE_ER);
			USART_ClearITPendingBit(USART3, USART_IT_NE);
			USART_ClearITPendingBit(USART3, USART_IT_FE);
			USART_ReceiveData(USART3);
		}
		
		OSIntExit();

}
