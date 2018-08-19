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
#include "math.h"
#define PAI 3.14
int car=44;
int Car=1;  
int Angle=0;     //蓝牙发数的整型转换
int X=0;         //蓝牙发数的整型转换
int Y=0;
int max=8000;//最大偏差
//调车参数///////////////
float Ierr=0;
float I_value=0;
float KI=-0.12;
float rate=0.09;//  1m/s距离转化角度的比例为（0.06）,,1.5m/s为0.04,           0.15
float duty=800;   //提前量：duty=650(1m/s)，   1.5m/s duty为900，
float KP=280; //Kp给300(1m/s)，，，kp给450（1.5m/s）
float V0=0.5;//车的基础速度(m/s)
float buff=500;//(最大偏离区域)
//////////////////////////////////////////////////////


//蓝牙发数的整型转换
static int isOKFlag=0;
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
void  go_straight(float V,int ElmoNum1,int ElmoNum2 ,CAN_TypeDef* CANx);
void  go_round(float V,float R,int ElmoNum1,int ElmoNum2 ,CAN_TypeDef* CANx);
void straight(float setValue,float feedbackValue);
float err(float distance,float ANGLE);//距离转化角度函数
float a_gen(float a,float b,int n);//角度生成函数（输入直线参数，给出角度）
void run_to(float BETA,float Vm);//跑向特定方向BETA
int line(float aa, float bb, float cc, float nn);//车在干扰下跑向任意直线·，注：aa<0,当aa=0时，bb=0；nn=1控制向X轴上方走，n=-1向x轴下方，n=-2向x轴正向走，n=2向x轴负向走
void loop(float corex,float corey,float Radium,float V_loop,int SN);//顺时针转
float meters(float V1)//米每秒转化成脉冲每秒
{
	float Va;
	Va=4096*V1/(PAI*0.12);
	return Va;
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
	int cycle_flag=0;//cycle_flag为全局变量，两个函数都能改变
	int change=1;//change为全局变量，两个函数都能改变
//	#define nn 1//(方向指定，1为向X轴上方，-1为向x轴下方，当倾斜角=0（aa=0）时，-2为x轴负向，2为x轴正向,)
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
	VelLoopCfg(CAN2, 1, 50000, 10000);
	//PosLoopCfg(CAN2, 1, 100, 100,0);
	//左轮
	VelLoopCfg(CAN2, 2, 50000, 10000);
	//PosLoopCfg(CAN2, 2, 100, 50000,10000);
	//电机使能
	MotorOn(CAN2, 01);
	MotorOn(CAN2, 02);
	//定位系统串口初始化
     USART3_Init(115200);
	 //蓝牙串口
	 UART4_Init(921600); 
	 delay_s(5);
     if(Car==4)
	 delay_s(10);
	 
	 if(Car==1)
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
	float err_1=0;
	  float Aa=0;
      float Bb=0;
      float Cc=0;
      float Nn=0;
    float last_change=0;//记录上次状态
	
	  
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{

		OSSemPend(PeriodSem, 0, &os_err);
		if(car==1)//1号车走正方形
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
      if(car==4)//4号车走正方形
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
		if(car==3)//测试
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
	  
	  loop(500,500,1000,1.0,1);//逆时针转为1,顺时针为-1
	  X=(int)action.x;
	  Y=(int)action.y;
      Angle=(int)action.angle;	  
	  USART_OUT( UART4, (uint8_t*)"%d ", X);
	  USART_OUT( UART4, (uint8_t*)"%d ", Y);
	  USART_OUT( UART4, (uint8_t*)"%d ", Angle);
	  USART_OUT(UART4,(uint8_t*)"\r\n");

	
	
	
  }	
	
	}
}

int cnt_flag=0;
float last_amount=0;
float last_P0=0;
float last_cycle_num=0;
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
		

void run_to(float BETA,float Vm)//由当前角度转向BETA
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
	
	
	
	
	
	
	
int line(float aa, float bb, float cc, float nn)
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
       run_to(aSetvalue,1);
	
	   X=(int)action.x;
	   Y=(int)action.y;
	   Angle=(int)action.angle;
	   Target=(int)target;
	   ASetvalue=(int)aSetvalue;
	   D=(int)d;
	   
	   USART_OUT( UART4, (uint8_t*)"%d ", X);
	   USART_OUT( UART4, (uint8_t*)"%d ", Y);
	   USART_OUT( UART4, (uint8_t*)"%d ", Angle);
	   USART_OUT( UART4, (uint8_t*)"%d ", D);
	   USART_OUT( UART4, (uint8_t*)"%d ", Target);
	   USART_OUT( UART4, (uint8_t*)"%d ", ASetvalue);
	   USART_OUT(UART4,(uint8_t*)"\r\n");
}
















void loop(float corex,float corey,float Radium,float V_loop,int SN)//闭环转圆
{
	float distance_err=0;//距离圆心的偏差
	float DISTANCE=0;//距离圆心的距离
	float tangent=0;
	float fake_tan=0; int Tangent=0;
	int Fake_tan=0;
	DISTANCE=sqrt((action.x-corex)*(action.x-corex)+(action.y-corey)*(action.y-corey));
	distance_err=DISTANCE-Radium;//偏离圆周的距离
	Ierr=Ierr+distance_err;
	I_value=Ierr*KI;
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
		Tangent=(int)(tangent);
		Fake_tan=(int)fake_tan;
		 USART_OUT(UART4,(uint8_t*)"%d ",Tangent);
		 USART_OUT(UART4,(uint8_t*)"%d ",Fake_tan);
		
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
		Tangent=(int)(tangent);
		Fake_tan=(int)fake_tan;
		USART_OUT(UART4,(uint8_t*)"%d ",Tangent);
		USART_OUT(UART4,(uint8_t*)"%d ",Fake_tan);
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
