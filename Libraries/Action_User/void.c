#include "void.h"
#include "elmo.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "timer.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "math.h"
#include "movebase.h"
#include "pps.h"
#include "fort.h"
#define PAI 3.14
#define mode 3
#define Angchange 1
#define recolong 4000
#define Pulse2mm COUNTS_PER_ROUND/(WHEEL_DIAMETER*Pi)
#define YAW_initangle 170
#define NEED_BALL_COLLOR (1)
#define NO_NEED_BALL_COLLOR (2)
/********************自定义变量区***************/
extern float run_r;
extern float run_R;
extern float run_V;
extern int Sn;
extern int trouble_flag;
float get_angle2(float a,float b,int n,int round);
typedef struct{

	float x;
	float y;
	float angle;
    float x_speed;
	float y_speed;	   
	}pos_t;
pos_t action;
extern uint8_t Ballcolor[5];//摄像机的接受数组//
float s;
int diagonal;
int change_compere_angle;
float body_angle_to_point;
static int push_balltime;
float body_angle;	

float buff=1000;//(最大偏离区域)
float rate=0.09;// 正方形  1m/s距离转化角度的比例为（0.06）,,1.5m/s为0.04,           0.15
float Ierr=0;
float I_value=0;
float KI=-0.09;
float KP=220; 
int  R=0;//右轮内轮
int  L=0;//左轮外轮
int record=0; //常规计圈数
int record1=0;//启动计圈数
float last_x=0;
float last_y=0;
float last_angle=0;
float last_x1=0;
float last_y1=0;
float last_angle1=0;
int R_switch=0;
int r_switch=0;
uint8_t R_switch1=0;
uint8_t r_switch1=0;
int round_cnt=-100;
uint8_t start_cnt=2;
uint8_t dec_start=0; 
uint8_t start_round_cnt=1;
uint8_t start_ready=0;
uint8_t changecoming=0;
float Ra=400;//(可变半径)
int dec_flag=0;//(半径增减的标志位)
int stable_flag=0;
extern float buff_cnt;
float Rbuff_cnt=0;
float Vbuff_cnt=0;
float add_angle;
extern uint8_t acceleration_ready;
float ADC_A;
float ADC_B;
int AreaChange;
int PushballFlag=0;

float fake_last_angle=0;
float real_send_angle=0;
float get_fake_angle;
/********************自定义变量区***************/	
/////////////////////////速度转脉冲////////////////////////////////
int change_buff=0;
/********************自定义变量区***************/	
/********************速度转脉冲*****************/
void systerm_change()
{
	action.angle=-GetAngle();
    action.x=-GetY();
    action.y=GetX();
	
//	action.x=action.x+OPS_TO_BACK_WHEEL*(cos(action.angle*PAI/180.0)-1);//	gety=getx+OPS_TO_BACK_WHEEL*(cos(-get（angle）*PAI/180.0)-1);
//    action.y=action.y-OPS_TO_BACK_WHEEL*sin(action.angle*PAI/180.0);	//   action.y=action.y-OPS_TO_BACK_WHEEL *sin(action.angle*PAI/180.0);
}

int AdcFlag(void)//启动ADC
{
		static int adc_num1,adc_num2,AdcFLAG=0;
	    int adc_cnt=0;
do
{
     	adc_num1=Get_Adc_Average(15,30);	///左轮
		adc_num2=Get_Adc_Average(14,30);  ///右轮
		if(adc_num1<1500||adc_num2<1500)
		adc_cnt++;	
		USART_OUT( UART4, (uint8_t*)"%d ", (int)adc_num2);//左
		  USART_OUT( UART4, (uint8_t*)"%d ", (int)adc_num1);//右
}while(adc_cnt<50); 


	if(adc_num1<300&&adc_num1>0&&AdcFLAG==0)
		{
			AdcFLAG=-1;  ////顺时针
		}
	if(adc_num1>300&&adc_num1<850&&AdcFLAG==0)
		{
			AdcFLAG=-2;  ////顺时针
		}
	if(adc_num1>=850&&adc_num1<1500&&AdcFLAG==0)
		{
			AdcFLAG=-3;  ////顺时针
		}
	 if(adc_num2<300&&adc_num2>0&&AdcFLAG==0)
		{
			AdcFLAG=1;  ////逆时针
		}
		if(adc_num2>300&&adc_num2<850&&AdcFLAG==0)
		{
			AdcFLAG=2;  ////逆时针
		}
		 if(adc_num2>=850&&adc_num2<1500&&AdcFLAG==0)
		{
			AdcFLAG=3;  ////逆时针
		}
	

	//	USART_OUT(UART4,(uint8_t*)"%d	%d	%d\n",adc_num1,adc_num2,AdcFLAG);	 /////////ADC  test///////////
		return AdcFLAG=1;
}

void start_mode()
{
	if(AdcFlag()==1||AdcFlag()==-1)//从400半径启动，内圈球较多，，记得增大内圈速度，博弈
	{
		start_cnt=1;
	}
	if(AdcFlag()==2||AdcFlag()==-2)//从900半径启动，中圈球多，先在中圈收球投球，再去1500半径收球再收最外圈
	{
        start_cnt=2;
	}
	if(AdcFlag()==3||AdcFlag()==-3)// 1800启动半径 别人跑内圈，我们选择从外圈开始扫
	{
		start_cnt=3;
	}
//	  USART_OUT( UART4, (uint8_t*)"%d ", (int)round_cnt);//左
		
}


int exchange(float v)//
{
	int pulse=0;
	pulse=(int)(v*32768/(PAI*0.12));
	return pulse;
}
//AngPID//////////////////////
float AnglePID(float Angle,float SetAngle)
{
	struct PID Ang;
	Ang.p=100;   				//////////300
	Ang.i=0;
	Ang.d=0;
	float err=0,u=0,err1=0,err2=0;
	err1=SetAngle-Angle;
	//////////////////劣弧调节//////////////////
	if(err1>=0)
		err2=err1-360;
	else
		err2=360+err1;
	if(err1>180||err1<-180)
	{
		err=err2;
	}
	else
	{
		err=err1;
	}
	u=Ang.p*err;
	return u;
}
float DirectionPID( float distance,float setdistance )//距离转角度
{
	struct PID Dir;
	float u=0,errd=0;
	errd=setdistance-distance;
	if(distance<300&&distance>-300)
	{
		Dir.p=0.06;
		u=Dir.p*errd;
	}
	else if(distance<500&&distance>-500)
	{
		Dir.p=0.06;
		u=Dir.p*errd;
	}
	else
	{
		Dir.p=0.06;
		u=Dir.p*errd;
	}
	return u;
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
	float R= exchange(Vm);//米每秒转化成脉冲每秒
	float L= exchange(Vm);
	
	float max=0;
	float D_value=0;
	float Deta_err=0;
	float last_err=0;
	B_err=BETA-action.angle;//当前角度与设定值的偏差
	Deta_err=B_err-last_err;//
	D_value=Deta_err*20;
	if(B_err>180)//偏差超过180，优弧转劣弧
		B_err=B_err-360;
	if(B_err<-180)
		B_err=B_err+360;
	//I设置限幅
	if(I_value>2500)
		I_value=2500;
	if(I_value<-2500)
		I_value=-2500;
/******   设置前馈 ******/
//	if((round_cnt==3||start_round_cnt==3)&&trouble_flag==0&&acceleration_ready==1)//防止1800半径撞墙
//	{
//		R=25000*32768/4096;//正常25000
//		L=18000*32768/4096;//正常18000
//	}
//		if(round_cnt==3&&trouble_flag==0&&acceleration_ready==0)//1800启动半径模式下，防止启动过冲
//	{
//		R=25000;
//		L=25000;
//	}
//	
	
/*********** PID调节	***********/	
	B_Uk=B_err*KP;//+D_value;
	 max=Vm*8000;
	if(B_Uk>max)//限幅，不至于反转
		B_Uk=max;
	if(B_Uk<-max)
		B_Uk=-max;
	R=(R-B_Uk*32768/4096);
	L=-(L+B_Uk*32768/4096);
	VelCrl(CAN1, 02,R);
	VelCrl(CAN1, 01,L);
//	USART_OUT(UART4,(uint8_t*)"speed: ");
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)L);
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)R);
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)I_value);

	if(round_cnt==3&&trouble_flag==0)	
	{
		if((((action.x+2200)<300)&&(action.x+2200>0))||((fabs(action.y)<150)&&(action.x<-2200)))
		  {
//		   VelCrl(CAN1, 01,30000);
//	       VelCrl(CAN1, 02,-30000);	
	      }
    }
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)action.x);
//    USART_OUT( UART4, (uint8_t*)"%d ", (int)action.y);
}
		
		
		
		
//////////////绕圈///////////////
float a_gen(float a,float b,int n)//输出给定直线的方向角  ax+by+c=0,y=(-a/b)*x+c0;k=-a/b=(action.y-y)/(action.x-x)----a=action.y-y,  ,b=-(action.x-x)
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









void Walkline( float corex,float corey,float Radium,float V_loop,int SN )//正常走行,输入圆心坐标，半径，速度，方向
{
	if(V_loop<1)
		V_loop=1;
	if(V_loop>2.5)
		V_loop=2.5;
	float mid=0;
    float index=1;
	corex=mid;
	corex=-corey;
	corey=mid;
	float distance_err=0;//距离圆心的偏差
	float DISTANCE=0;//距离圆心的距离
	float tangent=0;
	float fake_tan=0;
	DISTANCE=sqrt((action.x-corex)*(action.x-corex)+(action.y-corey)*(action.y-corey));
	distance_err=DISTANCE-Radium;//偏离圆周的距离
	if (fabs(distance_err)>500)
		index=0.02;
	if(fabs(distance_err)<50&&fabs(distance_err)>100)
		index=(300-fabs(distance_err))/400;
	if(distance_err<100)
		index=1;
	Ierr=Ierr+distance_err;
	I_value=Ierr*KI*index;

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
		if(action.y<corey)
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
		//特殊情况
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
	
//		USART_OUT( UART4, (uint8_t*)"%d ", (int)action.x);
//        USART_OUT( UART4, (uint8_t*)"%d ", (int)action.y);
//	   	USART_OUT( UART4, (uint8_t*)"%d ", (int)action.angle);
//		 	USART_OUT( UART4, (uint8_t*)"\r\n");
	
}

void Walkback(float v)//故障处理
{
	VelCrl(CAN1,0x01,-exchange(v));
	VelCrl(CAN1,0x02,exchange(v));
}
void Walk_left_away(float v,int time)//故障处理
{
	for(int i=0;i<=time;i++)
	{
    	VelCrl(CAN1,0x01,-exchange(v+0.5));
	    VelCrl(CAN1,0x02,exchange(v-0.5));	
	}
}
void Walk_right_away(float v,int time)//故障处理
{   
	for(int i=0;i<=time;i++)
	{
	VelCrl(CAN1,0x01,-exchange(v-0.5));
	VelCrl(CAN1,0x02,exchange(v+0.5));	
	}
}
void Walkahead(float v)//故障处理
{
	VelCrl(CAN2,0x01,exchange(v));
	VelCrl(CAN2,0x02,-exchange(v));
}

float R_buff(int buff_type,int buff_degree,int per_val,float standard_R)
{  
	//R_buff(-1,6,4,900);R_buff(1,4,2,900);
	Rbuff_cnt+=per_val;
	       if(Rbuff_cnt>=buff_degree*100)
	       {
			   Rbuff_cnt=buff_degree*100;
			   acceleration_ready=1;
		   }
    run_r=standard_R-buff_type*buff_degree*100+buff_type*Rbuff_cnt;  
return run_r;
}


float V_buff(float Buff_type,float Buff_degree,float Per_val,float standard_V)
{
		Vbuff_cnt+=Per_val;
	       if(Vbuff_cnt>=Buff_degree*0.1)
	       {
			   Vbuff_cnt=Buff_degree*0.1;
			   acceleration_ready=1;
		   }
    run_V=standard_V-Buff_type*Buff_degree*0.1+Buff_type*Vbuff_cnt;  
return run_V;
}


float round_setting(int radiuss)
{
	float ra_set=0;
	if(radiuss==901)//启动圈900
	{
		ra_set=900;
		KP=200;
		buff=1000;
		stable_flag=0;//不射球
	}
	if(radiuss==900)
	{
		  buff=800;
		  ra_set=900;
		  stable_flag=0;
		  KP=200;
	}
	if(radiuss==1900)
	{
		ra_set=2000;
		stable_flag=0;
		dec_flag=-1;
		KP=250;
	}
    if(radiuss==600)
	{
		ra_set=600;
		stable_flag=0;
		dec_flag=1;//缓冲区给大
		KP=550;
	}
	if(radiuss==1500)
	{
		stable_flag=1;
	    KP=150;	
	}
		return ra_set;
	
}


float startRadius()//启动 走形模式
{
	/*********8*****定义变量区*****/
typedef struct{
	float R;
	uint8_t buff1;
	uint8_t buff2;
	}a;
	a start;
	
	/***************定义变量区*****/
	
	
	
if(changecoming==1||start_round_cnt==1)
{
    changecoming=0;	
	if(start_cnt==1)
		start_ready=1;

	if(start_cnt==2)//启动半径为900,启动这个圆得好好调，目标速度两米
  { 
    /*********记得设置缓冲****/
	  if(start_round_cnt==1)//快速收球圈
	  {
		  start.R=round_setting(901);
	  }
	  if(start_round_cnt==2)//慢速射球圈1500
		start.R=round_setting(1500);
	  
	  if(start_round_cnt==3)//快速收球圈1800
		start.R=round_setting(1900);
	  
	  if(start_round_cnt==4)
	  {
	     start.R=round_setting(1500);
	     start_ready=1;
		 round_cnt=0;
	  }
	  
//  if(start_round_cnt==5)
//	 {
//	  start_ready=1;
//	  round_cnt=1;
//	  start.R=round_setting(900);
//  } 
//  
//  if(start_round_cnt==6)
//  {
//	  start_ready=1;
//	  round_cnt=1;
//  }
}
  if(start_cnt==3)
  {
	  if(start_round_cnt==1)
		  start.R=round_setting(1900);
	  
	  if(start_round_cnt==2)
		  start.R=round_setting(1500);
	  
	  if(start_round_cnt==3)
		  start.R=round_setting(900);
	  
	  if(start_round_cnt==4)
		  start.R=round_setting(600);
	  
	  if(start_round_cnt==5)
	  {
		  start_ready=1;
			
	  }
  }
  }
if(trouble_flag==0)
{	
	if(start_cnt==2&&stable_flag==0)
	{
		if(start_round_cnt==1)//   一上来冲900  int buff_type,int buff_degree,int per_val,float standard_R
		{
			start.R=R_buff(-1,6,4,900);//标准半径为900.缓冲半径为900+600=1500,并在2秒内从1500降到900   (2s)
			V_buff(1,15,0.005,2);//速度从0.5增到2米每秒（3秒）
		}
//		if(start_round_cnt==2)//从900上到1500
//			R_buff(1,4,4,1500);
		if(start_round_cnt==3)
		{
			start.R=R_buff(1,4,3,1900);//从1500上到1900,速度从1米每秒到2米每秒
			V_buff(1,10,0.008,2);
			
		}
//		if(start_round_cnt==4)
//			R_buff(-1,4,2,1500);
		if(start_round_cnt==5)
		{
			start.R=R_buff(-1,9,5,600);//从1500下到600
			V_buff(1,5,0.005,1.5);
		}
}
	if(start_cnt==3&&stable_flag==0)
	{
		if(start_round_cnt==1)//速度直接飙2米每秒
		{
			start.R=R_buff(1,3,3,1900);
			V_buff(1,20,0.005,2);
		}
		if(start_round_cnt==3)//从1500下到900
		{
			start.R=R_buff(-1,4,2,900);
		}
		if(start_round_cnt==4)//从900下到600
		{
			start.R=R_buff(-1,3,3,600);
		}
	
    }
  
	
}
	record1++;
	         if(record1==10)
			{
				last_angle1=action.angle;
                last_x1=action.x;
				last_y1=action.y; 
				record1=0;	
            }
		  if(Sn==1)
			  change_buff=-400;
		  if(Sn==-1)
			  change_buff=400;
	   if((action.y>change_buff&&last_y1<change_buff&&action.x>-2350&&Sn==1)||(action.y<change_buff&&last_y1>change_buff&&action.x>-2350&&Sn==-1))//换到
	{
		R_switch1=1;  
	    if(R_switch1>r_switch1)  
       {
			start_round_cnt++;
		    changecoming=1;
	   }
   }else R_switch1=0;
	r_switch1=R_switch1;
	
  return start.R;
}


int Radius(void)//常规走形模式
{
	     record++;
	         if(record==20)
			{
				last_angle=action.angle;
                last_x=action.x;
				last_y=action.y; 
				record=0;	
            }
		
		   	//
		  if(Sn==1)
			  change_buff=-200;
		  if(Sn==-1)
			  change_buff=200;
	   if((round_cnt==-100)||(action.y>change_buff&&last_y<change_buff&&action.x>-2350&&Sn==1)||(action.y<change_buff&&last_y>change_buff&&action.x>-2350&&Sn==-1))//换到
{
		R_switch=1;  
	    if(R_switch>r_switch||round_cnt==-100)  
{ 
	
	      trouble_flag=0;
		  round_cnt++;
		  buff_cnt=1;
		  Rbuff_cnt=0;
	      Vbuff_cnt=0;
//		  if(round_cnt%2==0)
//			  stable_flag=1;
//		  if(round_cnt%2==1)

if(round_cnt==-99)//启动圈
		  {
			  round_cnt=0;
			  Ra=400;
			  stable_flag=0;
              dec_flag=-1;
			  KP=180;
			  
		  }
if(round_cnt==1)
		  {
			Ra=round_setting(900);
		  }
if(round_cnt==2)
		  {
			  start_cnt=0;
			  Ra=1500;
			  stable_flag=1;
              dec_flag=-1;
			  KP=100;
		  }
	
if(round_cnt==3)
		  {
			dec_flag=-1;
		    Ra=round_setting(1900);
		  }  
if(round_cnt==4)
		  {
			  
		      Ra=1500;
			  stable_flag=1;
			  dec_flag=1;
			  KP=100;
		  }  
if(round_cnt==5)
		  {
		      Ra=1500;
			  stable_flag=1;
			  dec_flag=1;
			  KP=100;
			  round_cnt=0;
			  
		  } 		  
		  
}  
}	  else R_switch=0;
	 r_switch =R_switch;
 //    USART_OUT( UART4, (uint8_t*)"%d ", (int)Ra);
		
if(trouble_flag==0)
{		
	if(round_cnt==1)
	{
		if(start_cnt==1)//内圈往外,速度从一米增到1米5
		{
		
		    run_V=V_buff(1,5,0.0025,1.5);
		    Ra=R_buff(1,4,4,900);
	    }
	else{
		    //从射球圈变向收球圈
			   V_buff(1,5,0.0025,1.5);
		   	   Ra=R_buff(-1,6,3,900);
	    }
    }
	if(round_cnt==3)
	{
		  Ra=R_buff(1,5,4,2000);
		  V_buff(1,5,0.002,2);
	}
	
}	
return Ra;
}


void errdeal(void)//故障处理主控制
{      
	 body_angle=get_angle2(GetY()-2400,-GetX(),(GetY()-2400)/fabs(GetY()-2400),Sn);
	   if( body_angle>=180)
		 body_angle-=360;
		static int errtime=0;
	    float now_car_v=sqrt(pow(GetSpeedY(),2)+pow(GetSpeedX(),2)),car_d_to_center=sqrt(pow(GetX(),2)+pow(GetY()-2400,2));
		if(now_car_v<=100&&Sn)
		{
			errtime++;
		}
		else if(now_car_v>150)
		{
			errtime=0;
		}
			if(errtime>100)   
			{
				trouble_flag=1;
				
				if(Sn==1)
				{	
					 
				
					if(car_d_to_center>=1300)
					  {
						 if(car_d_to_center>=2750) 
						 {
							 if(((body_angle>90&&body_angle<180)&&(GetAngle()>-2&&GetAngle()<2))||((body_angle>0&&body_angle<90)&&(GetAngle()<-88&&GetAngle()>-92))||((body_angle>-90&&body_angle<0)&&((GetAngle()<180&&GetAngle()>178)||(GetAngle()<-178&&GetAngle()>-180)))||((body_angle>-180&&body_angle<-90)&&(GetAngle()<92&&GetAngle()>88)))
							 {
								 Walk_right_away(1.0,1500);
								 stable_flag=1;
							 }
						 }	 
					 						
						  else {
							 
							  Walk_left_away(1.0,1000);
							   if(round_cnt==3)
							  {
								  stable_flag=1;
							  }
							  
							  else  if (stable_flag==1)
							  {   
								  
								  Sn=-Sn;
							  }else run_r-=500;
						  }
							
						
					  }
				 	else 
					{						
						Walk_right_away(1.0,1000);	
                        run_r+=500;						
					}
		     	}else if(Sn==-1)
				{	
					if(car_d_to_center<=1300)
					{
					   Walk_left_away(1.0,1000);
						run_r+=500;
						
				    }else 
					{
						if(car_d_to_center>=2750) 
						 {
							 if(((body_angle>0&&body_angle<90)&&(GetAngle()>-2&&GetAngle()<2))||((body_angle>0&&body_angle<-90)&&(GetAngle()<-88&&GetAngle()>-92))||((body_angle>-180&&body_angle<-90)&&((GetAngle()<180&&GetAngle()>178)||(GetAngle()<-178&&GetAngle()>-180)))||((body_angle>90&&body_angle<180)&&(GetAngle()<92&&GetAngle()>88)))
							 {
								 
								 Walk_left_away(1.0,1500);
								 stable_flag=1;
							 }
						 }else							
					     {
							
							 Walk_right_away(1.0,1000);
							  if(round_cnt==3)
							  {
								  stable_flag=1;
							  }else  if (stable_flag==1)
							  {   
								  
								  Sn=-Sn;
							  }else
                               run_r-=500;
						 }
					 }
				
		     	}
				
				
//				for(int i=0;i<2000;i++)
//				{
//					Walkahead(0.7);
//					i++;
//				}				
				errtime=0;
			  for(int i=0;i<1000;i++)
				{
					Walkback(1.0);					
				}

			}			
//			USART_OUT(UART4,(uint8_t*)"%d %d %d\n",Lastx,Lasty,errtime);          /////////errtime  test////////////
}	

//得到炮台转轮速度//
float get_roll_v(void)
{   
	//四号车的炮台速度比较小需要加大//
	//一号车在顺时针 -1 4   1 3.5//	四号炮台+0.2
	if(s<=4000)
	{
	if(Sn==1)	//逆时针
	{
		if( diagonal==1)
			return((sqrt(19600*pow(s,2)/((sqrt(3))*s-584)))/377*3.44);//左下右上
		else
			return((sqrt(19600*pow(s,2)/((sqrt(3))*s-584)))/377*3.44);//左上右下
	}else//
	{   if( diagonal==-1)
		return((sqrt(19600*pow(s,2)/((sqrt(3))*s-584)))/377*3.44);
		else
		return((sqrt(19600*pow(s,2)/((sqrt(3))*s-584)))/377*3.44);
	}
    }
	else return(50);
	
}
//炮台的角度——射球主控制//
void get_sendangle(void)
{   float get_d(float,float);
	float get_angle2(float a,float b,int n,int round);
	void move_gun(float point_x,float point_y);
	void push_ball();
	float point_x;
	float point_y;
    float sendsend_angle;
	float getget_angle;
	ADC_A=fort.laserAValueReceive*2.5112+38.72;		///激光与距离的拟合
	ADC_B=fort.laserBValueReceive*2.4267+358.54;
	change_compere_angle=get_angle2(GetY()-2300,-GetX(),(GetY()-2300)/fabs(GetY()-2300),Sn);
	if(change_compere_angle>180)
	  change_compere_angle-=360;
	if(Sn==-1)	///顺时针
	{	if(change_compere_angle<=-35&&change_compere_angle>=-125)
		{
			point_x=-2400;
			point_y=-200;
			diagonal=1;
			move_gun(point_x,point_y);
			if(change_compere_angle<=-45)
				PushballFlag=1;
	
		}else if((change_compere_angle<=180&&change_compere_angle>=145)||(change_compere_angle>=-180&&change_compere_angle<=-125))
		{
			point_x=-2400;
			point_y=4900;
		    diagonal=-1;
			move_gun(point_x,point_y);
			if((change_compere_angle<=-135)||(change_compere_angle<=180&&change_compere_angle>=145))
			PushballFlag=1;
		
		}else if(change_compere_angle<=145&&change_compere_angle>=55)
		{
			point_x=2400;
			point_y=4700;
           	diagonal=1;
			move_gun(point_x,point_y);
		    if(change_compere_angle<=135)
			PushballFlag=1;
			
		}else if(change_compere_angle<=55&&change_compere_angle>=-35)
		{
			point_x=2280;
			point_y=-90;
			diagonal=-1 ;
			move_gun(point_x,point_y);
			
            if(change_compere_angle<=45)
			 PushballFlag=1;
		}
		s=get_d(point_x,point_y);
	}else 		///逆时针
	{
		if((change_compere_angle<=-145&&change_compere_angle>=-180)||(change_compere_angle>=125&&change_compere_angle<=180))
		{
			diagonal=1;
			point_x=-2400;
			point_y=-100;
			move_gun(point_x,point_y);
			if((change_compere_angle>=135)||(change_compere_angle>=-180&&change_compere_angle<=-155))
			{
				PushballFlag=1;
			}
			 

		}else if(change_compere_angle<=125&&change_compere_angle>=35)
		{ 
			diagonal=-1;
			point_x=-2500;
			point_y=4600;
			move_gun(point_x,point_y);
			body_angle_to_point=get_angle2(GetY()-point_y,-GetX()+point_x,(GetY()-point_y)/fabs((GetY()-point_y)),1);
	        if(change_compere_angle>=55)
				PushballFlag=1;
		}else if(change_compere_angle<=35&&change_compere_angle>=-55)
		{ 
			diagonal=1;
			point_x=2400;
			point_y=4700;		
            move_gun(point_x,point_y);		
                 if(change_compere_angle>=-35&&change_compere_angle<=35)
				PushballFlag=1;		

		}else if(change_compere_angle<=-55&&change_compere_angle>=-145)
		{
			diagonal=-1;
			point_x=2500;
			point_y=200;
			move_gun(point_x,point_y);
            if(change_compere_angle>=-125&&change_compere_angle<=-55)
		     PushballFlag=1;
		}
		s=get_d(point_x,point_y);
	}

	
	
}

//得到车与任意点的角度//
float get_angle2(float a,float b,int n,int round)
{ 	
	if(b!=0)
	{ if(n==1)
		{ if(-a/b>0)
		  {   
			  return(atan2(-a/b,1)/3.14159*180);
		  }
		  else if(-a/b<0)
		  {    
			  return(atan2(a/b,-1)/3.14159*180);
		  }
		}else
		{
			if(-a/b>0)
			{
		    return(atan2(a/b,-1)/3.14159*180);
				
			}
		  else if(-a/b<0)
		  {   
			 return(atan2(-a/b,1)/3.14159*180);
		  }
		}
	}
	else if(b==0)
	{
		if(round==-1)//顺指针
		{   if(n==1)
		   	{
				
				return(90);
			}
			else 
			{
				return(-90);
		       
				
			}
		}
        else 
		{   
			if(n==1)
		   {
			
				return(90);
			  
			}
			
			else 
			{
				return(-90);
		      
			}       			
		}
	} 
     
	if(a==0)
	{
		if(n==1)
		{  
			return(0);
		
		}
        else 
		{   
			return(-180);
            			
		}
		
	}	
}
//推球//
static int PushBallPosition=32768/2;
static int color_mode=0;//黑白球模式 白1 黑2 无3 
static int White_ball_time;//白球的时间//
static int Black_and_no_time;//黑球与没球的时间//
static int add_or_reduce=-1;
void push_ball(void)
{ 
	if(color_mode==0)
	{ 
		if(Ballcolor[2]==NEED_BALL_COLLOR)
		{
			color_mode=1;
			
		}else if(Ballcolor[2]==NO_NEED_BALL_COLLOR)
		{
			color_mode=2;
			
		}else if(Ballcolor[2]==0)
		{
			color_mode=3;
		}
	}
	
	
	switch(color_mode)
	{
		case 1:
			if(PushballFlag==1)
			White_ball_time++;
		    if(White_ball_time==100)
			{				
				PushBallPosition+=32768/2;
								
			}else if(White_ball_time==200)
			{
				White_ball_time=0;
				color_mode=0;
			}
			break;
		case 2:
            Black_and_no_time++;
            if(Black_and_no_time==1)	
			{
				PushBallPosition-=327682/2;

				
			}else if(White_ball_time==100)
            {
				Black_and_no_time=0;
				color_mode=0;
            }
			break;
        case 3:
            Black_and_no_time++; 
            
				if(Black_and_no_time==1)
				{
					PushBallPosition=PushBallPosition+add_or_reduce*32768/2;
					add_or_reduce=-add_or_reduce;
				}else if(Black_and_no_time>=80)
				{
					if(Ballcolor[2])
					{
						color_mode=0;
						
					}else if(Black_and_no_time>=160&&!Ballcolor[2])
					{
						Black_and_no_time=0;
					}
					
					
				}
			break;	
					
	}
	PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,PushBallPosition);
	
}
//得到车与四个点的距离//
float get_d(float x,float y)
{
	return(sqrt(pow (GetX()-x,2)+pow(GetY()-y,2)));
}
//得到车与四个点的角度与 车的速度角度的插值//
float get_differ_angle(float angle)
{   
	float make_angle_in_wide(float angle,float point_angle);
	float nowdiffer_angle;
	nowdiffer_angle=make_angle_in_wide(angle-GetAngle(),-180);
	return(nowdiffer_angle);
}
//得到炮台相对于算出的角度应该加或者减的角//
float get_addorreduce_angle(float differ_angle,float x)
{   float Add_V=sqrt(pow(GetSpeedX(),2)+pow(GetSpeedX(),2));
	float Pi=3.1415926;
	float t=sqrt(2*(sqrt(3)*x-800)/9800);
	float x_v=x/t;
	float need_v=sqrt(pow(Add_V,2)+pow(x_v,2)-2*x_v*Add_V*cos(fabs(differ_angle)/180*Pi));
	float cos_x=(pow(need_v,2)+pow(x_v,2)-pow(Add_V,2))/(2*x_v*need_v);
	return(acos(cos_x)/Pi*180);
}
//将角度限制在0~360度 point angle=0//  //将角度限制在-180~180度 point_angle=-180//
float make_angle_in_wide(float angle,float point_angle)
{
	
	if(angle>360+point_angle)
		angle-=360;
	else if(angle<0+point_angle)
		angle+=360;
	
	return(angle);
}
//计算并得到炮台需要运动到的角//
float last_body_angle=0;
void move_gun(float point_x,float point_y)
{
		float x;
		float cartopoint_angle;//车指向固定点的角度//
		float getget_angle;//从bodyangle与y轴正方向的角度//	
	    float begain_angle=0;//炮台的起始角度//       	
	    float fake_new_angle;
	    
		x=get_d(point_x,point_y);
		body_angle=get_angle2(GetY()-point_y,-GetX()+point_x,(GetY()-point_y)/fabs((GetY()-point_y)),Sn);//得到固定点指向车的角度//
	    if(fabs(make_angle_in_wide(last_body_angle-body_angle,-180))>175)
		body_angle=last_body_angle;	
		else last_body_angle=body_angle;

		cartopoint_angle=make_angle_in_wide(body_angle+180,-180);//将角度限制在-180~180//

		add_angle=get_addorreduce_angle(get_differ_angle(cartopoint_angle),x);
		body_angle=body_angle-add_angle*0.6;//弥补误差//
		
		getget_angle=make_angle_in_wide(90+body_angle,-180);//将角度限制在-180~180//
	    fake_new_angle=make_angle_in_wide(begain_angle-getget_angle+GetAngle(),-180);
	    get_fake_angle=make_angle_in_wide(fake_new_angle-fake_last_angle,-180);
	    real_send_angle=real_send_angle+get_fake_angle;
	
 //   	USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t%d\t%d\t\r\n",(int)body_angle,(int)fake_new_angle,(int)fake_last_angle,(int)get_fake_angle,(int)real_send_angle);
	    fake_last_angle=fake_new_angle;
		//发送角度//
		YawPosCtrl(real_send_angle-15);
	   
		USART_OUT(UART4,(uint8_t*)"add_angle=%d\t\n",(int)add_angle);
}











