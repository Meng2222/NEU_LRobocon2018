#include "elmo.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "timer.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_gpio.h"
#include "math.h"
#include "movebase.h"
#include "pps.h"
#include "fort.h"
#include "string.h" 
#include "PushBall.h" 


#define push_mode (2)
extern int last_round_cnt;
extern int BeforePushposition;
extern int LastpushBallposition;
extern int if_white_shoot;
extern int stable_flag;
extern union push_p push_position;
extern int trouble_flag;
float add_or_reduce_angle;
float get_angle2(float a,float b,int n,int round);
float add_angle;
extern uint8_t Ballcolor[5];//摄像机的接受数组//
float s;
int diagonal;
int change_compere_angle;
float body_angle_to_point;
static int push_balltime;
float fake_last_angle=0;
float get_fake_angle;
float ShootBili;
int PushballFlag=0;
extern int Sn;
extern int roundCnt;
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
			return((sqrt(19600*pow(s,2)/((sqrt(3))*s-800)))/377*3.55);//左下右上\\3.45
		else
			return((sqrt(19600*pow(s,2)/((sqrt(3))*s-800)))/377*3.55);//左上右下
	}else//
	{   if( diagonal==-1)
			return((sqrt(19600*pow(s,2)/((sqrt(3))*s-800)))/377*3.55);
		else
			return((sqrt(19600*pow(s,2)/((sqrt(3))*s-800)))/377*3.55);
	}
    }
	else return(50);
	
	
}
int N_point_data[4][2];
int S_point_data[4][2];
int point_number;
//炮台的角度——射球主控制//
int now_shooter_v;
static int last_shooter_v=0;
static int a=0;
int if_shoot[4]={1,1,1,1};
int real_number=0;
static float last_get_shooter_v;
void get_sendangle(void)
{  
	
	float point_x;
	float point_y;
    float sendsend_angle;
	float getget_angle;
	float now_get_rool_v;
    real_number=0;
	//逆时针//
	//1号//
	N_point_data[0][0]=-2200;
	N_point_data[0][1]=100;
	//2号//
	N_point_data[1][0]=-2200;
	N_point_data[1][1]=4500;
	//3号//
	N_point_data[2][0]=2200;
	N_point_data[2][1]=4500;
	//4号//
	N_point_data[3][0]=2200;
	N_point_data[3][1]=250;
	//顺时针//
	//1号//
	S_point_data[0][0]=-2200;
	S_point_data[0][1]=100;
	//2号//
	S_point_data[1][0]=-2200;
	S_point_data[1][1]=4500;
	//3号//
	S_point_data[2][0]=2200;
	S_point_data[2][1]=4500;
	//4号//
	S_point_data[3][0]=2200;
	S_point_data[3][1]=100;
	
	change_compere_angle=get_angle2(GetY()-2300,-GetX(),(GetY()-2300)/fabs(GetY()-2300),Sn);
	if(change_compere_angle>180)
	  change_compere_angle-=360;
	if(Sn==-1)	///顺时针
	{	if(change_compere_angle<=-35&&change_compere_angle>=-125)
		{
		
			diagonal=1;
		    point_number=1;
			if(change_compere_angle<=-55&&change_compere_angle>=-120)
				PushballFlag=1;
			else PushballFlag=0;
	
		}else if((change_compere_angle<=180&&change_compere_angle>=145)||(change_compere_angle>=-180&&change_compere_angle<=-125))
		{
		
		    diagonal=-1;
			point_number=2;
			if((change_compere_angle<=-145)||(change_compere_angle<=180&&change_compere_angle>=150))
			PushballFlag=1;
			else PushballFlag=0;
		
		}else if(change_compere_angle<=145&&change_compere_angle>=55)
		{
			
           	diagonal=1;
			point_number=3;
		    if(change_compere_angle<=125&&change_compere_angle>=60)
			PushballFlag=1;
			else PushballFlag=0;
			
		}else if(change_compere_angle<=55&&change_compere_angle>=-35)
		{
		
			diagonal=-1 ;
		    point_number=4;
			
            if(change_compere_angle<=35&&change_compere_angle>=-30)
			 PushballFlag=1;
			else PushballFlag=0;
		}

	}else 		///逆时针
	{
		if((change_compere_angle<=-145&&change_compere_angle>=-180)||(change_compere_angle>125&&change_compere_angle<=180))
		{
			diagonal=1;
			point_number=1;
	   
			if((change_compere_angle>=145)||(change_compere_angle>=-180&&change_compere_angle<=-150))
				PushballFlag=1;
			else PushballFlag=0;
			
		}else if(change_compere_angle<=125&&change_compere_angle>35)
		{ 
			diagonal=-1;
			point_number=2;
	
	        if(change_compere_angle>=55&&change_compere_angle<=120)
				PushballFlag=1;
			else PushballFlag=0;
		}else if(change_compere_angle<=35&&change_compere_angle>-55)
		{ 
			diagonal=1;
			point_number=3;
			
			 if(change_compere_angle>=-35&&change_compere_angle<=30)
			PushballFlag=1;		
			 else PushballFlag=0;
		}else if(change_compere_angle<=-55&&change_compere_angle>-145)
		{
			diagonal=-1;
			point_number=4;

            if(change_compere_angle>=-125&&change_compere_angle<=-60)
		     PushballFlag=1;
			else PushballFlag=0;
		}
		
	}
 //   point_number=3;
	if(Sn==1)
	{
		s=get_d(N_point_data[point_number-1][0],N_point_data[point_number-1][1]);
		ShooterVelCtrl(get_roll_v());  
		move_gun(N_point_data[point_number-1][0],N_point_data[point_number-1][1]);
	}else 
	{
		s=get_d(S_point_data[point_number-1][0],S_point_data[point_number-1][1]);
		ShooterVelCtrl(get_roll_v());  
		move_gun(S_point_data[point_number-1][0],S_point_data[point_number-1][1]);
	}
    now_shooter_v=fort.shooterVelReceive;
	now_get_rool_v=get_roll_v();
	if(Sn==1)
	{
	if(roundCnt==2&&point_number==4&&last_round_cnt==1)
		PushballFlag=0;
    }else
	{
		if(roundCnt==2&&point_number==1&&last_round_cnt==1)
		PushballFlag=0;
	}
//	if(last_shooter_v-now_get_rool_v<-8)
//	   now_shooter_v=now_get_rool_v;
//	else if(last_shooter_v-now_get_rool_v>5)
//	   now_get_rool_v=last_shooter_v;
	
	if(now_shooter_v-last_get_shooter_v>5&&now_shooter_v-now_get_rool_v>5)
		now_shooter_v=last_get_shooter_v;
		
	
	if(stable_flag)
	{
		if(PushballFlag) 
		{
			if(if_white_shoot/*last_get_shooter_v-now_shooter_v<=-3&&last_get_shooter_v-now_shooter_v>=-7&&if_shoot[point_number-1]&&now_get_rool_v-now_shooter_v<15*/)
			{				
				if_shoot[point_number-1]=0;
				if_white_shoot=0;
			}



            PushballFlag=if_shoot[point_number-1];			
		}
		
//		USART_OUT(UART4,(uint8_t*)"%d\t",(int)last_get_shooter_v);
		last_get_shooter_v=now_shooter_v;
				
    }
	
	if(if_shoot[0]==0&&if_shoot[1]==0&&if_shoot[2]==0&&if_shoot[3]==0&&roundCnt!=5)
	{
		for( int i=0;i<4;i++)
		 if_shoot[i]=1;
	}
	
	
//	USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t%d\t%d\t\r\n",(int)now_get_rool_v,if_shoot[0],if_shoot[1],if_shoot[2],if_shoot[3]);

}

//得到车与任意点的角度//

//推球//
//向推球电机发出的位置//起始为0//
int PushBallPosition=0;
//黑白球模式 白1 黑2 无3 //
static int color_mode=0;
//白球的时间//
static int White_ball_time;
//黑球与没球的时间//
static int Black_and_no_time;
//增加或减少/
static int add_or_reduce=-1;
//推球电机在范围内的时间//
static int position_in_wide=0;
//白球的发射时间//
static int fire_wait_time=50;//1m/s 为90s 
//推球轮子翻转后需要等待的时间//
static int wheel_need_wait_time=65;
//推球电机故障时间//
static int wheel_error_time=0;
int if_add_position=1;
int add_mode;
int abs(int);
void push_ball(void)
{
	if(color_mode==0)
	{ 
		if(Ballcolor[2]==Need_ball_collor)
		{	
			color_mode=1;
			
		}else if(Ballcolor[2]==No_need_ball_collor)
		{
			color_mode=2;
			
		}else if(Ballcolor[2]==0)
		{
			color_mode=3;
		}
	}
	
	
	switch(color_mode)
	{
		case 0:break;
		case 1://白球//
		if(stable_flag)
		{  
			if(PushballFlag)
			{   
				White_ball_time++;
			}
			
						
		    if(White_ball_time==fire_wait_time)//设定的白球投掷时间//
			{				
				PushBallPosition+=32768;
				White_ball_time+=1;
								
			}else if(White_ball_time>fire_wait_time)//缓冲时间//
			{
				if( abs(push_position.push_pos[1]-PushBallPosition)<=500)
                   position_in_wide++;
				else wheel_error_time++;
				//当位置范围在误差范围之内时//
				if(position_in_wide>=wheel_need_wait_time)
				{
					White_ball_time=0;
				    position_in_wide=0;
					color_mode=0;
					wheel_error_time=0;
					if_add_position=1;
				}
				//故障处理//
				if(wheel_error_time>=200)
				{
					PushBallPosition-=32768;
					White_ball_time=0;
				    position_in_wide=0;
					wheel_error_time=0;
					color_mode=0;
					
				}
			}
			
			if(PushballFlag==0)
			{
				White_ball_time=0;
				position_in_wide=0;

			}
		}
			break;
		case 2://黑球//
            Black_and_no_time++;
            if(Black_and_no_time==1)	
			{
				PushBallPosition-=32768;
			
			}else if(Black_and_no_time>=2)
            {
				if( abs(push_position.push_pos[1]-PushBallPosition)<=500)
                   position_in_wide++;
				else wheel_error_time++;
				//当位置范围在误差范围之内时//
				if(position_in_wide>=wheel_need_wait_time)
				{
					Black_and_no_time=0;
				    position_in_wide=0;
					color_mode=0;
					wheel_error_time=0;
				}
				//故障处理//
				if(wheel_error_time>=200)
				{
					PushBallPosition+=32768;
					Black_and_no_time=0;
				    position_in_wide=0;
					color_mode=0;
					wheel_error_time=0;				
				}
            }
			
			
			break;
        case 3://没球//
            Black_and_no_time++; 
            #if push_mode == 1//左右舱室互换//每次旋转180°
				if(Black_and_no_time==1)
				{
					PushBallPosition=PushBallPosition+add_or_reduce*32768/2;
					add_or_reduce=-add_or_reduce;
				}else if(Black_and_no_time>=2)
				{
					if( abs(push_position.push_pos[1]-PushBallPosition)<=500)
                      position_in_wide++;
		            else wheel_error_time++;
						if(position_in_wide>=wheel_need_wait_time)
					{
						if(Ballcolor[2])
						{
							color_mode=0;						
						}
						position_in_wide=0;
						Black_and_no_time=0;
						wheel_error_time=0;
			             
					}	
					
					//故障处理//
					if(wheel_error_time>=200)
				    {
					add_or_reduce=-add_or_reduce;
					PushBallPosition=PushBallPosition+add_or_reduce*32768/2;
					Black_and_no_time=0;
				    position_in_wide=0;
					color_mode=0;
					wheel_error_time=0;
				    }
				}
		#elif push_mode == 2//先用一个舱室，再用另一个//每次旋转360°//
		        if(Black_and_no_time<200)
				{
				   if(Ballcolor[2])
					{
						color_mode=0;						
					}
				}else
				{
					if(Black_and_no_time==200)
					{PushBallPosition=PushBallPosition+add_or_reduce*32768/2;
				     add_or_reduce=-add_or_reduce;
					}
					if( abs(push_position.push_pos[1]-PushBallPosition)<=500)
                      position_in_wide++;
					else wheel_error_time++;
						if(position_in_wide>=wheel_need_wait_time)
					{
						if(Ballcolor[2])
						{
							color_mode=0;						
						}
						position_in_wide=0;
						Black_and_no_time=0;
						wheel_error_time=0;
					}	
					
					//故障处理//
					if(wheel_error_time>=200)
				    {
					add_or_reduce=-add_or_reduce;
					PushBallPosition=PushBallPosition+add_or_reduce*32768/2;
					Black_and_no_time=0;
				    position_in_wide=0;
					color_mode=0;
					wheel_error_time=0;
					
			    	}
				 
				}
				
		#endif
			break;	
					
	}
		
	if(PushballFlag==0)
	{
		White_ball_time=0;
	}
	PosCrl(CAN2, PUSH_BALL_ID,ABSOLUTE_MODE,PushBallPosition);
//	USART_OUT( UART4, (uint8_t*)"%d\t%d\t",push_position.push_pos[1],(int)PushBallPosition);
//	USART_OUT(UART4,(uint8_t*)" color_mode=%d\r\n",color_mode);
	

}

//得到车与四个点的距离//
float get_d(float x,float y)
{
	return(sqrt(pow (GetX()-x,2)+pow(GetY()-y,2)));
}
//得到车与四个点的角度与 车的速度角度的插值//
float get_differ_angle(float angle)
{
	
	float nowdiffer_angle=0;
	float car_run_v=make_angle_in_wide(90-atan2(GetSpeedX(),GetSpeedY())/pi*180,-180);
	nowdiffer_angle=make_angle_in_wide(angle-car_run_v,-180);
	if(nowdiffer_angle>0)
		add_or_reduce_angle=-1 ;
	else if(nowdiffer_angle<0)
		add_or_reduce_angle=1;
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
//将角度限制在0~360度 point_angle=0//  //将角度限制在-180~180度 point_angle=-180//
float make_angle_in_wide(float angle,float point_angle)
{
	
	if(angle>360+point_angle)
		angle-=360.0f;
	else if(angle<=0+point_angle)
		angle+=360.0f;
//	USART_OUT( UART4, (uint8_t*)"hanshunei %d ", (int)angle);
	return(angle);
}
//计算并得到炮台需要运动到的角//
float last_body_angle=0;
int i=0;
float go_real_send_angle=0;
extern int roundCnt;
void move_gun(float point_x,float point_y)
{
	float car_angle_to_point=0;
	float x=0;
	float cartopoint_angle=0;//车指向固定点的角度//
	float getget_angle=0;//从bodyangle与y轴正方向的角度//	
	float begain_angle=0;//炮台的起始角度//       	
	float fake_new_angle=0;//假的新角度//
	float bili=0;
	float addadd=0;
	x=get_d(point_x,point_y);
	car_angle_to_point=get_angle2(GetY()-point_y,-GetX()+point_x,(GetY()-point_y)/fabs((GetY()-point_y)),Sn);//得到固定点指向车的角度//
//	if(fabs(make_angle_in_wide(last_body_angle-car_angle_to_point,-180))>178)
//	car_angle_to_point=last_body_angle;	
//	else last_body_angle=car_angle_to_point;
	USART_OUT( UART4, (uint8_t*)"angle ");
	cartopoint_angle=make_angle_in_wide(car_angle_to_point+180,-180);//将角度限制在-180~180//

	add_angle=get_addorreduce_angle(get_differ_angle(cartopoint_angle),x);	
	
//	if(Sn==1)	//逆时针
//	{   if( diagonal==1)//左下右上
//		bili=1.2;
//		else bili=0.7;
//	}else
//    {
//		if( diagonal==1)//左下右上
//		bili=1.2;
//		else bili=2.6;
//	}
	
	if(Sn==1)
	{
		if(roundCnt==2)
			bili=0.1;
		else if(roundCnt==3||roundCnt==4)
			bili=0.05;
		else bili=0.05;
    }else if(Sn==-1)
	{
		if(roundCnt==2)
			bili=0.1;
		else if(roundCnt==3||roundCnt==4)
			bili=0.05;
		else bili=0.05;
	}
	
 //   USART_OUT( UART4, (uint8_t*)"%d ", (int)go_real_send_angle);
	addadd=add_or_reduce_angle*bili*add_angle;
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)addadd);
	car_angle_to_point=make_angle_in_wide(car_angle_to_point+add_or_reduce_angle*bili*add_angle,-180);	//1m/s 为0.6//	
	getget_angle=make_angle_in_wide(90+car_angle_to_point,-180);//将角度限制在-180~180//
	fake_new_angle=make_angle_in_wide(begain_angle-getget_angle+GetAngle(),-180);
	get_fake_angle=make_angle_in_wide(fake_new_angle-fake_last_angle,-180);
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)go_real_send_angle);
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)get_fake_angle);
	go_real_send_angle=go_real_send_angle+get_fake_angle;//弥补误差//
//	USART_OUT( UART4, (uint8_t*)"%d ", (int)go_real_send_angle);
//	USART_OUT( UART4, (uint8_t*)"angle ");
//   	USART_OUT(UART4,(uint8_t*)"%d\t%d\t%d\t%d\t%d\t",(int)car_angle_to_point,(int)fake_new_angle,(int)fake_last_angle,(int)get_fake_angle,(int)real_send_angle);
	fake_last_angle=fake_new_angle;
	//发送角度
	
	YawPosCtrl(go_real_send_angle);
//	    USART_OUT(UART4,(uint8_t*)"gun_v=%d\t\r\n",(int)fort.shooterVelReceive);
//		USART_OUT(UART4,(uint8_t*)"add_angle=%d\t\r\n",(int)add_angle);
}

int if_add_time=0;
int out_position_time=0;
int normal_push=1;
int ShaveTime=0;
int deal_measure=0;
int measure_time=0;
int last_normal_push;
extern int No_StableTime;
extern int Out_AreaTime;
int shave_mode=0;
void PushBallErrorDeal(void)
{
	if(!if_add_time)
	{
		if(abs(PushBallPosition-push_position.push_pos[1])>500)
			out_position_time++;
		else if(abs(PushBallPosition-push_position.push_pos[1])<500)
		{
			out_position_time=0;
			normal_push=1;
			shave_mode=0;
		}		
    }
	if(out_position_time==(int)200/(shave_mode+1))
	{
		shave_mode++;
		if_add_time=1;
		PushBallPosition=(push_position.push_pos[1]/16384)*16384;
		LastpushBallposition=PushBallPosition;
		normal_push=0;
	}
	if(if_add_time)
	{
		ShaveTime++;
	}
	if(ShaveTime==50)
		PushBallPosition+=5461;
	else if(ShaveTime==100)
	{
		PushBallPosition-=5461;
	}else if(ShaveTime==150)
	{
		PushBallPosition+=5461;
	}else if(ShaveTime==200)
	{
		PushBallPosition-=5461;
	}
	else if(ShaveTime==250)
	{
		if_add_time=0;
		out_position_time=0;
		ShaveTime=0;
	}
    if(shave_mode>=3)
		shave_mode=3;
}
