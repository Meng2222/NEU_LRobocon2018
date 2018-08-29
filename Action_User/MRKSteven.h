#ifndef MRKSTEVEN_H
#define MRKSTEVEN_H  


typedef struct CAngle
	{
	float m_angle_Target;//目标角度 
    float m_angle_Dvalue;//角度差（测量值angl-预期量ang）
	}CAngle;	
	
	
typedef struct CLine
	{
	float m_line_angle;	//直线方向角（车坐标）
    float m_line_yintercept;//y轴截距（标准坐标）
    float m_line_xintercept;//x轴坐标（标准坐标）
	float m_line_distance;
	float m_line_slope;
	}CLine;
	
	 
typedef struct CSquare
	{
	int   m_square_side;
	float m_square_middle;
	float m_square_halfedges;
	int   m_switch_distance;
	float m_square_distance;
	}CSquare;
	

typedef struct CCircle
	{
	float m_radius;   //半径
	float m_center_x; //圆心坐标
	float m_center_y; //圆心坐标
		
	float m_angle_tpcenter;  //机器人与圆心连线角度
	float m_angle_tptangent; //机器人（切线）偏差角度
	float m_distance_tpcenter;//机器人到圆心距离
	float m_d_angle; //机器人偏差距离校正
	float m_angle_target;   //机器人最终行进角度
	}CCircle;

//====================================================================================   
// _   _   _   _   _   _   _   _   _                _   _   _   _   _   _   _   _   _ 
//| |_| |_| |_| |_| |_| |_| |_| |_| |_【参数选择】_| |_| |_| |_| |_| |_| |_| |_| |_| |
//
//====================================================================================	
#define veh         0
	
#define v           1      //【Mode1】【Mode2】【Mode3】【Mode4】【Mode5】车身速度（m/s）
#define Mode        3      // 0调试状态（目前设置为静止）
				           // 1直行（r=0）||圆周运动 前进/后退; 
                           // 2直行（r=0）||多边形运动（此时r为多边形边长）（带自动校正）
                           // 3直线闭环
                           // 4正方形闭环
	                       // 5圆闭环
	                       // 6正方形扫荡+ADC激光爆炸
#define pi          3.14

#define Kp_A        200    //【P】角度闭环————|| 300的Kp_A可能导致角度积分错误
#define Ki_A        0      //【I】角度闭环————|| 
#define Kd_A        0      //【D】角度闭环————|| 
                           //
#define Kp_l        0.03   //【D】直线闭环————||  0.09（Kp_l=90度/p_a(1000)）——————（Mode3直线用90ok）
#define Ki_l        0      //【I】直线闭环————||
#define Kd_l        0      //【D】直线闭环————||

#define Kp_A0        4251   //【P】角度闭环————|| 300的Kp_A可能导致角度积分错误
#define Ki_A0        0      //【I】角度闭环————|| 
#define Kd_A0        0      //【D】角度闭环————|| 
                            //
#define Kp_l0        0.03   //【D】直线闭环————||  0.09（Kp_l=90度/p_a(1000)）——————（Mode3直线用90ok）
#define Ki_l0        0      //【I】直线闭环————||
#define Kd_l0        0      //【D】直线闭环————||

//====================================================================================			
#define switch_distance  500 //【Mode3】直线闭环校正开始距离（mm）
//角度闭环参量
#define angle            180.f
//直线闭环参量
#define line_angle       0.f
#define line_xintercept  0.f
#define line_yintercept  0.f
//square闭环参量
#define	square_middle    2000
#define square_halfedges 1000
//circle闭环参量
#define radius           1.f      //半径
#define	center_x         0.f       //圆心坐标
#define center_y         2000.f    //圆心坐标

#define direction  0       //【Mode1】：方向（0为前进，1为后退）
                           //【Mode5】：顺时针/逆时针走圆（0为逆时针，1为顺时针）		
						   
						   
#define r          0     //【Mode1】【Mode2】半径 || 边长（m）
                           //【Mode1】：车身旋转半径（填入0则直行）（r>0逆时针运动；r<0顺时针运动）
                           //【Mode2】：正方形边长（r=0为直行）
						   
//====================================================================================						   
						   
// 宏定义棍子收球电机ID

#define COLLECT_BALL_ID (8)

// 宏定义推球电机ID

#define PUSH_BALL_ID (6)

// 宏定义送弹机构送弹时电机应该到达位置：单位位脉冲

#define PUSH_POSITION (4500)

// 宏定义送弹机构收回时电机位置

#define PUSH_RESET_POSITION (5)

// 宏定义发射机构航向电机ID

#define GUN_YAW_ID (7)

// 电机旋转一周的脉冲数

#define COUNT_PER_ROUND (4096.0f)

// 宏定义每度对应脉冲数

#define COUNT_PER_DEGREE  (COUNT_PER_ROUND/360.0f)

// 宏定义航向角减速比

#define YAW_REDUCTION_RATIO (4.0f)

//正方形闭环：1m/s: Kp_A=200 p_a=500 Kp_l=0.03 （距离Kp过小导致机器人无法快速回到直线）

//====================================================================================
float YawTransform(float yawAngle);
void YawAngleCtr(float yawAngle);

void Coordinate_Reverse(void); 
void Angle_Lock4(float angle_target);
void Move(int V1,int V2);
void Move_0(float V01,float V02);
void Line_Lock4(float lineangle, float yintercept, float xintercept);
void Circle_Lock1(void);
void Square_Lock1(int squaremiddle , int squareHalfedges);
void Square_Sweep_Right1(int square_m , int square_e);
void Square_Sweep_Left1(int square_m , int square_e);
void Adc(void);
void Position_Record(void);
void Adc_Check(void);
void Square_Movement (void);
void Move_Basic(void);
void square_edg_jump(void);
void Collision_Processing(void);
void Angle_Lock5_plus(float angle_target);

void Target_Angle_Calculate(void);
void Target_Angle(void);
void Target_Relative_Angle_Lock(void);
void Yaw_Scanning (void);
void ShooterVelControl(void);
#endif
