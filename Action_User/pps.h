#ifndef __PPS_H
#define __PPS_H
#include "stdint.h"


/*接受几个来自定位系统的float数据*/
#define GET_PPS_VALUE_NUM      6
/*接受几个来自定位系统的uint8_t数据*/ /* 6 * 4byte = 24*/
#define GET_PPS_DATA_NUM       24
// 宏定义棍子收球电机ID
#define COLLECT_BALL1_ID (5)
#define COLLECT_BALL2_ID (6)
// 宏定义推球电机ID
#define PUSH_BALL_ID (7)
// 宏定义送弹机构送弹时电机应该到达位置：单位位脉冲
#define PUSH_POSITION (4500)
// 宏定义送弹机构收回时电机位置
#define PUSH_RESET_POSITION (5)
//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
// 宏定义发射机构航向电机ID
#define GUN_YAW_ID (7)
// 电机旋转一周的脉冲数
#define COUNT_PER_ROUND (4096.0f)
// 宏定义每度对应脉冲数
#define COUNT_PER_DEGREE  (COUNT_PER_ROUND/360.0f)
// 宏定义航向角减速比
#define YAW_REDUCTION_RATIO (4.0f)
// 发射航向角转换函数 由度转换为脉冲
// yawAngle为角度，范围180到-180之间，初始位置为0度。

typedef union{
	uint8_t data[GET_PPS_DATA_NUM];
	float  value[GET_PPS_VALUE_NUM];
}PosSend_t;

/*定位系统返回的值*/
typedef struct{
		/*定位系统返回的角度*/
		float ppsAngle ;
		/*定位系统返回的X值*/
		float ppsX ;
		/*定位系统返回的Y值*/
		float ppsY ;
		/*定位系统返回的X轴速度*/
		float ppsSpeedX;
		/*定位系统返回的Y轴速度*/
		float ppsSpeedY;
		/*定位系统的z轴角速度*/
		float ppsWZ ;
}Pos_t;


void TalkOpsToGetReady(void);
/*初始化并且让程序等待定位系统发数*/
void WaitOpsPrepare(void);

void SetOpsReady(uint8_t flag);
void SetAngle(float setValue);
void SetX(float setValue);
void SetY(float setValue);
void SetSpeedX(float setValue);
void SetSpeedY(float setValue);
void SetWZ(float setValue);

/*定位系统准备完毕*/
uint8_t GetOpsReady(void);
/*返回定位系统的角度*/
float GetAngle(void);
/*返回定位系统的X值*/
float GetX(void);
/*返回定位系统的Y值*/
float GetY(void);
/*返回定位系统的X轴的速度*/
float GetSpeedX(void);
/*返回定位系统的角度*/
float GetSpeedY(void);
/*返回定位系统的Z轴角速度值*/
float GetWZ(void);


#endif 

