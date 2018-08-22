#ifndef __GUN_H
#define __GUN_H
#include "stdint.h"
typedef union
{
    //这个32位整型数是给电机发送的速度（脉冲/s）
    int32_t Int32 ;
    //通过串口发送数据每次只能发8位
    uint8_t Uint8[4];

}num_t;
void SendUint8(void);
float YawTransform(float yawAngle);
void YawAngleCtr(float yawAngle);

// 宏定义棍子收球电机ID
#define COLLECT_BALL_ID (8)

// 宏定义推球电机ID
#define PUSH_BALL_ID (6)
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
#endif
