#include "gun.h"
#include "elmo.h"
#include "can.h"
#include "stm32f4xx.h"
#include "usart.h"
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





//航向电机
// 将角度转换为脉冲
float YawTransform(float yawAngle)
{
	return (yawAngle * YAW_REDUCTION_RATIO * COUNT_PER_DEGREE);
}

//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
void YawAngleCtr(float yawAngle)
{
	PosCrl(CAN1, GUN_YAW_ID, ABSOLUTE_MODE, YawTransform(yawAngle));
}
// 同样要配置位置环
