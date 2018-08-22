#include "othermoto.h"
#include "usart.h"
#include "elmo.h"
#include "can.h"
#include "stm32f4xx_gpio.h"

void OtherMotoInit(void)
{
    USART1_Init(115200);
    CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
    VelLoopCfg(CAN1, COLLECT_BALL_ID, 50000, 50000);
    PosLoopCfg(CAN1, PUSH_BALL_ID, 50000,50000,20000);
    PosLoopCfg(CAN1, GUN_YAW_ID, 50000,50000,20000);
    ElmoInit(CAN1);
    MotorOn(CAN1, COLLECT_BALL_ID);
    MotorOn(CAN1, PUSH_BALL_ID);
    MotorOn(CAN1, GUN_YAW_ID);
    LauncherWheelSpeedCtrl(LAUNCHER_SPEED_DEFINE);
    CollecterWheelSpeedCtrl(COLLECTER_SPEED_DEFINE);
}

void LauncherWheelSpeedCtrl(int32_t speed)
{
    union
    {
        int32_t Int32;
        uint8_t Uint8[4];
    }u_Num;
    u_Num.Int32 = speed;
    USART_SendData(USART1, 'A');
    for(uint8_t i  = 0; i < 4; i++)
    {
        USART_SendData(USART1, u_Num.Uint8[i]);
    }
    USART_SendData(USART1, 'J');
}

void CollecterWheelSpeedCtrl(int32_t speed)
{
    VelCrl(CAN1, COLLECT_BALL_ID, speed);
}

void SendBall2Launcher(void)
{
    static uint16_t counter = 0;
    counter++;
    if(counter <= 500)
    {
        // 推球
        PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);
    }
    else if(counter > 500 && counter <= 1000)
    {
        // 复位
        PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);
    }
    if(counter == 1000)
    {
        counter = 0;
    }
}

// 将角度转换为脉冲
float YawTransform(float yawAngle)
{
    return (yawAngle * YAW_REDUCTION_RATIO * COUNT_PER_DEGREE);
}

//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
void LauncherYAWCtrl(float yawAngle)
{
    PosCrl(CAN1, GUN_YAW_ID, ABSOLUTE_MODE, YawTransform(yawAngle));
}
