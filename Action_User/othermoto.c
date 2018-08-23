#include "othermoto.h"
#include "usart.h"
#include "elmo.h"
#include "can.h"
#include "stm32f4xx_gpio.h"
#include "fort.h"
#include "gpio.h"

float GetLauncherSpeed(void)
{
    return ReadShooterVel();
}

float GetLauncherAngle(void)
{
    return ReadYawPos();
}

float GetLaserA(void)
{
    return ReadLaserAValue();
}

float GetLaserB(void)
{
    return ReadLaserBValue();
}

void OtherMotoInit(void)
{
    UART5_Init(921600);
    CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
    VelLoopCfg(CAN1, COLLECT_BALL_ID, 50000, 50000);
    PosLoopCfg(CAN1, PUSH_BALL_ID, 50000,50000,20000);
    ElmoInit(CAN1);
    MotorOn(CAN1, COLLECT_BALL_ID);
    MotorOn(CAN1, PUSH_BALL_ID);
    BEEP_ON;
    LauncherWheelSpeedCtrl(LAUNCHER_SPEED_DEFINE);
    CollecterWheelSpeedCtrl(COLLECTER_SPEED_DEFINE);
}

void LauncherWheelSpeedCtrl(int32_t speed)
{
    ShooterVelCtrl(speed);
}

void CollecterWheelSpeedCtrl(int32_t speed)
{
    VelCrl(CAN1, COLLECT_BALL_ID, speed * COUNT_PER_ROUND);
}

void SendBall2Launcher(void)
{
    static uint16_t counter = 0;
    counter++;
    //设置推球状态维持次数
    if(counter <= 200)
    {
        // 推球
        PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);
    }
    //设置复位状态维持次数
    else if(counter > 200 && counter <= 400)
    {
        // 复位
        PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);
    }
    //计数器清零
    if(counter == 400)
    {
        counter = 0;
    }
}

//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
void LauncherYAWCtrl(float yawAngle)
{
    YawPosCtrl(yawAngle);
}
