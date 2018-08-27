#include "othermoto.h"
#include "usart.h"
#include "elmo.h"
#include "can.h"
#include "stm32f4xx_gpio.h"
#include "fort.h"
#include "gpio.h"
#include "Pos.h"

// 宏定义送弹机构送弹时电机应该到达位置：单位位脉冲
#define PUSH_POSITION (4500)
// 宏定义送弹机构收回时电机位置
#define PUSH_RESET_POSITION (5)
//各轮速度默认值
#define LAUNCHER_SPEED_DEFINE (0)
#define COLLECTER_SPEED_DEFINE (60)

//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
// 宏定义发射机构航向电机ID
// 电机旋转一周的脉冲数
#define COUNT_PER_ROUND (4096.0f)
// 宏定义每度对应脉冲数
#define COUNT_PER_DEGREE  (COUNT_PER_ROUND/360.0f)
// 宏定义航向角减速比
#define YAW_REDUCTION_RATIO (4.0f)
//发射器到定位器的距离
#define RPOS4LAUNCHER (95)
//推球次数
#define TIMES_4_PUSH_BALL (50)
//复位次数
#define TIMES_4_RESET (50)
//激光器最大误差
#define MAX_ERR_4_LASER (15)

static point storageInAirPointlb, storageInAirPointlf, storageInAirPointrb, storageInAirPointrf;

extern point nowPoint;

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
    PosLoopCfg(CAN1, PUSH_BALL_ID, 500000,500000,500000);
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

void CollecterWheelSpeedCtrl(float speed)
{
    VelCrl(CAN1, COLLECT_BALL_ID, speed * COUNT_PER_ROUND);
}

void SendBall2Launcher(void)
{
    static uint16_t counter = 0;
    counter++;
    //设置推球状态维持次数
    if(counter <= TIMES_4_PUSH_BALL)
    {
        // 推球
        if(counter == 1)
        {
            PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_POSITION);
        }
    }
    //设置复位状态维持次数
    else if(counter > TIMES_4_PUSH_BALL && counter <= (TIMES_4_PUSH_BALL + TIMES_4_RESET))
    {
        // 复位
        if(counter == TIMES_4_PUSH_BALL + 1)
        {
            PosCrl(CAN1, PUSH_BALL_ID,ABSOLUTE_MODE,PUSH_RESET_POSITION);
        }
    }
    //计数器清零
    if(counter >= (TIMES_4_PUSH_BALL + TIMES_4_RESET))
    {
        counter = 0;
    }
}

//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
void LauncherYAWCtrl(float yawAngle)
{
    YawPosCtrl(yawAngle);
}

//设置储藏区位置
void StoragePosSet(void)
{
    storageInAirPointrf = setPointXY(2400 - LENGTH_TO_THE_SIDE, 4800 - LENGTH_TO_THE_SIDE);
    storageInAirPointrb = setPointXY(2400 - LENGTH_TO_THE_SIDE, LENGTH_TO_THE_SIDE);
    storageInAirPointlf = setPointXY(LENGTH_TO_THE_SIDE - 2400, 4800 - LENGTH_TO_THE_SIDE);
    storageInAirPointlb = setPointXY(LENGTH_TO_THE_SIDE - 2400, LENGTH_TO_THE_SIDE);
}

//返回某个储藏区相对于炮台的位置
point GetPosToLauncher(const point storageInAirPoint)
{
    point LauncherPos, nowpoint;
    StoragePosSet();
    nowpoint = GetNowPoint();
    LauncherPos = setPointRA(RPOS4LAUNCHER, GetA());
    LauncherPos = StrPos(nowpoint, LauncherPos);
    return RelPos(LauncherPos, storageInAirPoint);
}

//返回最近的炮台的炮台
point SuitableStoragePos(void)
{
    point nowpoint;
    nowpoint = GetNowPoint();
    StoragePosSet();
    if(nowpoint.x >= 0)
    {
        if(nowpoint.y <= 2400)
        {
            return storageInAirPointrb;
        }
        else
        {
            return storageInAirPointrf;
        }
    }
    else
    {
        if(nowpoint.y <= 2400)
        {
            return storageInAirPointlb;
        }
        else
        {
            return storageInAirPointlf;
        }
    }
}

/**
 * [SuitableSpeed2Launch 返回适合投球的发射轮转速]
 * @param  distance [与储藏区的距离]
 * @return          [适合发射的发射轮转速]
 */
float SuitableSpeed2Launch(float distance)
{
    float speed = 0;
    speed = 0.0153f * distance + 34.046f;
//    speed = 0.0129f * distance + 35.070f;
    return speed;
}

//判断激光瞄准是否在误差范围内
_Bool Delta4Laser(void)
{
    extern point storagePos;
    point launcherPos;
    float DlaserA = 0, DlaserB = 0;
    float dirY = 0, exOfLaser = 0;
    launcherPos = setPointRA(RPOS4LAUNCHER, GetA());
    launcherPos = StrPos(nowPoint, launcherPos);
    if(launcherPos.y > 2400)
    {
        dirY = 2400 - launcherPos.y;
    }
    if(launcherPos.y < 2400)
    {
        dirY = launcherPos.y;
    }
    exOfLaser = storagePos.r * (1 + (LENGTH_TO_THE_SIDE / dirY));
    DlaserA = GetLaserA() - exOfLaser;
    DlaserB = GetLaserB() - exOfLaser;
    USART_OUT(UART4, (uint8_t *)"%d %d\r\n", (int32_t)GetLaserA(), (int32_t)GetLaserB());
    if(DlaserA > (225 - MAX_ERR_4_LASER) && DlaserB > (225 - MAX_ERR_4_LASER) && DlaserA < (225 + MAX_ERR_4_LASER) && DlaserB < (225 + MAX_ERR_4_LASER))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
