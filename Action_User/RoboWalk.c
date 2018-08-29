#include "RoboWalk.h"

//LengthProcessing中使用,当x小于X1时,返回0,大于X2时返回1,之间是返回一个之间的量
#define X1 100
#define X2 200
//两侧激光遮挡的临界值,大于它时认为没有阻挡,小于它时认为激光被挡住了
#define THRESHOLD4ADC 150

extern point nowPoint;

enum {clockwise, anticlockwise} Dir2TurnAround = clockwise; 

#define CCX 0
#define CCY 2400
point circlecentre;//{2400, 0}
reldir circledir = right;
float circleradius = 800;

/**
 * [RoboWalkInit 机器人行走初始化]
 */
void RoboWalkInit(void)
{
    extern PIDCtrlStructure PidA;
    extern PIDCtrlStructure PidB;	
    MoveBaseInit();
    Adc_Init();
    PidA.KP = 20; //20
    PidA.KI= 0.01; //0.01
    PidA.KD = 1; //1
    PidA.GetVar = GetA;
    PidA.ExOut = 0;
    PidB.KP = 10; //10
    PidB.KI= 0.0001; //0.0001
    PidB.KD = 0.5; //0.5
    PidB.GetVar = GetA;
    PidB.ExOut = 0;
    circlecentre = setPointXY(CCX, CCY);
    PIDCtrlInit1();
    PIDCtrlInit2();
}

/**
 * [LengthProcessing 把距离关系转换成0~1之间的数的关系]
 * @param  x [采到的距离]
 * @return   [转换后的数]
 */
float LengthProcessing(float x)
{
    if(x <= X1)
    {
        return 0;
    }
    else if(x >= X2)
    {
        return 1;
    }
    else
    {
        return (atan((x - X1) / (X2 - x)) / 1.57);
    }
}

/**
 * [GetP4circle 把采到的距离按照pid的输入量返回]
 * @more  当车为顺时针时,在圆内返回负值,圆外返回正值,逆时针相反.关于circle取值请参看Getpid部分
 * @return  [符合pid输入量关系的量]
 */
float GetP4circle(void)
{
    float tempNum = 0, tempAns = 0;
    tempNum = Point2Point(circlecentre, nowPoint) - circleradius;
    tempAns = LengthProcessing(__fabs(tempNum));
    if(circledir == left)
    {
        tempAns = -1 * tempAns;
    }
    if(tempNum > 0)
    {
        return tempAns;
    }
    else if(tempNum < 0)
    {
        return -1 * tempAns;
    }
    else
    {
        return 0;
    }
}

/**
 * [SetDir2TurnAround 根据Dir2TurnAround的值改变相关线路方向]
 */
void SetDir2TurnAround(void)
{
    if(Dir2TurnAround == clockwise)
    {
        circledir = right;
    }
    else if(Dir2TurnAround == anticlockwise)
    {
        circledir = left;
    }
}

/**
 * [ChangeRoad 改变线路]
 */
void ChangeRoad(void)
{
    extern PIDCtrlStructure PidA;
    extern PIDCtrlStructure PidB;	
    static _Bool yChangeFlag = 0;
    static float lasta = 0;
    _Bool changFlag = 0;
    linewithdir line2centre;
    point relPoint;
    SetDir2TurnAround();
    relPoint = RelPos(circlecentre, nowPoint);
    if(relPoint.y > 0)
    {
        yChangeFlag = 1;
    }
    if(relPoint.y < 0 && yChangeFlag)
    {
        if(lasta * relPoint.a < 0)
        {
            yChangeFlag = 0;
            if(changFlag == 0)
            {
                circleradius += 300;
            }
            if(changFlag == 1)
            {
                circleradius -= 300;
            }
        }
    }
    lasta = relPoint.a;
    if(circleradius > 1700 && changFlag == 0)
    {
        while(1)
        {
            WheelSpeed(0, 1);
            WheelSpeed(0, 2); 
        }
        changFlag = 1;
        circleradius = 1700;
    }
    if(circleradius < 800 && changFlag == 1)
    {
        changFlag = 0;
        circleradius = 800;
    }
    line2centre = DirlinePoint2Point(circlecentre, nowPoint);
    PidA.ExOut = VDirForLine(line2centre, circledir);
    PidB.ExOut = PidA.ExOut - 90 * GetP4circle();
    if(PidB.ExOut >= 180)
    {
        PidB.ExOut -= 360;
    }
    if(PidB.ExOut < -180)
    {
        PidB.ExOut += 360;
    }
}

/**
 * [Wait4ADC 等待激光被遮挡]
 */
void Wait4ADC(void)
{	
    _Bool ADCFlag = 1;
    while(ADCFlag)
    {
        static _Bool leftFlag = 0, rightFlag = 0;
        if(GETADCLEFT < THRESHOLD4ADC)
        {
            leftFlag = 1;
        }
        if(GETADCRIGHT < THRESHOLD4ADC)
        {
            rightFlag = 1;
        }
        if(GETADCLEFT > THRESHOLD4ADC && leftFlag)
        {
            Dir2TurnAround = clockwise;
            ADCFlag = 0;
        }
        if(GETADCRIGHT >THRESHOLD4ADC && rightFlag)
        {
            Dir2TurnAround = anticlockwise;
            ADCFlag = 0;
        }
    }
}

void Change_Dir_2_Turn_Around(void)
{	
    if(Dir2TurnAround == clockwise)
    {
        Dir2TurnAround = anticlockwise;
    }
    if(Dir2TurnAround == anticlockwise)
    {
        Dir2TurnAround = clockwise;
    }
    SetDir2TurnAround();
}
