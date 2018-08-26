#include "RoboWalk.h"

//LengthProcessing中使用,当x小于X1时,返回0,大于X2时返回1,之间是返回一个之间的量
#define X1 50
#define X2 80
//两侧激光遮挡的临界值,大于它时认为没有阻挡,小于它时认为激光被挡住了
#define THRESHOLD4ADC 150

extern point nowPoint;

enum {clockwise, anticlockwise} Dir2TurnAround = clockwise; 

linewithdir line1 = {0, -1, 0, forward};
linewithdir line2 = {-1, 0, 0, forward};
linewithdir line3 = {0, -1, 0, backward};
linewithdir line4 = {-1, 0, 0, backward};

const linewithdir corss1 = {1, 1, -2400, forward};
const linewithdir corss2 = {-1, 1, -2400, forward};

#define CCX 0
#define CCY 2400
point circlecentre;//{2400, 0}
reldir circledir = right;
float circleradius = 800;
float length = 0;
enum {circle, fang} roadsignal = circle;

linewithdir Target =
{
    .a = 0,
    .b = 1,
    .c = 0,
    .linedir = forward,
};

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
    PidA.KD = 20; //20
    PidA.GetVar = GetA;
    PidA.ExOut = 0;
    PidB.KP = 10; //10
    PidB.KI= 0.001; //0.0001
    PidB.KD = 20; //20
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
 * [GetP4fang 把采到的距离按照pid的输入量返回]
 * @more  当车在带方向直线右边时,返回正值,左边反之
 * @return  [符合pid输入量关系的量]
 */
float GetP4fang(void)
{
    float tempNum = 0;
    tempNum =Point2Line(Target, nowPoint);
    if(RelDir2Line(Target, nowPoint) == right)
    {
        return LengthProcessing(__fabs(tempNum));
    }
    else if(RelDir2Line(Target, nowPoint) == left)
    {
        return -1 * LengthProcessing(__fabs(tempNum));
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
        line1.linedir = forward;
        line2.linedir = forward;
        line3.linedir = backward;
        line4.linedir = backward;
    }
    else if(Dir2TurnAround == anticlockwise)
    {
        circledir = left;
        line1.linedir = backward;
        line2.linedir = backward;
        line3.linedir = forward;
        line4.linedir = forward;
    }
}

/**
 * [setFangArea 设置方形区域的相关线路]
 * @param length [方形区域的半边长]
 */
void setFangArea(float length)
{
    line1.c = 2400 - length;
    line2.c = -length;
    line3.c = 2400 + length;
    line4.c = length;
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
            if(roadsignal == circle)
            {
                circleradius += 500;
            }
            if(roadsignal == fang)
            {
                length -= 500;
                setFangArea(length);
            }
        }
    }
    lasta = relPoint.a;
    if(circleradius >= 1700 && roadsignal == circle)
    {
        roadsignal = fang;
        length = 1700;
        circleradius = 0;
        setFangArea(length);
    }
    if(length <= 1000 && roadsignal == fang)
    {
        roadsignal = circle;
        circleradius = 800;
        length = 0;
    }
    if(roadsignal == fang)
    {
        reldir dir2corss1,dir2corss2;
        dir2corss1 = RelDir2Line(corss1, nowPoint);
        dir2corss2 = RelDir2Line(corss2, nowPoint);
        if(dir2corss1 == left)
        {
            if(dir2corss2 == right)
            {
                Target = line1;
            }
            else if(dir2corss2 == left)
            {
                Target = line2;
            }
            else
            {
                Target = Target;
            }
        }
        else if(dir2corss1 == right)
        {
            if(dir2corss2 == left)
            {
                Target = line3;
            }
            else if(dir2corss2 == right)
            {
                Target = line4;
            }
            else
            {
                Target = Target;
            }
        }
        else
        {
            Target = Target;
        }
    }
    if(roadsignal == circle)
    {
        line2centre = DirlinePoint2Point(circlecentre, nowPoint);
        PidA.ExOut = VDirForLine(line2centre, circledir);
        PidB.ExOut = PidA.ExOut - 90 * GetP4circle();
    }
    else if(roadsignal == fang)
    {
        PidA.ExOut = LineDir(Target);
        PidB.ExOut = PidA.ExOut + 90 * GetP4fang();
    }
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
