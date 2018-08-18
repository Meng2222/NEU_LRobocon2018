#ifndef POS_H
#define POS_H

#include "math.h"
#include "environmental.h"

//位置,成员为x,y,角度
typedef struct
{
    float posX;
    float posY;
    float angle;
}Pos;

//位置,用直角坐标和极坐标两种表示方法
typedef struct
{
    float x;
    float y;
    float a;
    float r;
}point;

//直线解析式,成员aX+bY+c=0的三个参数
typedef struct
{
    float a;
    float b;
    float c;
}line;

//直线方向枚举类型,可以为前或后
typedef enum
{
    forward,
    backward,
}dir;

//相对位置枚举类型,可以为左和右
typedef enum
{
    right,
    left,
    ontheline,
}reldir;

//获取当前位置
float GetX(void);
float GetY(void);
float GetA(void);

//计算车所在点距离特定直线的距离
float Point2Line(line * thisline);

/**计算直线方向问题
*  thisline 为要求的直线,Dir是前进方向取值可以为forWard和backward
*  返回值为-180到180间的一个数,与小车定位器的返回值所取坐标系相同(y轴正向为0度)
*/
float LineDir(const line * thisline ,dir Dir);

//设置一个点的位置,通过直角坐标或极坐标
void setPointXY(point * thispoint ,float x, float y);
void setPointRA(point * thispoint ,float r, float a);

//获取当前车的绝对位置;
void GetNowPoint(point * nowpoint);

//返回当前车与目标直线的相对位置,返回左右
reldir RelDir2Line(const line * thisline, dir thisdir);
#endif
