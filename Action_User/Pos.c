#include "Pos.h"

#define LENGTH_OF_LOCATOR_2_BACK 65

Pos PosGet_t;
speed speed_t;

float GetX(void)
{
    return PosGet_t.posX;
}

float GetY(void)
{
    return PosGet_t.posY + LENGTH_OF_LOCATOR_2_BACK;
}

float GetA(void)
{
    return PosGet_t.angle;
}

float GetSpeedX(void)
{
    return speed_t.speedX;
}

float GetSpeedY(void)
{
    return speed_t.speedY;
}

float GetSpeedA(void)
{
    return speed_t.angle;
}

float Point2Line(linewithdir thisline, point thispoint)
{
    float x = 0, y = 0;
    x = thispoint.x;
    y = thispoint.y;
    return (__fabs(thisline.a * x + thisline.b * y + thisline.c) / pow((thisline.a * thisline.a + thisline.b * thisline.b), 0.5));
}

void xy2ra(point * thispoint)
{
    float a = 0;
    (*thispoint).r = pow(((*thispoint).x * (*thispoint).x + (*thispoint).y * (*thispoint).y), 0.5);
    if((*thispoint).y == 0)
    {
        if((*thispoint).x > 0)
        {
            (*thispoint).a = -90;
        }
        else if((*thispoint).x < 0)
        {
            (*thispoint).a = 90;
        }
        else
        {
            (*thispoint).a = 0;
        }
    }
    else
    {
        a = -1 * (atan((*thispoint).x / (*thispoint).y) * (180 / 3.14f));
        if((*thispoint).y > 0)
        {
            (*thispoint).a = a;
        }
        else
        {
            a += 180;
            if(a >= 180)
            {
                (*thispoint).a = a - 360;
            }
            else
            {
                (*thispoint).a = a;
            }
        }
    }
}

void ra2xy(point * thispoint)
{
    (*thispoint).x = -1 * (*thispoint).r * sin((*thispoint).a);
    (*thispoint).y = (*thispoint).r * cos((*thispoint).a);
}

point setPointXY(float x, float y)
{
    point thispoint;
    thispoint.x = x;
    thispoint.y = y;
    xy2ra(&thispoint);
    return thispoint;
}

point setPointRA(float r, float a)
{
    point thispoint;
    thispoint.r = r;
    thispoint.a = a;
    ra2xy(&thispoint);
    return thispoint;
}

point GetNowPoint(void)
{
    point nowpoint;
    nowpoint = setPointXY(GetX(), GetY());
    return nowpoint;
}

float LineDir(const linewithdir thisline)
{
    float tempangle = 0;
    if(thisline.a == 0)
    {
        if(thisline.b == 0)
        {
            return GetA();
        }
        else if(thisline.linedir == forward)
        {
            return 90;
        }
        else
        {
            return -90;
        }
    }
    else
    {
        tempangle = (atan(thisline.b / thisline.a) * (180 / 3.14f));
        if(thisline.linedir == forward)
        {
            return tempangle;
        }
        else
        {
            tempangle += 180;
            if(tempangle >= 180)
            {
                return tempangle - 360;
            }
            else
            {
                return tempangle;
            }
        }
    }
}

reldir RelDir2Line(const linewithdir thisline, const point thispoint)
{
    float angleOfLine = 0;
    float ExY = 0;
    angleOfLine = LineDir(thisline);
    if(angleOfLine == 0)
    {
        if(thispoint.x > -1 * (thisline.c / thisline.a))
        {
            return right;
        }
        if(thispoint.x < -1 * (thisline.c / thisline.a))
        {
            return left;
        }
        if(thispoint.x == -1 * (thisline.c / thisline.a))
        {
            return ontheline;
        }
    }
    else if(angleOfLine == 180 || angleOfLine == -180)
    {
        if(thispoint.x > -1 * (thisline.c / thisline.a))
        {
            return left;
        }
        if(thispoint.x < -1 * (thisline.c / thisline.a))
        {
            return right;
        }
        if(thispoint.x == -1 * (thisline.c / thisline.a))
        {
            return ontheline;
        }
    }
    else
    {
        ExY = (-1 * ((thisline.a * thispoint.x) + thisline.c) / thisline.b);
        if(angleOfLine > 0)
        {
            if(thispoint.y > ExY)
            {
                return right;
            }
            if(thispoint.y < ExY)
            {
                return left;
            }
            if(thispoint.y == ExY)
            {
                return ontheline;
            }
        }
        if(angleOfLine < 0)
        {
            if(thispoint.y > ExY)
            {
                return left;
            }
            if(thispoint.y < ExY)
            {
                return right;
            }
            if(thispoint.y == ExY)
            {
                return ontheline;
            }
        }
    }
    return ontheline;
}

//返回点之间的距离
float Point2Point(const point thispoint, const point thatpoint)
{
    float x = 0, y = 0;
    x = thispoint.x - thatpoint.x;
    y = thispoint.y - thatpoint.y;
    return pow((x * x + y * y), 0.5);
}

//返回thispoint指向thatpoint的带方向的直线
linewithdir DirlinePoint2Point(const point thispoint, const point thatpoint)
{
    linewithdir tempdirline;
    tempdirline.a = (thispoint.y - thatpoint.y);
    tempdirline.b = (thatpoint.x - thispoint.x);
    tempdirline.c = (thispoint.x * thatpoint.y - thatpoint.x * thispoint.y);
    if(thispoint.y == thatpoint.y)
    {
        if(thispoint.x > thatpoint.x)
        {
            tempdirline.linedir = forward;
        }
        else
        {
            tempdirline.linedir = backward;
        }
    }
    else
    {
        if(thispoint.y < thatpoint.y)
        {
            tempdirline.linedir = forward;
        }
        else
        {
            tempdirline.linedir = backward;
        }
    }
    return tempdirline;
}

//返回thatpoint相对于thispoint的相对位置
point RelPos(const point thispoint, const point thatpoint)
{
    point temppoint;
    temppoint = setPointXY(thatpoint.x - thispoint.x, thatpoint.y - thispoint.y);
    return temppoint;
}

//返回相对于thispoint位置为relpoint的点的绝对位置
point StrPos(const point thispoint, const point relpoint)
{
    point thatpoint;
    thatpoint = setPointXY(thispoint.x + relpoint.x, thispoint.y + relpoint.y);
    return thatpoint;
}

//返回有向直线thisline的thisdir侧的垂直于其的方向
float VDirForLine(const linewithdir thisline, reldir thisdir)
{
    float tempangle = 0;
    tempangle = LineDir(thisline);
    if(thisdir == right)
    {
        tempangle -= 90;
    }
    else if (thisdir == left)
    {
        tempangle += 90;
    }
    else
    {
        return 0;
    }
    if(tempangle >= 180)
    {
        tempangle -= 360;
    }
    if(tempangle < -180)
    {
        tempangle += 360;
    }
    return tempangle;
}
