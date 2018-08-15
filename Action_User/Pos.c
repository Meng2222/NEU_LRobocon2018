#include "Pos.h"

Pos Pos_t;

#ifdef CAR1
float GetX(void)
{
    return Pos_t.posY;
}

float GetY(void)
{
    return -1 * Pos_t.posX;
}
float GetA(void)
{
    if(Pos_t.angle >= -90)
    {
        return Pos_t.angle - 90;
    }
    else
    {
        return Pos_t.angle + 270;
    }
}
#endif
#ifdef CAR4
float GetX(void)
{
    return Pos_t.posX;
}

float GetY(void)
{
    return Pos_t.posY;
}
float GetA(void)
{
    return Pos_t.angle;
}
#endif

float Point2Line(line * thisline)
{
    float x = 0, y = 0;
    x = GetX();
    y = GetY();
    return (__fabs((*thisline).a * x + (*thisline).b * y + (*thisline).c) / pow(((*thisline).a * (*thisline).a + (*thisline).b * (*thisline).b), 0.5));
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
        a = (atan((*thispoint).x / (*thispoint).y) * (180 / 3.14f));
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

void setPointXY(point * thispoint ,float x, float y)
{
    (*thispoint).x = x;
    (*thispoint).y = y;
    xy2ra(thispoint);
}

void setPointRA(point * thispoint ,float r, float a)
{
    (*thispoint).r = r;
    (*thispoint).a = a;
    ra2xy(thispoint);
}

void GetNowPoint(point * nowpoint)
{
    setPointXY(nowpoint, GetX(), GetY());
}

float LineDir(line * thisline ,dir thisdir)
{
    float tempangle = 0;
    if((*thisline).a == 0)
    {
        if(thisdir == forward)
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
        tempangle = (atan((*thisline).b / (*thisline).a) * (180 / 3.14f));
        if(thisdir == forward)
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

reldir RelDir2Line(line * thisline, dir thisdir)
{
    float angleOfLine = 0;
    float ExY = 0;
    angleOfLine = LineDir(thisline, thisdir);
    if(angleOfLine == 0)
    {
        if(GetX() > -1 * ((*thisline).c / (*thisline).a))
        {
            return right;
        }
        if(GetX() < -1 * ((*thisline).c / (*thisline).a))
        {
            return left;
        }
        if(GetX() == -1 * ((*thisline).c / (*thisline).a))
        {
            return ontheline;
        }
    }
    else if(angleOfLine == 180 || angleOfLine == -180)
    {
        if(GetX() > -1 * ((*thisline).c / (*thisline).a))
        {
            return left;
        }
        if(GetX() < -1 * ((*thisline).c / (*thisline).a))
        {
            return right;
        }
        if(GetX() == -1 * ((*thisline).c / (*thisline).a))
        {
            return ontheline;
        }
    }
    else
    {
        ExY = (-1 * (((*thisline).a * GetX()) + (*thisline).c) / (*thisline).b);
        if(angleOfLine > 0)
        {
            if(GetY() > ExY)
            {
                return right;
            }
            if(GetY() < ExY)
            {
                return left;
            }
            if(GetY() == ExY)
            {
                return ontheline;
            }
        }
        if(angleOfLine < 0)
        {
            if(GetY() > ExY)
            {
                return left;
            }
            if(GetY() < ExY)
            {
                return right;
            }
            if(GetY() == ExY)
            {
                return ontheline;
            }
        }
    }
    return ontheline;
}
