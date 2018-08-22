#include "trans.h"



struct trans{
float oldPosX;
float oldPosY;
float newPosX;
float newPosY;
float t_angle;
}transform;

void Transformation(void)
{
	transform.t_angle=GetAngle();
	transform.oldPosX=GetPosX();
	transform.oldPosY=GetPosY();
	
	transform.newPosX=transform.oldPosX+(OPS_TO_BACK_WHEEL*sin(transform.t_angle*PI/180));
	transform.newPosY=transform.oldPosY+OPS_TO_BACK_WHEEL-(OPS_TO_BACK_WHEEL*cos(transform.t_angle*PI/180));

}
