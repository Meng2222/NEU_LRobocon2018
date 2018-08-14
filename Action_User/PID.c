#include "PID.h"
float OUT=0;
static float errAngle;
static float errLast;
static float errIntergral;
	float PID(float setAngle,float anglePos )
	{   
		float Kp=1,Ki=0,Kd=0;
		errAngle=anglePos-setAngle;
		errIntergral=errIntergral+errAngle;
		OUT=Kp*errAngle+Ki*errIntergral+Kd*(errAngle-errLast);
		errLast=errAngle;
	    return OUT;
	}