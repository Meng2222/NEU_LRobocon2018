#include "PID.h"


void Init_PID(PID_Value *pid_init)                                           //PID参数初始化（重开pid时需要用到）
{
	pid_init->Angle_Last = pid_init->Angle;
	pid_init->ITerm = pid_init->vel;
}

Line_Value *Line_N[12];
Arc_Value *Arc_N[12];
Coordinate_Value *Coordinate_N[12];
void PID_Init(PID_Value *PID_a)
{
	Line_N[0]->x1 = 0;
	Line_N[0]->y1 = 500;
	Line_N[0]->x2 = 1000;
	Line_N[0]->y2 = 1000;
	Line_N[0]->line_kp = 20;
	Line_N[0]->line_A = 0;
	Line_N[0]->line_B = 0;
	Line_N[0]->line_C = 0;
	Line_N[0]->line_Angle = 0;
	Line_N[0]->line_Error = 0;
	PID_a->l = Line_N[0];
	
	Arc_N[0]->x0 = 0;
	Arc_N[0]->y0 = 1000;
	Arc_N[0]->r0 = 500;
	Arc_N[0]->arc_Angle = 0;
	Arc_N[0]->arc_Error = 0;
	Arc_N[0]->arc_Direction = ACW;
	Arc_N[0]->arc_kp = 10;
	PID_a->r = Arc_N[0];
	
	Coordinate_N[0]->x3 = 1000;
	Coordinate_N[0]->y3 = 1000;
	Coordinate_N[0]->coordinate_Angle = 0;
	PID_a->c = Coordinate_N[0];
	
	PID_a->Angle_Last = 0;
	PID_a->Angle_Set = 0;
	PID_a->DTerm = 0;
	PID_a->ITerm = 0;
	PID_a->Error = 0;
	PID_a->kp = 20;
	PID_a->ki = 0;
	PID_a->kd = 100;
	PID_a->Mode = manual;
	PID_a->Mode_Last = manual;
	PID_a->V = 500;
	PID_a->vel = 0;	
}

void PID_Control(PID_Value *p)
{
	if(p->Mode == manual)
	{
		p->Mode_Last = manual;
		return;
	}
	if(p->Mode_Last == manual) Init_PID(p);	
	p->Angle = GetAngle();
	p->X = GetX();
	p->Y = GetY();
	p->X_Speed = GetSpeedX();
	p->Y_Speed = GetSpeedY();
	switch(p->Mode)
	{
		case Arc:
			p->r->arc_Error = (float)sqrt((double)(((p->X)-(p->r->x0))*((p->X)-(p->r->x0))-((p->Y)-(p->r->y0))*((p->Y)-(p->r->y0)))) - (p->r->r0);
			if((p->X) == (p->r->x0))  p->r->arc_Angle = Compare((p->Y),(p->r->y0))*90;
			else if((p->X)>(p->r->x0)) p->r->arc_Angle = ((float)atan((double)(((p->Y)-(p->r->y0))/((p->X)-(p->r->x0)))))*(180/Pi);
			else if((p->X)<(p->r->x0)) p->r->arc_Angle = 180+((float)atan((double)(((p->Y)-(p->r->y0))/((p->X)-(p->r->x0)))))*(180/Pi);
			if(p->r->arc_Direction == CW) (p->r->arc_Angle) = (p->r->arc_Angle)-180-(p->r->arc_Error)/(p->r->arc_kp);
			else if(p->r->arc_Direction == ACW) (p->r->arc_Angle) = (p->r->arc_Angle)+(p->r->arc_Error)/(p->r->arc_kp);
			if((p->r->arc_Angle)>180) (p->r->arc_Angle) -= 360;
			else if((p->r->arc_Angle)<-180) (p->r->arc_Angle) += 360;
			p->Angle_Set = p->r->arc_Angle;
			break;
		case Line:
			p->l->line_A = (p->l->y1)-(p->l->y2);
			p->l->line_B = (p->l->x2)-(p->l->x1);
			p->l->line_C = (p->l->x1)*(p->l->y2)-(p->l->x2)*(p->l->y1);
			p->l->line_Error = ((p->l->line_A)*(p->X)+(p->l->line_B)*(p->Y)+(p->l->line_C))/(sqrt((p->l->line_A)*(p->l->line_A)+(p->l->line_B)*(p->l->line_B)));
			if(p->l->line_B != 0) p->l->line_Angle = (((0-(p->l->line_B))/(ABS(p->l->line_B)))*90-(((p->l->line_A)*(p->l->line_B))/(ABS((p->l->line_A)*(p->l->line_B))))*(((float)(atan((double)(ABS((p->l->line_A)/(p->l->line_B))))))*(180/Pi)));
			else if(p->l->line_A < 0) p->l->line_Angle = 0;
			else if(p->l->line_A > 0) p->l->line_Angle = 180;
			p->l->line_Angle -= ((p->l->line_Error)/(p->l->line_kp));
			if((p->l->line_Angle)>180) (p->l->line_Angle) -= 360;
			else if((p->l->line_Angle)<-180) (p->l->line_Angle) += 360;
			p->Angle_Set = p->l->line_Angle;
			break;
		case Coordinate:
			if((p->X) != (p->c->x3)) p->c->coordinate_Angle = (Compare((p->X),(p->c->x3)))*90+((float)atan((double)(((p->Y)-(p->c->y3))/((p->X)-(p->c->x3)))))*(180/Pi);
			else if((p->Y) > (p->c->y3)) p->c->coordinate_Angle = 180;
			else if((p->Y) < (p->c->y3)) p->c->coordinate_Angle = 0;
			p->Angle_Set = p->c->coordinate_Angle;
			break;
		default:
			break;
	}
	p->Error = p->Angle_Set - p->Angle;
	if((p->Error) > 180) p->Error -= 180;
	else if((p->Error) < -180) p->Error += 180;
	if ((p->Error) > -3 && (p->Error) < 3) 
		{
			p->kd = 1000;
		}
	else 
		{
			p->kd = 50;
		}
	p->ITerm += p->ki * p->Error;
	p->ITerm = constrain(p->ITerm,2000.0f,-2000.0f);
	p->DTerm = p->Angle_Last - p->Angle;
	p->vel = p->kp * p->Error + p->ki * p->ITerm + p->kd * p->DTerm;
	p->vel = constrain(p->vel,2000.0f,-2000.0f);
	p->V = p->V/(5.0f*ABS(p->DTerm) + 1);
	p->Angle_Last = p->Angle;
	p->Mode_Last = p->Mode;
}

void GO(PID_Value *p_GO)
{
	VelCrl(CAN2,1,(int)(((4096/378)*p_GO->vel)+(4096/378)*p_GO->V));
	VelCrl(CAN2,2,(int)(((4096/378)*p_GO->vel)-(4096/378)*p_GO->V));
}



void PID_Control_Competition(PID_Value *pid)
{
	pid->Mode = Line;
	PID_Control(pid);
	GO(pid);
}
