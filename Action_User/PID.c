#include "PID.h"


void Init_PID(PID_Value *pid_init)                                           //PID参数初始化（重开pid时需要用到）
{
	pid_init->Angle_Last = pid_init->Angle;
	pid_init->ITerm = pid_init->vel;
}

Line_Value Line_N[26];
Arc_Value Arc_N[12];
Coordinate_Value Coordinate_N[12];
void PID_Line_Init(void)
{
	Line_N[0].x1 = 700;
	Line_N[0].y1 = 1500;
	Line_N[0].x2 = 700;
	Line_N[0].y2 = 2900;
	Line_N[0].line_kp = 20;
	Line_N[0].line_A = 0;
	Line_N[0].line_B = 0;
	Line_N[0].line_C = 0;
	Line_N[0].line_Angle = 0;
	Line_N[0].line_Error = 0;
	
	Line_N[1].x1 = 700;
	Line_N[1].y1 = 2900;
	Line_N[1].x2 = -700;
	Line_N[1].y2 = 2900;
	Line_N[1].line_kp = 20;
	Line_N[1].line_A = 0;
	Line_N[1].line_B = 0;
	Line_N[1].line_C = 0;
	Line_N[1].line_Angle = 0;
	Line_N[1].line_Error = 0;
	
	Line_N[2].x1 = -700;
	Line_N[2].y1 = 2900;
	Line_N[2].x2 = -700;
	Line_N[2].y2 = 1500;
	Line_N[2].line_kp = 20;
	Line_N[2].line_A = 0;
	Line_N[2].line_B = 0;
	Line_N[2].line_C = 0;
	Line_N[2].line_Angle = 0;
	Line_N[2].line_Error = 0;
	
	Line_N[3].x1 = -700;
	Line_N[3].y1 = 1500;
	Line_N[3].x2 = 700;
	Line_N[3].y2 = 1500;
	Line_N[3].line_kp = 20;
	Line_N[3].line_A = 0;
	Line_N[3].line_B = 0;
	Line_N[3].line_C = 0;
	Line_N[3].line_Angle = 0;
	Line_N[3].line_Error = 0;
	
	Line_N[4].x1 = 1200;
	Line_N[4].y1 = 1000;
	Line_N[4].x2 = 1200;
	Line_N[4].y2 = 3400;
	Line_N[4].line_kp = 20;
	Line_N[4].line_A = 0;
	Line_N[4].line_B = 0;
	Line_N[4].line_C = 0;
	Line_N[4].line_Angle = 0;
	Line_N[4].line_Error = 0;
	
	Line_N[5].x1 = 1200;
	Line_N[5].y1 = 3400;
	Line_N[5].x2 = -1200;
	Line_N[5].y2 = 3400;
	Line_N[5].line_kp = 20;
	Line_N[5].line_A = 0;
	Line_N[5].line_B = 0;
	Line_N[5].line_C = 0;
	Line_N[5].line_Angle = 0;
	Line_N[5].line_Error = 0;
	
	Line_N[6].x1 = -1200;
	Line_N[6].y1 = 3400;
	Line_N[6].x2 = -1200;
	Line_N[6].y2 = 1000;
	Line_N[6].line_kp = 20;
	Line_N[6].line_A = 0;
	Line_N[6].line_B = 0;
	Line_N[6].line_C = 0;
	Line_N[6].line_Angle = 0;
	Line_N[6].line_Error = 0;
	
	Line_N[7].x1 = -1200;
	Line_N[7].y1 = 1000;
	Line_N[7].x2 = 1200;
	Line_N[7].y2 = 1000;
	Line_N[7].line_kp = 20;
	Line_N[7].line_A = 0;
	Line_N[7].line_B = 0;
	Line_N[7].line_C = 0;
	Line_N[7].line_Angle = 0;
	Line_N[7].line_Error = 0;
	
	Line_N[8].x1 = 1700;
	Line_N[8].y1 = 500;
	Line_N[8].x2 = 1700;
	Line_N[8].y2 = 3900;
	Line_N[8].line_kp = 20;
	Line_N[8].line_A = 0;
	Line_N[8].line_B = 0;
	Line_N[8].line_C = 0;
	Line_N[8].line_Angle = 0;
	Line_N[8].line_Error = 0;
	
	Line_N[9].x1 = 1700;
	Line_N[9].y1 = 3900;
	Line_N[9].x2 = -1700;
	Line_N[9].y2 = 3900;
	Line_N[9].line_kp = 20;
	Line_N[9].line_A = 0;
	Line_N[9].line_B = 0;
	Line_N[9].line_C = 0;
	Line_N[9].line_Angle = 0;
	Line_N[9].line_Error = 0;
	
	Line_N[10].x1 = -1700;
	Line_N[10].y1 = 3900;
	Line_N[10].x2 = -1700;
	Line_N[10].y2 = 500;
	Line_N[10].line_kp = 20;
	Line_N[10].line_A = 0;
	Line_N[10].line_B = 0;
	Line_N[10].line_C = 0;
	Line_N[10].line_Angle = 0;
	Line_N[10].line_Error = 0;
	
	Line_N[11].x1 = -1700;
	Line_N[11].y1 = 500;
	Line_N[11].x2 = 1700;
	Line_N[11].y2 = 500;
	Line_N[11].line_kp = 20;
	Line_N[11].line_A = 0;
	Line_N[11].line_B = 0;
	Line_N[11].line_C = 0;
	Line_N[11].line_Angle = 0;
	Line_N[11].line_Error = 0;
	
	Line_N[12].x1 = 1700;
	Line_N[12].y1 = 500;
	Line_N[12].x2 = 1700;
	Line_N[12].y2 = 3900;
	Line_N[12].line_kp = 20;
	Line_N[12].line_A = 0;
	Line_N[12].line_B = 0;
	Line_N[12].line_C = 0;
	Line_N[12].line_Angle = 0;
	Line_N[12].line_Error = 0;
	
	
	
	
	Line_N[13].x1 = -700;
	Line_N[13].y1 = 1500;
	Line_N[13].x2 = -700;
	Line_N[13].y2 = 2900;
	Line_N[13].line_kp = 20;
	Line_N[13].line_A = 0;
	Line_N[13].line_B = 0;
	Line_N[13].line_C = 0;
	Line_N[13].line_Angle = 0;
	Line_N[13].line_Error = 0;
	
	Line_N[14].x1 = -700;
	Line_N[14].y1 = 2900;
	Line_N[14].x2 = 700;
	Line_N[14].y2 = 2900;
	Line_N[14].line_kp = 20;
	Line_N[14].line_A = 0;
	Line_N[14].line_B = 0;
	Line_N[14].line_C = 0;
	Line_N[14].line_Angle = 0;
	Line_N[14].line_Error = 0;
	
	Line_N[15].x1 = 700;
	Line_N[15].y1 = 2900;
	Line_N[15].x2 = 700;
	Line_N[15].y2 = 1500;
	Line_N[15].line_kp = 20;
	Line_N[15].line_A = 0;
	Line_N[15].line_B = 0;
	Line_N[15].line_C = 0;
	Line_N[15].line_Angle = 0;
	Line_N[15].line_Error = 0;
	
	Line_N[16].x1 = 700;
	Line_N[16].y1 = 1500;
	Line_N[16].x2 = -700;
	Line_N[16].y2 = 1500;
	Line_N[16].line_kp = 20;
	Line_N[16].line_A = 0;
	Line_N[16].line_B = 0;
	Line_N[16].line_C = 0;
	Line_N[16].line_Angle = 0;
	Line_N[16].line_Error = 0;
	
	Line_N[17].x1 = -1200;
	Line_N[17].y1 = 1000;
	Line_N[17].x2 = -1200;
	Line_N[17].y2 = 3400;
	Line_N[17].line_kp = 20;
	Line_N[17].line_A = 0;
	Line_N[17].line_B = 0;
	Line_N[17].line_C = 0;
	Line_N[17].line_Angle = 0;
	Line_N[17].line_Error = 0;
	
	Line_N[18].x1 = -1200;
	Line_N[18].y1 = 3400;
	Line_N[18].x2 = 1200;
	Line_N[18].y2 = 3400;
	Line_N[18].line_kp = 20;
	Line_N[18].line_A = 0;
	Line_N[18].line_B = 0;
	Line_N[18].line_C = 0;
	Line_N[18].line_Angle = 0;
	Line_N[18].line_Error = 0;
	
	Line_N[19].x1 = 1200;
	Line_N[19].y1 = 3400;
	Line_N[19].x2 = 1200;
	Line_N[19].y2 = 1000;
	Line_N[19].line_kp = 20;
	Line_N[19].line_A = 0;
	Line_N[19].line_B = 0;
	Line_N[19].line_C = 0;
	Line_N[19].line_Angle = 0;
	Line_N[19].line_Error = 0;
	
	Line_N[20].x1 = 1200;
	Line_N[20].y1 = 1000;
	Line_N[20].x2 = -1200;
	Line_N[20].y2 = 1000;
	Line_N[20].line_kp = 20;
	Line_N[20].line_A = 0;
	Line_N[20].line_B = 0;
	Line_N[20].line_C = 0;
	Line_N[20].line_Angle = 0;
	Line_N[20].line_Error = 0;
	
	Line_N[21].x1 = -1700;
	Line_N[21].y1 = 500;
	Line_N[21].x2 = -1700;
	Line_N[21].y2 = 3900;
	Line_N[21].line_kp = 20;
	Line_N[21].line_A = 0;
	Line_N[21].line_B = 0;
	Line_N[21].line_C = 0;
	Line_N[21].line_Angle = 0;
	Line_N[21].line_Error = 0;
	
	Line_N[22].x1 = -1700;
	Line_N[22].y1 = 3900;
	Line_N[22].x2 = 1700;
	Line_N[22].y2 = 3900;
	Line_N[22].line_kp = 20;
	Line_N[22].line_A = 0;
	Line_N[22].line_B = 0;
	Line_N[22].line_C = 0;
	Line_N[22].line_Angle = 0;
	Line_N[22].line_Error = 0;
	
	Line_N[23].x1 = 1700;
	Line_N[23].y1 = 3900;
	Line_N[23].x2 = 1700;
	Line_N[23].y2 = 500;
	Line_N[23].line_kp = 20;
	Line_N[23].line_A = 0;
	Line_N[23].line_B = 0;
	Line_N[23].line_C = 0;
	Line_N[23].line_Angle = 0;
	Line_N[23].line_Error = 0;
	
	Line_N[24].x1 = 1700;
	Line_N[24].y1 = 500;
	Line_N[24].x2 = -1700;
	Line_N[24].y2 = 500;
	Line_N[24].line_kp = 20;
	Line_N[24].line_A = 0;
	Line_N[24].line_B = 0;
	Line_N[24].line_C = 0;
	Line_N[24].line_Angle = 0;
	Line_N[24].line_Error = 0;
	
	Line_N[25].x1 = -1700;
	Line_N[25].y1 = 500;
	Line_N[25].x2 = -1700;
	Line_N[25].y2 = 3900;
	Line_N[25].line_kp = 20;
	Line_N[25].line_A = 0;
	Line_N[25].line_B = 0;
	Line_N[25].line_C = 0;
	Line_N[25].line_Angle = 0;
	Line_N[25].line_Error = 0;
}

void PID_Arc_Init(void)
{
	Arc_N[0].x0 = 0;
	Arc_N[0].y0 = 1000;
	Arc_N[0].r0 = 500;
	Arc_N[0].arc_Angle = 0;
	Arc_N[0].arc_Error = 0;
	Arc_N[0].arc_Direction = ACW;
	Arc_N[0].arc_kp = 10;
}

void PID_Coordinate_Init(void)
{
	Coordinate_N[0].x3 = 1000;
	Coordinate_N[0].y3 = 1000;
	Coordinate_N[0].coordinate_Angle = 0;
}
void PID_Init(PID_Value *PID_a)
{
	PID_a->Angle_Last = 0;
	PID_a->Angle_Set = 0;
	PID_a->DTerm = 0;
	PID_a->ITerm = 0;
	PID_a->Error = 0;
	PID_a->kp = 20;
	PID_a->ki = 0;
	PID_a->kd = 100;
	PID_a->Mode = Line;
	PID_a->Mode_Last = Line;
	PID_a->V = 2000;
	PID_a->Coordinate_Num = 0;
	PID_a->Line_Num = 0;
	PID_a->Arc_Num = 0;
	PID_a->V_Set = 2000;
	PID_a->vel = 0;
	
	PID_Line_Init();
	PID_a->l = &Line_N[PID_a->Line_Num];
	
	PID_Arc_Init();
	PID_a->r = &Arc_N[PID_a->Arc_Num];
	
	PID_Coordinate_Init();
	PID_a->c = &Coordinate_N[PID_a->Coordinate_Num];
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
	p->l = &Line_N[p->Line_Num];
	p->r = &Arc_N[p->Arc_Num];
	p->c = &Coordinate_N[p->Coordinate_Num];
	switch(p->Mode)
	{
		case Arc:
			p->r->arc_Error = (float)sqrt((double)(((p->X)-(p->r->x0))*((p->X)-(p->r->x0))-((p->Y)-(p->r->y0))*((p->Y)-(p->r->y0)))) - (p->r->r0);
			p->r->arc_Error = constrain(p->r->arc_Error,1800,-1800);
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
			if((p->l->line_B != 0) && (p->l->line_A != 0)) p->l->line_Angle = (((0-(p->l->line_B))/(ABS(p->l->line_B)))*90-(((p->l->line_A)*(p->l->line_B))/(ABS((p->l->line_A)*(p->l->line_B))))*(((float)(atan((double)(ABS((p->l->line_A)/(p->l->line_B))))))*(180/Pi)));
			else if((p->l->line_A == 0) && (p->l->line_B > 0)) p->l->line_Angle = -90;
			else if((p->l->line_A == 0) && (p->l->line_B < 0)) p->l->line_Angle = 90;
			else if((p->l->line_A < 0) && (p->l->line_B == 0)) p->l->line_Angle = 0;
			else if((p->l->line_A > 0) && (p->l->line_B == 0)) p->l->line_Angle = 180;
			p->l->line_Error = constrain(p->l->line_Error,1800,-1800);
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
	if((p->Error) > 180) p->Error -= 360;
	else if((p->Error) < -180) p->Error += 360;
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
	p->V = p->V_Set/((2.0f*ABS(p->DTerm)) + 1);
	p->Angle_Last = p->Angle;
	p->Mode_Last = p->Mode;
}

void PID_Pre(PID_Value *p)
{
	p->l = &Line_N[p->Line_Num];
	p->r = &Arc_N[p->Arc_Num];
	p->c = &Coordinate_N[p->Coordinate_Num];
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
			break;
		case Line:
			p->l->line_A = (p->l->y1)-(p->l->y2);
			p->l->line_B = (p->l->x2)-(p->l->x1);
			p->l->line_C = (p->l->x1)*(p->l->y2)-(p->l->x2)*(p->l->y1);
			p->l->line_Error = ((p->l->line_A)*(p->X)+(p->l->line_B)*(p->Y)+(p->l->line_C))/(sqrt((p->l->line_A)*(p->l->line_A)+(p->l->line_B)*(p->l->line_B)));
			if((p->l->line_B != 0) && (p->l->line_A != 0)) p->l->line_Angle = (((0-(p->l->line_B))/(ABS(p->l->line_B)))*90-(((p->l->line_A)*(p->l->line_B))/(ABS((p->l->line_A)*(p->l->line_B))))*(((float)(atan((double)(ABS((p->l->line_A)/(p->l->line_B))))))*(180/Pi)));
			else if((p->l->line_A == 0) && (p->l->line_B > 0)) p->l->line_Angle = -90;
			else if((p->l->line_A == 0) && (p->l->line_B < 0)) p->l->line_Angle = 90;
			else if((p->l->line_A < 0) && (p->l->line_B == 0)) p->l->line_Angle = 0;
			else if((p->l->line_A > 0) && (p->l->line_B == 0)) p->l->line_Angle = 180;
			p->l->line_Angle -= ((p->l->line_Error)/(p->l->line_kp));
			if((p->l->line_Angle)>180) (p->l->line_Angle) -= 360;
			else if((p->l->line_Angle)<-180) (p->l->line_Angle) += 360;
			break;
		case Coordinate:
			if((p->X) != (p->c->x3)) p->c->coordinate_Angle = (Compare((p->X),(p->c->x3)))*90+((float)atan((double)(((p->Y)-(p->c->y3))/((p->X)-(p->c->x3)))))*(180/Pi);
			else if((p->Y) > (p->c->y3)) p->c->coordinate_Angle = 180;
			else if((p->Y) < (p->c->y3)) p->c->coordinate_Angle = 0;
			break;
		default:
			break;
	}
}

void GO(PID_Value *p_GO)
{
	VelCrl(CAN2,1,(int)(((4096/378)*(p_GO->vel))+(4096/378)*(p_GO->V)));
	VelCrl(CAN2,2,(int)(((4096/378)*(p_GO->vel))-(4096/378)*(p_GO->V)));
}

void PID_Control_Competition(PID_Value *pid,u8 dir)
{
	if(dir == Right)
	{
		pid->Mode = Line;
		PID_Control(pid);
		GO(pid);
		pid->Line_Num += 1;
		if((pid->Line_Num) == 13) pid->Line_Num = 9;
		PID_Pre(pid);
		if((pid->l->line_Error>500) || (pid->l->line_Error<-500)) pid->Line_Num -= 1;
	}
	else if(dir == Left)
	{
		pid->Mode = Line;
		pid->Line_Num = 13;
		PID_Control(pid);
		GO(pid);
		pid->Line_Num += 1;
		if((pid->Line_Num) == 26) pid->Line_Num = 22;
		PID_Pre(pid);
		if((pid->l->line_Error>500) || (pid->l->line_Error<-500)) pid->Line_Num -= 1;
	}	
}
