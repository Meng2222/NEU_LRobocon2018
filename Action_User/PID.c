#include "PID.h"
Line_Value Line_N[34];
Arc_Value Arc_N[12];
Coordinate_Value Coordinate_N[12];
int line_order1[24] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,8,9,10,11,4,5,6,7};
int line_order2[24] = {17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,25,26,27,28,21,22,23,24};
int line_order3[36] = {0,5,14,15,12,13,10,11,8,9,6,7,12,13,14,15,12,13,14,15,12,13,14,15,12,13,14,15,12,13,14,15,12,13,14,15};
int line_order4[36] = {17,22,31,32,29,30,27,28,25,26,23,24,29,30,31,32,29,30,31,32,29,30,31,32,29,30,31,32,29,30,31,32,29,30,31,32};
float ABS(float thing)                                                       //浮点绝对值函数
{
	if(thing > 0) return thing;
	else return (0-thing);
}

float Compare(float a1,float b1)                                             //比较函数，输出1或-1
{
	if(a1>b1) return 1.0f;
	else return -1.0f;
}

float constrain(float amt, float high, float low)                            //浮点数限幅函数
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

void Init_PID(PID_Value *pid_init)                                           //PID参数初始化（重开pid时需要用到）
{
	pid_init->Angle_Last = pid_init->Angle;
	pid_init->ITerm = pid_init->vel;
}

void PID_Line_Init(void)                                                     //直线参数初始化
{
	Line_N[0].x1 = 600;
	Line_N[0].y1 = 1800;
	Line_N[0].x2 = 600;
	Line_N[0].y2 = 3000;
	Line_N[0].line_kp = 20;
	Line_N[0].line_A = 0;
	Line_N[0].line_B = 0;
	Line_N[0].line_C = 0;
	Line_N[0].line_Angle = 0;
	Line_N[0].line_Error = 0;
	
	Line_N[1].x1 = 600;
	Line_N[1].y1 = 3000;
	Line_N[1].x2 = -600;
	Line_N[1].y2 = 3000;
	Line_N[1].line_kp = 20;
	Line_N[1].line_A = 0;
	Line_N[1].line_B = 0;
	Line_N[1].line_C = 0;
	Line_N[1].line_Angle = 0;
	Line_N[1].line_Error = 0;
	
	Line_N[2].x1 = -600;
	Line_N[2].y1 = 3000;
	Line_N[2].x2 = -600;
	Line_N[2].y2 = 1800;
	Line_N[2].line_kp = 20;
	Line_N[2].line_A = 0;
	Line_N[2].line_B = 0;
	Line_N[2].line_C = 0;
	Line_N[2].line_Angle = 0;
	Line_N[2].line_Error = 0;
	
	Line_N[3].x1 = -600;
	Line_N[3].y1 = 1800;
	Line_N[3].x2 = 600;
	Line_N[3].y2 = 1800;
	Line_N[3].line_kp = 20;
	Line_N[3].line_A = 0;
	Line_N[3].line_B = 0;
	Line_N[3].line_C = 0;
	Line_N[3].line_Angle = 0;
	Line_N[3].line_Error = 0;
	
	Line_N[4].x1 = 1000;
	Line_N[4].y1 = 1400;
	Line_N[4].x2 = 1000;
	Line_N[4].y2 = 3400;
	Line_N[4].line_kp = 20;
	Line_N[4].line_A = 0;
	Line_N[4].line_B = 0;
	Line_N[4].line_C = 0;
	Line_N[4].line_Angle = 0;
	Line_N[4].line_Error = 0;
	
	Line_N[5].x1 = 1000;
	Line_N[5].y1 = 3400;
	Line_N[5].x2 = -1000;
	Line_N[5].y2 = 3400;
	Line_N[5].line_kp = 20;
	Line_N[5].line_A = 0;
	Line_N[5].line_B = 0;
	Line_N[5].line_C = 0;
	Line_N[5].line_Angle = 0;
	Line_N[5].line_Error = 0;
	
	Line_N[6].x1 = -1000;
	Line_N[6].y1 = 3400;
	Line_N[6].x2 = -1000;
	Line_N[6].y2 = 1400;
	Line_N[6].line_kp = 20;
	Line_N[6].line_A = 0;
	Line_N[6].line_B = 0;
	Line_N[6].line_C = 0;
	Line_N[6].line_Angle = 0;
	Line_N[6].line_Error = 0;
	
	Line_N[7].x1 = -1000;
	Line_N[7].y1 = 1400;
	Line_N[7].x2 = 1000;
	Line_N[7].y2 = 1400;
	Line_N[7].line_kp = 20;
	Line_N[7].line_A = 0;
	Line_N[7].line_B = 0;
	Line_N[7].line_C = 0;
	Line_N[7].line_Angle = 0;
	Line_N[7].line_Error = 0;
	
	Line_N[8].x1 = 1400;
	Line_N[8].y1 = 1000;
	Line_N[8].x2 = 1400;
	Line_N[8].y2 = 3800;
	Line_N[8].line_kp = 20;
	Line_N[8].line_A = 0;
	Line_N[8].line_B = 0;
	Line_N[8].line_C = 0;
	Line_N[8].line_Angle = 0;
	Line_N[8].line_Error = 0;
	
	Line_N[9].x1 = 1400;
	Line_N[9].y1 = 3800;
	Line_N[9].x2 = -1400;
	Line_N[9].y2 = 3800;
	Line_N[9].line_kp = 20;
	Line_N[9].line_A = 0;
	Line_N[9].line_B = 0;
	Line_N[9].line_C = 0;
	Line_N[9].line_Angle = 0;
	Line_N[9].line_Error = 0;
	
	Line_N[10].x1 = -1400;
	Line_N[10].y1 = 3800;
	Line_N[10].x2 = -1400;
	Line_N[10].y2 = 1000;
	Line_N[10].line_kp = 20;
	Line_N[10].line_A = 0;
	Line_N[10].line_B = 0;
	Line_N[10].line_C = 0;
	Line_N[10].line_Angle = 0;
	Line_N[10].line_Error = 0;
	
	Line_N[11].x1 = -1400;
	Line_N[11].y1 = 1000;
	Line_N[11].x2 = 1400;
	Line_N[11].y2 = 1000;
	Line_N[11].line_kp = 20;
	Line_N[11].line_A = 0;
	Line_N[11].line_B = 0;
	Line_N[11].line_C = 0;
	Line_N[11].line_Angle = 0;
	Line_N[11].line_Error = 0;
	
	Line_N[12].x1 = 1800;
	Line_N[12].y1 = 600;
	Line_N[12].x2 = 1800;
	Line_N[12].y2 = 4200;
	Line_N[12].line_kp = 20;
	Line_N[12].line_A = 0;
	Line_N[12].line_B = 0;
	Line_N[12].line_C = 0;
	Line_N[12].line_Angle = 0;
	Line_N[12].line_Error = 0;
		
	Line_N[13].x1 = 1800;
	Line_N[13].y1 = 4200;
	Line_N[13].x2 = -1800;
	Line_N[13].y2 = 4200;
	Line_N[13].line_kp = 20;
	Line_N[13].line_A = 0;
	Line_N[13].line_B = 0;
	Line_N[13].line_C = 0;
	Line_N[13].line_Angle = 0;
	Line_N[13].line_Error = 0;
	
	Line_N[14].x1 = -1800;
	Line_N[14].y1 = 4200;
	Line_N[14].x2 = -1800;
	Line_N[14].y2 = 600;
	Line_N[14].line_kp = 20;
	Line_N[14].line_A = 0;
	Line_N[14].line_B = 0;
	Line_N[14].line_C = 0;
	Line_N[14].line_Angle = 0;
	Line_N[14].line_Error = 0;
	
	Line_N[15].x1 = -1800;
	Line_N[15].y1 = 600;
	Line_N[15].x2 = 1800;
	Line_N[15].y2 = 600;
	Line_N[15].line_kp = 20;
	Line_N[15].line_A = 0;
	Line_N[15].line_B = 0;
	Line_N[15].line_C = 0;
	Line_N[15].line_Angle = 0;
	Line_N[15].line_Error = 0;
	
	Line_N[16].x1 = 1800;
	Line_N[16].y1 = 600;
	Line_N[16].x2 = 1800;
	Line_N[16].y2 = 4200;
	Line_N[16].line_kp = 20;
	Line_N[16].line_A = 0;
	Line_N[16].line_B = 0;
	Line_N[16].line_C = 0;
	Line_N[16].line_Angle = 0;
	Line_N[16].line_Error = 0;
	
	
	
	
	
	Line_N[19].x1 = -600;
	Line_N[19].y1 = 1800;
	Line_N[19].x2 = -600;
	Line_N[19].y2 = 3000;
	Line_N[19].line_kp = 20;
	Line_N[19].line_A = 0;
	Line_N[19].line_B = 0;
	Line_N[19].line_C = 0;
	Line_N[19].line_Angle = 0;
	Line_N[19].line_Error = 0;
	
	Line_N[18].x1 = -600;
	Line_N[18].y1 = 3000;
	Line_N[18].x2 = 600;
	Line_N[18].y2 = 3000;
	Line_N[18].line_kp = 20;
	Line_N[18].line_A = 0;
	Line_N[18].line_B = 0;
	Line_N[18].line_C = 0;
	Line_N[18].line_Angle = 0;
	Line_N[18].line_Error = 0;
	
	Line_N[17].x1 = 600;
	Line_N[17].y1 = 3000;
	Line_N[17].x2 = 600;
	Line_N[17].y2 = 1800;
	Line_N[17].line_kp = 20;
	Line_N[17].line_A = 0;
	Line_N[17].line_B = 0;
	Line_N[17].line_C = 0;
	Line_N[17].line_Angle = 0;
	Line_N[17].line_Error = 0;
	
	Line_N[20].x1 = 600;
	Line_N[20].y1 = 1800;
	Line_N[20].x2 = -600;
	Line_N[20].y2 = 1800;
	Line_N[20].line_kp = 20;
	Line_N[20].line_A = 0;
	Line_N[20].line_B = 0;
	Line_N[20].line_C = 0;
	Line_N[20].line_Angle = 0;
	Line_N[20].line_Error = 0;
	
	Line_N[23].x1 = -1000;
	Line_N[23].y1 = 1400;
	Line_N[23].x2 = -1000;
	Line_N[23].y2 = 3400;
	Line_N[23].line_kp = 20;
	Line_N[23].line_A = 0;
	Line_N[23].line_B = 0;
	Line_N[23].line_C = 0;
	Line_N[23].line_Angle = 0;
	Line_N[23].line_Error = 0;
	
	Line_N[22].x1 = -1000;
	Line_N[22].y1 = 3400;
	Line_N[22].x2 = 1000;
	Line_N[22].y2 = 3400;
	Line_N[22].line_kp = 20;
	Line_N[22].line_A = 0;
	Line_N[22].line_B = 0;
	Line_N[22].line_C = 0;
	Line_N[22].line_Angle = 0;
	Line_N[22].line_Error = 0;
	
	Line_N[21].x1 = 1000;
	Line_N[21].y1 = 3400;
	Line_N[21].x2 = 1000;
	Line_N[21].y2 = 1400;
	Line_N[21].line_kp = 20;
	Line_N[21].line_A = 0;
	Line_N[21].line_B = 0;
	Line_N[21].line_C = 0;
	Line_N[21].line_Angle = 0;
	Line_N[21].line_Error = 0;
	
	Line_N[24].x1 = 1000;
	Line_N[24].y1 = 1400;
	Line_N[24].x2 = -1000;
	Line_N[24].y2 = 1400;
	Line_N[24].line_kp = 20;
	Line_N[24].line_A = 0;
	Line_N[24].line_B = 0;
	Line_N[24].line_C = 0;
	Line_N[24].line_Angle = 0;
	Line_N[24].line_Error = 0;
	
	Line_N[27].x1 = -1400;
	Line_N[27].y1 = 1000;
	Line_N[27].x2 = -1400;
	Line_N[27].y2 = 3800;
	Line_N[27].line_kp = 20;
	Line_N[27].line_A = 0;
	Line_N[27].line_B = 0;
	Line_N[27].line_C = 0;
	Line_N[27].line_Angle = 0;
	Line_N[27].line_Error = 0;
	
	Line_N[26].x1 = -1400;
	Line_N[26].y1 = 3800;
	Line_N[26].x2 = 1400;
	Line_N[26].y2 = 3800;
	Line_N[26].line_kp = 20;
	Line_N[26].line_A = 0;
	Line_N[26].line_B = 0;
	Line_N[26].line_C = 0;
	Line_N[26].line_Angle = 0;
	Line_N[26].line_Error = 0;
	
	Line_N[25].x1 = 1400;
	Line_N[25].y1 = 3800;
	Line_N[25].x2 = 1400;
	Line_N[25].y2 = 1000;
	Line_N[25].line_kp = 20;
	Line_N[25].line_A = 0;
	Line_N[25].line_B = 0;
	Line_N[25].line_C = 0;
	Line_N[25].line_Angle = 0;
	Line_N[25].line_Error = 0;
	
	Line_N[28].x1 = 1400;
	Line_N[28].y1 = 1000;
	Line_N[28].x2 = -1400;
	Line_N[28].y2 = 1000;
	Line_N[28].line_kp = 20;
	Line_N[28].line_A = 0;
	Line_N[28].line_B = 0;
	Line_N[28].line_C = 0;
	Line_N[28].line_Angle = 0;
	Line_N[28].line_Error = 0;
	
	Line_N[31].x1 = -1800;
	Line_N[31].y1 = 600;
	Line_N[31].x2 = -1800;
	Line_N[31].y2 = 4200;
	Line_N[31].line_kp = 20;
	Line_N[31].line_A = 0;
	Line_N[31].line_B = 0;
	Line_N[31].line_C = 0;
	Line_N[31].line_Angle = 0;
	Line_N[31].line_Error = 0;
	
	Line_N[30].x1 = -1800;
	Line_N[30].y1 = 4200;
	Line_N[30].x2 = 1800;
	Line_N[30].y2 = 4200;
	Line_N[30].line_kp = 20;
	Line_N[30].line_A = 0;
	Line_N[30].line_B = 0;
	Line_N[30].line_C = 0;
	Line_N[30].line_Angle = 0;
	Line_N[30].line_Error = 0;
	
	Line_N[29].x1 = 1800;
	Line_N[29].y1 = 4200;
	Line_N[29].x2 = 1800;
	Line_N[29].y2 = 600;
	Line_N[29].line_kp = 20;
	Line_N[29].line_A = 0;
	Line_N[29].line_B = 0;
	Line_N[29].line_C = 0;
	Line_N[29].line_Angle = 0;
	Line_N[29].line_Error = 0;
	
	Line_N[32].x1 = 1800;
	Line_N[32].y1 = 600;
	Line_N[32].x2 = -1800;
	Line_N[32].y2 = 600;
	Line_N[32].line_kp = 20;
	Line_N[32].line_A = 0;
	Line_N[32].line_B = 0;
	Line_N[32].line_C = 0;
	Line_N[32].line_Angle = 0;
	Line_N[32].line_Error = 0;
	
	Line_N[33].x1 = -1800;
	Line_N[33].y1 = 600;
	Line_N[33].x2 = -1800;
	Line_N[33].y2 = 4200;
	Line_N[33].line_kp = 20;
	Line_N[33].line_A = 0;
	Line_N[33].line_B = 0;
	Line_N[33].line_C = 0;
	Line_N[33].line_Angle = 0;
	Line_N[33].line_Error = 0;
}

void PID_Arc_Init(void)                                                      //圆形参数初始化
{
	Arc_N[0].x0 = 0;
	Arc_N[0].y0 = 2200;
	Arc_N[0].r0 = 600;
	Arc_N[0].arc_Angle = 0;
	Arc_N[0].arc_Error = 0;
	Arc_N[0].arc_Direction = CW;
	Arc_N[0].arc_kp = 20;
	
	Arc_N[1].x0 = 0;
	Arc_N[1].y0 = 2200;
	Arc_N[1].r0 = 1100;
	Arc_N[1].arc_Angle = 0;
	Arc_N[1].arc_Error = 0;
	Arc_N[1].arc_Direction = CW;
	Arc_N[1].arc_kp = 20;
	
	Arc_N[2].x0 = 0;
	Arc_N[2].y0 = 2200;
	Arc_N[2].r0 = 1600;
	Arc_N[2].arc_Angle = 0;
	Arc_N[2].arc_Error = 0;
	Arc_N[2].arc_Direction = CW;
	Arc_N[2].arc_kp = 20;
}

void PID_Coordinate_Init(void)                                               //坐标参数初始化
{
	Coordinate_N[0].x3 = 1000;
	Coordinate_N[0].y3 = 1000;
	Coordinate_N[0].coordinate_Angle = 0;
}
void PID_Init(PID_Value *PID_a)                                              //PID总参数初始化
{
	PID_a->direction = 0;
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
	PID_a->V = 1000;
	PID_a->Coordinate_Num = 0;
	PID_a->Line_Num = 0;
	PID_a->Line_Cnt = 0;
	PID_a->Arc_Num = 0;
	PID_a->V_Set = 1000;
	PID_a->vel = 0;
	for(int i = 0;i < 24;i++) PID_a->Line_Order[i] = line_order1[i];
		
	PID_Line_Init();
	PID_a->l = &Line_N[PID_a->Line_Num];
	
	PID_Arc_Init();
	PID_a->r = &Arc_N[PID_a->Arc_Num];
	
	PID_Coordinate_Init();
	PID_a->c = &Coordinate_N[PID_a->Coordinate_Num];
}

void PID_Control(PID_Value *p)                                               //PID控制函数，输出电机转速
{
	if(p->Mode == manual)
	{
		p->Mode_Last = manual;
		return;
	}
	if(p->Mode_Last == manual) Init_PID(p);	
	p->Angle = GetAngle();
	p->X = GetX();
	p->Y = GetY() + 95;
	p->X_Speed = GetSpeedX();
	p->Y_Speed = GetSpeedY();
	p->l = &Line_N[p->Line_Num];
	p->r = &Arc_N[p->Arc_Num];
	p->c = &Coordinate_N[p->Coordinate_Num];
	switch(p->Mode)
	{
		case Arc:
			p->r->arc_Error = sqrt((((p->X)-(p->r->x0))*((p->X)-(p->r->x0))+((p->Y)-(p->r->y0))*((p->Y)-(p->r->y0)))) - (p->r->r0);
			p->r->arc_Error = constrain(p->r->arc_Error,1800,-1800);
			if((p->X) == (p->r->x0))  p->r->arc_Angle = Compare((p->Y),(p->r->y0))*90;
			else if((p->X)>(p->r->x0)) p->r->arc_Angle = ((float)atan((double)(((p->Y)-(p->r->y0))/((p->X)-(p->r->x0)))))*(180/Pi);
			else if((p->X)<(p->r->x0)) p->r->arc_Angle = 180+((float)atan((double)(((p->Y)-(p->r->y0))/((p->X)-(p->r->x0)))))*(180/Pi);
			if((p->r->arc_Angle)>180) (p->r->arc_Angle) -= 360;
			else if((p->r->arc_Angle)<-180) (p->r->arc_Angle) += 360;
			if(p->r->arc_Direction == CW) (p->r->arc_Angle) -= 180;
			if(p->r->arc_Direction == CW) (p->r->arc_Angle) = (p->r->arc_Angle)-(p->r->arc_Error)/(p->r->arc_kp);
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
	if ((p->Error) > -10 && (p->Error) < 10)
	{
		p->kp = 30;
		p->kd = 50;
		p->ki = 0.05;
	}
	else 
	{
		p->kp = 30;
		p->kd = 50;
		p->ki = 0;
	}
	p->ITerm += p->ki * p->Error;
	p->ITerm = constrain(p->ITerm,2000.0f,-2000.0f);
	p->DTerm = p->Angle_Last - p->Angle;
	p->vel = p->kp * p->Error + p->ki * p->ITerm + p->kd * p->DTerm;
	p->vel = constrain(p->vel,2000.0f,-2000.0f);
	if(p->Line_Num < 4 || (p->Line_Num > 16 && p->Line_Num < 21))
	p->V = (p->V_Set);
	p->Angle_Last = p->Angle;
	p->Mode_Last = p->Mode;
}

void PID_Pre(PID_Value *p)                                                   //去掉PID部分的PID函数^_^
{
	p->l = &Line_N[p->Line_Num];
	p->r = &Arc_N[p->Arc_Num];
	p->c = &Coordinate_N[p->Coordinate_Num];
	switch(p->Mode)
	{
		case Arc:
			p->r->arc_Error = sqrt((((p->X)-(p->r->x0))*((p->X)-(p->r->x0))+((p->Y)-(p->r->y0))*((p->Y)-(p->r->y0)))) - (p->r->r0);
			p->r->arc_Error = constrain(p->r->arc_Error,1800,-1800);
			if((p->X) == (p->r->x0))  p->r->arc_Angle = Compare((p->Y),(p->r->y0))*90;
			else if((p->X)>(p->r->x0)) p->r->arc_Angle = ((float)atan((double)(((p->Y)-(p->r->y0))/((p->X)-(p->r->x0)))))*(180/Pi);
			else if((p->X)<(p->r->x0)) p->r->arc_Angle = 180+((float)atan((double)(((p->Y)-(p->r->y0))/((p->X)-(p->r->x0)))))*(180/Pi);
			if((p->r->arc_Angle)>180) (p->r->arc_Angle) -= 360;
			else if((p->r->arc_Angle)<-180) (p->r->arc_Angle) += 360;
			if(p->r->arc_Direction == CW) (p->r->arc_Angle) -= 180;
			if(p->r->arc_Direction == CW) (p->r->arc_Angle) = (p->r->arc_Angle)-(p->r->arc_Error)/(p->r->arc_kp);
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
			p->l->line_Error = constrain(p->l->line_Error,1800,-1800);
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
	float kw = ABS(GetWZ());
	if(kw < 100) kw = 1;
	else kw = kw/100;
	p->V_Set = constrain(2*ABS(p->l->line_Error)/kw,3000,1000);
}

void GO(PID_Value *p_GO)                                                     //电机控制函数
{
	VelCrl(CAN2,1,(int)(((4096/378)*(p_GO->vel))+(4096/378)*(p_GO->V)));
	VelCrl(CAN2,2,(int)(((4096/378)*(p_GO->vel))-(4096/378)*(p_GO->V)));
}

void UART4_OUT(PID_Value *pid_out)                                           //串口输出函数
{
	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetX());
	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetY());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->Angle_Set);
	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->l->line_Error);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetWZ());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->vel);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->V);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)(sqrt(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY())));
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)ReadShooterVel());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetShooterVelCommand());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)ReadYawPos());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetYawPosCommand());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)ReadLaserAValue());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)ReadLaserBValue());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->Angle);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)Get_Adc_Average(15,10));
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)Get_Adc_Average(14,10));
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetShooterVelCommand());
	USART_SendData(UART4,'\r');
	USART_SendData(UART4,'\n');
}

void PID_Competition_testVersion(PID_Value *pid, u8 dir)
{
	static u8 flag = 0;
	if(flag == 0 && dir == Right)
	{
		for(int i = 0;i < 24;i++) pid->Line_Order[i] = line_order1[i];
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		flag = 1;
	}
	if(flag == 0 && dir == Left)
	{
		for(int i = 0;i < 24;i++) pid->Line_Order[i] = line_order2[i];
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		flag = 1;
	}
	if(pid->Line_Num < 17)
	{
		pid->direction = ACW;
		pid->Mode = Line;
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		PID_Control(pid);
		GO(pid);
		pid->Line_Cnt += 1;
		if(pid->Line_Cnt == 24) pid->Line_Cnt = 0;
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		PID_Pre(pid);
		if(pid->l->line_Error>600) pid->Line_Cnt -= 1;
		if(pid->Line_Cnt == -1) pid->Line_Cnt = 23;
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		pid->target_Num = pid->Line_Num % 4 +1;
		if(pid->target_Num == 4) pid->target_Num = 0;
	}
	else if(pid->Line_Num > 16)
	{
		pid->direction = CW;
		pid->Mode = Line;
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		PID_Control(pid);
		GO(pid);
		pid->Line_Cnt += 1;
		if(pid->Line_Cnt == 24) pid->Line_Cnt = 0;
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		PID_Pre(pid);
		if(pid->l->line_Error<-600) pid->Line_Cnt -= 1;
		if(pid->Line_Cnt == -1) pid->Line_Cnt = 23;
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		pid->target_Num = 3 - pid->Line_Num % 4;
	}
}

void PID_Competition(PID_Value *pid, u8 dir)
{
	static u8 flag = 0;
	if(flag == 0 && dir == Right)
	{
		for(int i = 0;i < 36;i++) pid->Line_Order[i] = line_order3[i];
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		flag = 1;
	}
	if(flag == 0 && dir == Left)
	{
		for(int i = 0;i < 36;i++) pid->Line_Order[i] = line_order4[i];
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		flag = 1;
	}
	if(pid->Line_Num < 17)
	{
		pid->direction = ACW;
		pid->Mode = Line;
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		PID_Control(pid);
		GO(pid);
		pid->Line_Cnt += 1;
		if(pid->Line_Cnt == 36) pid->Line_Cnt = 0;
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		PID_Pre(pid);
		if(pid->l->line_Error>800) pid->Line_Cnt -= 1;
		if(pid->Line_Cnt == -1) pid->Line_Cnt = 35;
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		pid->target_Num = pid->Line_Num % 4 +1;
		if(pid->target_Num == 4) pid->target_Num = 0;
	}
	else if(pid->Line_Num > 16)
	{
		pid->direction = CW;
		pid->Mode = Line;
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		PID_Control(pid);
		GO(pid);
		pid->Line_Cnt += 1;
		if(pid->Line_Cnt == 36) pid->Line_Cnt = 0;
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		PID_Pre(pid);
		if(pid->l->line_Error<-800) pid->Line_Cnt -= 1;
		if(pid->Line_Cnt == -1) pid->Line_Cnt = 35;
		pid->Line_Num = pid->Line_Order[pid->Line_Cnt];
		pid->target_Num = 3 - pid->Line_Num % 4;
	}
}

void ErrorDisposal(PID_Value *pid)
{
	
}
