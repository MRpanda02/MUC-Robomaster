#ifndef _MUC_PID_H__
#define _MUC_PID_H__
#include "main.h"


typedef struct
{
    //PID ������
    double Kp;
    double Ki;
    double Kd;
	double Ka;
	
	//������ ����
	double max_out;  //������
	double dead_band;//PIDƫ������
	double intergral_band;//������
	double max_input;//�������
	
	//PID���ֵ
	double output;
	double output_compensation;
	
	//���
	double e_max;
	double e[3];//2���� 1��һ�� 0���ϴ�
	double d_last;
	double intergral;  //����
	
} PidTypeDef;

extern PidTypeDef Yaw_speed_pid;

void PID_Init(PidTypeDef * pid,double kp,double ki,double kd,double ka,double max_out,double dead_band,double i_band,double max_input);
void PID_Calc(PidTypeDef * pid, double rel_val, double set_val );

#endif
