#ifndef _MUC_PID_H__
#define _MUC_PID_H__
#include "main.h"


typedef struct
{
    //PID 三参数
    double Kp;
    double Ki;
    double Kd;
	double Ka;
	
	//最大输出 死区
	double max_out;  //最大输出
	double dead_band;//PID偏差死区
	double intergral_band;//积分区
	double max_input;//最大输入
	
	//PID输出值
	double output;
	double output_compensation;
	
	//误差
	double e_max;
	double e[3];//2最新 1上一次 0上上次
	double d_last;
	double intergral;  //积分
	
} PidTypeDef;

extern PidTypeDef Yaw_speed_pid;

void PID_Init(PidTypeDef * pid,double kp,double ki,double kd,double ka,double max_out,double dead_band,double i_band,double max_input);
void PID_Calc(PidTypeDef * pid, double rel_val, double set_val );

#endif
