#include "muc_pid.h"


//初始化PID
void PID_Init(PidTypeDef * pid,double kp,double ki,double kd,double ka,double max_out,double dead_band,double i_band,double max_input)
{
	pid->Kp=kp;
	pid->Ki=ki;
	pid->Kd=kd;
	pid->Ka=ka;
	pid->max_out=max_out;
	pid->dead_band=dead_band;
	pid->intergral_band=i_band;
	pid->max_input=max_input;
	pid->output=0;
	pid->e[0]=0;
	pid->e[1]=0;
	pid->e[2]=0;
	pid->d_last=0;
}


//PID计算
void PID_Calc(PidTypeDef * pid, double rel_val, double set_val )
{
	double p = 0;
	double i = 0;
	double d = 0;
	//把输入值与限制的最大输入做匹配
	VAL_LIMIT(set_val,-pid->max_input,pid->max_input);
	
	pid->e[2] = set_val - rel_val; //当前误差
	
	//如果误差在死区内 则直接输出为0 不用再进行PID运算
	if(MyAbs(pid->e[2]) >=pid->dead_band)
	{
		if(MyAbs(pid->e[2]) <= pid->intergral_band)
			pid->intergral = pid->intergral+ (pid->Ki) * (pid->e[2]);	
		else
			{pid->intergral=pid->intergral*0.99;}
			
		VAL_LIMIT(pid->intergral,-pid->max_out,pid->max_out);
		
				
  		p = pid->Kp * (pid->e[2]);
		i = pid->intergral;
		d = pid->Kd*(pid->e[2]-pid->e[1]) * (1-pid->Ka) + (pid->d_last)*(pid->Ka); //pid->Kd * (pid->e[2]-pid->e[1]) 
		
		pid->d_last=d;
		pid->output =p+i+d;
		
		

		VAL_LIMIT(pid->output,-pid->max_out,pid->max_out);

	} else {
		pid->output=0;
	}
	pid->e[0] = pid->e[1];
	pid->e[1] = pid->e[2];
}


