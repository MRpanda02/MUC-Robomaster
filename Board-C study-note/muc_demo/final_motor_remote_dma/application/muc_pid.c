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
	
	double P = 0;
	double I = 0;
	double D = 0;
	//把输入值与限制的最大输入做匹配
	VAL_LIMIT(set_val,-pid->max_input,pid->max_input);
	
	pid->e[2] = set_val - rel_val; //当前误差
	
	/*如果误差不在死区内则运算，否则直接输出为0 不用再进行PID运算*/
	if(MyAbs(pid->e[2]) >=pid->dead_band) //误差大于一定范围
	{
		if(MyAbs(pid->e[2]) <= pid->intergral_band) //误差小于积分上限
		{
			pid->intergral += pid->intergral+(pid->Ki) * (pid->e[2]);	
		}
		else
		{
			pid->intergral = pid->intergral * 0.99;
		}
			
		VAL_LIMIT(pid->intergral,-pid->max_out,pid->max_out);
		
				
  		p = pid->Kp * (pid->e[2]);
		i = pid->intergral;
		d = pid->Kd*(pid->e[2]-pid->e[1]) * (1-pid->Ka) + (pid->d_last)*(pid->Ka); //pid->Kd * (pid->e[2]-pid->e[1]) 
		
		pid->d_last=d;
		pid->output =p+i+d;
		
		
		P=pid->Kp*(pid->e[2]-pid->e[1]);
		
//		if(MyAbs(pid->e[2] )<= MyAbs((set_val/2))  ) //误差小于积分上限
//		{
//			
//		pid->Ki=pid->KiSpeed*MyAbs(rel_val);
//		I=pid->Ki * pid->e[2];
//			
//			
//		}
//		else
//		{
//		
//		pid->Ki=0;
//		I=0;
//		}
//		
//		if(pid->e[2]<-MyAbs(set_val)*0.01  )
//		{
//		I=0;
//		
//		}
//		
	I=pid->Ki * pid->e[2];	
		
		
		D= pid->Kd*(pid->e[2]-2*pid->e[1]+pid->e[0]);
		
		

		
		pid->output_compensation+=  P+I+D;

		
		VAL_LIMIT(pid->output_compensation,-pid->max_out,pid->max_out);
		VAL_LIMIT(pid->output,-pid->max_out,pid->max_out);
		
		if(pid->output_compensation==-pid->max_out||pid->output_compensation==pid->max_out)

		{
		I=0;
		}
	} 
	else 
	{
		pid->output=0;
	}
	pid->e[0] = pid->e[1];
	pid->e[1] = pid->e[2];
}


