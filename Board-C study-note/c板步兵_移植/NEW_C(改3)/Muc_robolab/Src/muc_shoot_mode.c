#include "muc_shoot_mode.h"
#include "muc_stir_wheel.h"
#include "muc_Shoot_Judge_Control.h"
extern int32_t handledShoot_P_Feedback;
extern int16_t shooting_position;



st_mode St_Mode =running;  //射击模式，初始化为连发

//射击函数，根据射击模式有不同射击效果,在需要打弹的时候统一调用

void Shooting(void)
{ 	
	if(Heat_Limit==0)
	{
		if(St_Mode == single){
		Single();
		
		}
		else if(St_Mode == running){
		Running();
		}		
	}
	else return;
}

int IgnoreFirst=0;
int ShootFeedBack[2]={0};
int StopTime=0;
void One_shoot(void)
{
		static int Tasktime=0;
	static int CircleTime=0;
    static int realPosTemp;
	
	ShootFeedBack[1]=ShootFeedBack[0];
	ShootFeedBack[0]=Stir_position_Feedback.ecd_value;
	if(MyAbs((ShootFeedBack[0]-ShootFeedBack[1]))<100)
	{
	StopTime++;
	}

	
	realPosTemp = shooting_position-Stir_position_Feedback.ecd_value;
	
	if( realPosTemp<-10000 )
	{
	realPosTemp += 24600;
	}
	
	if(StopTime>4)
	{

	Tasktime++;
    CircleTime=(CircleTime+1)%6;
    
	if(   CircleTime<3)
	{
	shooting_position=Stir_position_Feedback.ecd_value+600;
	}
	
	else
	{
	shooting_position=Stir_position_Feedback.ecd_value-600;
		
	}
	if(Tasktime>6)
	{
	CircleTime=0;
	StopTime=0;	
	Tasktime=0;
	}
}
	



	else if( (realPosTemp<291)||(realPosTemp>20000) )
	{
		 CircleTime=0;
	shooting_position-=(590.4);
    Remain_Shoot_Number--;
	}

	else
	{
	 CircleTime=0;
	}
	
	

	
 }

 void Single(void)
{
     
	if(MouseLeftPressTime==1){
	One_shoot();	
	}
	
	else{
	if(MouseLeftPressTime>250){
	MouseLeftPressTime=400;}
	}
}


void Running(void)
{
	
	static int RunningtTime=0;
	static int WaitingTime=0;

	if(MouseLeftPressTime==1)
	{
	One_shoot();  
    RunningtTime++;
	}
	else
	{
	if(MouseLeftPressTime>100)
	MouseLeftPressTime=0;
	}

}
void TenRunning()
{

	static int RunningtTime=0;
	static int WaitingTime=0;
	if(RunningtTime<11)
	{
	if(MouseLeftPressTime==1)
	{
	One_shoot();  
    RunningtTime++;
	}
	else
	{
	if(MouseLeftPressTime>60)
	MouseLeftPressTime=0;
	}
	
    }
	
	else
	{
	WaitingTime++;
	if(WaitingTime>250)	
	{
	WaitingTime=0;
	RunningtTime=0;
	}
	
	}

}

//设置射击模式
void set_st_mode(st_mode mode)
{
	St_Mode = mode;
}


st_mode get_st_mode(void)
{
	return St_Mode;
}


//读取模式函数
void change_st_mode(void)
{
	static st_mode mode;
	
	mode = get_st_mode();
	
	if(mode==single){
		set_st_mode(running);
	}
	else if(mode==running){		
		set_st_mode(single);
	}
}
