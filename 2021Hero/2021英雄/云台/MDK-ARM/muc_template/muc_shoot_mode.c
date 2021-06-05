#include "muc_shoot_mode.h"
#include "muc_stir_wheel.h"
#include "muc_Shoot_Judge_Control.h"
#include "muc_shoot_wheel.h"

extern int32_t BigTurn_Position_ecd;
extern int16_t BigTurn_Position_ref;


//С����
st_mode St_Mode =single;  //���ģʽ����ʼ��Ϊ����

 //С��������������������ģʽ�в�ͬ���Ч��,����Ҫ�򵯵�ʱ��ͳһ����
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


void One_shoot(void)
{
  static int realPosTemp;
//	realPosTemp = SmaTurn_Position_ref-Stir_position_Feedback.ecd_value;
	
	if( realPosTemp<-10000 ){
	realPosTemp += 24600;
	}
	
	if( (realPosTemp<291)||(realPosTemp>20000) ){
	SmaTurn_Position_ref+=(320);	
	}

}

void Single(void)
{
	if(MouseLeftPressTime==1){
	One_shoot();
	}
	
	else
	{
		if(MouseLeftPressTime>250)
		{
			MouseLeftPressTime=400;
		}
	}
}


void Running(void)
{
	if(MouseLeftPressTime==1){
	One_shoot();  
	}
	else{
	if(MouseLeftPressTime>100)
	MouseLeftPressTime=0;
	}
}

void set_st_mode(st_mode mode)//�������ģʽ
{
	St_Mode = mode;
}


st_mode get_st_mode(void)
{
	return St_Mode;
}

void change_st_mode(void)//��ȡģʽ����
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








