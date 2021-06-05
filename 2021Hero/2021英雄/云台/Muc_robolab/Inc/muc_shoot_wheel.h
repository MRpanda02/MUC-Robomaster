#ifndef _MUC_SHOOT_WHEEL_H__
#define _MUC_SHOOT_WHEEL_H__
#include "main.h"
#include "muc_pid.h"

//��Ħ����
    //Ħ����ת��
#define SHOOT_LEFT_CONFIG_FrictionWheelSpeed_HIGH  12000  //before 3000  //24
#define SHOOT_LEFT_CONFIG_FrictionWheelSpeed_LOW   7000  //18

#define SHOOT_RIGHT_CONFIG_FrictionWheelSpeed_HIGH  -12000  //before 3000  //24
#define SHOOT_RIGHT_CONFIG_FrictionWheelSpeed_LOW   -7000  //18


#define LASER_ON()  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET)
#define LASER_OFF()  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET)

typedef enum//����
{
    Spd_Stop = 0,
	Spd_Low = 1,
	Spd_High = 2,
}ShootSpeedSta_t;

typedef enum//Ħ����״̬
{
  WheelStop = 0,
  WheelRun
}WheelSta_t;

typedef struct
{
  int16_t real[8];
	uint8_t count;   //���ݽ��ռ���
	int16_t round_cnt;	//Ȧ��
	int16_t ecd_value;	//��������������ı�����ֵ
	int16_t sum;		//������ֵ�ۻ���		
	int16_t bias;		//��ʼ������ֵ	
	int16_t calc;
	int16_t position_calc;	//λ�ü���ֵ
	int16_t calc_diff;	//���α�����֮��Ĳ�ֵ
	int16_t calc_last;
	int16_t turns;
} ShootReceiveTypeDef;


void Shoot_Speed_Data_deal(ShootReceiveTypeDef *Receive, uint8_t * msgData);
ShootSpeedSta_t MucGetShootSpeed( void );
void SmallShoot_Loop_Task( void );
void MucShootInit( void );
void Shoot_Loop_Task(void);


extern ShootReceiveTypeDef Shoot_Left_Feedback;
extern ShootReceiveTypeDef Shoot_Right_Feedback;
extern uint16_t MouseLeftPressTime;
extern uint16_t MouseRightPressTime; 
extern uint32_t BigWheelRunTime;
extern WheelSta_t BigWheelSta ;
extern PidTypeDef Shoot_left_speed_pid;
extern PidTypeDef Shoot_right_speed_pid;

//СĦ����
extern WheelSta_t SmallWheelSta;
extern uint32_t SmallWheelRunTime;

void BigShoot_Loop_Task();



#endif
