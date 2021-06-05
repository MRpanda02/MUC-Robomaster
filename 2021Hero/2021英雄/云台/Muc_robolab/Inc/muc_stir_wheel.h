#ifndef _MUC_STIR_WHEEL_H__
#define _MUC_STIR_WHEEL_H__

#include "main.h"
#include "muc_chassis.h"
#include "muc_pid.h"
#include "muc_gimbal.h"

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
} StirReceiveTypeDef;

void Shooting_Speed_Data_deal(StirReceiveTypeDef *Receive, uint8_t *msgData);
void Position_Round_Data_deal(StirReceiveTypeDef *Receive, uint8_t *msgData );
void SMControlLoop_Task(void);
void MucStirWheelInit( void );
//���ָ�   ���(2006)
extern CMReceiveTypeDef MagazinePossionFeedback;   //λ������Ϣ
extern CMReceiveTypeDef MagazineSpeedFeedback;    //�ٶȷ�����Ϣ
extern PidTypeDef Magazine_speed_pid;//�ٶȻ�PID
extern int16_t MagazinePossion_ref;//Ŀ��λ��
extern int16_t MagazinePossion_ecd;//ʵ��λ��

void Magazine_Init(void);
void MagazinePossionDataDeal(CMReceiveTypeDef *Receive,uint8_t * msgData);
void MagazineSpeedDataDeal(CMReceiveTypeDef *Receive,uint8_t * msgData);

//���� ���(3508)
extern PidTypeDef Big_Stir_speed_pid;
extern int32_t BigTurn_Speed_ecd;
extern volatile Encoder BigTurn_Encoder;

void BigTurn_Data_Deal(volatile Encoder *v, uint8_t *msgData);
void SetBigTurn(int Num);
//С����
extern volatile Encoder SmallTurn_Encoder;

extern PidTypeDef Sma_Stir_speed_pid;
extern int32_t SmaTurn_Speed_ecd;
extern float SmaTurn_Position_ecd;
extern float SmaTurn_Position_ref;
extern uint8_t SmallInitFlag;

void SetSmalTurn(int Num);
void SmaTurn_Data_Deal(volatile Encoder *v, uint8_t *msgData);




extern uint16_t MouseLeftPressTime;
extern uint16_t MouseRightPressTime; 

enum//��������
{
	Big_OneTurn=0,
	Small_OneTurn,
};

//��������Ƶ�
extern volatile Encoder NewTurn_Encoder;
extern PidTypeDef NewTurn_Speed_pid;
extern  uint32_t BigPauseTime;//��ÿ�������ʱ��,��ֹ����


void NewTurnInit(void);
void NewTurnTask(int Long);



extern int HavePill;  //��ǰǹ�����е��ӵ���
void OneTurnSet(void);//ʹ��Ӧ�Ĳ���תһ���ӵ��ĽǶ�

extern int16_t ShootNum_42;
extern int16_t ShootNum_17;
void ShootDataDeal( uint8_t * msgData );
void SensorInit(void);
void ShootBigPill();

#endif
