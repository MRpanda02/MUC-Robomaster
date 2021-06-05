#ifndef _MUC_STIR_WHEEL_H__
#define _MUC_STIR_WHEEL_H__

#include "main.h"
#include "muc_chassis.h"
#include "muc_pid.h"
#include "muc_gimbal.h"

typedef struct
{
  int16_t real[8];
	uint8_t count;   //数据接收计数
	int16_t round_cnt;	//圈数
	int16_t ecd_value;	//经过处理后连续的编码器值
	int16_t sum;		//编码器值累积和		
	int16_t bias;		//初始编码器值	
	int16_t calc;
	int16_t position_calc;	//位置计算值
	int16_t calc_diff;	//两次编码器之间的差值
	int16_t calc_last;
	int16_t turns;
} StirReceiveTypeDef;

void Shooting_Speed_Data_deal(StirReceiveTypeDef *Receive, uint8_t *msgData);
void Position_Round_Data_deal(StirReceiveTypeDef *Receive, uint8_t *msgData );
void SMControlLoop_Task(void);
void MucStirWheelInit( void );
//弹仓盖   电机(2006)
extern CMReceiveTypeDef MagazinePossionFeedback;   //位置馈信息
extern CMReceiveTypeDef MagazineSpeedFeedback;    //速度反馈信息
extern PidTypeDef Magazine_speed_pid;//速度环PID
extern int16_t MagazinePossion_ref;//目标位置
extern int16_t MagazinePossion_ecd;//实际位置

void Magazine_Init(void);
void MagazinePossionDataDeal(CMReceiveTypeDef *Receive,uint8_t * msgData);
void MagazineSpeedDataDeal(CMReceiveTypeDef *Receive,uint8_t * msgData);

//大波轮 电机(3508)
extern PidTypeDef Big_Stir_speed_pid;
extern int32_t BigTurn_Speed_ecd;
extern volatile Encoder BigTurn_Encoder;

void BigTurn_Data_Deal(volatile Encoder *v, uint8_t *msgData);
void SetBigTurn(int Num);
//小波轮
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

enum//波轮类型
{
	Big_OneTurn=0,
	Small_OneTurn,
};

//大发射机构推弹
extern volatile Encoder NewTurn_Encoder;
extern PidTypeDef NewTurn_Speed_pid;
extern  uint32_t BigPauseTime;//大弹每两发间隔时间,防止超速


void NewTurnInit(void);
void NewTurnTask(int Long);



extern int HavePill;  //当前枪管中有的子弹量
void OneTurnSet(void);//使相应的波轮转一个子弹的角度

extern int16_t ShootNum_42;
extern int16_t ShootNum_17;
void ShootDataDeal( uint8_t * msgData );
void SensorInit(void);
void ShootBigPill();

#endif
