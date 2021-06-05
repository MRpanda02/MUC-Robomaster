#ifndef _MUC_SHOOT_JUDGE_CONTROL_H__
#define _MUC_SHOOT_JUDGE_CONTROL_H__


#include "main.h"

void JudgementDataCheck_Task(void);



extern uint16_t Shoot_Heat;  //17mm弹丸热量
extern uint16_t Shoot_Last_Heat;   //上一次17mm弹丸热量
extern uint8_t  Shoot_Speed;  //射速
extern uint16_t Remain_Shoot_Number;  //剩余可发射子弹数
extern uint16_t Shoot_Max_Heat;  //当前热量上限
extern uint8_t  Robot_Level;   //英雄等级
extern uint8_t  Heat_Limit;  //热量限制射击标志
extern uint8_t  Heat_Wait; //超热量强制停止射击1s标志
extern uint16_t Heat_Wait_Time; //超热量强制停止发弹一秒
extern uint16_t Have_Shoot_Done; //已发射




typedef struct
{
   uint16_t shoot_heat;
   uint16_t robot_level;
   uint16_t remain_hp;
   uint16_t max_hp;
	
} receive_from_chassis;

void Deal_Data_From_Chassis(receive_from_chassis  *FromC, uint8_t * msgData);

extern receive_from_chassis shoot_control_consulation;
#endif
