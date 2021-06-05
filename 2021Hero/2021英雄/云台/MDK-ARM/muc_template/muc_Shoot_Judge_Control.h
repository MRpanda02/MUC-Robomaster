#ifndef _MUC_SHOOT_JUDGE_CONTROL_H__
#define _MUC_SHOOT_JUDGE_CONTROL_H__


#include "main.h"

void JudgementDataCheck_Task(void);



extern uint16_t Shoot_Heat;  //17mm��������
extern uint16_t Shoot_Last_Heat;   //��һ��17mm��������
extern uint8_t  Shoot_Speed;  //����
extern uint16_t Remain_Shoot_Number;  //ʣ��ɷ����ӵ���
extern uint16_t Shoot_Max_Heat;  //��ǰ��������
extern uint8_t  Robot_Level;   //Ӣ�۵ȼ�
extern uint8_t  Heat_Limit;  //�������������־
extern uint8_t  Heat_Wait; //������ǿ��ֹͣ���1s��־
extern uint16_t Heat_Wait_Time; //������ǿ��ֹͣ����һ��
extern uint16_t Have_Shoot_Done; //�ѷ���




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
