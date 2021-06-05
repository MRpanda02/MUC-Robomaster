#ifndef _MUC_MAIN_TASK_H__
#define _MUC_MAIN_TASK_H__
#include "main.h"

typedef enum
{
    PREPARE_STATE,     		//�ϵ���ʼ��״̬ 
    STANDBY_STATE,			//��ֹ̨ͣ��ת״̬
    NORMAL_STATE,			//����״̬
    STOP_STATE,        	    //ֹͣ�˶�״̬
}WorkState_t;


void TaskAllBeginRun( void );
void TaskAllRunningLoop( void );
WorkState_t GetWorkState(void);
static uint32_t mainTaskTimes=0;


#endif  // _MUC_MAIN_TASK_H__
