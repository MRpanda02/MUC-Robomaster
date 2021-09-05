#ifndef _MUC_MAIN_TASK_H__
#define _MUC_MAIN_TASK_H__
#include "main.h"

typedef enum
{
    PREPARE_STATE=0,     		//�ϵ���ʼ��״̬ 
    STANDBY_STATE=1,			//��ֹ̨ͣ��ת״̬
    NORMAL_STATE=2,			//����״̬
    STOP_STATE=3,        	    //ֹͣ�˶�״̬
}WorkState_t;


void TaskAllBeginRun( void );
void TaskAllRunningLoop( void );
WorkState_t GetWorkState(void);
static uint32_t mainTaskTimes=0;


#endif  // _MUC_MAIN_TASK_H__
