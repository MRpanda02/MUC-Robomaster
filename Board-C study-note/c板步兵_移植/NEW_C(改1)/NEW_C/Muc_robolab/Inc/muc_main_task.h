#ifndef _MUC_MAIN_TASK_H__
#define _MUC_MAIN_TASK_H__
#include "main.h"

typedef enum
{
    PREPARE_STATE=0,     		//上电后初始化状态 
    STANDBY_STATE=1,			//云台停止不转状态
    NORMAL_STATE=2,			//正常状态
    STOP_STATE=3,        	    //停止运动状态
}WorkState_t;


void TaskAllBeginRun( void );
void TaskAllRunningLoop( void );
WorkState_t GetWorkState(void);
static uint32_t mainTaskTimes=0;


#endif  // _MUC_MAIN_TASK_H__
