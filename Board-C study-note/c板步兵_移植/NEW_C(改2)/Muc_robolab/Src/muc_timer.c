#include "muc_timer.h"
#include "muc_main_task.h"



static uint16_t runTimes = 0;
void HandlerTim6CallBlack( void )
{
    /* 线程运行时间变量 */
    
    
    /* 设置初始时间 */
    __HAL_TIM_SET_COUNTER(&htim14,1);

    
    TaskAllRunningLoop();
    
    /* 获取截至时间,可得到整个任务的运行时间 */
    runTimes = __HAL_TIM_GET_COUNTER(&htim14);
    if( runTimes>900 )
    {
        //整个主进程的运行时间过长，危险警告
    }
}

