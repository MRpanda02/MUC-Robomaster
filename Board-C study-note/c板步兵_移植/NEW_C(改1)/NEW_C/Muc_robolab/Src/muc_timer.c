#include "muc_timer.h"
#include "muc_main_task.h"



static uint16_t runTimes = 0;
void HandlerTim6CallBlack( void )
{
    /* �߳�����ʱ����� */
    
    
    /* ���ó�ʼʱ�� */
    __HAL_TIM_SET_COUNTER(&htim14,1);

    
    TaskAllRunningLoop();
    
    /* ��ȡ����ʱ��,�ɵõ��������������ʱ�� */
    runTimes = __HAL_TIM_GET_COUNTER(&htim14);
    if( runTimes>900 )
    {
        //���������̵�����ʱ�������Σ�վ���
    }
}

