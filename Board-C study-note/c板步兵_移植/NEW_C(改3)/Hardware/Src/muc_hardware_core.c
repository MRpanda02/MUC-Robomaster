#include "muc_hardware_core.h"


/* 系统重启函数 */
void MucSystemReboot( void )
{
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}


