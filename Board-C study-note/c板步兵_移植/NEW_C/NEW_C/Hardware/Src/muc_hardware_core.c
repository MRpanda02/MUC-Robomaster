#include "muc_hardware_core.h"


/* ϵͳ�������� */
void MucSystemReboot( void )
{
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}


