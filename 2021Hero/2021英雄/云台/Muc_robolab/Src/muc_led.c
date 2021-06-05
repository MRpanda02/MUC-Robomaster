#include "muc_led.h"

void LedStaInit( void )
{
    LED_Red_Off();
    LED_Green_Off();
}

void LedTaskLoop( void )
{
    static uint16_t ledTimes=0;
    
    if( ledTimes++>300)
    {
        ledTimes=0;
        LED_Green_Toggle();
    }
	
}

