#include "muc_led.h"

void LedStaInit( void )
{
    LED_Red_Off();
    LED_Green_Off();
	LED_Blue_Off();
}

static uint16_t ledTimes=0;
void LedTaskLoop( void )
{
	
    if( ledTimes++>300)
    {
        ledTimes=0;
        LED_Green_Toggle();
    }
	
}

