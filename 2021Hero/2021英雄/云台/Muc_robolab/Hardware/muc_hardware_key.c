#include "muc_hardware_key.h"

ButtonSta_t GetKeySta( void )
{
    return (KEY()==GPIO_PIN_SET) ? BtnPressDown:BtnPressUp;
}

uint8_t GetKeyPressUpAction( void )
{
    static uint8_t getPressDown=0;
    if(GetKeySta()==BtnPressDown)
    {
        getPressDown=1;
        return 0;
    }
    if( (1==getPressDown)&&(GetKeySta()==BtnPressUp) )
    {
        getPressDown=0;
        return 1;
    }
        
    return 0;
}
