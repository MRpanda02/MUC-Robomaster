#include "muc_hardware_super_capacity.h"
#include "muc_remote.h"
#include "muc_hardware_can.h"


extern uint16_t ShiftToChassis; 

void Deal_Data_ForShift( uint16_t ShiftFlag, uint8_t * msgData)
{
ShiftFlag =(msgData[0]<<8)|msgData[1];
}

void SuperCapControlLoopTask( void )
{
	
//	
	if(CC_Condition.Key_SHIFT==PressDown)
	{
	  CAN_Send_Message( &hcan2, 0X140, 1, 0, 0, 0 );
	}
	else{
	  CAN_Send_Message( &hcan2, 0X140, 0, 0, 0, 0 );
	}
		
    if(CC_Condition.S1Sta == StaN1 )
    {
        // SuperCapInputOn();  /* 打开超级电容充电 */
        // SuperCapInputOff();  /* 关闭超级电容充电 */
        // SuperCapOutputOn();  /* 打开超级电容放电 */
		// SuperCapOutputOff();  /* 关闭超级电容放电 */
    }
	
	
	
}
