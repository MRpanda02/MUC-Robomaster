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
	//SHIFT±êÖ¾·¢ËÍ
	if(CC_Condition.Key_SHIFT==PressDown)
	{
	  CAN_Send_Message( &hcan2, 0X140, 1, 0, 0, 0 );
	}
	else{
	  CAN_Send_Message( &hcan2, 0X140, 0, 0, 0, 0 );
	}	
}
