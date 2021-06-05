#include "muc_gimbal_chassis.h"

gimbal_to_chassis_t g_chassisDataFromGM;

/***********************上板反馈速度和状态处理**************************/
void Deal_Data_From_Gimbal( uint8_t * msgData )
{
	g_chassisDataFromGM.rawFrontBack = (int16_t)((msgData[0]<<8)|msgData[1]);
    g_chassisDataFromGM.rawLeftRight = (int16_t)((msgData[2]<<8)|msgData[3]);
    g_chassisDataFromGM.rawRotate = (int16_t)((msgData[4]<<8)|msgData[5]);
    g_chassisDataFromGM.rawStatus = (msgData[6]<<8)|msgData[7];
    
    if( g_chassisDataFromGM.rawStatus & (0x1<<MUC_GIMBAL_CHASSIS_CONNECT_STA_BIT) )
    {
        g_chassisDataFromGM.sta.status = GC_Normal;
    }else{
        g_chassisDataFromGM.sta.status = GC_Stop;
        
    }
    
    g_chassisDataFromGM.sta.count = 0;
}

void MucGimbalChassisLoopTask( void )
{
    g_chassisDataFromGM.sta.count ++;
    if(g_chassisDataFromGM.sta.count>MUC_GIMBAL_CHASSIS_LOST_COUNT)
    {
        g_chassisDataFromGM.sta.status = GC_Lost;
        g_chassisDataFromGM.rawFrontBack = 0;
        g_chassisDataFromGM.rawLeftRight = 0;
        g_chassisDataFromGM.rawRotate = 0;
        g_chassisDataFromGM.sta.count = MUC_GIMBAL_CHASSIS_LOST_COUNT+1;
        return ;
    }  
}

