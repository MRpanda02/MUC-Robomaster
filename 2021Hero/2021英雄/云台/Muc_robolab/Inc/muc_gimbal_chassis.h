#ifndef _MUC_GIMBAL_CHASSIS_H__
#define _MUC_GIMBAL_CHASSIS_H__
#include "main.h"

/*  */
#define MUC_GIMBAL_CHASSIS_CONNECT_STA_BIT 0
/*  */

#define GC_NORMAL (0X1<<MUC_GIMBAL_CHASSIS_CONNECT_STA_BIT)
#define GC_STOP (0X0<<MUC_GIMBAL_CHASSIS_CONNECT_STA_BIT)


#define MUC_GIMBAL_CHASSIS_LOST_COUNT 100

/* ״̬λ */

typedef enum
{
	GC_Lost=0,
	GC_Normal=1,
	GC_Stop=2, 
}gc_sta_t;

typedef struct{
    uint16_t count;
    gc_sta_t status;   
}gc_status_t;


typedef struct{
    int16_t rawFrontBack;
    int16_t rawLeftRight;
    int16_t rawRotate;
    uint16_t rawStatus;    
    gc_status_t sta;
}gimbal_to_chassis_t;

void Deal_Data_From_Gimbal( uint8_t * msgData );
void MucGimbalChassisLoopTask( void );

extern gimbal_to_chassis_t g_chassisDataFromGM;

#endif // _MUC_GIMBAL_CHASSIS_H__
