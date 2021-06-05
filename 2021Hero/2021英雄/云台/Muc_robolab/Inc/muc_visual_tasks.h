#ifndef MUC_VISUAL_TASKS_H__
#define MUC_VISUAL_TASKS_H__
#include "muc_hardware_uart.h"

#define MINIPC_BUF_LEN  100
#define MyAbs(x)  ( (x)>0?(x):-(x) )

typedef struct{
	float Yaw_orientation;
	float Pitch_orientation;
	float attackCount;
	
}Shooting_orientation;

extern Shooting_orientation Version_Shooting; 
extern uint8_t g_MiniPCRxBuf[MINIPC_BUF_LEN];
extern uint8_t out_view;
extern int8_t toward_follow;
void AutoShootingAndWindmill(void);
void view_Init(void);
extern void Dma_Callback_MS_Handle(Shooting_orientation*View,uint8_t* buff);
#endif
