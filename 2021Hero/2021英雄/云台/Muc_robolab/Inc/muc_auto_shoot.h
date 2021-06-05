#ifndef _MUC_AUTO_SHOOT_H__
#define _MUC_AUTO_SHOOT_H__
#include "main.h"

#define MINIPC_BUF_LEN  100

void MucAutoShootInit( void );
void MucDmaCallbackMiniPCHandle( char* final ,uint16_t size );
//void AutoShootingAndWindmill(void);
void Data_Deal(uint16_t  Size);
extern int Vision_Task_Windmill;
extern int Vision_Task_Autoshooting;
//extern unsigned char g_MiniPCRxBuf[MINIPC_BUF_LEN];

#endif
