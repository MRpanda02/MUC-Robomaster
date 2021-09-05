#ifndef _MUC_AUTO_SHOOT_H__
#define _MUC_AUTO_SHOOT_H__
#include "main.h"
#include "muc_remote.h"

#define MINIPC_BUF_LEN  40

void MucAutoShootInit( void );
void MucDmaCallbackMiniPCHandle( int* final ,uint16_t size );
void AutoShootingAndWindmill(CC_TypeDef *GimbalRC);
void Data_Deal(uint16_t  Size);
extern int Vision_Task_Windmill;
extern int Vision_Task_Autoshooting;
extern unsigned char g_MiniPCRxBuf[MINIPC_BUF_LEN];

#endif
