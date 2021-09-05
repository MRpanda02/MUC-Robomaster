#include "muc_test.h"
#include "muc_hardware_uart.h"
#include "can.h"
#include "muc_stmflash.h"
#include "muc_hardware_stmflash.h"

int16_t g_testNum1=0;
int16_t g_testNum2=0;

void TestLoop( int dTimes )
{
//    static int count=0;
    uint32_t* bufferTest[20];
    //HAL_Delay(dTimes);
    //STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)"ttr345678909876543210",5);//写入一个字 
    STMFLASH_Read(FLASH_SAVE_ADDR,(uint32_t*)bufferTest,5);
    printf("%s\n",(char *)bufferTest);
}

void TestLoopOnce()
{
//    u32 WriteData[20];
//    STMFLASH_Write(FLASH_VAL_ADDR,&WriteData,FlashData_LEN);//写入一个字 
    // STMFLASH_Read
}

