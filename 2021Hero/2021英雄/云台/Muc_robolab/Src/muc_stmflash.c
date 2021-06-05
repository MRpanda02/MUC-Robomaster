#include "muc_stmflash.h"
#include "muc_hardware_stmflash.h"
#include "muc_hardware_key.h"
#include "muc_hardware_beep.h"

flash_data_t g_flash_data;


uint8_t CheckDataUseful( void )
{
    int32_t CRCValue;
    
    if( (FlashData_Header == g_flash_data.head)&&(g_flash_data.dataLen == FlashData_LEN) )
    {
        CRCValue = (g_flash_data.XGygoMpuCaliValue+g_flash_data.YGygoMpuCaliValue+ \
        g_flash_data.ZGygoMpuCaliValue+g_flash_data.GimbalPitchMidPos+g_flash_data.GimbalYawMidPos)%0XFAFB;  
        
        if(g_flash_data.checkSum == CRCValue )
        {
            return 1;
        }    
    }
    
    return 0;
}

void SetUsefulData( flash_data_t *flash_data )
{
    if( FlashData_Header==flash_data->head )
    {
        g_flash_data.head = FlashData_Header;
        g_flash_data.dataLen = FlashData_LEN;
        
        g_flash_data.GimbalPitchMidPos = flash_data->GimbalPitchMidPos;
        g_flash_data.GimbalYawMidPos = flash_data->GimbalYawMidPos;
        g_flash_data.XGygoMpuCaliValue = flash_data->XGygoMpuCaliValue;
        g_flash_data.YGygoMpuCaliValue = flash_data->YGygoMpuCaliValue;
        g_flash_data.ZGygoMpuCaliValue = flash_data->ZGygoMpuCaliValue;
        
        g_flash_data.checkSum = (g_flash_data.XGygoMpuCaliValue+g_flash_data.YGygoMpuCaliValue+ \
        g_flash_data.ZGygoMpuCaliValue+g_flash_data.GimbalPitchMidPos+g_flash_data.GimbalYawMidPos)%0XFAFB;  
        
        STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)&g_flash_data,FlashData_LEN);
       
    }
    
}

uint8_t GetCaliDataSta( uint8_t pos )
{
    
    return (g_flash_data.status&(0X1<<pos)) && CheckDataUseful();
}


/* 清空云台的校准数据 */
void CleanGimbalCaliData( void )
{
    g_flash_data.status = g_flash_data.status & (~(0x1<<GIMBAL_CALI_DATA));
    SetUsefulData(&g_flash_data);
}

/* 清空IMU的校准数据 */
void CleanIMUCaliData( void )
{
    g_flash_data.status = g_flash_data.status & (~(0x1<<MPU6050_CALI_DATA));
    SetUsefulData(&g_flash_data);
}


void CheckConfigParams( void )
{
    // 从flash中读取信息
	STMFLASH_Read(FLASH_SAVE_ADDR,(uint32_t*)&g_flash_data,FlashData_LEN);
    
    if( CheckDataUseful() )
    {
        // 数据有效
        /* 是否需要重新配置云台位置信息 */
        if(GetKeySta()==BtnPressDown)
        {
            bibiSing(1);
            CleanGimbalCaliData();
            CleanIMUCaliData();
            while(GetKeySta()==BtnPressDown)
            {
                HAL_Delay(10);
            }
        }
        return ;
    }
    
    g_flash_data.head = FlashData_Header;
    g_flash_data.dataLen = FlashData_LEN;
    
    g_flash_data.status = 0;
    
    g_flash_data.GimbalPitchMidPos = 0;
    g_flash_data.GimbalYawMidPos = 0;
    g_flash_data.XGygoMpuCaliValue = 0;
    g_flash_data.YGygoMpuCaliValue = 0;
    g_flash_data.ZGygoMpuCaliValue = 0;
    
    g_flash_data.checkSum = (g_flash_data.XGygoMpuCaliValue+g_flash_data.YGygoMpuCaliValue+ \
    g_flash_data.ZGygoMpuCaliValue+g_flash_data.GimbalPitchMidPos+g_flash_data.GimbalYawMidPos)%0XFAFB; 
    
    SetUsefulData(&g_flash_data);
    
}

void UpdateConfigParamsForMpu6050( int16_t XGygoMpuCaliValue, int16_t YGygoMpuCaliValue, int16_t ZGygoMpuCaliValue )
{
    g_flash_data.XGygoMpuCaliValue = XGygoMpuCaliValue;
    g_flash_data.YGygoMpuCaliValue = YGygoMpuCaliValue;
    g_flash_data.ZGygoMpuCaliValue = ZGygoMpuCaliValue;
    
    g_flash_data.status = g_flash_data.status | (0X1<<MPU6050_CALI_DATA);
    
    g_flash_data.checkSum = (g_flash_data.XGygoMpuCaliValue+g_flash_data.YGygoMpuCaliValue+ \
    g_flash_data.ZGygoMpuCaliValue+g_flash_data.GimbalPitchMidPos+g_flash_data.GimbalYawMidPos)%0XFAFB; 
    
    SetUsefulData(&g_flash_data);
    
}

void UpdateConfigParamsForGimbal( int16_t GimbalPitchMidPos, int16_t GimbalYawMidPos )
{
    g_flash_data.GimbalPitchMidPos = GimbalPitchMidPos;
    g_flash_data.GimbalYawMidPos = GimbalYawMidPos;
    
    g_flash_data.status = g_flash_data.status | (0X1<<GIMBAL_CALI_DATA);
    
    g_flash_data.checkSum = (g_flash_data.XGygoMpuCaliValue+g_flash_data.YGygoMpuCaliValue+ \
    g_flash_data.ZGygoMpuCaliValue+g_flash_data.GimbalPitchMidPos+g_flash_data.GimbalYawMidPos)%0XFAFB; 
    
    SetUsefulData(&g_flash_data);
    
}

void GetMpu6050CaliData( int16_t *XGygoMpuCaliValue, int16_t *YGygoMpuCaliValue, int16_t *ZGygoMpuCaliValue )
{
    *XGygoMpuCaliValue = g_flash_data.XGygoMpuCaliValue;
    *YGygoMpuCaliValue = g_flash_data.YGygoMpuCaliValue;
    *ZGygoMpuCaliValue = g_flash_data.ZGygoMpuCaliValue;
}

void GetGimbalCaliData( int16_t *gimbalPitchMidPos, int16_t *gimbalYawMidPos)
{
    *gimbalPitchMidPos = g_flash_data.GimbalPitchMidPos;
    *gimbalYawMidPos = g_flash_data.GimbalYawMidPos;
 
}




