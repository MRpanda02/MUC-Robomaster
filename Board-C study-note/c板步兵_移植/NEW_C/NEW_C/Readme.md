###RoboMaster_MUC_C

####### 注意事项 ########

--  muc_imu.c/h、muc_gimbal.c 文件已修改，移植请注意
--  串口输出的 UART7 用 UART5 代替
--  STM32F407IG 共有12个页分区 FLASH_SECTOR_12 ~ FLASH_SECTOR_23已经注释 



####### 版本说明 ########

### 2020.4.2  16:25  V0.1.0 草木灰

-- 移植C板硬件配置
-- 移植步兵Hardware、Robolab
-- spi5报错，没有测试

### 2020.4.11  16:39  V0.1.1 草木灰

-- 原步兵程序用于串口输出的 UART7 用 UART5 代替
-- 定时器 TIM8 无法修改
-- CAN 通信参数无法修改 9.4.1 ，10.3.1
-- INS任务有os移植报错
-- STM32F407IG 共有12个页分区 FLASH_SECTOR_12-23已经注释 

### 2020.4.18  14:27  V0.2.0 草木灰

-- INS移植成功，全部功能实现，云台函数已经被修改
-- ”pitch_angle_vel_ecd = IMU_Real_Data.Gyro_Z;    //云台PITCH速度“
	可能需要将角度转化成速度

