2019年中央民大参北斗战队 RoboLab Template Code

### 全国大学生机器人竞赛Robomaster — 嵌入式代码工程模板

> 芯片型号:STM32F427HII
> STM32CubeMX Ver:5.0.1
> Keil5 for MDK Ver:5.23.0
>
> ------
>
> ##### 硬件接口说明：
>
> CAN1: Pitch Motor(0X205),Yaw Motor(0X206), Stir Motor(0X207)
> CAN2: Nothing
> UART1: Remote Control DR16 Receiver(DT7)
> UART3: Referee System Student Interface
> UART7: Use For Printf Debug Message

[TOC]

------

#### 2019年04月16日 03:00	V0.0.1    周旺生

- 串口7支持Printf输出
- 串口1可以读取遥控器的数据
- 定时器6正常工作 中断时间:1ms
- CAN1/CAN2正常工作,CAN接收数据使用了不同的FIFO. CAN1 FIFO0  &&  CAN2 FIFO1
- 云台工作正常

------

#### 2019年04月16日 17:30	V0.0.2    周旺生

- 遥控器正常控制云台的Pitch和Yaw的移动
- 启用了Flash功能，如果校准云台，则需要在开机之前按住白色按钮进入校准模式。具体操作详情，请查看校准说明。
- 添加了主进程运行时间统计功能，使用Tim14.
- 添加了蜂鸣器的功能.
------

#### 2019年04月21日 20:42	V0.0.3    周旺生

- 添加了对裁判系统读取的功能(2018版裁判系统) -- 未测试
------

#### 2019年04月21日 22:05	V0.0.4    周旺生

- 裁判系统数据正常读取 -- 已测试
- 添加了超级电容的相关文件
- 添加了系统复位函数 void MucSystemReboot( void );
- 优化了系统校准的功能
------

#### 2019年04月27日 09:26	V0.0.5    周旺生

- 添加了云台电机的位置ID  Pitch Motor(0X205),Yaw Motor(0X206)
- 修改了加速度计计算的轴向:Z轴改为X轴
- 删除了云台对YAW轴的角度限制
- 屏蔽了底盘的计算和输出
- 此代码已可以在19版步兵上 正确初始化云台
------

#### 2019年04月27日 10:23	V0.0.6    周旺生

- 添加了拨轮电机的PID控制,未加卡弹反转。Stir Motor(0X207)
------

#### 2019年04月27日 11:15	V0.0.7    周旺生

- 添加了摩擦轮电机的PID控制。Shoot Motor Left(0X201) Right(0X202)
------

#### 2019年04月27日 12:15	V0.0.8    周旺生

- 支持读取2019版裁判系统
------

#### 2019年04月27日 13:25	V0.0.9    周旺生

- 支持遥控器S2的第三档停机的支持	
------

#### 2019年04月27日 16:52	V0.0.10    杨斯元

- 云台控制数据处理分离，测试可用,不稳需调
- 底盘控制数据处理分离，添加了360部分并与跟随部分整合，未测试
- 摩擦轮数据处理分离完成，未测试
------

#### 2019年04月27日 19:56	V0.0.11    周旺生

- 把串口6作为和MINIPC的通信接口，DMA接收，数据最大长度60.波特率9600.
------

#### 2019年04月27日 22:34	V0.0.12    杨斯元

- 拨轮验证成功
- 摩擦轮控制实现分档，按住右键2秒左右切换
------

#### 2019年04月28日 11:31	V0.0.13    杨斯元

- 枪口热量限制部分移植完成,未测试
- 部分代码的可读性改善
------

#### 2019年04月28日 12:17	V0.0.14    杨斯元

- 键盘和鼠标读取回归REMOTE.C
------

#### 2019年04月28日 14:39	V0.0.15    杨斯元

- 视觉部分键盘控制及处理添加，未测试
------

#### 2019年04月28日 14:39	V0.0.16    杨斯元

- 添加视觉离线判断
- 修正遥控器控制参数
------

#### 2019年04月29日 12:06	V0.0.17    周旺生

- 完成地盘控制板和云台的通信(gimbal:CAN2 CHASSIS:CAN2 ) 0X320
- CHASSIS指示灯说明: 绿灯闪烁表示地盘正常模式,地盘电机有PID计算;红灯闪烁表示停止模式,地盘电机无PID计算;红灯常亮绿灯闪烁，表示未收到云台发来的数据，检查CAN线连接
------

#### 2019年05月01日 20:31	V0.0.18    杨斯元

- 云台参数调整
- 视觉初步功能
------

#### 2019年05月02日 23:33	V0.0.19    杨斯元

- 先做个保存
------

#### 2019年05月03日 14:36	V0.0.20    杨斯元

- 先做个保存
------

#### 2019年05月04日 04:38	V0.0.21    杨斯元

- 视觉基本达成,但是启动还是比较慢。参数全是玄学。主要问题还是接收数据的问题。
- 添加了卡尔曼滤波器部分，有时间弄二阶的且最好使用FPU运算单元
- 云台控制前后顺序的改变，先视觉后遥控
------

#### 2019年05月04日 13:03	V0.0.22    杨斯元

- 视觉校验BUG处理,对面移动太快时稍有延迟，其他七七八八，稳定性可以再进步
------

#### 2019年05月04日 17:14	V0.0.23    杨斯元

- 采用了新的视觉控制方案，还是学长强
- 增加了KD控制
------

#### 2019年05月04日 17:14	V0.0.24    杨斯元

- 正儿八经的卡尔曼滤波函数编写
------

#### 2019年05月08日 20:50	V0.0.25    杨斯元

- 视觉部分代码控制 按V开始
- 摩擦轮鼠标右键开启，开启后键盘F变速
- 射击模式键盘B改变控制
- 删除了三连发模式，感觉没有必要
------

#### 2019年05月10日 21:08	V0.0.26    杨斯元

- 接收底盘发送的枪口热量数据于shoot_control_consulation结构体中，具体效果未测试
- 删除云台板子裁判系统部分，交给底盘
- 开启can2的fifo1数据缓存区，现在can2收发都可以
------

#### 2019年05月13日 22:24	V0.0.27    杨斯元

- 枪口热量相关数据底盘发送接收  can2 0x141
- 枪口热量接收数据处理
- 裁判系统接收任务屏蔽
------

#### 2019年05月14日 14:19	V0.0.28    周旺生&&杨斯元

- 云台磁力计校准由Uint16改为Uint32，原因这台车误差较大，5000次累加之后，超界
------

#### 2019年05月16日 16:08	V0.0.29    杨斯元

- 做个保存
------

#### 2019年05月16日 19:04	V0.0.30    杨斯元

- 自旋时移动已解决
- 打风车控制逻辑构建，未测试
- 自瞄键盘控制成功 Shift and V = AutoShooting || Ctrl and V = Windmill || V = Close
------

#### 2019年05月17日 12:24	V0.0.31    杨斯元

- 底盘跟随优化
------

#### 2019年05月18日 18:51	V0.0.32    杨斯元

- 摩擦轮优化
------

#### 2019年05月19日 11:37	V0.0.33    杨斯元

- 系统状态检查任务中漏洞修补
- 遥控器离线处理，如接收器脱落等情况，车关闭
- 激光与摩擦轮同步
- 拨轮与摩擦轮同步
- 射击模式改变 B按住超过0.5s才可以执行
------

#### 2019年05月19日 20:05	V0.0.34    杨斯元

- QE相反问题修复
- 摩擦轮开启时间缩短
- 视觉代码优化
------

#### 2019年05月20日 21:07	V0.0.35    杨斯元

- 模糊整定PID算法尝试，先做个保存
------

#### 2019年05月22日 11:28	V0.0.36    杨斯元

- 底盘跟随最终优化完成
- 模糊pid删除，STM32处理不了
------

#### 2019年05月22日 13:28	V0.0.37    杨斯元

- 视觉代码优化
------

#### 2019年05月23日 08:20	V0.0.38    杨斯元

- 自旋控制逻辑优化，移动速度慢，停止速度快
- 底盘加速时，斜坡函数中间量加大
- 视觉发现换个场地就不行，有待优化
------

#### 2019年05月24日 06:34	V0.0.39    梁永才&& 杨斯元

- 视觉yaw轴电控部分完成
------

#### 2019年05月24日 17:13	V0.0.40    杨斯元

- QE自旋优化
- 枪口热量限制BUG解决
- 摩擦轮BUG解决
- 自旋进入PID运算
- 自瞄与自旋开启优化 PS:下一版准备右键自瞄，摩擦轮换地方
------

#### 2019年05月24日 17:13	V0.0.41    杨斯元

- 之前的自瞄开启与自旋优化有问题，返回
- 在使用Cubemax之前，保个存
------

#### 2019年05月24日 22:05	V0.0.42    杨斯元

- 视觉Yaw Pitch完成
- 弹仓盖完成，测试成功
------