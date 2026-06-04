=============================================================================
             Emm_V5.0 闭环步进电机驱动库 (STM32 HAL库适配版)
=============================================================================

一、文件组成：
  - Emm_V5.h : 驱动库头文件，包含宏定义、系统参数枚举、电机句柄结构体及函数声明。
  - Emm_V5.c : 驱动库源文件，包含所有电机控制协议的封装、阻塞式参数读取及异步帧解析。

二、编码格式：
  - GB2312 (中文注释，完美兼容 Keil/IAR 等 Windows 下的嵌入式 IDE，无中文乱码)。

三、使用说明：

1. 初始化：
   在 main.c 中引入头文件：
   #include "Emm_V5.h"

   实例化电机并初始化：
   Emm_V5_Motor motor1;
   Emm_V5_Init(&motor1, &huart2, 1); // 绑定串口2，电机总线地址为1

2. 使能电机：
   Emm_V5_En_Control(&motor1, true, false); // 使能电机，不启用同步

3. 运动控制：
   - 速度模式：
     Emm_V5_Vel_Control(&motor1, EMM_CW, 1000, 10, false); // 顺时针，1000 RPM，加速度10
   - 位置模式 (相对运动)：
     Emm_V5_Pos_Control(&motor1, EMM_CW, 1000, 10, 3200, false, false); // 顺时针，相对运动3200脉冲
   - 位置模式 (绝对运动)：
     Emm_V5_Pos_Control(&motor1, EMM_CW, 1000, 10, 0, true, false); // 运动到绝对零点位置
   - 立即停止：
     Emm_V5_Stop_Now(&motor1, false);

4. 回零操作：
   - 触发单圈就近回零：
     Emm_V5_Origin_Trigger_Return(&motor1, 0, false);
   - 触发无限位碰撞回零：
     Emm_V5_Origin_Trigger_Return(&motor1, 2, false);
   - 修改回零参数 (例如碰撞回零：速度 50RPM，超时 10000ms，检测转速 10RPM，检测电流 300mA，检测时间 100ms)：
     Emm_V5_Origin_Modify_Params(&motor1, true, 2, EMM_CCW, 50, 10000, 10, 300, 100, false);

5. 状态读取：
   - 阻塞查询模式 (适用于简单流程控制，会产生几毫秒的阻塞延迟)：
     if (Emm_V5_Read_Position_Blocking(&motor1)) {
         float current_angle = motor1.real_pos; // 获取当前角度
     }
     if (Emm_V5_Read_Speed_Blocking(&motor1)) {
         float current_speed = motor1.real_vel; // 获取当前转速 (RPM)
     }

   - 异步非阻塞解析模式 (推荐，适用于高实时性系统)：
     当您配置了串口的空闲中断 (IDLE) 或 DMA 接收，在接收到完整的一帧数据包后，直接调用：
     Emm_V5_Parse_Frame(&motor1, rx_buffer, rx_length);
     解析器会自动校验地址与校验码，并自动解算数据更新到 `motor1.real_pos`, `motor1.real_vel` 等对应的结构体变量中。
