# 起重机协同状态机与 Emm_V5 电机控制系统操作指南 (完整版)

本控制系统是一套为 STM32F103 开发的非阻塞起重机主控状态机（Master FSM）库。系统彻底弃用了冗余的 ZDT 步进电机旧库，改为全面采用 Emm_V5 闭环步进升降电机与总线旋转舵机联控的控制架构。

---

## 1. 目录结构与文件职责

*   **[app_servo_fsm.h](file:///d:/stm32hal/slewing%20ring/crane_system_fsm/app_servo_fsm.h) / [app_servo_fsm.c](file:///d:/stm32hal/slewing%20ring/crane_system_fsm/app_servo_fsm.c)**：
    **舵机管理模块**。负责旋转（ID1）、对齐（ID2）和抓爪（ID3）三个总线舵机的非阻塞状态维护（包含 `UNINIT`, `DETECTING`, `READY`, `MOVING`, `ERROR` 等），支持指令异步触发发送与非阻塞到位检测。
*   **[app_system_fsm.h](file:///d:/stm32hal/slewing%20ring/crane_system_fsm/app_system_fsm.h) / [app_system_fsm.c](file:///d:/stm32hal/slewing%20ring/crane_system_fsm/app_system_fsm.c)**：
    **系统主状态机模块**。以固定的 50ms（20Hz）分频节拍更新，负责控制整机 10 个工步动作的非阻塞流转，并提供上位机动态对齐宏参数 `g_grab_align_pos_box` 及单步/自动调试的流转判定。

---

## 2. 出厂配置参数定义清单

在 [app_system_fsm.h](file:///d:/stm32hal/slewing%20ring/crane_system_fsm/app_system_fsm.h) 中预留了以下核心工艺参数定义，您可以根据机械结构的实际行程和响应时间对其进行微调：

```c
/* A. 升降高度位置参数 (单位: mm) */
#define ELEV_HEIGHT_SAFE            (20.0f)     /* 提升至安全回缩高度，防止底座转动时产生碰撞 */
#define ELEV_HEIGHT_GRAB            (150.0f)    /* 抓取落料斗时的预备下压高度 */
#define ELEV_HEIGHT_DROP            (100.0f)    /* 放置货物至货箱上方的安全落料高度 */
#define ELEV_HEIGHT_MAX_LIMIT       (3500.0f)   /* 丝杠模组物理最高安全限位行程 */

/* B. 舵机旋转角度参数 (参数范围: 0 ~ 1000) */
#define BASE_ROT_POS_START          (100)       /* 底座转向初始抓取点偏角 (舵机1) */
#define BASE_ROT_POS_BOX            (600)       /* 底座转向卸货货箱点偏角 (舵机1) */
#define BASE_ROT_MIN_LIMIT          (50)        /* 底座舵机顺时针最小限位角 */
#define BASE_ROT_MAX_LIMIT          (950)       /* 底座舵机逆时针最大限位角 */
#define GRAB_ALIGN_POS_START        (100)       /* 抓取点时夹爪水平对齐偏角 (舵机2) */
#define GRAB_ALIGN_POS_BOX          (400)       /* 卸货点时夹爪对齐缺省值，由变量 g_grab_align_pos_box 动态改写 */

/* C. 夹爪开合角度参数 (参数范围: 0 ~ 1000) */
#define GRAB_CLAW_POS_OPEN          (200)       /* 夹爪完全张开时的目标偏角 (舵机3) */
#define GRAB_CLAW_POS_CLOSE         (750)       /* 夹爪全力咬合时的目标偏角 (舵机3) */

/* D. "顿戳微张式深挖" 特色工艺参数 */
#define ELEV_FIRST_DIG_DEPTH        (6.0f)      /* 第一阶段下压深挖距离 (mm) */
#define ELEV_RETRACT_HEIGHT         (8.0f)      /* 回缩微张卸荷距离 (mm) */
#define ELEV_SECOND_DIG_DEPTH       (15.0f)     /* 第二阶段全力咬合深挖下压距离 (mm) */
#define CLAW_MID_CLOSE_POS          (450)       /* 第一阶段浅挖时夹爪半闭合目标角 (舵机3) */
#define CLAW_MID_BACK_POS           (350)       /* 回缩微张时夹爪稍微张开的目标角 (舵机3) */

/* E. 运动时间与速度参数 */
#define STEPPER_SPEED_ELEV          (800)       /* 丝杠升降电机的移动设定速度 (RPM) */
#define STEPPER_ACC_ELEV            (15)        /* 升降电机的 S 型加减速曲线斜率档位 */
#define BASE_ROT_DURATION_MS        (1800)      /* 底座转向预估最大耗时 (ms) */
#define GRAB_ALIGN_DURATION_MS      (800)       /* 夹爪水平对准预估最大耗时 (ms) */
#define GRAB_CLAW_DURATION_MS       (600)       /* 夹爪全开/全闭动作最大耗时 (ms) */
#define DELAY_GRAB_SETTLE_MS        (1000)      /* 夹爪全力闭合抓紧后，维持力矩的等待时间 (ms) */
#define DELAY_DROP_SETTLE_MS        (800)       /* 放置货物手爪张开后，等待颗粒倾倒完毕的静默时间 (ms) */

/* F. 位移判定公差 */
#define TOLERANCE_STEPPER_MM        (1.5f)      /* 步进升降高度的允许到位判定误差 (mm) */
```

---

## 3. 全自动抓取工艺（10个工步）与“顿戳微张式深挖”工艺

输入 `seq` 指令触发后，起重机自动流转执行以下 10 个动作工步：
1.  **工步 1 (SYS_TASK_STEP_1_RAISE_SAFE)**：升降滑块自动移至安全高度（20mm），避开转弯时的机械障碍。
2.  **工步 2 (SYS_TASK_STEP_2_OPEN_CLAW)**：底座转向初始抓取点，同时夹爪全力张开（200），对齐对准角度。
3.  **工步 3 (SYS_TASK_STEP_3_DESCEND_GRAB)**：滑块垂直下降至抓取预备点（150mm）。
4.  **工步 4 (SYS_TASK_STEP_4_1_FIRST_DIG)**：**[微挖下压]** 夹爪半闭合至 **450**（浅挖），滑块在抓取预备点上继续下压 **6mm** 戳入料斗。
5.  **工步 5 (SYS_TASK_STEP_4_2_RETRACT_SETTLE)**：**[抬升微张]** 滑块上抬 **8mm**，同时夹爪稍退张开至 **350**，使料斗内结拱卡死的豆子坍塌以填满斗瓣。
6.  **工步 6 (SYS_TASK_STEP_4_3_SECOND_DIG_LOCK)**：**[深挖死咬]** 滑块再次全力深压下挫 **15mm**，夹爪闭合至最大值 **750** 咬死货物，延时稳定。
7.  **工步 7 (SYS_TASK_STEP_5_RAISE_SAFE)**：夹爪保持抓死，滑块快速回缩提升至安全高度（20mm）。
8.  **工步 8 (SYS_TASK_STEP_6_ROTATE_TO_BOX)**：底座转向卸货货箱点（ID1 移至 600），夹爪自适应角度对齐。
9.  **工步 9 (SYS_TASK_STEP_7_DESCEND_DROP)**：滑块垂直下降至放货高度（100mm）。
10. **工步 10 (SYS_TASK_STEP_8_RELEASE_CLAW)**：夹爪完全张开，静待物料倒出后，滑块升回安全高度，底座返回抓取点，系统重归 READY。

---

## 4. 全局工作模式运行机制

本系统将运行模式解耦，并在到位流转函数 `transition_to_next_step` 中融入了如下双模拦截逻辑：

### A. 自动运行模式 (SYS_MODE_AUTO)
*   **出厂与上电默认状态**：上电复位后，系统默认工作在此模式。
*   **控制逻辑**：接收到 `seq` 命令后，主循环状态机每 50ms 自动在后台流转跳转，连续、不停顿地执行完所有 10 步动作直到复位。

### B. 手动调试模式 (SYS_MODE_MANUAL)
*   **切换方式**：在调试控制台输入 `mode manual` 指令随时切入。
*   **控制逻辑**：此模式下，即使您输入 `seq` 触发了循环，状态机也会在**每一个单一工步动作执行完毕到位后，立刻进行强制挂起拦截**，系统退回 `READY`，等待您的下一步指令，适合前期调试。

---

## 5. 调试控制台 CLI 串口指令大全 (波特率 115200)

请在串口助手（USART3 接口）中发送以下指令（请勾选“发送新行/回车”）：

### 状态机与电机核心控制：
*   **`seq`**：启动一轮全自动抓取搬运工艺序列（若在手动模式下，仅触发执行第一步）。
*   **`next`**：**[顺次单步指令]** 自动推算并触发运行下一步动作。例如连续输入 `next` 发送，即可顺次一步一步驱动起重机完成全部动作，调试极为丝滑。
*   **`step <1~10>`**：强制触发执行第 1 至第 10 步指定的单一工步动作，到位后自动暂停拦截。
*   **`mode <auto/manual>`**：动态切换系统工作模式（默认 auto）。
*   **`motor_pos <pos> <speed>`**：**[Emm_V5 步进控制]** 控制升降电机向绝对位置 `pos` (mm) 位移，最大速度 `speed` (RPM)。
*   **`set_align <pos>`**：动态调整卸货对准偏角变量 `g_grab_align_pos_box` (范围 0~1000)。
*   **`status`**：遥测打印系统运行模式、前级状态、实时高度、底座偏角、对齐角度、夹爪角度及它们的在线通信状况。

### 舵机独立硬件操作：
*   **`pos <id> <pos> <time>`**：控制指定 ID (1,2,3) 的总线舵机在 `time` (ms) 内转动到目标位置 `pos` (0~1000)。
*   **`read <id>`**：强行读回指定 ID 舵机的当前角度、当前电压以及芯片温度。
*   **`stop <id>`**：下发紧急制动指令。
*   **`free <id>`**：释放舵机力矩，进入手动示教模式。
*   **`lock <id>`**：锁定舵机力矩。

---

## 6. 工程挂载与移植细节说明

为了使您的 Keil 工程能够正确整合新驱动并支持状态机调度，代码中已完成以下改动和挂载：

### A. 移除与添加编译文件 (Keil 工程树操作)
1.  **废除旧 ZDT 电机库**：在项目组中找到 `Control.c` 和 `Data.c` 并移出编译组（Remove）。
2.  **添加 Emm_V5 与状态机文件**：
    - 将 `Emm_V5_stepper` 组中的 `Emm_V5.c` 与 `app_stepper_ctrl.c` 添加进编译。
    - 将 `crane_system_fsm` 组中的 `app_servo_fsm.c` 与 `app_system_fsm.c` 添加进编译。

### B. 时钟周期中断服务函数挂载 (`main.c`)
在 `main.c` 的 TIM3 中断服务中已正确配置 10ms 递减及 50ms 降频节拍分发：
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        /* 1. 软件定时计数器递减 (10ms 节拍) */
        if (g_servo_reply_timeout_counter > 0) {
            g_servo_reply_timeout_counter--;
        }
        if (g_sequence_settle_counter > 0) {
            g_sequence_settle_counter--;
        }
        if (g_settle_delay_counter > 0) {
            g_settle_delay_counter--;
        }

        /* 2. 降频分频，当累加到 50ms (20Hz) 时将刷新标志置位 */
        static uint8_t tick_divider = 0;
        tick_divider++;
        if (tick_divider >= 5)
        {
            tick_divider = 0;
            g_fsm_update_flag = 1;
        }
    }
}
```

### C. 主循环状态机轮询挂载 (`main.c`)
在 `main.c` 循环的 `while (1)` 中已剔除阻塞语句，改写为以下非阻塞处理：
```c
  while (1)
  {
      /* 1. 串口 3 命令行交互处理器 */
      Debug_CLI_Process();

      /* 2. 状态机 50ms 节拍非阻塞查询更新 */
      if (g_fsm_update_flag)
      {
          g_fsm_update_flag = 0;
          System_FSM_Process();
      }
  }
```
