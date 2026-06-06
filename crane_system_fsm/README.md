# 起重器（升降 + 三总线舵机）联合控制状态机库使用说明

本库是专门为起重器搬运系统定制的非阻塞动作控制库，部署在独立文件夹 [crane_system_fsm](file:///d:/stm32hal/slewing%20ring/crane_system_fsm) 下。本库实现了一个主控系统状态机（Master FSM）和三个舵机的独立状态机，将高度升降步进电机和三路总线舵机有机结合，并为上位机微调预留了非阻塞接口。

---

## 1. 目录结构与文件职责

- [app_servo_fsm.h](file:///d:/stm32hal/slewing%20ring/crane_system_fsm/app_servo_fsm.h) / [app_servo_fsm.c](file:///d:/stm32hal/slewing%20ring/crane_system_fsm/app_servo_fsm.c)：
  **舵机状态机模块**。
  管理底座（ID:1）、抓斗对齐（ID:2）、爪子开合（ID:3）三个舵机独立的生命周期状态（`UNINIT`, `DETECTING`, `READY`, `MOVING`, `ERROR`），实现非阻塞的到位容差匹配和移动超时时间防卡死保护。
- [app_system_fsm.h](file:///d:/stm32hal/slewing%20ring/crane_system_fsm/app_system_fsm.h) / [app_system_fsm.c](file:///d:/stm32hal/slewing%20ring/crane_system_fsm/app_system_fsm.c)：
  **系统级联合状态机主模块**。
  核心心跳频率为 **50ms (20Hz)**，大幅降低串口总线硬件带宽开销，抗高频干扰能力极强。实现了十步**“顿戳微张式二次深挖”**颗粒（豆类）搬运工艺控制逻辑。同时导出上位机控制的对齐角度变量 `g_grab_align_pos_box` 及微调接口。

---

## 2. 调试参数宏定义清单

以下是 [app_system_fsm.h](file:///d:/stm32hal/slewing%20ring/crane_system_fsm/app_system_fsm.h) 中定义的完整可调参数清单，您可以直接在头文件中修改这些数值来适配您的实际硬件尺寸和机械行程：

```c
/* =============================================================================
 *                               调试参数宏定义清单
 * =============================================================================
 */

/* A. 升降高度位置参数 (步进电机 - 精度: 0.01mm) */
#define ELEV_HEIGHT_SAFE            (20.0f)     /* 搬运与旋转过程中的安全悬挂高度 (mm)，防拖地碰壁 */
#define ELEV_HEIGHT_GRAB            (150.0f)    /* 抓取货物时的下降高度 (mm) */
#define ELEV_HEIGHT_DROP            (100.0f)    /* 释放货物时的安全下降高度 (mm) */
#define ELEV_HEIGHT_MAX_LIMIT       (3500.0f)   /* 升降机构最大安全物理行程软限位 (mm) */

/* B. 水平角度及对齐参数 (总线舵机 1 & 2 - 范围: 0 ~ 1000) */
#define BASE_ROT_POS_START          (100)       /* 初始对准货物的底盘角度 (舵机1) */
#define BASE_ROT_POS_BOX            (600)       /* 货箱正上方的底盘旋转角度 (舵机1) */
#define BASE_ROT_MIN_LIMIT          (50)        /* 底座旋转安全最小限位 */
#define BASE_ROT_MAX_LIMIT          (950)       /* 底座旋转安全最大限位 */
#define GRAB_ALIGN_POS_START        (100)       /* 抓取时抓斗的初始对齐朝向 (舵机2) */
#define GRAB_ALIGN_POS_BOX          (400)       /* 货箱方向抓斗对准角度缺省值 (仅作为上电初值) */

/* C. 爪子完全开合位置 (总线舵机 3 - 范围: 0 ~ 1000) */
#define GRAB_CLAW_POS_OPEN          (200)       /* 爪子完全张开位置脉冲值 */
#define GRAB_CLAW_POS_CLOSE         (750)       /* 爪子完全闭合夹紧位置脉冲值 */

/* D. "顿戳微张式二次深挖" 专有工艺参数 */
#define ELEV_FIRST_DIG_DEPTH        (6.0f)      /* 第一次伴随浅压挖掘深度 (mm) */
#define ELEV_RETRACT_HEIGHT         (8.0f)      /* 第一次压完后，向上抬起释放硬应力的距离 (mm) */
#define ELEV_SECOND_DIG_DEPTH       (15.0f)     /* 第二次全力深入挖掘压入的绝对深度 (mm) */
#define CLAW_MID_CLOSE_POS          (450)       /* 第一次下压时，爪子半闭合聚拢角度 (脉冲) */
#define CLAW_MID_BACK_POS           (350)       /* 抬起释放应力时，爪子微幅向外退回张开的角度 (脉冲) */

/* E. 时间与速度配置参数 (S曲线及运动时间) */
#define STEPPER_SPEED_ELEV          (800)       /* 升降步进电机的移动速度 (RPM) */
#define STEPPER_ACC_ELEV            (15)        /* 升降电机加减速档位 (0 ~ 15，S曲线平滑防抖) */
#define BASE_ROT_DURATION_MS        (1800)      /* 底座大范围水平旋转时间 (ms)，缓慢旋转防晃 */
#define GRAB_ALIGN_DURATION_MS      (800)       /* 抓斗对齐旋转所用时间 (ms) */
#define GRAB_CLAW_DURATION_MS       (600)       /* 爪子张合运行所用时间 (ms) */
#define DELAY_GRAB_SETTLE_MS        (1000)      /* 爪子完全咬紧后，起吊前的物理稳定延时 (ms) */
#define DELAY_DROP_SETTLE_MS        (800)       /* 爪子完全张开后，物料脱开落稳的等待延时 (ms) */

/* F. 到位判定容差 */
#define TOLERANCE_STEPPER_MM        (1.5f)      /* 步进电机到位判定绝对差值容差 (mm) */
```

---

## 3. 工艺核心：“顿戳微张式二次深挖”

针对很小的豆子颗粒，本状态机在工步 4 中实施了如下控制时序，可大幅增加抓取饱和度并减小机构过载：
1. **初次下压**：升降降到预备高度后，爪子闭合至 **450**，升降电机同步下压 **6mm** 聚拢豆子。
2. **顿戳微张**：升降电机**向上抬起 8mm** 以释放阻力应力，同时爪子**向外微退至 350**。局部豆堆压力被释放，在重力作用下发生流动坍塌，顺畅填满爪内空腔。
3. **二次咬死**：等待 150ms 后，升降电机**向下全力深压 15mm**（探底），同时爪子以最大扭矩**闭合至 750** 彻底咬死提吊。

---

## 4. 后续系统集成移植指南 (待进一步操作)

为了不改变您当前写好的其他程序，我们只在新文件夹中生成了上述代码。当您过目完毕准备开始集成时，仅需按如下三步修改原有工程：

### 步骤 A: 定时中断接口挂载
在您的 `main.c` 里的 TIM3 定时器中断回调函数 `HAL_TIM_PeriodElapsedCallback` 中挂载计数递减与 50ms 更新分发：

```c
/* 引入系统状态机头文件 */
#include "../crane_system_fsm/app_system_fsm.h"

// 计时变量累加器
static uint8_t fsm_div_counter = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        // 1. 每 5 次 10ms 中断（即 50ms），将更新标志置 1 一次
        fsm_div_counter++;
        if (fsm_div_counter >= 5)
        {
            fsm_div_counter = 0;
            g_fsm_update_flag = 1;
        }
        
        // 2. 软件计时器以 10ms 步长自动递减
        if (g_servo_reply_timeout_counter > 0) {
            g_servo_reply_timeout_counter--;
        }
        if (g_sequence_settle_counter > 0) {
            g_sequence_settle_counter--;
        }
        if (g_settle_delay_counter > 0) {
            g_settle_delay_counter--;
        }
    }
}
```

### 步骤 B: 主循环 Process 与初始化挂载
在 `main.c` 的主入口中挂载初始化与轮询处理：

```c
int main(void)
{
    // ... 原有初始化代码 ...
    
    // 初始化起重状态机
    System_FSM_Init();
    
    while (1)
    {
        // 原有 Debug_CLI_Process() 等轮询
        Debug_CLI_Process();
        
        // 以 50ms 稳定节拍调用状态机核心 Process
        if (g_fsm_update_flag)
        {
            g_fsm_update_flag = 0;
            System_FSM_Process();
        }
    }
}
```

### 步骤 C: 上位机串口输入接口挂载 (CLI命令扩展)
在您的 `serial_servo_debug_cli.c` 的 `Debug_CLI_Process` 解析逻辑中，新增如下分支指令以供上位机连接：

```c
#include "../crane_system_fsm/app_system_fsm.h"

// 在交互控制命令行处理分支中添加：
if (strcmp(cmd, "set_align") == 0) {
    char *p1 = strtok(NULL, " ");
    if (p1) {
        int pos = atoi(p1);
        // 调用状态机留好的上位机动态输入接口
        System_FSM_SetGrabAlignPos((uint16_t)pos);
    } else {
        printf(">> 参数错误! 格式应为: set_align <pos>\r\n");
    }
}
else if (strcmp(cmd, "status") == 0) {
    // 统一状态一键打印回显
    static char status_buf[600];
    System_FSM_GetStatusString(status_buf, sizeof(status_buf));
    printf("%s", status_buf);
}
else if (strcmp(cmd, "seq") == 0) {
    // 一键运行连续联动搬运流程
    uint8_t ok = System_FSM_StartSequence();
    if (!ok) {
        printf(">> [警告]: 系统未处于 READY 状态，拒绝执行序列搬运动作。\r\n");
    }
}
else if (strcmp(cmd, "step") == 0) {
    char *p1 = strtok(NULL, " ");
    if (p1) {
        int step_num = atoi(p1);
        // 调用状态机单步执行接口
        uint8_t ok = System_FSM_StartSingleStep((uint8_t)step_num);
        if (!ok) {
            printf(">> [警告]: 系统未就绪或步骤号 %d 错误，拒绝执行单步动作。\r\n", step_num);
        }
    } else {
        printf(">> 参数错误! 格式应为: step <1~10>\r\n");
    }
}
else if (strcmp(cmd, "estop") == 0) {
    // 紧急停车
    System_FSM_EmergencyStop();
    printf(">> [ESTOP]: 起重器已紧急停车并断电力矩！\r\n");
}
```
