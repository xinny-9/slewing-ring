# 串口总线舵机 STM32F103C8T6 HAL 库裸机驱动说明书 (双版本分发版)

本驱动库专为 **STM32F103C8T6** 基于 **STM32CubeMX HAL 库** 开发，支持裸机（Bare-metal）高稳定运行。移除了对 FreeRTOS 等操作系统的依赖，针对您的**单线半双工 (TX引脚外接上拉)** 硬件电路以及传统的**双使能引脚 GPIO 控制电路**提供了完美适配。

为了给您提供最适合的开发体验，我将驱动库整理并重构为了 **两套相互独立、分开放置** 的版本，您可以根据项目需求自由选择：

---

## 📂 驱动库版本对比与选择指南

它们被存放在 `serial_servo_f103_drv/` 下的两个独立子文件夹中：

| 文件夹名称 | 版本定位 | 核心工作原理 | 优缺点对比 | 适用场景 |
| :--- | :--- | :--- | :--- | :--- |
| **`fast_irq_version/`** | **高性能直接中断拦截版** | 在 `stm32f1xx_it.c` 的串口 ISR 头部第一行直接强行拦截接收字节并送入状态机，处理完后直接 `return` 退出中断。 | **优点**：极低开销、极致响应。规避了 HAL 库臃肿的接收回调分配流程，在 115200 高波特率下极佳稳定。<br>**缺点**：需要手动微调 `stm32f1xx_it.c` 的串口中断函数。 | **极度推荐**。多舵机通信、对实时性和稳定性要求极高、裸机控制的主力版本。 |
| **`standard_hal_version/`** | **标准 HAL 接收回调版** | 采用 HAL 库标准的 `HAL_UART_Receive_IT` 异步接收 1 字节，并在标准的 `HAL_UART_RxCpltCallback` 接收完成回调中解析并循环挂载接收。 | **优点**：100% 契合 STM32 标准 HAL 开发规范，不修改 `stm32f1xx_it.c`。<br>**缺点**：HAL 中断调度开销较大，极高负荷下抗抖动性略逊于拦截版。 | 追求标准 HAL 开发规范、不希望侵入修改默认生成的中断服务函数、调试轻量应用的场景。 |

> [!TIP]
> 无论您选择哪套版本，底层的通信时序控制、8ms SysTick 自适应超时防死锁机制、双线电回波消除算法，以及应用层的控制 API 都是完全相同的。
> 两套驱动的 `serial_servo_hal.h` 中均已将 **`SERIAL_SERVO_USE_SINGLE_WIRE`** 宏设为 **`1`**（默认开启内置单线半双工模式）。

---

## 🖥️ 调试上位机方案一：Web 网页版极速直连上位机 (神仙推荐)

为了让您能够以最快的速度、在不需要任何复杂下位机中转的硬件状况下调试舵机，我已经为您在驱动的根目录下生成了一款基于现代 HTML5 Web Serial API 编写的**网页版直连调试上位机：`serial_servo_web_monitor.html`**。

### 🌟 亮点与特点：
1. **完全免安装**：**零环境搭建、无需安装 Python 依赖或第三方软件**。双击即可在现代浏览器（如 Chrome、Edge、Opera）中完美打开。
2. **直连调试**：将您的 USB-TTL 串口转换器模块连接在电脑上（接线时只需把 TX 引脚接到舵机总线上并外接 4.7K~10KΩ 的上拉电阻拉至总线电平），在网页上点击右上角 **[连接设备]**，选中该 COM 串口，即可立刻对总线进行调试！
3. **极佳交互体验**：
   - 支持通过拖动**平滑滑块**和 **3D 指针物理偏角预览** 实时控制舵机位置。
   - **一键诊断**：一键循环回读舵机的实时物理角度、芯片温度和供电电压，超高响应渲染。
   - **参数配置**：支持紧急刹车、使能/卸载力矩开关（自由拖动机械臂示教），甚至内置了修改舵机 ID（带广播安全提示警告）的高阶配置。
   - **报文监视器**：底部内嵌了高响应的十六进制数据报文控制台，能够实时清晰呈现出发送和接收到的原始二进制报文（如 `55 55 01 07 01 ...`），方便您对照下位机代码进行学习。

---

## 💻 调试上位机方案二：STM32 裸机串口命令行测试控制台 (实战必备)

如果您需要将 PC 端的通用串口调试助手连接到 **STM32 开发板**上，通过输入英文字符命令的形式让 STM32 内部的驱动库去执行操作并反馈结果，您可以直接在您的工程中包含 **`serial_servo_debug_cli.h`** 和 **`serial_servo_debug_cli.c`** 这套独立的命令行组件。

### 📥 支持的命令行（CLI）交互指令格式：
* **`pos <id> <pos> <time>\r\n`**：例如 `pos 1 800 1500\r\n`（控制 1 号舵机在 1.5 秒内转动到位置 800）。
* **`read <id>\r\n`**：例如 `read 1\r\n`（回读并打印 1 号舵机的实时位置、电压、温度）。
* **`stop <id>\r\n`**：例如 `stop 1\r\n`（刹车 ID 1 舵机并锁死位置）。
* **`free <id>\r\n`**：例如 `free 1\r\n`（断开 ID 1 舵机力矩，可用手随意拖动旋转示教）。
* **`lock <id>\r\n`**：例如 `lock 1\r\n`（重新给 ID 1 舵机上电锁死）。

---

## 🛠️ 第三步：STM32CubeMX 配置指南 (两版本通用)

对于您**只连接 TX 引脚、外接上拉电阻**的硬件设计，请在 STM32CubeMX 中进行如下配置：

### 1. 配置串口 (以 USART1 为例)
* **Mode**: 选择 **Single Wire (Half-Duplex)** (单线半双工通信)。
  *(注：此时硬件上只使用并连接 MCU 的 TX 引脚，且该 TX 引脚在外部必须接一个 4.7KΩ ~ 10KΩ 的上拉电阻拉至总线参考电压！)*
* **Baud Rate**: **115200 Bits/s** (总线舵机的默认通信速率)
* **Word Length**: **8 Bits**
* **Parity**: **None**
* **Stop Bits**: **1**
* **NVIC Settings**: 勾选 **USART1 global interrupt** 旁的 **Enabled** 框（**必须开启串口中断**）。

### 2. GPIO 引脚配置
* **无需配置任何方向切换 GPIO 端口！** 驱动底层的方向切换宏会自动控制 STM32 硬件内部的收发状态，为您节省宝贵的 GPIO 资源。
* *(如果您日后切换为外置芯片使能模式，只需在 `serial_servo_hal.h` 中将 `SERIAL_SERVO_USE_SINGLE_WIRE` 改为 `0`，并配置 PB0/PB1 为 GPIO 推挽输出即可)*。

---

## 📝 第四步：工程代码整合与挂载 (差异对比)

请根据您挑选的版本，将对应子文件夹下的 4 个文件复制进您的 Keil 或 STM32CubeIDE 工程，并按照以下方法进行挂载：

### 选项 A：如果您选择 `fast_irq_version/` (直接中断拦截版)

打开 `Core/Src/stm32f1xx_it.c` 文件，找到对应的串口中断服务函数，在第一行加入拦截器调用。
以 `USART1_IRQHandler` 为例：

```c
/* USER CODE BEGIN Includes */
#include "serial_servo_hal.h"  // 1. 引入移植头文件
/* USER CODE END Includes */

...

void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  Serial_Servo_UART_IRQHandler();  // 2. 在中断入口第一行拦截
  return;                          // 3. 拦截后直接退出，避免进入标准 HAL 库处理
  /* USER CODE END USART1_IRQn 0 */
  
  HAL_UART_IRQHandler(&huart1);
}
```

---

### 选项 B：如果您选择 `standard_hal_version/` (标准 HAL 接收回调版)

无需修改 `stm32f1xx_it.c`！直接在 `main.c` 尾部（或任意实现标准回调的文件中）实现 `HAL_UART_RxCpltCallback` 即可：

```c
/* USER CODE BEGIN 4 */
// 挂载标准标准接收回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Serial_Servo_RxCallback(huart); // 调用本驱动的标准处理接口
}
/* USER CODE END 4 */
```

---

## 🚀 第五步：高性能直接中断拦截版 完整的 main.c 代码范例

如果您选择了 **`fast_irq_version/`** (直接中断拦截版)，以下为您提供了一款工业级标准的 **`main.c` 完整主程序应用模板**：

```c
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 高性能中断拦截版 - 串口总线舵机控制与调试完整程序范例
 ******************************************************************************
 */

#include "main.h"
#include "serial_servo_hal.h"
#include "serial_servo_debug_cli.h"
#include <stdio.h>

/* 声明 CubeMX 自动生成的串口句柄 */
UART_HandleTypeDef huart1; // 绑定总线舵机 (Single Wire 115200)
UART_HandleTypeDef huart2; // 绑定 PC 调试命令行控制台 (115200)

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();  // 串口 1 初始化
  MX_USART2_UART_Init();  // 串口 2 初始化

  /* 初始化总线物理层 */
  Serial_Servo_HAL_Init();

  /* 初始化 PC 串口命令行调试控制台，开启 USART2 异步接收中断 */
  Debug_CLI_Init();

  printf(">> 高性能直接中断拦截版就绪！\r\n");

  uint32_t last_action_tick = 0;
  uint8_t toggle_pos_flag = 0;

  while (1)
  {
    /* ================= 核心任务一：调试控制台进程 (高响应优先) ================= */
    Debug_CLI_Process();

    /* ================= 核心任务二：周期性交替运动与传感器健康监视 ================= */
    /* 基于裸机非阻塞时钟 Tick，每隔 3000ms (3秒) 自动触发一次自动位置交替与诊断 */
    if (HAL_GetTick() - last_action_tick >= 3000)
    {
      last_action_tick = HAL_GetTick();
      
      printf(">> [自动调度开始]\r\n");

      // 自动交替控制 ID 1 运动到位置 200 和 800
      if (toggle_pos_flag == 0)
      {
        printf(">> 调度: 控制舵机 1 平滑旋转至位置 200 (耗时1.2秒)...\r\n");
        serial_servo_set_position(&g_serial_servo_controller, 1, 200, 1200);
        toggle_pos_flag = 1;
      }
      else
      {
        printf(">> 调度: 控制舵机 1 平滑旋转至位置 800 (耗时1.2秒)...\r\n");
        serial_servo_set_position(&g_serial_servo_controller, 1, 800, 1200);
        toggle_pos_flag = 0;
      }

      HAL_Delay(50); // 避开发送到读取转换的总线物理静默死区

      // 周期性主动诊断 (读取实时电压与芯片温度)
      uint16_t voltage_mv = 0;
      uint8_t temperature = 0;
      
      int vin_status = serial_servo_read_vin(&g_serial_servo_controller, 1, &voltage_mv);
      HAL_Delay(40); // 40ms 物理通道切换防冲突死区
      int temp_status = serial_servo_read_temp(&g_serial_servo_controller, 1, &temperature);

      if (vin_status == 0 && temp_status == 0)
      {
        printf(">> [健康状况诊断]: 舵机 1 电压: %.2f V | 实时温度: %d ℃\r\n", 
               voltage_mv / 1000.0f, temperature);
               
        // 过热安全自保护逻辑
        if (temperature >= 75)
        {
          printf(">> [⚠️过热报警] 温度过高，自动断电释放舵机力矩以防烧毁！\r\n");
          serial_servo_load_unload(&g_serial_servo_controller, 1, 0); 
        }
      }
      else
      {
        printf(">> [诊断警告]: 舵机 1 离线或总线断开！\r\n");
      }
      
      printf(">> [自动调度结束]\r\n\r\n");
    }
  }
}
```

---

## 🚀 第六步：标准 HAL 接收回调版 完整的 main.c 代码范例 (选项 B 专属)

如果您选择了 **`standard_hal_version/`** (标准 HAL 接收回调版)，您**完全不需要修改 `stm32f1xx_it.c` 文件**。

以下为您提供专属的 **`main.c` 完整程序范例**。在该范例中，演示了如何通过标准的 **`HAL_UART_RxCpltCallback`** 接收完成回调接口，以极高内聚性、无干扰地同时挂载与分发“总线舵机串口数据”与“PC调试命令行串口数据”：

```c
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 标准 HAL 接收回调版 - 串口总线舵机控制与调试完整程序范例
 ******************************************************************************
 */

#include "main.h"
#include "serial_servo_hal.h"
#include "serial_servo_debug_cli.h"
#include <stdio.h>

/* 声明 CubeMX 自动生成的串口句柄 */
UART_HandleTypeDef huart1; // 总线舵机串口 (Single Wire 115200)
UART_HandleTypeDef huart2; // PC 调试控制台串口 (Asynchronous 115200)

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

int main(void)
{
  /* 1. 初始化 HAL 库 */
  HAL_Init();

  /* 2. 配置系统时钟 */
  SystemClock_Config();

  /* 3. 初始化配置好的外设 */
  MX_GPIO_Init();
  MX_USART1_UART_Init();  // 开启总线舵机串口
  MX_USART2_UART_Init();  // 开启 PC 调试串口

  /* 4. 初始化总线物理层驱动并挂载回调，同时启动 huart1 的首次 HAL 中断接收监听 */
  Serial_Servo_HAL_Init();

  /* 5. 初始化独立的 PC 串口命令行调试控制台，开启 huart2 的首次标准 1 字节中断接收监听 */
  Debug_CLI_Init();

  printf(">> 标准 HAL 接收回调版就绪，系统正常运行中！\r\n");

  uint32_t last_action_tick = 0;
  uint8_t toggle_pos_flag = 0;

  while (1)
  {
    /* ================= 调试控制台命令行解析处理 (高响应优先) ================= */
    Debug_CLI_Process();

    /* ================= 周期性交替位置控制与环境诊断监视 ================= */
    /* 每隔 3500ms 自动交替执行运动并回读舵机环境参数，基于裸机非阻塞 Tick */
    if (HAL_GetTick() - last_action_tick >= 3500)
    {
      last_action_tick = HAL_GetTick();
      
      printf(">> [标准自动调度任务]\r\n");

      // 自动交替驱动 ID 1 运动到 150 和 850
      if (toggle_pos_flag == 0)
      {
        printf(">> 调度: 控制 1 号舵机匀速旋转至位置 150 (耗时1.0秒)...\r\n");
        serial_servo_set_position(&g_serial_servo_controller, 1, 150, 1000);
        toggle_pos_flag = 1;
      }
      else
      {
        printf(">> 调度: 控制 1 号舵机匀速旋转至位置 850 (耗时1.0秒)...\r\n");
        serial_servo_set_position(&g_serial_servo_controller, 1, 850, 1000);
        toggle_pos_flag = 0;
      }

      HAL_Delay(50); // 避开发送完到接收转换的总线物理静默死区

      // 主动监测并读取 1 号舵机的环境电压与温度
      uint16_t voltage_mv = 0;
      uint8_t temperature = 0;
      
      int vin_r = serial_servo_read_vin(&g_serial_servo_controller, 1, &voltage_mv);
      HAL_Delay(40); // 给半双工总线以稳定的物理翻转静默
      int temp_r = serial_servo_read_temp(&g_serial_servo_controller, 1, &temperature);

      if (vin_r == 0 && temp_r == 0)
      {
        printf(">> [舵机健康数据]: 输入电压: %.2f V | 芯片实时温度: %d ℃\r\n", 
               voltage_mv / 1000.0f, temperature);
      }
      else
      {
        printf(">> [诊断警告]: 回读超时！请检查物理接线是否松脱。\r\n");
      }
      
      printf(">> [标准自动调度结束]\r\n\r\n");
    }
  }
}

/**
  * @brief  标准 HAL 库串口接收中断完成回调函数 (USER CODE 4 区域)
  * @note   在此处极度优雅、对称地分发处理来自不同串口设备的数据包，零侵入 stm32f1xx_it.c！
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 1. 处理来自串口 1 (总线舵机) 的回传接收字节数据 */
    Serial_Servo_RxCallback(huart);
    
    /* 2. 处理来自串口 2 (PC 调试控制台) 的英文字符命令字节数据 */
    Debug_CLI_RxCallback(huart);
}
```

---

## 📘 核心应用层控制 API 完整使用指南 (全部 19 个函数)

下面将驱动核心头文件 `serial_servo.h` 中声明的**全部 19 个应用层驱动 API 函数**进行归类整理，并提供详尽的原型、入参、返回值说明及调用示例，方便您随时查阅。

---

### 1. 控制器对象初始化与 ID 配置

#### 🟢 ① 控制器对象初始化：`serial_servo_controller_object_init`
* **函数原型**：`void serial_servo_controller_object_init(SerialServoControllerTypeDef *self);`
* **功能描述**：用于将全局 of 舵机控制器管理对象进行内部数据结构和状态机的清零与就绪初始化。
* **调用示例**：
  ```c
  // 驱动内部已在 Serial_Servo_HAL_Init() 中自动调用，用户一般无需手动调用。
  serial_servo_controller_object_init(&g_serial_servo_controller);
  ```

#### 🟢 ② 写入并配置舵机 ID 号：`serial_servo_set_id`
* **函数原型**：`void serial_servo_set_id(SerialServoControllerTypeDef *self, uint32_t old_id, uint32_t new_id);`
* **功能描述**：修改指定舵机的 ID 号。
* **参数说明**：
  - `old_id`: 原舵机 ID（若不知道原 ID，可传入广播 ID `0xFE` 强行修改）。
  - `new_id`: 目标新 ID（取值范围：`0 ~ 253`）。
* **⚠️警告**：进行此修改时，**总线上必须有且仅有 1 个舵机**！若挂载了多个舵机，它们均会被同时修改为新 ID。
* **调用示例**：
  ```c
  // 将当前连接的舵机（不管其原本ID是多少）强制修改为 ID 5
  serial_servo_set_id(&g_serial_servo_controller, 0xFE, 5);
  ```

#### 🟢 ③ 读取舵机当前 ID 号：`serial_servo_read_id`
* **函数原型**：`int serial_servo_read_id(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t *ret_servo_id);`
* **功能描述**：回读总线上舵机的真实 ID。
* **参数说明**：
  - `servo_id`: 需要检测的舵机目标 ID（若在总线上只接了一个舵机，传入 `0xFE` 进行广播检测最方便）。
  - `ret_servo_id`: 接收读出 ID 结果的指针。
* **返回值**：成功返回 `0`；超时未响应或校验错误返回 `-1`。
* **调用示例**：
  ```c
  uint8_t detected_id = 0;
  if (0 == serial_servo_read_id(&g_serial_servo_controller, 0xFE, &detected_id)) {
      printf("检测到总线上的舵机 ID 为: %d\r\n", detected_id);
  }
  ```

---

### 2. 运动控制与物理力矩调节

#### 🔵 ④ 控制舵机运动到指定位置（最常用）：`serial_servo_set_position`
* **函数原型**：`void serial_servo_set_position(SerialServoControllerTypeDef *self, uint32_t servo_id, int position, uint32_t duration);`
* **功能描述**：控制舵机在设定的时间参数内，匀速平滑旋转到指定目标位置。
* **参数说明**：
  - `servo_id`: 舵机 ID（若控制总线全部舵机同步运动，可传入广播 ID `0xFE`）。
  - `position`: 目标位置值，有效取值范围为 `0 ~ 1000`（对应舵机物理旋转的 `0° ~ 240°` 范围，其中 `500` 为几何中位）。
  - `duration`: 到达目标位置所需的时间，单位为 `毫秒`（取值范围：`0 ~ 30000ms`）。
* **调用示例**：
  ```c
  // 让 ID 为 1 的舵机在 2000毫秒 (2秒) 内旋转到位置值 800
  serial_servo_set_position(&g_serial_servo_controller, 1, 800, 2000);
  ```

#### 🔵 ⑤ 回读舵机实时位置值：`serial_servo_read_position`
* **函数原型**：`int serial_servo_read_position(SerialServoControllerTypeDef *self, uint32_t servo_id, int16_t *position);`
* **功能描述**：向舵机请求获取当前的实时角度/位置值。
* **参数说明**：
  - `position`: 保存回读结果的变量指针，回读结果范围 `0 ~ 1000`。
* **返回值**：成功返回 `0`；超时或校验错返回 `-1`。
* **调用示例**：
  ```c
  int16_t current_pos = 0;
  if (0 == serial_servo_read_position(&g_serial_servo_controller, 1, &current_pos)) {
      printf("ID 1 舵机实时位置: %d\r\n", current_pos);
  }
  ```

#### 🔵 ⑥ 紧急停止：`serial_servo_stop`
* **函数原型**：`void serial_servo_stop(SerialServoControllerTypeDef *self, uint32_t servo_id);`
* **功能描述**：让正在转动中的舵机立刻紧急刹车并在当前位置锁死。
* **调用示例**：
  ```c
  // 紧急制动 ID 为 1 的舵机
  serial_servo_stop(&g_serial_servo_controller, 1);
  ```

#### 🔵 ⑦ 力矩开关控制（手动教学关键）：`serial_servo_load_unload`
* **函数原型**：`void serial_servo_load_unload(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t load);`
* **功能描述**：使能或断开舵机内部无刷/有刷电机的驱动力矩。常用于手动拖动示教（拖动机械臂摆好姿势后读取阻力，记录坐标）。
* **参数说明**：
  - `load`: 力矩模式。`0` 代表掉电释放（卸载力矩，舵机变为无力状态，可用手随意转动）；`1` 代表重新上电锁死（加载力矩，恢复锁定并在当前位置提供额定扭力）。
* **调用示例**：
  ```c
  // 释放 ID 2 舵机力矩，进入自由手动旋转模式
  serial_servo_load_unload(&g_serial_servo_controller, 2, 0);
  
  // 重新锁定 ID 2 舵机力矩，恢复定位锁死
  serial_servo_load_unload(&g_serial_servo_controller, 2, 1);
  ```

#### 🔵 ⑧ 回读舵机力矩使能状态：`serial_servo_read_load_unload`
* **函数原型**：`int serial_servo_read_load_unload(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t* load_unload);`
* **功能描述**：获取当前舵机力矩是否处于锁定状态。
* **参数说明**：
  - `load_unload`: 接收结果的指针。`0` 表示卸载释放，`1` 表示锁死使能。
* **返回值**：成功返回 `0`；超时未响应或校验错误返回 `-1`。
* **调用示例**：
  ```c
  uint8_t torque_state = 0;
  if (0 == serial_servo_read_load_unload(&g_serial_servo_controller, 1, &torque_state)) {
      printf("舵机 1 力矩锁定状态：%s\r\n", torque_state ? "锁定上电" : "释放掉电");
  }
  ```

---

### 3. 机械偏差校准

#### 🟡 ⑨ 临时写入角度偏差：`serial_servo_set_deviation`
* **函数原型**：`void serial_servo_set_deviation(SerialServoControllerTypeDef *self, uint32_t servo_id, int new_deviation);`
* **功能描述**：用于在出厂机械装配有微小偏差时，通过软件设置校正偏差量（此操作立刻生效，但重新上电会丢失）。
* **参数说明**：
  - `new_deviation`: 偏差补偿值，取值范围 `-125 ~ 125`（对应中间零位）。
* **调用示例**：
  ```c
  // 临时给 ID 1 舵机写入 +10 的位置补偿偏差
  serial_servo_set_deviation(&g_serial_servo_controller, 1, 10);
  ```

#### 🟡 ⑩ 永久保存/固化偏差：`serial_servo_save_deviation`
* **函数原型**：`void serial_servo_save_deviation(SerialServoControllerTypeDef *self, uint32_t servo_id);`
* **功能描述**：将上面临时写入的角度偏差值，永久写入到舵机内部主控芯片的 EEPROM 闪存中，保证断电不丢失。
* **调用示例**：
  ```c
  // 固化 ID 1 舵机的当前角度偏差配置
  serial_servo_save_deviation(&g_serial_servo_controller, 1);
  ```

#### 🟡 ⑪ 读取当前已保存的偏差值：`serial_servo_read_deviation`
* **函数原型**：`int serial_servo_read_deviation(SerialServoControllerTypeDef *self, uint32_t servo_id, int8_t *deviation);`
* **功能描述**：读取舵机内部 EEPROM 中当前固化保存的角度偏差值。
* **参数说明**：
  - `deviation`: 指向带符号 8 位整型的指针，用以接收回读的偏差值。
* **返回值**：成功返回 `0`；超时返回 `-1`。
* **调用示例**：
  ```c
  int8_t saved_dev = 0;
  if (0 == serial_servo_read_deviation(&g_serial_servo_controller, 1, &saved_dev)) {
      printf("舵机 1 内部固化的角度偏差值: %d\r\n", saved_dev);
  }
  ```

---

### 4. 阈值保护与安全限制设置

#### 🔴 ⑫ 设置角度限位保护：`serial_servo_set_angle_limit`
* **函数原型**：`void serial_servo_set_angle_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t limit_l, uint32_t limit_h);`
* **功能描述**：限制舵机的最大最小物理旋转区间，防止机械臂发生关节过度弯曲自撞受损。
* **参数说明**：
  - `limit_l`: 最小角度限制值（`0 ~ 1000`）。
  - `limit_h`: 最大角度限制值（`0 ~ 1000`且必须大于最小限位）。
* **调用示例**：
  ```c
  // 限制 ID 1 舵机只能在 200 到 800 位置区间内旋转
  serial_servo_set_angle_limit(&g_serial_servo_controller, 1, 200, 800);
  ```

#### 🔴 ⑬ 读取已配置的角度限位：`serial_servo_read_angle_limit`
* **函数原型**：`int serial_servo_read_angle_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint16_t limit[2]);`
* **功能描述**：获取指定舵机当前固化的限位区间。
* **参数说明**：
  - `limit`: 长度为 2 的 `uint16_t` 数组指针。`limit[0]` 存放下限，`limit[1]` 存放上限。
* **返回值**：成功返回 `0`；超时返回 `-1`。
* **调用示例**：
  ```c
  uint16_t angle_limits[2] = {0};
  if (0 == serial_servo_read_angle_limit(&g_serial_servo_controller, 1, angle_limits)) {
      printf("舵机 1 安全角度区间：[%d, %d]\r\n", angle_limits[0], angle_limits[1]);
  }
  ```

#### 🔴 ⑭ 设置最高安全工作温度：`serial_servo_set_temp_limit`
* **函数原型**：`void serial_servo_set_temp_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t limit);`
* **功能描述**：写入舵机的最高安全工作温度阈值。当温度超过此阈值，舵机红灯闪烁报警，并自动卸载力矩停转以避免烧毁马达。
* **参数说明**：
  - `limit`: 最高温度上限值，单位：`摄氏度`（常用默认值：`85`）。
* **调用示例**：
  ```c
  // 设置安全温度上限为 80 摄氏度
  serial_servo_set_temp_limit(&g_serial_servo_controller, 1, 80);
  ```

#### 🔴 ⑮ 读取最高安全工作温度：`serial_servo_read_temp_limit`
* **函数原型**：`int serial_servo_read_temp_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t *limit);`
* **功能描述**：回读舵机中设定的最高温度报警保护阈值。
* **返回值**：成功返回 `0`；超时返回 `-1`。
* **调用示例**：
  ```c
  uint8_t temp_limit = 0;
  if (0 == serial_servo_read_temp_limit(&g_serial_servo_controller, 1, &temp_limit)) {
      printf("当前设定的最高温度报警阀值: %d ℃\r\n", temp_limit);
  }
  ```

#### 🔴 ⑯ 设置安全工作电压区间：`serial_servo_set_vin_limit`
* **函数原型**：`void serial_servo_set_vin_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t limit_l, uint32_t limit_h);`
* **功能描述**：设置供电输入电压的正常波动范围。若供电电压偏低或偏高超出此范围，舵机会闪烁 LED 报警并锁转。
* **参数说明**：
  - `limit_l`: 最小工作电压限值，单位：`毫伏 (mV)`（取值范围：`4500 ~ 12000mV`）。
  - `limit_h`: 最大工作电压限值，单位：`毫伏 (mV)`。
* **调用示例**：
  ```c
  // 限制 ID 1 舵机在 6.0V ~ 8.4V (6000mV ~ 8400mV) 正常工作
  serial_servo_set_vin_limit(&g_serial_servo_controller, 1, 6000, 8400);
  ```

#### 🔴 ⑰ 读取安全工作电压区间：`serial_servo_read_vin_limit`
* **函数原型**：`int serial_servo_read_vin_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint16_t limit[2]);`
* **功能描述**：获取当前设定的安全电压保护范围。
* **参数说明**：
  - `limit`: 长度为 2 的 `uint16_t` 数组。`limit[0]` 存放低电压限值，`limit[1]` 存放高电压限值。
* **返回值**：成功返回 `0`；超时返回 `-1`。
* **调用示例**：
  ```c
  uint16_t vin_limits[2] = {0};
  if (0 == serial_servo_read_vin_limit(&g_serial_servo_controller, 1, vin_limits)) {
      printf("安全工作电压范围: %.2fV ~ %.2fV\r\n", vin_limits[0]/1000.0f, vin_limits[1]/1000.0f);
  }
  ```

---

### 5. 实时运行状况监视 (诊断与回读)

#### 🔎 ⑱ 读取实时温度值：`serial_servo_read_temp`
* **函数原型**：`int serial_servo_read_temp(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t *temp);`
* **功能描述**：读取舵机内部实时检测出的驱动板主芯片当前温度。
* **参数说明**：
  - `temp`: 接收结果 of 单字节指针，单位：`摄氏度 (℃)`。
* **返回值**：成功返回 `0`；发生断线超时未响应则返回 `-1`。
* **调用示例**：
  ```c
  uint8_t cur_temp = 0;
  if (0 == serial_servo_read_temp(&g_serial_servo_controller, 1, &cur_temp)) {
      printf("舵机 1 实时工作温度: %d ℃\r\n", cur_temp);
  }
  ```

#### 🔎 ⑲ 读取实时供电电压值：`serial_servo_read_vin`
* **函数原型**：`int serial_servo_read_vin(SerialServoControllerTypeDef *self, uint32_t servo_id, uint16_t *vin);`
* **功能描述**：回读舵机接线端的实时供电电压，方便主控程序实时监测电池电量和压降。
* **参数说明**：
  - `vin`: 接收结果 of 16 位无符号整型指针，单位为：`毫伏 (mV)`。
* **返回值**：成功返回 `0`；超时响应失败返回 `-1`。
* **调用示例**：
  ```c
  uint16_t current_vin_mv = 0;
  if (0 == serial_servo_read_vin(&g_serial_servo_controller, 1, &current_vin_mv)) {
      printf("舵机总线供电电压为: %.2f V\r\n", current_vin_mv / 1000.0f);
  }
  ```

---

## 🛡️ 工业级稳定通信保证设计

1. **硬件级等待传输完成 (TC)**：在半双工模式下，如果发送数据没被完全排空到物理总线上就提前拉低/拉高方向引脚，会导致数据丢尾。本驱动物理层增加了严格 of `USART_SR_TC` 标志轮询，实现无缝、无时差的方向翻转。
2. **免重入垃圾数据清理**：对于双线合一的单线半双工电路，发送引脚上的数据通常会产生电回波从而灌入接收引脚（自发自收）。本物理层在完成发送并进入接收模式的第一时间，执行了硬件寄存器废数丢弃操作，完美过滤回波干扰。
3. **基于 SysTick 的超时**：裸机下所有的读指令都配备了超时强退机制（默认8ms），若舵机损坏或物理线断开，系统决不会陷入死循环，保证机械臂整体主控系统的绝对安全。
