# PipeBot - 多功能管道内检测小车

一个基于 **STM32F407VET6** 微控制器的自主移动机器人项目，集成了电机控制、IMU 传感、测距模块和串口通信，用于管道内环境检测和导航。

## 📋 项目概述

PipeBot 是一个嵌入式实时系统，采用 **FreeRTOS** 实时操作系统和 **STM32CubeMX** 硬件抽象层，实现了多任务并发控制。系统与上位机（Raspberry Pi 4B 运行 ROS2 Humble）通过串口进行通信，支持远程控制和数据反馈。

### 核心特性
- ✅ 双电机独立控制，支持前进、后退、原地转向
- ✅ 编码器反馈的闭环速度控制（PID 算法）
- ✅ 9 轴 IMU 传感器（加速度、角速度、欧拉角）
- ✅ 双路 TOF 测距模块（左右距离检测）
- ✅ 实时串口通信协议（11 字节固定帧格式）
- ✅ 多任务并发调度（FreeRTOS）
- ✅ 按键控制和 LED 状态指示

---

## 🛠️ 硬件平台

| 组件 | 规格 | 备注 |
|------|------|------|
| **MCU** | STM32F407VET6 | 168MHz, 512KB FLASH, 192KB RAM |
| **电机驱动** | DRV8833 双路 H 桥 | 支持 PWM 速度控制 |
| **编码器** | MG513P30 | 13PPR + 30 减速比 + 4 倍频 = 1560 脉冲/圈 |
| **IMU** | JY61P (9 轴) | I2C 接口，地址 0x50 |
| **测距** | TOF050F × 2 | UART3/UART4，地址 0x02/0x03 |
| **通信** | UART1 | 与 ROS2 上位机通信 |
| **LED** | 红蓝双色 | PC5 (红), PB2 (蓝) |
| **按键** | 1 个 | PA0，用于 LED 切换 |

### 电机参数（可配置）

在 [`Core/Bsp/bsp_motor.h`](Core/Bsp/bsp_motor.h) 中定义：

```c
#define WHEEL_DIAMETER    0.0423f   // 轮径 42.3mm
#define WHEEL_BASE        0.148f    // 轮距 148mm
#define ENCODER_TICKS     1560.0f   // 编码器脉冲数
#define SAMPLE_TIME       0.02f     // 采样周期 20ms (50Hz)
#define MOTOR_MAX_PWM     16799     // PWM 最大值
```

**⚠️ 更换电机或轮子时必须同时修改这些宏，否则速度计算错误。**

---

## 📍 引脚定义

### 电机控制
| 功能 | 引脚 | 端口 | 说明 |
|------|------|------|------|
| 左电机 PWM | PE9 | TIM1_CH1 | 速度控制 |
| 左电机方向 1 | PA3 | GPIO | AIN1 |
| 左电机方向 2 | PA4 | GPIO | AIN2 |
| 左编码器 A | PA6 | TIM4_CH1 | 编码器反馈 |
| 左编码器 B | PA7 | TIM4_CH2 | 编码器反馈 |
| 右电机 PWM | PE11 | TIM1_CH2 | 速度控制 |
| 右电机方向 1 | PA1 | GPIO | BIN1 |
| 右电机方向 2 | PA2 | GPIO | BIN2 |
| 右编码器 A | PB6 | TIM3_CH1 | 编码器反馈 |
| 右编码器 B | PB7 | TIM3_CH2 | 编码器反馈 |
| 电机使能 | PA5 | GPIO | STBY (待机控制) |

### 传感器接口
| 功能 | 引脚 | 接口 | 说明 |
|------|------|------|------|
| IMU (JY61P) | PB8/PB9 | I2C1 | 9 轴传感器 |
| TOF 左 | PC10/PC11 | UART3 | 测距模块 |
| TOF 右 | PC12/PD2 | UART4 | 测距模块 |

### 用户接口
| 功能 | 引脚 | 端口 | 说明 |
|------|------|------|------|
| 按键 | PA0 | GPIO | 用于 LED 切换 |
| 红 LED | PC5 | GPIO | 状态指示 |
| 蓝 LED | PB2 | GPIO | 状态指示 |
| 调试串口 | PA9/PA10 | UART1 | 与 ROS2 通信 |

---

## 🏗️ 软件架构

### 分层设计

```
┌─────────────────────────────────────┐
│   应用层 (Core/App/Tasks/)          │
│  - SerialRxTask: 串口接收           │
│  - SerialTxTask: 串口发送           │
│  - MotorTask: 电机控制              │
│  - IMUTask: IMU 数据采集            │
│  - TOFTask: 测距数据采集            │
│  - LEDTask: LED 控制                │
│  - KeyTask: 按键扫描                │
└─────────────────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│   硬件抽象层 (Core/Bsp/)            │
│  - bsp_motor.c/h: 电机驱动          │
│  - bsp_jy61p.c/h: IMU 驱动          │
│  - bsp_tof050f.c/h: 测距驱动        │
│  - bsp_usart_dma.c/h: 串口 DMA      │
└─────────────────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│   底层驱动 (Core/Src/)              │
│  - HAL 库初始化代码 (CubeMX 生成)   │
│  - UART, I2C, TIM, DMA 配置         │
└─────────────────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│   中间件 (Middlewares/)             │
│  - FreeRTOS 内核                    │
│  - CMSIS-RTOS V2 适配层             │
└─────────────────────────────────────┘
```

### 任务列表

| 任务 | 优先级 | 周期 | 功能 |
|------|--------|------|------|
| **SerialRxTask** | 高 | 事件驱动 | 接收来自 ROS2 的控制命令 |
| **SerialTxTask** | 高 | 事件驱动 | 发送传感器数据到 ROS2 |
| **MotorTask** | 中 | 20ms | 读取编码器，计算速度，发送反馈 |
| **IMUTask** | 中 | 10ms | 读取 IMU 数据，解析欧拉角 |
| **TOFTask** | 中 | 事件驱动 | 读取测距数据，打包发送 |
| **LEDTask** | 低 | 事件驱动 | 处理 LED 控制消息 |
| **KeyTask** | 低 | 10ms | 扫描按键，发送 LED 控制消息 |

### 通信协议

#### 帧格式（11 字节固定长度）

```
[帧头] [ID] [长度] [数据 0-5] [校验] [帧尾]
 0x55  0x0*  0x06   6 字节    0x**  0xBB
```

#### 协议 ID 映射表

| ID | 方向 | 功能 | 数据定义 |
|----|------|------|---------|
| **0x01** | ↓ | 速度控制 | Data0: 左电机方向, Data1: 左电机速度, Data3: 右电机方向, Data4: 右电机速度 |
| **0x02** | ↑ | 速度反馈 | Data0: 左电机方向, Data1: 左电机速度, Data3: 右电机方向, Data4: 右电机速度 |
| **0x03** | ↑ | IMU 加速度 | Data0-1: X轴(H/L), Data2-3: Y轴(H/L), Data4-5: Z轴(H/L) |
| **0x04** | ↑ | IMU 角速度 | Data0-1: X轴(H/L), Data2-3: Y轴(H/L), Data4-5: Z轴(H/L) |
| **0x05** | ↑ | IMU 欧拉角 | Data0-1: Roll(H/L), Data2-3: Pitch(H/L), Data4-5: Yaw(H/L) |
| **0x06** | ↑ | 测距数据 | Data0-1: 左距离(H/L), Data2-3: 右距离(H/L) |
| **0x07** | ↓ | 系统状态 | Data0: LED使能, Data1: LED状态, Data2: 蜂鸣器使能, Data3: 蜂鸣器状态, Data4: IMU校准 |
| **0x08** | ↕ | PID 参数 | Data0-1: P(H/L), Data2-3: I(H/L), Data4-5: D(H/L) |

**校验算法**: 累加和校验，计算范围为前 9 字节（帧头 + ID + 长度 + 6 字节数据）

**字节序**: 所有 16-bit 数据采用高低字节拆分，接收端用 `(H << 8 | L)` 拼接

---

## 🚀 快速开始

### 环境要求
- **IDE**: CLion 或 STM32CubeIDE
- **编译器**: GCC ARM Embedded (arm-none-eabi-gcc)
- **构建系统**: CMake 3.22+
- **调试器**: CMSIS-DAP (配置文件: `stm32f4_dap.cfg`)

---

## 📊 运动控制 API

### 基础运动函数

```c
// 前进（mm/s）
void Move_Forward(int16_t speed_mm_s);

// 后退（mm/s）
void Move_Backward(int16_t speed_mm_s);

// 原地左转（mm/s）
void Turn_Left(int16_t speed_mm_s);

// 原地右转（mm/s）
void Turn_Right(int16_t speed_mm_s);

// 停止
void Motor_Stop(void);

// 单电机速度控制（mm/s）
void MotorL_Speed(int16_t speed_mm_s);
void MotorR_Speed(int16_t speed_mm_s);
```

### PID 参数

电机 PID 参数定义在 [`Core/Bsp/bsp_motor.c`](Core/Bsp/bsp_motor.c:13-14)：

```c
PID_Params_t MotorL_PID = {4.1f, 0.15f, 0.0f};  // Kp, Ki, Kd
PID_Params_t MotorR_PID = {4.1f, 0.15f, 0.0f};
```

可通过协议 ID `0x08` 动态下发 PID 参数。

---

## 🔧 开发规范

### 关键约束

1. **禁止使用 HAL_Delay()**
   - 在任何 FreeRTOS 任务中必须使用 `vTaskDelay()` 或 `osDelay()`
   - `HAL_Delay()` 会阻塞整个内核调度

2. **CubeMX 代码保护**
   - 所有业务逻辑必须在 `/* USER CODE BEGIN/END */` 块内
   - 每次重新生成后，删除 `STM32F407XX_FLASH.ld` 中所有 `(READONLY)` 关键字

3. **新文件注册**
   - 创建新的 `.c` 文件后，必须在 `CMakeLists.txt` 的 `add_executable()` 中添加

4. **DMA 接收重启**
   - 每次 `HAL_UARTEx_RxEventCallback()` 触发后，必须调用 `HAL_UART_AbortReceive()` 重启 DMA
   - 否则 DMA 计数器不会重置，导致数据丢失

5. **信号量初始化**
   - TX/RX 任务启动时必须先 `osSemaphoreRelease()` 一次，表示硬件初始状态空闲
   - 否则第一次发送会永久阻塞

### 堆大小限制

FreeRTOS 堆仅 **15360 字节**（见 `Core/Inc/FreeRTOSConfig.h:71`）。新任务的栈大小需谨慎计算，避免堆溢出导致系统崩溃。

### 代码风格

- **标准**: C11
- **编译器**: GCC ARM
- **宏定义**: 用于硬件抽象（见 `bsp_motor.h` 的 `MOTOR_*` 宏）
- **结构体**: 用于数据传输（`ProtocolFrame_t`, `Motor_TypeDef`, `PID_Params_t`）

---

## 🐛 常见问题

### 1. 编译失败：链接脚本错误
**症状**: `section '.text' will not fit in region 'FLASH'`

**解决**: 删除 `STM32F407XX_FLASH.ld` 中所有 `(READONLY)` 关键字

### 2. 串口接收数据丢失
**症状**: 接收到的数据不完整或乱码

**原因**: `HAL_UARTEx_RxEventCallback()` 后未调用 `HAL_UART_AbortReceive()` 重启 DMA

**解决**: 见 [`SerialRxTask.c:45-46`](Core/App/Tasks/SerialRxTask.c:45)

### 3. 串口发送永久阻塞
**症状**: `HAL_UART_Transmit_DMA()` 调用后程序卡住

**原因**: TX 任务启动时未调用 `osSemaphoreRelease()` 初始化信号量

**解决**: 见 [`SerialTxTask.c:14`](Core/App/Tasks/SerialTxTask.c:14)

### 4. 电机速度计算完全错误
**症状**: 电机转速反馈值与实际不符

**原因**: `bsp_motor.h` 中的 `WHEEL_DIAMETER`、`WHEEL_BASE` 或 `ENCODER_TICKS` 与实际硬件不匹配

**解决**: 修改这些宏，重新编译

### 5. printf 输出不显示
**症状**: `printf()` 调用但串口助手无输出

**原因**: `_write()` 函数未正确重定向或 UART1 未初始化

**解决**: 检查 `main.c:66` 的 `_write()` 实现

---

## 📚 文件结构

```
PipeBot/
├── Core/
│   ├── App/
│   │   ├── Tasks/              # FreeRTOS 任务
│   │   │   ├── SerialRxTask.c
│   │   │   ├── SerialTxTask.c
│   │   │   ├── MotorTask.c
│   │   │   ├── IMUTask.c
│   │   │   ├── TOFTask.c
│   │   │   ├── LEDTask.c
│   │   │   └── KeyTask.c
│   │   └── Types/
│   │       └── LEDType.h       # 协议数据结构体
│   ├── Bsp/                    # 硬件抽象层
│   │   ├── bsp_motor.c/h
│   │   ├── bsp_jy61p.c/h
│   │   ├── bsp_tof050f.c/h
│   │   └── bsp_usart_dma.c/h
│   ├── Inc/                    # CubeMX 生成的头文件
│   └── Src/                    # CubeMX 生成的源文件
├── Drivers/                    # STM32 HAL 库和 CMSIS
├── Middlewares/                # FreeRTOS 内核
├── cmake/                      # CMake 配置
├── CMakeLists.txt
├── PipeBot.ioc                 # STM32CubeMX 项目文件
├── stm32f4_dap.cfg             # CMSIS-DAP 调试配置
└── README.md
```

---

## 📖 相关文档

- **AGENTS.md**: AI 助手开发指南（非显而易见的项目规则）

---

