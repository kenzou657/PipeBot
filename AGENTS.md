# AGENTS.md

This file provides guidance to agents when working with code in this repository.

## 项目栈
- **硬件**: STM32F407VET6 (512KB FLASH, 192KB RAM)
- **框架**: FreeRTOS + HAL库 (STM32CubeMX 生成)
- **构建**: CMake (C11 标准)
- **IDE**: CLion
- **调试**: CMSIS-DAP (`stm32f4_dap.cfg`)

## 关键非显而易见的规则

### 1. 串口通信协议 (STM32 <-> ROS2)
- **帧格式**: 固定 11 字节 `[0x55|ID|0x06|Data[6]|Checksum|0xBB]`
- **校验**: 累加和，计算范围为前 9 字节（帧头到数据）
- **字节序**: 16-bit 数据必须拆分为高/低字节，接收端用 `(H << 8 | L)` 拼接
- **关键**: [`ProtocolFrame_t`](Core/App/Types/LEDType.h:27) 有 12 字节（含 reserved），但只发送 11 字节
- **ID 映射**: 见 `.roorules` 第 3.2 节

### 2. DMA 串口接收的隐藏陷阱
- 使用 `HAL_UARTEx_ReceiveToIdle_DMA()` 时，每次 IDLE 中断后**必须**调用 `HAL_UART_AbortReceive()` 重启 DMA
- 否则 DMA 计数器 (NDTR) 不会重置，导致数据丢失
- 见 [`SerialRxTask.c`](Core/App/Tasks/SerialRxTask.c:45) 的关键注释

### 3. FreeRTOS 任务延时
- **禁止**: 任何任务中使用 `HAL_Delay()`（会阻塞内核调度）
- **必须**: 使用 `vTaskDelay()` 或 `osDelay()`
- **堆大小**: 仅 15360 字节（`configTOTAL_HEAP_SIZE`），新任务需谨慎

### 4. CubeMX 代码保护与链接脚本
- 所有业务逻辑必须在 `/* USER CODE BEGIN/END */` 块内
- **每次重新生成后**: 检查 [`STM32F407XX_FLASH.ld`](STM32F407XX_FLASH.ld) 删除所有 `(READONLY)` 关键字，否则编译失败
- 新增 `.c/.h` 文件必须在 [`CMakeLists.txt`](CMakeLists.txt) 的 `add_executable` 中添加

### 5. 电机控制的参数化
- 轮径、轮距、编码器脉冲数在 [`bsp_motor.h`](Core/Bsp/bsp_motor.h:11-16) 定义
- 更换驱动轮/电机时**必须**修改这些宏，否则速度计算错误
- PID 参数通过协议 ID `0x08` 动态下发

### 6. 串口发送的信号量同步
- TX 任务使用信号量等待硬件完成（见 [`SerialTxTask.c`](Core/App/Tasks/SerialTxTask.c:21)）
- DMA 完成回调释放信号量，确保帧不会重叠发送
- 队列 + 信号量的双重同步机制

### 7. printf 重定向
- 已在 [`main.c`](Core/Src/main.c:66) 重定向至 UART1
- 使用 `printf()` 直接输出调试信息到串口助手

## 代码风格
- C11 标准，GCC ARM 编译器
- 宏定义用于硬件抽象（见 `bsp_motor.h` 的 `MOTOR_*` 宏）
- 结构体用于数据传输（`ProtocolFrame_t`, `Motor_TypeDef`, `PID_Params_t`）
