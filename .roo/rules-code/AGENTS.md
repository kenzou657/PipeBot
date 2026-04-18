# AGENTS.md

This file provides guidance to agents when working with code in this repository (Code mode).

## 关键编码规则（非显而易见）

### 1. ProtocolFrame_t 的 12 字节陷阱
- 结构体定义为 12 字节（含 `reserved` 补齐位），但协议只发送 11 字节
- TX 任务中必须显式指定 `HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&txFrame, 11)` 而非 `sizeof(txFrame)`
- 见 [`SerialTxTask.c:27`](../../Core/App/Tasks/SerialTxTask.c:27)

### 2. DMA 接收必须重启
- 每次 `HAL_UARTEx_RxEventCallback()` 触发后，**必须**调用 `HAL_UART_AbortReceive()` 然后重新初始化 DMA
- 不重启会导致 NDTR 计数器不重置，下一帧数据丢失
- 见 [`SerialRxTask.c:45-46`](../../Core/App/Tasks/SerialRxTask.c:45)

### 3. 信号量初始化顺序
- TX/RX 任务启动时必须先 `osSemaphoreRelease()` 一次，表示硬件初始状态空闲
- 否则第一次发送会永久阻塞
- 见 [`SerialTxTask.c:14`](../../Core/App/Tasks/SerialTxTask.c:14)

### 4. 电机参数必须同步修改
- 更换电机/轮子时，必须同时修改 [`bsp_motor.h`](../../Core/Bsp/bsp_motor.h) 中的：
  - `WHEEL_DIAMETER` (轮径)
  - `WHEEL_BASE` (轮距)
  - `ENCODER_TICKS` (编码器脉冲数)
- 这些宏直接影响速度计算公式 `TICKS_TO_METERS_PER_SEC`

### 5. 新文件必须在 CMakeLists.txt 中注册
- 创建新的 `.c` 文件后，必须在 [`CMakeLists.txt:35-46`](../../CMakeLists.txt:35) 的 `add_executable()` 中添加
- 否则编译时文件被忽略

### 6. CubeMX 重新生成后的必做项
- 删除 [`STM32F407XX_FLASH.ld`](../../STM32F407XX_FLASH.ld) 中所有 `(READONLY)` 关键字
- 检查 `/* USER CODE BEGIN/END */` 块是否被保留
- 验证 `CMakeLists.txt` 中的源文件列表是否被覆盖

### 7. 禁止在任务中使用 HAL_Delay
- 必须使用 `vTaskDelay()` 或 `osDelay()`
- `HAL_Delay()` 会阻塞整个 FreeRTOS 内核，导致其他任务无法运行

### 8. 堆大小限制
- FreeRTOS 堆仅 15360 字节（见 [`FreeRTOSConfig.h:71`](../../Core/Inc/FreeRTOSConfig.h:71)）
- 新任务的栈大小需谨慎计算，避免堆溢出导致系统崩溃
