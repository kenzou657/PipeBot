# AGENTS.md

This file provides guidance to agents when working with code in this repository (Ask mode).

## 关键文档与架构理解（非显而易见）

### 1. 串口通信协议的完整映射
- 协议采用固定 11 字节帧格式：`[0x55|ID|0x06|Data[6]|Checksum|0xBB]`
- **ID 映射**（见 `.roorules` 第 3.2 节）：
  - `0x01`: 下发速度控制（左右电机方向和速度）
  - `0x02`: 上传速度反馈
  - `0x03-0x05`: IMU 数据（加速度、角速度、欧拉角）
  - `0x06`: 测距模块数据（左右距离）
  - `0x07`: 系统状态（LED、蜂鸣器、IMU 校准）
  - `0x08`: PID 参数（P/I/D 系数）
- 所有 16-bit 数据采用高低字节拆分，接收端用 `(H << 8 | L)` 拼接

### 2. ProtocolFrame_t 结构体的隐藏设计
- 定义在 [`Core/App/Types/LEDType.h:27`](../../Core/App/Types/LEDType.h:27)
- 结构体大小为 12 字节（含 `reserved` 补齐位）
- 但协议只发送 11 字节，`reserved` 不被传输
- 这是为了内存对齐而设计的，避免编译器自动填充导致的问题

### 3. 电机控制的参数化设计
- 所有电机参数集中在 [`bsp_motor.h`](../../Core/Bsp/bsp_motor.h:11-16)
- 关键宏：
  - `WHEEL_DIAMETER`: 轮径（单位：米）
  - `WHEEL_BASE`: 轮距（单位：米）
  - `ENCODER_TICKS`: 编码器脉冲数（包含减速比和倍频）
  - `SAMPLE_TIME`: 采样周期（20ms）
- 这些宏直接影响速度计算公式 `TICKS_TO_METERS_PER_SEC`
- 更换电机或轮子时必须同时修改这些宏

### 4. FreeRTOS 堆大小的严格限制
- 堆大小仅 15360 字节（见 [`FreeRTOSConfig.h:71`](../../Core/Inc/FreeRTOSConfig.h:71)）
- 这是 STM32F407VET6 的 192KB RAM 中分配的
- 所有任务栈、队列、信号量都从这个堆中分配
- 新增任务需谨慎计算栈大小，避免堆溢出

### 5. DMA 接收的设计陷阱
- 使用 `HAL_UARTEx_ReceiveToIdle_DMA()` 实现空闲中断接收
- 每次中断后**必须**重启 DMA（调用 `HAL_UART_AbortReceive()` 然后重新初始化）
- 否则 DMA 计数器 (NDTR) 不会重置，导致数据丢失
- 这是 STM32 HAL 库的已知行为，不是 bug

### 6. 串口发送的双重同步机制
- TX 任务使用**队列 + 信号量**的组合：
  - 队列：存储待发送的帧数据
  - 信号量：等待 DMA 硬件完成
- 初始化时必须先 `osSemaphoreRelease()` 一次，表示硬件空闲
- DMA 完成回调释放信号量，确保帧不会重叠发送

### 7. printf 重定向的实现
- 已在 [`main.c:66`](../../Core/Src/main.c:66) 重定向 `_write()` 函数
- 输出直接发送到 UART1（通过 `HAL_UART_Transmit()` 阻塞模式）
- 可用于实时调试和参数可视化

### 8. CubeMX 代码保护的重要性
- 所有业务逻辑必须在 `/* USER CODE BEGIN/END */` 块内
- CubeMX 重新生成时会保留这些块，但删除块外的代码
- 每次重新生成后必须检查 `STM32F407XX_FLASH.ld` 删除 `(READONLY)` 关键字

### 9. CMake 构建系统的文件注册
- 新增 `.c` 文件必须在 [`CMakeLists.txt:35-46`](../../CMakeLists.txt:35) 的 `add_executable()` 中添加
- 否则编译时文件被忽略，导致链接错误
- 这是 CMake 的标准行为，不是项目特定的

### 10. 项目架构的分层设计
- **应用层** (`Core/App/Tasks/`): FreeRTOS 任务实现
- **硬件抽象层** (`Core/Bsp/`): 电机、IMU、测距模块驱动
- **底层驱动** (`Core/Src/`): CubeMX 生成的硬件初始化
- **中间件** (`Middlewares/`): FreeRTOS 内核
