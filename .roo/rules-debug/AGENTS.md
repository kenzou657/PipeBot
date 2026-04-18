# AGENTS.md

This file provides guidance to agents when working with code in this repository (Debug mode).

## 关键调试发现（非显而易见）

### 1. DMA 接收数据丢失的隐藏原因
- 症状：接收到的数据不完整或乱码
- 根本原因：`HAL_UARTEx_RxEventCallback()` 后未调用 `HAL_UART_AbortReceive()` 重启 DMA
- DMA 计数器 (NDTR) 不会自动重置，导致下一帧数据被覆盖
- 解决：见 [`SerialRxTask.c:45-46`](../../Core/App/Tasks/SerialRxTask.c:45)

### 2. 串口发送永久阻塞
- 症状：`HAL_UART_Transmit_DMA()` 调用后程序卡住
- 根本原因：TX 任务启动时未调用 `osSemaphoreRelease()` 初始化信号量
- 第一次发送时信号量计数为 0，导致 `osSemaphoreAcquire()` 永久等待
- 解决：见 [`SerialTxTask.c:14`](../../Core/App/Tasks/SerialTxTask.c:14)

### 3. 校验和计算错误导致协议解析失败
- 症状：接收到的帧被丢弃或触发错误处理
- 常见错误：计算校验和时包含了 `reserved` 字节或计算范围错误
- 正确范围：前 9 字节（帧头 + ID + 长度 + 6 字节数据）
- 见 [`SerialTxTask.c:24`](../../Core/App/Tasks/SerialTxTask.c:24)

### 4. 电机速度计算完全错误
- 症状：电机转速反馈值与实际不符，PID 控制失效
- 根本原因：`bsp_motor.h` 中的 `WHEEL_DIAMETER`、`WHEEL_BASE` 或 `ENCODER_TICKS` 与实际硬件不匹配
- 这些宏直接影响 `TICKS_TO_METERS_PER_SEC` 公式
- 调试方法：打印原始编码器计数和计算后的速度，对比实际转速

### 5. printf 输出不显示
- 症状：`printf()` 调用但串口助手无输出
- 根本原因：`_write()` 函数未正确重定向或 UART1 未初始化
- 检查：`main.c:66` 的 `_write()` 实现，确保 `huart1` 已初始化
- 调试：在 `main()` 早期添加 `printf("System started\n")` 测试

### 6. FreeRTOS 任务卡死
- 症状：某个任务无响应，系统似乎冻结
- 常见原因：任务中使用了 `HAL_Delay()` 而非 `vTaskDelay()`
- `HAL_Delay()` 会阻塞整个内核调度，其他任务无法运行
- 调试：搜索所有任务代码中的 `HAL_Delay`，替换为 `osDelay()`

### 7. 堆溢出导致系统崩溃
- 症状：系统随机重启或行为异常
- 根本原因：新任务栈大小过大，超过 FreeRTOS 堆限制（15360 字节）
- 调试：检查 `FreeRTOSConfig.h:71` 的 `configTOTAL_HEAP_SIZE`
- 计算：所有任务栈大小之和不能超过堆大小

### 8. 编译失败：链接脚本错误
- 症状：编译时出现 "section `.text' will not fit in region `FLASH'" 错误
- 根本原因：`STM32F407XX_FLASH.ld` 中存在 `(READONLY)` 关键字
- 解决：删除所有 `(READONLY)` 关键字后重新编译
- 这是 CubeMX 重新生成后的常见问题

### 9. 新文件编译被忽略
- 症状：创建新的 `.c` 文件但编译时未被包含
- 根本原因：未在 `CMakeLists.txt` 的 `add_executable()` 中添加
- 解决：在 [`CMakeLists.txt:35-46`](../../CMakeLists.txt:35) 中添加新文件路径
