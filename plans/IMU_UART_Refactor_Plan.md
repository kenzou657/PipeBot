# JY61P IMU 从 I2C 迁移到 UART2 重构方案

## 📋 概述

将 JY61P IMU 传感器的通信方式从 **I2C 阻塞式** 改为 **UART2 非阻塞式 DMA**，与项目中 TOF 传感器的实现方式保持一致。

---

## 🎯 设计目标

1. **保持架构一致性**: 与 TOF 传感器的 DMA 接收模式相同
2. **非阻塞式操作**: 避免阻塞 IMUTask，提高系统响应性
3. **支持三种数据帧**: 加速度(0x51)、角速度(0x52)、欧拉角(0x53)
4. **校验和验证**: 确保数据完整性
5. **最小化改动**: 复用现有的 DMA 框架

---

## 📊 JY61P UART 协议分析

### 三种数据帧格式

#### 1. 加速度帧 (0x51)
```
[0x55] [0x51] [AxL] [AxH] [AyL] [AyH] [AzL] [AzH] [TL] [TH] [SUM]
 11字节
```
- 加速度: X=((AxH<<8)|AxL)/32768*16g
- 温度: T=((TH<<8)|TL)/100°C
- 校验: SUM = 0x55+0x51+AxL+AxH+AyL+AyH+AzL+AzH+TL+TH

#### 2. 角速度帧 (0x52)
```
[0x55] [0x52] [WxL] [WxH] [WyL] [WyH] [WzL] [WzH] [VolL] [VolH] [SUM]
 11字节
```
- 角速度: X=((WxH<<8)|WxL)/32768*2000°/s
- 电压: V=((VolH<<8)|VolL)/100V
- 校验: SUM = 0x55+0x52+WxL+WxH+WyL+WyH+WzL+WzH+VolL+VolH

#### 3. 欧拉角帧 (0x53)
```
[0x55] [0x53] [RollL] [RollH] [PitchL] [PitchH] [YawL] [YawH] [VL] [VH] [SUM]
 11字节
```
- 欧拉角: Roll=((RollH<<8)|RollL)/32768*180°
- 版本号: V=(VH<<8)|VL
- 校验: SUM = 0x55+0x53+RollL+RollH+PitchL+PitchH+YawL+YawH+VL+VH

---

## 🔧 需要修改的文件

### 1. **Core/Bsp/bsp_jy61p.h** (头文件)
**修改内容**:
- 删除 I2C 相关的寄存器定义
- 添加 UART 帧格式定义
- 添加接收缓冲区声明
- 添加新的解析函数声明

**新增定义**:
```c
#define JY61P_FRAME_SIZE        11      // 每帧 11 字节
#define JY61P_FRAME_ACC         0x51    // 加速度帧
#define JY61P_FRAME_GYRO        0x52    // 角速度帧
#define JY61P_FRAME_ANGLE       0x53    // 欧拉角帧

// 接收缓冲区 (双缓冲防溢出)
extern uint8_t g_jy61p_buf[JY61P_FRAME_SIZE * 2];
extern uint16_t g_jy61p_rx_len;

// 解析函数
int8_t JY61P_Parse_Stream(uint8_t *raw_buf, uint16_t len);
void JY61P_Decode_Acc(JY61P_Data_t *pDest, uint8_t *pSrc);
void JY61P_Decode_Gyro(JY61P_Data_t *pDest, uint8_t *pSrc);
void JY61P_Decode_Angle(JY61P_Data_t *pDest, uint8_t *pSrc);
```

### 2. **Core/Bsp/bsp_jy61p.c** (实现文件)
**修改内容**:
- 删除 I2C 读取相关代码
- 添加接收缓冲区定义
- 重写 `JY61P_Decode()` 为三个独立的解析函数
- 添加流式解析函数 `JY61P_Parse_Stream()`
- 添加校验和计算函数

**新增函数**:
```c
// 接收缓冲区
uint8_t g_jy61p_buf[JY61P_FRAME_SIZE * 2];
uint16_t g_jy61p_rx_len = 0;

// 校验和计算
uint8_t JY61P_Calculate_Checksum(uint8_t *pData, uint16_t len);

// 流式解析 (返回 0 成功, -1 未找到帧头, -2 长度不足, -3 校验失败)
int8_t JY61P_Parse_Stream(uint8_t *raw_buf, uint16_t len);

// 三种帧的解析函数
void JY61P_Decode_Acc(JY61P_Data_t *pDest, uint8_t *pSrc);
void JY61P_Decode_Gyro(JY61P_Data_t *pDest, uint8_t *pSrc);
void JY61P_Decode_Angle(JY61P_Data_t *pDest, uint8_t *pSrc);
```

### 3. **Core/App/Tasks/IMUTask.c** (任务文件)
**修改内容**:
- 删除 I2C 非阻塞读取逻辑
- 删除 `HAL_I2C_MemRxCpltCallback()` 和 `HAL_I2C_ErrorCallback()`
- 改为 UART DMA 接收模式
- 使用信号量等待 UART IDLE 中断
- 调用新的流式解析函数

**新的任务流程**:
```c
void StartIMUTask(void *argument) {
    // 1. 初始化 UART2 DMA 接收
    UART_Device_Init(&huart2, g_jy61p_buf, sizeof(g_jy61p_buf));
    osSemaphoreRelease(Sem_IMURxCompleteHandle);
    
    for(;;) {
        // 2. 等待 UART IDLE 中断
        if (osSemaphoreAcquire(Sem_IMURxCompleteHandle, osWaitForever) == osOK) {
            
            // 3. 流式解析缓冲区中的所有帧
            JY61P_Parse_Stream(g_jy61p_buf, g_jy61p_rx_len);
            
            // 4. 重置接收长度
            g_jy61p_rx_len = 0;
        }
    }
}

// UART IDLE 中断回调 (在 stm32f4xx_it.c 中)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2) {
        g_jy61p_rx_len = Size;
        osSemaphoreRelease(Sem_IMURxCompleteHandle);
        
        // 重启 DMA
        HAL_UART_AbortReceive(huart);
        UART_Device_Init(&huart2, g_jy61p_buf, sizeof(g_jy61p_buf));
    }
}
```

### 4. **Core/Inc/main.h** (全局声明)
**修改内容**:
- 删除 `Sem_I2CHandle` 信号量声明
- 添加 `Sem_IMURxCompleteHandle` 信号量声明

```c
// 删除
// extern osSemaphoreId_t Sem_I2CHandle;

// 添加
extern osSemaphoreId_t Sem_IMURxCompleteHandle;
```

### 5. **Core/Src/freertos.c** (FreeRTOS 配置)
**修改内容**:
- 删除 I2C 信号量创建代码
- 添加 IMU UART 信号量创建代码
- 删除 IMUTask 中的 I2C 相关初始化

```c
// 删除
// Sem_I2CHandle = osSemaphoreNew(1, 0, NULL);

// 添加
Sem_IMURxCompleteHandle = osSemaphoreNew(1, 0, NULL);
```

### 6. **Core/Src/stm32f4xx_it.c** (中断处理)
**修改内容**:
- 删除 I2C 中断处理代码
- 在 `HAL_UARTEx_RxEventCallback()` 中添加 USART2 处理分支

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        // 现有的 UART1 处理...
    }
    else if (huart->Instance == USART2) {
        // 新增的 UART2 (IMU) 处理
        g_jy61p_rx_len = Size;
        osSemaphoreRelease(Sem_IMURxCompleteHandle);
        HAL_UART_AbortReceive(huart);
        UART_Device_Init(&huart2, g_jy61p_buf, sizeof(g_jy61p_buf));
    }
    // ... 其他 UART 处理
}
```

### 7. **CMakeLists.txt** (构建配置)
**修改内容**:
- 无需修改（bsp_jy61p.c 已在列表中）

### 8. **PipeBot.ioc** (CubeMX 配置)
**修改内容**:
- 禁用 I2C1
- 启用 USART2 (UART 模式)
- 配置 USART2 DMA 接收
- 配置 USART2 IDLE 中断
- 重新生成代码

**USART2 配置**:
- 波特率: 9600 bps (根据 JY61P 实际配置)
- 数据位: 8
- 停止位: 1
- 校验: 无
- DMA: 启用接收 DMA

---

## 📝 实现步骤

### 第一阶段: 准备工作
1. ✅ 在 CubeMX 中禁用 I2C1，启用 USART2
2. ✅ 配置 USART2 DMA 和 IDLE 中断
3. ✅ 重新生成 CubeMX 代码

### 第二阶段: 修改 BSP 层
1. ✅ 更新 `bsp_jy61p.h` (添加 UART 定义)
2. ✅ 重写 `bsp_jy61p.c` (实现流式解析)
3. ✅ 添加三个独立的解码函数

### 第三阶段: 修改应用层
1. ✅ 重写 `IMUTask.c` (改为 UART DMA 模式)
2. ✅ 更新 `main.h` (修改信号量声明)
3. ✅ 更新 `freertos.c` (修改信号量创建)

### 第四阶段: 修改中断处理
1. ✅ 更新 `stm32f4xx_it.c` (添加 USART2 处理)

### 第五阶段: 测试验证
1. ✅ 编译验证
2. ✅ 烧录测试
3. ✅ 串口调试验证数据

---

## 🔄 数据流对比

### 原 I2C 方式
```
IMUTask (阻塞)
  ↓
HAL_I2C_Mem_Read_IT() (非阻塞发起)
  ↓
等待 I2C 中断完成 (osSemaphoreAcquire)
  ↓
JY61P_Decode() (解析单次读取的 24 字节)
  ↓
更新 imu_data
```

### 新 UART DMA 方式
```
IMUTask (非阻塞)
  ↓
等待 UART IDLE 中断 (osSemaphoreAcquire)
  ↓
JY61P_Parse_Stream() (流式解析缓冲区)
  ↓
根据帧类型调用对应解码函数
  ↓
更新 imu_data
```

---

## ⚠️ 关键注意事项

### 1. 帧类型识别
- 需要根据第二字节 (0x51/0x52/0x53) 识别帧类型
- 每种帧类型的解析方式不同

### 2. 校验和计算
- 加速度帧: SUM = 0x55+0x51+AxL+AxH+AyL+AyH+AzL+AzH+TL+TH
- 角速度帧: SUM = 0x55+0x52+WxL+WxH+WyL+WyH+WzL+WzH+VolL+VolH
- 欧拉角帧: SUM = 0x55+0x53+RollL+RollH+PitchL+PitchH+YawL+YawH+VL+VH

### 3. 缓冲区大小
- 设为 `JY61P_FRAME_SIZE * 2 = 22 字节`
- 防止 DMA 溢出

### 4. 波特率配置
- 确认 JY61P 的实际波特率（通常 9600 或 115200）
- 在 CubeMX 中正确配置

### 5. 数据更新频率
- UART 接收频率取决于 JY61P 的输出频率
- 通常为 50Hz 或 100Hz

---

## 🧪 测试验证清单

- [ ] 编译无错误
- [ ] 烧录成功
- [ ] UART2 能接收到数据
- [ ] 校验和验证通过
- [ ] 加速度数据正确解析
- [ ] 角速度数据正确解析
- [ ] 欧拉角数据正确解析
- [ ] IMUTask 不阻塞其他任务
- [ ] printf 输出数据合理

---

## 📚 参考文件

- 现有 TOF 实现: `Core/Bsp/bsp_tof050f.c`
- 现有 UART DMA 框架: `Core/Bsp/bsp_usart_dma.c`
- 现有 IMU 任务: `Core/App/Tasks/IMUTask.c`

---

## 🎓 架构优势

1. **一致性**: 与 TOF 传感器采用相同的 DMA 接收模式
2. **非阻塞**: IMUTask 不再阻塞，系统响应性提高
3. **可扩展**: 支持多种帧类型，易于添加新功能
4. **可靠性**: 校验和验证确保数据完整性
5. **易维护**: 流式解析框架与 TOF 类似，便于理解和维护
