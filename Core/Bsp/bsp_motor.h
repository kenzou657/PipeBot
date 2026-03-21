//
// Created by asus on 2026/3/17.
//
#ifndef PIPEBOT_BSP_MOTOR_H
#define PIPEBOT_BSP_MOTOR_H

#include "stm32f4xx_hal.h"
#include "Types/LEDType.h"
#include "main.h"

#define WHEEL_DIAMETER    0.0423f   // 轮径 42.3mm (0.0423m)
#define WHEEL_BASE        0.148f   // 轮距 148mm (0.148m)，根据实际履带中心距修改
#define ENCODER_TICKS     1560.0f  // MG513P30 4倍频+30减速比+13ppr后的总脉冲数
#define SAMPLE_TIME       0.02f    // 采样周期 20ms (50Hz)
#define MOTOR_MAX_PWM     16799
#define TICKS_TO_METERS_PER_SEC ((3.1415926f * WHEEL_DIAMETER) / (ENCODER_TICKS * SAMPLE_TIME))

extern const float TICKS_TO_MM_PER_SEC;

#define MOTOR_LEFT_TIM            &htim1
#define MOTOR_LEFT_CHANNEL        TIM_CHANNEL_4
#define MOTOR_LEFT_ENCODER_TIM    &htim4
#define AIN1_GPIO_Port            GPIOA
#define AIN1_Pin                  GPIO_PIN_3
#define AIN2_GPIO_Port            GPIOA
#define AIN2_Pin                  GPIO_PIN_4

#define MOTOR_RIGHT_TIM           &htim1
#define MOTOR_RIGHT_CHANNEL       TIM_CHANNEL_1
#define MOTOR_RIGHT_ENCODER_TIM   &htim3
#define BIN1_GPIO_Port            GPIOA
#define BIN1_Pin                  GPIO_PIN_1
#define BIN2_GPIO_Port            GPIOA
#define BIN2_Pin                  GPIO_PIN_2

#define MOTOR_PWM_START() do { \
HAL_TIM_PWM_Start(MOTOR_LEFT_TIM, MOTOR_LEFT_CHANNEL);   \
HAL_TIM_PWM_Start(MOTOR_RIGHT_TIM, MOTOR_RIGHT_CHANNEL); \
} while(0)

#define MOTOR_PWM_STOP() do { \
HAL_TIM_PWM_Stop(MOTOR_LEFT_TIM, MOTOR_LEFT_CHANNEL);    \
HAL_TIM_PWM_Stop(MOTOR_RIGHT_TIM, MOTOR_RIGHT_CHANNEL);  \
} while(0)

#define MOTOR_ENCODER_START() do { \
HAL_TIM_Encoder_Start(MOTOR_LEFT_ENCODER_TIM, TIM_CHANNEL_ALL);  \
HAL_TIM_Encoder_Start(MOTOR_RIGHT_ENCODER_TIM, TIM_CHANNEL_ALL); \
} while(0)

#define MOTOR_ENCODER_STOP() do { \
HAL_TIM_Encoder_Stop(MOTOR_LEFT_ENCODER_TIM, TIM_CHANNEL_ALL);  \
HAL_TIM_Encoder_Stop(MOTOR_RIGHT_ENCODER_TIM, TIM_CHANNEL_ALL); \
} while(0)

// 左电机方向控制
#define MOTOR_L_FORWARD()  do{ HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET); \
HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET); }while(0)
#define MOTOR_L_BACKWARD() do{ HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET); \
HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET); }while(0)
#define MOTOR_L_STOP()     do{ HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET); \
HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET); }while(0)

// 右电机方向控制
#define MOTOR_R_FORWARD()  do{ HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET); \
HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET); }while(0)
#define MOTOR_R_BACKWARD() do{ HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET); \
HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET); }while(0)
#define MOTOR_R_STOP()     do{ HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET); \
HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET); }while(0)

typedef struct {
    int16_t  encoder_count;    // 原始脉冲计数值
    int16_t  velocity;         // 实际物理速度 (mm/s)
    float    target_velocity;  // 目标速度
    int32_t  total_count;      // 累计总脉冲数 (用于路程计算)
} Motor_TypeDef;

// 外部声明
extern Motor_TypeDef MotorL, MotorR;

void MotorL_Speed(int16_t speed);
void MotorR_Speed(int16_t speed);
void Move_Forward(int16_t speed);
void Move_Backward(int16_t speed);
void Turn_Left(int16_t speed);
void Turn_Right(int16_t speed);
void Motor_Stop(void);
void Motor_Read_Encoder(void);
int16_t Speed_To_PWM(int16_t speed_mm_s);
void Pack_Kinematics_To_Frame(ProtocolFrame_t *frame, int16_t v_l_mm, int16_t v_r_mm);
float PID_Incremental_Compute(PID_Params_t *pid, float target, float measure);

#endif //PIPEBOT_BSP_MOTOR_H
