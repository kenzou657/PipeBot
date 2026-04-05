//
// Created by asus on 2026/3/17.
//

#include "bsp_motor.h"

#include "bsp_usart_dma.h"
#include "tim.h"
#include "Types/LEDType.h"

Motor_TypeDef MotorL, MotorR;

PID_Params_t MotorL_PID = {4.1f, 0.15f, 0.0f};
PID_Params_t MotorR_PID = {4.1f, 0.15f, 0.0f};

// 计算比例因子：将脉冲增量直接转为 mm/s
// 公式：(delta_ticks / 1320) * (PI * D) / 0.02
const float TICKS_TO_MM_PER_SEC = TICKS_TO_METERS_PER_SEC * 1000.0f;

void MotorL_Speed(int16_t speed_mm_s) {
    int16_t speed = Speed_To_PWM(speed_mm_s);

    if (speed > 0) {
        MOTOR_L_FORWARD();
        __HAL_TIM_SET_COMPARE(MOTOR_LEFT_TIM, MOTOR_LEFT_CHANNEL, speed);
    } else if (speed < 0) {
        MOTOR_L_BACKWARD();
        __HAL_TIM_SET_COMPARE(MOTOR_LEFT_TIM, MOTOR_LEFT_CHANNEL, -speed);
    } else {
        MOTOR_L_STOP();
        __HAL_TIM_SET_COMPARE(MOTOR_LEFT_TIM, MOTOR_LEFT_CHANNEL, 0);
    }
}

void MotorR_Speed(int16_t speed_mm_s) {
    int16_t speed = Speed_To_PWM(speed_mm_s);

    if (speed > 0) {
        MOTOR_R_FORWARD();
        __HAL_TIM_SET_COMPARE(MOTOR_RIGHT_TIM, MOTOR_RIGHT_CHANNEL, speed);
    } else if (speed < 0) {
        MOTOR_R_BACKWARD();
        __HAL_TIM_SET_COMPARE(MOTOR_RIGHT_TIM, MOTOR_RIGHT_CHANNEL, -speed);
    } else {
        MOTOR_R_STOP();
        __HAL_TIM_SET_COMPARE(MOTOR_RIGHT_TIM, MOTOR_RIGHT_CHANNEL, 0);
    }
}

/* 前进：两轮同速向前 */
void Move_Forward(int16_t speed_mm_s) {
    MotorL.target_velocity = (float)speed_mm_s;
    MotorR.target_velocity = (float)speed_mm_s;

    float MotorL_output = PID_Incremental_Compute(&MotorL_PID, MotorL.target_velocity, MotorL.velocity);
    float MotorR_output = PID_Incremental_Compute(&MotorR_PID, MotorR.target_velocity, MotorR.velocity);

    MotorL_Speed((int16_t)MotorL_output);
    MotorR_Speed((int16_t)MotorR_output);
}

/* 后退：两轮同速向后 */
void Move_Backward(int16_t speed_mm_s) {
    MotorL.target_velocity = (float)speed_mm_s;
    MotorR.target_velocity = (float)speed_mm_s;

    float MotorL_output = PID_Incremental_Compute(&MotorL_PID, MotorL.target_velocity, MotorL.velocity);
    float MotorR_output = PID_Incremental_Compute(&MotorR_PID, MotorR.target_velocity, MotorR.velocity);

    MotorL_Speed(-(int16_t)MotorL_output);
    MotorR_Speed(-(int16_t)MotorR_output);
}

/* 原地左转：左轮后退，右轮前进 */
void Turn_Left(int16_t speed_mm_s) {
    MotorL.target_velocity = (float)speed_mm_s;
    MotorR.target_velocity = (float)speed_mm_s;

    float MotorL_output = PID_Incremental_Compute(&MotorL_PID, MotorL.target_velocity, MotorL.velocity);
    float MotorR_output = PID_Incremental_Compute(&MotorR_PID, MotorR.target_velocity, MotorR.velocity);

    MotorL_Speed(-(int16_t)MotorL_output);
    MotorR_Speed((int16_t)MotorR_output);
}

/* 原地右转：左轮后退，右轮前进 */
void Turn_Right(int16_t speed_mm_s) {
    MotorL.target_velocity = (float)speed_mm_s;
    MotorR.target_velocity = (float)speed_mm_s;

    float MotorL_output = PID_Incremental_Compute(&MotorL_PID, MotorL.target_velocity, MotorL.velocity);
    float MotorR_output = PID_Incremental_Compute(&MotorR_PID, MotorR.target_velocity, MotorR.velocity);

    MotorL_Speed((int16_t)MotorL_output);
    MotorR_Speed(-(int16_t)MotorR_output);
}

/* 停止 */
void Motor_Stop(void) {
    MotorL_Speed(0);
    MotorR_Speed(0);
}

void Motor_Read_Encoder(void) {
    // 1. 读取左右编码器的计数值
    MotorL.encoder_count = -(int16_t)__HAL_TIM_GET_COUNTER(MOTOR_LEFT_ENCODER_TIM);
    MotorR.encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(MOTOR_RIGHT_ENCODER_TIM);

    // 2. 清零计数器，确保下次读取的是这 10ms 内的增量
    __HAL_TIM_SET_COUNTER(MOTOR_LEFT_ENCODER_TIM, 0);
    __HAL_TIM_SET_COUNTER(MOTOR_RIGHT_ENCODER_TIM, 0);

    // 3. 计算物理速度(mm/s)
    MotorL.velocity = (int16_t)((float)MotorL.encoder_count * TICKS_TO_MM_PER_SEC);
    MotorR.velocity = (int16_t)((float)MotorR.encoder_count * TICKS_TO_MM_PER_SEC);

    // 4. 累计总里程 (用于里程计/定位)
    MotorL.total_count += MotorL.encoder_count;
    MotorR.total_count += MotorR.encoder_count;
}

int16_t Speed_To_PWM(int16_t speed_mm_s) {
    // 电机在 100% 占空比下跑 670mm/s
    float pwm = (float)speed_mm_s * (MOTOR_MAX_PWM / 670.0f);

    if (pwm > MOTOR_MAX_PWM) pwm = MOTOR_MAX_PWM;
    if (pwm < -MOTOR_MAX_PWM) pwm = -MOTOR_MAX_PWM;

    return (int16_t)pwm;
}

void Pack_Kinematics_To_Frame(ProtocolFrame_t *frame, int16_t v_l_mm, int16_t v_r_mm) {
    if (frame == NULL) return;

    // 4. 按照“方向+高低位”格式填充 Data 域 (共 6 字节)
    // [0-2]: 左轮方向, 高8, 低8
    frame->data[0] = (v_l_mm >= 0) ? 0x00 : 0x01;   //0正1反
    uint16_t abs_l = (uint16_t)(v_l_mm >= 0 ? v_l_mm : -v_l_mm);
    frame->data[1] = (uint8_t)(abs_l >> 8);
    frame->data[2] = (uint8_t)(abs_l & 0xFF);

    // [3-5]: 右轮方向, 高8, 低8
    frame->data[3] = (v_r_mm >= 0) ? 0x00 : 0x01;
    uint16_t abs_r = (uint16_t)(v_r_mm >= 0 ? v_r_mm : -v_r_mm);
    frame->data[4] = (uint8_t)(abs_r >> 8);
    frame->data[5] = (uint8_t)(abs_r & 0xFF);

}

/**
 * @brief  增量式 PID 计算函数
 * @formula: Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
 */
float PID_Incremental_Compute(PID_Params_t *pid, float target, float measure) {
    pid->target = target;
    pid->current = measure;

    // 1. 计算当前误差
    pid->err = pid->target - pid->current;

    // 2. 增量式 PID 公式计算
    float p_term = pid->Kp * (pid->err - pid->last_err);
    float i_term = pid->Ki * pid->err;
    float d_term = pid->Kd * (pid->err - 2.0f * pid->last_err + pid->prev_err);

    float delta_out = p_term + i_term + d_term;

    // 3. 累加到最终输出
    pid->output += delta_out;

    // 4. 更新误差记录，为下一次计算做准备
    pid->prev_err = pid->last_err;
    pid->last_err = pid->err;

    // 5. 输出限幅（重要！防止 PWM 超过定时器自动重装载值）
    if (pid->output > 670.0f)  pid->output = 670.0f;
    if (pid->output < -670.0f) pid->output = -670.0f;

    return pid->output;
}

/**
 * @brief 从协议数据位更新 PID 参数
 * @param pData 指向 ProtocolFrame_t 中的 data 数组起始地址 (即 frame->data)
 */
void Motor_Update_PID_From_Protocol(uint8_t *pData) {
    /* * 协议定义：大端模式 (高位在前，低位在后)
     * Data[0-1]: Kp, Data[2-3]: Ki, Data[4-5]: Kd
     * 这里的 pData[0] 对应结构体中的 data[0]
     */

    // Kp 解析：data[0]是高8位，data[1]是低8位
    uint16_t kp_raw = (uint16_t)((pData[0] << 8) | pData[1]);
    MotorL_PID.Kp = (float)kp_raw / 100.0f;
    MotorR_PID.Kp = (float)kp_raw / 100.0f;

    // Ki 解析：data[2]是高8位，data[3]是低8位
    uint16_t ki_raw = (uint16_t)((pData[2] << 8) | pData[3]);
    MotorL_PID.Ki = (float)ki_raw / 100.0f;
    MotorR_PID.Ki = (float)ki_raw / 100.0f;

    // Kd 解析：data[4]是高8位，data[5]是低8位
    uint16_t kd_raw = (uint16_t)((pData[4] << 8) | pData[5]);
    MotorL_PID.Kd = (float)kd_raw / 100.0f;
    MotorR_PID.Kd = (float)kd_raw / 100.0f;

    /* * 顶级工程师提示：更新完 PID 后，如果是运行中调节，
     * 建议在此处调用 PID_Init 或清空积分项，防止参数突变导致系统震荡。
     */
}
