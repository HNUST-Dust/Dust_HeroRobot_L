/**
 * @file app_chassis.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-01-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef APP_CHASSIS_H_
#define APP_CHASSIS_H_

/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"
// device
#include "dvc_motor_dji.h"
// bsp
#include "cmsis_os2.h"
#include "bsp_can.h"
#include "stdio.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘控制模式枚举
 * 
 */
enum ChassisOperationMode
{
    CHASSIS_OPERATION_MODE_SPIN = 0,
    CHASSIS_OPERATION_MODE_NORMAL,
    CHASSIS_OPERATION_MODE_FOLLOW_HEAD,
    CHASSIS_OPERATION_MODE_FOLLOW_BACK,
};


class Chassis
{
public:
    // 底盘4个3508，控制全向轮
    MotorDjiC620 motor_chassis_1_,
                 motor_chassis_2_,
                 motor_chassis_3_,
                 motor_chassis_4_;

    // 底盘跟随pid
    Pid chassis_follow_pid_;

    void Init();

    void Task();

    inline void SetChassisOperationMode(ChassisOperationMode chassis_opreation_mode);

    inline void SetTargetVxInGimbal(float target_velocity_x);

    inline void SetTargetVyInGimbal(float target_velocity_y);

    inline void SetTargetVelocityRotation(float target_velocity_rotation);

    inline void SetNowYawRadianDiff(float yaw_radian_diff);

    inline void SetMaxOmegaSpeed(float max_omega_speed);

    inline float GetMaxOmegaSpeed();

protected:
    // 底盘操作模式
    ChassisOperationMode chassis_opreation_mode_ = CHASSIS_OPERATION_MODE_NORMAL;

    // 云台坐标系目标速度
    float target_vx_in_gimbal_ = 0.0f;
    float target_vy_in_gimbal_ = 0.0f;

    // 底盘坐标系目标速度
    float target_vx_in_chassis_ = 0.0f;
    float target_vy_in_chassis_ = 0.0f;

    // 目标速度 旋转
    float target_velocity_rotation_ = 0.0f;

    // yaw轴角度差
    float yaw_radian_diff_ = 0.0f;

    // 斜坡规划参数
    float max_accel_xy_ = 100.f;
    float max_accel_r_  = 100.f;

    float now_accel_x_ = 0.0f;
    float now_accel_y_ = 0.0f;
    float now_accel_r_ = 0.0f;

    float last_target_vx_ = 0.0f;
    float last_target_vy_ = 0.0f;
    float last_target_rotation_ = 0.0f;

    float max_omega_speed_ = 12.f;

    void OperationMode();

    void RotationMatrixTransform();

    void SlopePlanning();

    void KinematicsInverseResolution();

    void OutputToMotor();

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/**
 * @brief 切换底盘操作模式
 * 
 * @param chassis_opreation_mode 
 */
inline void Chassis::SetChassisOperationMode(ChassisOperationMode chassis_opreation_mode)
{
    chassis_opreation_mode_ = chassis_opreation_mode;
}

/**
 * @brief 设定目标速度X
 *
 * @param target_velocity_x 目标速度X
 */
inline void Chassis::SetTargetVxInGimbal(float target_vx_in_gimbal)
{
    target_vx_in_gimbal_ = target_vx_in_gimbal;
}

/**
 * @brief 设定目标速度Y
 *
 * @param target_velocity_y 目标速度Y
 */
inline void Chassis::SetTargetVyInGimbal(float target_vy_in_gimbal)
{
    target_vy_in_gimbal_ = target_vy_in_gimbal;
}

/**
 * @brief 设定目标速度旋转
 *
 * @param target_velocity_rotation 目标速度Y
 */
inline void Chassis::SetTargetVelocityRotation(float target_velocity_rotation)
{
    target_velocity_rotation_ = target_velocity_rotation;
}

/**
 * @brief 设定现在yaw角偏差
 * 
 * @param now_yawdiff 
 */
inline void Chassis::SetNowYawRadianDiff(float yaw_radian_diff)
{
    yaw_radian_diff_ = yaw_radian_diff;
}

/**
 * @brief 设置底盘最大角速度
 * 
 * @param max_omega_speed
 */
inline void Chassis::SetMaxOmegaSpeed(float max_omega_speed)
{
    max_omega_speed_ = max_omega_speed;
}

/**
 * @brief 获取底盘最大角速度
 * 
 * @return float
 */
inline float Chassis::GetMaxOmegaSpeed()
{
    return max_omega_speed_;
}

#endif