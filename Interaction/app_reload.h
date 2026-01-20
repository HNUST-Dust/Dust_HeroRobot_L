/**
 * @file app_reload.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-11-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __APP_RELOAD_H__
#define __APP_RELOAD_H__

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dm.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "stdio.h"

/* Exported macros -----------------------------------------------------------*/

#define MAX_RELORD_TORQUE       4.5f

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reload类
 * 
 */
class Reload
{
public:

    // 拨弹盘1个2006，控制进退弹
    MotorDmNormal motor_reload_;

    void Init();

    void Task();

    inline void SetTargetReloadOmega(float target_reload_omega);

    inline void SetTargetReloadTorque(float target_yaw_torque);

    inline float GetTargetReloadOmega();

    inline float GetTargetReloadTorque();

protected:

    // Reload当前值
    float now_reload_angle_ = 0.0f;
    float now_reload_omega_ = 0.0f;
    float now_reload_torque_ = 0.0f;
    float now_reload_radian_ = 0.0f;

    MotorDmStatus now_reload_status_ = MOTOR_DM_STATUS_ENABLE;

    // Reload目标角度
    float target_reload_angle_ = 0.0f;
    float target_reload_omega_ = 0.0f;
    float target_reload_torque_ = 0.0f;
    float target_reload_radian_ = 0.0f;

    // Reload角度差
    float reload_angle_diff = 0.0f;

    void SelfResolution();

    void Output();

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/**
 * @brief 设定Reload角速度
 * 
 * @param target_reload_rotation 
 */
inline void Reload::SetTargetReloadOmega(float target_reload_omega)
{
    target_reload_omega_ = target_reload_omega;
}

/**
 * @brief 设定Reload力矩
 * 
 * @param target_reload_rotation 
 */
inline void Reload::SetTargetReloadTorque(float target_yaw_torque)
{
    target_reload_torque_ = target_yaw_torque;
}

/**
 * @brief 
 * 
 * @return float 
 */
inline float Reload::GetTargetReloadOmega()
{
    return (target_reload_omega_);
}

/**
 * @brief 
 * 
 * @return float 
 */
inline float Reload::GetTargetReloadTorque()
{
    return (target_reload_torque_);
}


#endif