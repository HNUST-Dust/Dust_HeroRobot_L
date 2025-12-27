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

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Reload
{
public:
    // 拨弹盘1个2006，控制进退弹
    MotorDmNormal motor_reload_;

    void Init();

    void Task();

    inline void SetTargetReloadOmega(float target_reload_rotation);

    inline void SetTargetReloadTorque(float target_yaw_torque);

protected:
    // Reload当前角度
    float now_reload_angle_ = 0.0f;

    // Reload当前角速度
    float now_reload_omega_ = 0.0f;

    // Reload当前力矩
    float now_reload_torque_ = 0.0f;

    // Reload当前弧度
    float now_reload_radian_ = 0.0f;

    
    // Reload目标角度
    float target_reload_angle_ = 0.0f;

    // Reload目标角速度
    float target_reload_omega_ = 0.0f;

    // Reload目标力矩
    float target_reload_torque_ = 0.0f;

    // Reload目标弧度
    float target_reload_radian_ = 0.0f;

    // Reload目标角速度
    float target_reload_rotation_ = 0.0f;

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
inline void Reload::SetTargetReloadOmega(float target_reload_rotation)
{
    target_reload_rotation_ = target_reload_rotation;
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


#endif