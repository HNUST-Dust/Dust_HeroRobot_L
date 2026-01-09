/**
 * @file app_reload.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "app_reload.h"
#include "alg_math.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Reload初始化函数
 * 
 */
void Reload::Init()
{
    // 拨弹盘4310电机初始化
    motor_reload_.Init(&hcan2, 0x08, 0x07, MOTOR_DM_CONTROL_METHOD_NORMAL_MIT, 12.5f, 30.f, 10.f);
    
    motor_reload_.CanSendClearError();
    osDelay(pdMS_TO_TICKS(1000));

    motor_reload_.CanSendEnter();
    osDelay(pdMS_TO_TICKS(1000));

    motor_reload_.SetKp(0);

    motor_reload_.SetKd(0);

    motor_reload_.SetControlTorque(0);

    motor_reload_.Output();

    static const osThreadAttr_t kReloadTaskAttr = 
    {
        .name = "reload_task",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Reload::TaskEntry, this, &kReloadTaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void Reload::TaskEntry(void *argument)
{
    Reload *self = static_cast<Reload *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief Reload自身解算
 * 
 */
void Reload::SelfResolution()
{
    now_reload_omega_ = motor_reload_.GetNowOmega();
    now_reload_angle_ = normalize_angle(motor_reload_.GetNowAngle() / PI * 180.f);
    now_reload_torque_ = motor_reload_.GetNowTorque();
}

/**
 * @brief Reload输出函数
 * 
 */
void Reload::Output()
{
    motor_reload_.SetControlTorque(target_reload_torque_);
    motor_reload_.Output();
}

/**
 * @brief Reload卡弹处理函数
 * 
 */
void Reload::MisFireProcess()
{
    static uint8_t entry_flag = 0;

    // 当稳定为开火模式时才允许进入卡弹处理
    if(pre_reload_fire_state_ == RELOAD_FIRE_STATE_FIRE && now_reload_fire_state_ == RELOAD_FIRE_STATE_FIRE)
    {
        entry_flag = 1;
    }

    if(entry_flag)
    {
        target_reload_torque_ = 0;
        entry_flag = 0;
    }

    misfire_count_ = 0;
}

/**
 * @brief Reload任务函数
 * 
 */
void Reload::Task()
{
    for(;;)
    {
        // 拨弹盘电机掉线处理
        motor_reload_.AlivePeriodElapsedCallback();
        // 自身姿态解算
        SelfResolution();
        // 拨弹盘卡弹处理
        // MisFireProcess();
        // 输出
        Output();
        
        osDelay(pdMS_TO_TICKS(1));
    }
}
