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
#include "stdio.h"

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
    motor_reload_.Init(&hcan2, 0x08, 0x07, MOTOR_DM_CONTROL_METHOD_NORMAL_MIT, 
                    12.5f, 50.0f, 10.0f, 10.261194f);
    
    motor_reload_.CanSendClearError();
    osDelay(pdMS_TO_TICKS(1000));

    motor_reload_.CanSendEnter();
    osDelay(pdMS_TO_TICKS(1000));

    motor_reload_.SetKp(0);

    motor_reload_.SetKd(0.3);

    motor_reload_.SetControlOmega(0);

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
    now_reload_angle_ = motor_reload_.GetNowAngle();
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
 * @brief Reload任务函数
 * 
 */
void Reload::Task()
{
    for(;;)
    {
        Output();
        SelfResolution();
        printf("%f,%f\n", target_reload_rotation_, now_reload_torque_);
        osDelay(pdMS_TO_TICKS(1));
    }
}
