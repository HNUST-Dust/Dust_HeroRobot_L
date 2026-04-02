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
#include "Timer.hpp"

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
    // 拨弹盘位置环PID初始化
    // kp, ki, kd, kf, i_out_max, out_max, dt
    pid_reload_angle_.Init(
        8.0f, 
        1.0f, 
        0.3f, 
        0.0f, 
        0.0f,
        MAX_RELORD_TORQUE, 
        0.001f);

    // 拨弹盘4310电机初始化
    motor_reload_.Init(&hcan2, 0x08, 0x07, MOTOR_DM_CONTROL_METHOD_NORMAL_MIT, 12.5f, 30.f, 10.f);

    motor_reload_.SetKp(0);

    motor_reload_.SetKd(0);

    motor_reload_.SetControlTorque(0);

    motor_reload_.Output();

    static const osThreadAttr_t kReloadTaskAttr = 
    {
        .name = "reload_task",
        .stack_size = 128 * 6,
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
    now_reload_status_ = motor_reload_.GetControlStatus();
    now_reload_omega_  = motor_reload_.GetNowOmega();
    now_reload_angle_  = normalize_angle(motor_reload_.GetNowAngle() / PI * 180.f);
    now_reload_torque_ = motor_reload_.GetNowTorque();
    now_reload_radian_ = normalize_pi(motor_reload_.GetNowAngle());
}

/**
 * @brief Reload输出函数
 * 
 */
void Reload::Output()
{
    // 位置环PID计算
    pid_reload_angle_.SetTarget(target_reload_radian_);
    pid_reload_angle_.SetNow(now_reload_radian_);
    pid_reload_angle_.CalculateAnglePid();
    float pid_torque = pid_reload_angle_.GetOut();

    // 当热量过高时，只有降下来才允许继续拨弹
    if (shooting_heat_ < 100)  {
        motor_reload_.SetControlTorque(target_reload_torque_);
    } else {
        motor_reload_.SetControlTorque(0);
    }

    motor_reload_.Output();
}

/**
 * @brief 增加拨弹盘目标角度
 * 
 * @param delta_deg 增量角度（度）
 */
void Reload::AddTargetAngle(float delta_deg)
{
    target_reload_radian_ += delta_deg * PI / 180.0f;
    target_reload_radian_ = normalize_pi(target_reload_radian_);
}

/**
 * @brief Reload任务函数
 * 
 */
void Reload::Task()
{
    Timer print(20);

    for (;;)
    {
        SelfResolution();

        if (now_reload_status_ == MOTOR_DM_CONTROL_STATUS_ENABLE)
        {
            Output();

            if (!NoConnectTimer.IsFinish()) {
                NoConnectTimer.Finish();
            }
        }
        else if(now_reload_status_ == MOTOR_DM_CONTROL_STATUS_DISABLE)
        {
            NoConnectTimer.Tick([&]()
            {
                uint16_t step = NoConnectTimer.GetTimerCounter();

                if (step == 0) {
                    motor_reload_.CanSendEnter();
                }

                now_reload_status_ = motor_reload_.GetControlStatus();

                if (now_reload_status_ == MOTOR_DM_CONTROL_STATUS_ENABLE) {
                    NoConnectTimer.Finish();
                }
            });
        }
        else
        {
            NoConnectTimer.Tick([&]()
            {
                uint16_t step = NoConnectTimer.GetTimerCounter();

                if (step == 0) {
                    motor_reload_.CanSendClearError();
                }

                now_reload_status_ = motor_reload_.GetControlStatus();

                if (now_reload_status_ == MOTOR_DM_CONTROL_STATUS_ENABLE) {
                    NoConnectTimer.Finish();
                }
            });
        }
        
        // print.Clock([&](){
        //     printf("%.2f\n", now_reload_torque_);
        // });

        osDelay(pdMS_TO_TICKS(1));
    }
}






