/**
 * @file app_robot.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "Robot.h"

/* Private macros ------------------------------------------------------------*/

#define K_NORM                  1.f / 660.f
#define C_NORM                  -256.f / 165.f
#define MAX_OMEGA_SPEED         20.f
#define REMOTE_YAW_RATIO        0.5f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Robot初始化函数
 * 
 */
void Robot::Init()
{
    dwt_init(168);

    // 上下板通讯组件初始化
    mcu_comm_.Init(&hcan2, 0x01, 0x00);

    // 云台初始化
    gimbal_.Init();

    // 底盘陀螺仪初始化
    imu_.Init();

    // 10s时间等待陀螺仪收敛
    osDelay(pdMS_TO_TICKS(1 * 1000));

    // 拨弹盘初始化
    reload_.Init();
    
    // 底盘初始化
    chassis_.Init();
    
    // 超级电容初始化 
    // supercap_.Init(&hcan1);

    static const osThreadAttr_t kRobotTaskAttr = 
    {
        .name = "robot_task",
        .stack_size = 1024,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Robot::TaskEntry, this, &kRobotTaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void Robot::TaskEntry(void *argument)
{
    Robot *self = static_cast<Robot *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief Robot任务函数
 * 
 */
void Robot::Task()
{
    // Mcu底盘数据
    McuChassisData mcu_chassis_data_local;
    mcu_chassis_data_local.chassis_speed_x     = 1024;
    mcu_chassis_data_local.chassis_speed_y     = 1024;
    mcu_chassis_data_local.rotation            = 1024;
    mcu_chassis_data_local.keyboard_l.all      = 0;

    // Mcu命令数据
    McuCommData mcu_comm_data_local;
    mcu_comm_data_local.switch_lr.all          = 15;
    mcu_comm_data_local.keyboard_h.all         = 0;
    mcu_comm_data_local.imu_yaw.f              = 0;

    // Mcu自瞄数据
    McuRecvAutoaimData mcu_autoaim_data_local;
    mcu_autoaim_data_local.autoaim_yaw_ang.f   = 0;

    for(;;)
    {
        /****************************   通讯   ****************************/


        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        mcu_chassis_data_local = *(static_cast<const McuChassisData*>(&(mcu_comm_.recv_chassis_data_)));
        mcu_comm_data_local = *(static_cast<const McuCommData*>(&(mcu_comm_.recv_comm_data_)));
        mcu_autoaim_data_local = *(static_cast<const McuRecvAutoaimData*>(&(mcu_comm_.recv_autoaim_data_)));
        __enable_irq();


        /****************************   云台   ****************************/


        if(mcu_comm_data_local.switch_lr.switchcode.switch_r == SWITCH_MID || mcu_comm_data_local.mouse_lr.mousecode.mouse_r == REMOTE_VT02_KEY_STATUS_FREE)
        {
            remote_yaw_radian_ += (M_PI / 180.f * (K_NORM * mcu_chassis_data_local.rotation + C_NORM)) * REMOTE_YAW_RATIO;

            remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

            gimbal_.SetTargetYawRadian(remote_yaw_radian_);
        }
        else if(mcu_comm_data_local.switch_lr.switchcode.switch_r == SWITCH_UP || mcu_comm_data_local.mouse_lr.mousecode.mouse_r == REMOTE_VT02_KEY_STATUS_PRESS)
        {
            switch (mcu_autoaim_data_local.mode) 
            {
                case(AUTOAIM_MODE_IDIE):
                {
                    remote_yaw_radian_ += (M_PI / 180.f * (K_NORM * mcu_chassis_data_local.rotation + C_NORM)) * REMOTE_YAW_RATIO;

                    remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

                    gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                    reload_.SetTargetReloadTorque(0);

                    break;
                }
                case(AUTOAIM_MODE_FOLLOW):
                {
                    float filtered_autoaim = gimbal_.yaw_autoaim_filter_.Update(mcu_autoaim_data_local.autoaim_yaw_ang.f);

                    remote_yaw_radian_ += filtered_autoaim / 150.f;

                    gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                    reload_.SetTargetReloadTorque(0);

                    break;
                }
                case(AUTOAIM_MODE_FIRE):
                {
                    float filtered_autoaim = gimbal_.yaw_autoaim_filter_.Update(mcu_autoaim_data_local.autoaim_yaw_ang.f);

                    remote_yaw_radian_ += filtered_autoaim / 150.f;

                    gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                    reload_.SetTargetReloadTorque(MAX_RELORD_TORQUE);

                    break;
                }
            }
        }
        
        // MCU掉线检测保护
        if(mcu_comm_.GetMcuAliveState() == MCU_ALIVE_STATE_ENABLE)
        {
            gimbal_.SetImuYawAngle(normalize_angle_pm_pi(mcu_comm_data_local.imu_yaw.f));
        }
        else if (mcu_comm_.GetMcuAliveState() == MCU_ALIVE_STATE_DISABLE) 
        {
            gimbal_.SetImuYawAngle(gimbal_.GetTargetYawRadian());
        }


        /****************************   底盘   ****************************/


        // 设置当前角度差
        chassis_.SetNowYawRadianDiff(gimbal_.GetNowYawRadian());

        // 设置目标映射速度
        chassis_.SetTargetVxInGimbal((K_NORM * mcu_chassis_data_local.chassis_speed_x + C_NORM) * MAX_OMEGA_SPEED);
        chassis_.SetTargetVyInGimbal((K_NORM * mcu_chassis_data_local.chassis_speed_y + C_NORM) * MAX_OMEGA_SPEED);
        
        
        /****************************   模式   ****************************/

        
        // 左按钮
        if(mcu_comm_data_local.switch_lr.switchcode.switch_l == SWITCH_MID || mcu_comm_data_local.mouse_lr.mousecode.mouse_l == REMOTE_VT02_KEY_STATUS_FREE) 
        {
            chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_NORMAL);

            reload_.SetTargetReloadTorque(0);
        }
        else if(mcu_comm_data_local.switch_lr.switchcode.switch_l == SWITCH_DOWN || mcu_comm_data_local.mouse_lr.mousecode.mouse_l == REMOTE_VT02_KEY_STATUS_PRESS)
        {
            chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_NORMAL);
        }

        if(mcu_comm_data_local.switch_lr.switchcode.switch_l == SWITCH_UP)
        {
            chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_SPIN);
        }

        osDelay(pdMS_TO_TICKS(1));
    }
}
