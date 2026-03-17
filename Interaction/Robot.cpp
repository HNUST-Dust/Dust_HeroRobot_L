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
#include "ui_interface.h"

/* Private macros ------------------------------------------------------------*/

#define K_NORM                  1.f / 660.f
#define C_NORM                  -256.f / 165.f
#define REMOTE_YAW_RATIO        0.5f
#define AUTOAIM_YAW_RATIO       150.f

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

    // 拨弹盘初始化
    reload_.Init();
    
    // 底盘初始化
    chassis_.Init();
    
    // 超级电容初始化 
    // supercap_.Init(&hcan1);

    // 裁判系统
    referee_.Init(&huart6, uart6_callback_function, UART_BUFFER_LENGTH);

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
    mcu_chassis_data_local.all                 = 0;
    mcu_chassis_data_local.cns                 = 1;

    // Mcu命令数据
    McuCommData mcu_comm_data_local;
    mcu_comm_data_local.mouse_lr.all           = 0;
    mcu_comm_data_local.keyboard.all           = 0;
    mcu_comm_data_local.imu_yaw                = 0.0f;

    // Mcu自瞄数据
    McuRecvAutoaimData mcu_autoaim_data_local;
    mcu_autoaim_data_local.autoaim_yaw_ang.f   = 0;
    mcu_autoaim_data_local.is_autoaim_start    = 0;

    uint8_t count = 0;

    for(;;)
    {
        /****************************   通讯   ****************************/


        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        mcu_chassis_data_local = *(static_cast<const McuChassisData*>(&(mcu_comm_.recv_chassis_data_)));
        mcu_comm_data_local = *(static_cast<const McuCommData*>(&(mcu_comm_.recv_comm_data_)));
        mcu_autoaim_data_local = *(static_cast<const McuRecvAutoaimData*>(&(mcu_comm_.recv_autoaim_data_)));
        __enable_irq();

        
        // MCU掉线检测保护
        if(mcu_comm_.GetMcuAliveState() == MCU_ALIVE_STATE_ENABLE)
        {
            if (mcu_comm_.first_power_on) 
            {
                remote_yaw_radian_ = normalize_angle_pm_pi(mcu_comm_data_local.imu_yaw);
                mcu_comm_.first_power_on = false;
            }

            gimbal_.SetImuYawAngle(normalize_angle_pm_pi(mcu_comm_data_local.imu_yaw));
        }
        else if (mcu_comm_.GetMcuAliveState() == MCU_ALIVE_STATE_DISABLE) 
        {
            if (mcu_comm_.first_power_on)
            {
                remote_yaw_radian_ = gimbal_.GetTargetYawRadian();
                mcu_comm_.first_power_on = false;
            }

            gimbal_.SetImuYawAngle(gimbal_.GetTargetYawRadian());
        }


        /****************************   云台   ****************************/


        // 自瞄开启
        if(mcu_chassis_data_local.fn2 || mcu_comm_data_local.mouse_lr.mouse_r == REMOTE_KEY_STATUS_PRESS)
        {
            if(mcu_autoaim_data_local.mode == PC_AUTOAIM_MODE_IDIE)
            {
                remote_yaw_radian_ += (M_PI / 180.f * (K_NORM * mcu_chassis_data_local.rotation + C_NORM)) * REMOTE_YAW_RATIO;
            }
            else
            {
                float filtered_autoaim = gimbal_.yaw_autoaim_filter_.Update(mcu_autoaim_data_local.autoaim_yaw_ang.f);

                remote_yaw_radian_ += filtered_autoaim / AUTOAIM_YAW_RATIO;
            }

            remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

            gimbal_.SetTargetYawRadian(remote_yaw_radian_);
        }
        else if(!mcu_chassis_data_local.fn2 || mcu_comm_data_local.mouse_lr.mouse_r == REMOTE_KEY_STATUS_FREE)
        {
            remote_yaw_radian_ += (M_PI / 180.f * (K_NORM * mcu_chassis_data_local.rotation + C_NORM)) * REMOTE_YAW_RATIO;

            remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

            gimbal_.SetTargetYawRadian(remote_yaw_radian_);
        }
        

        /****************************   底盘   ****************************/


        // 设置当前角度差
        chassis_.SetNowYawRadianDiff(-gimbal_.GetNowYawRadian());

        // 设置目标映射速度
        chassis_.SetTargetVxInGimbal((K_NORM * mcu_chassis_data_local.chassis_speed_x + C_NORM) * chassis_.GetMaxOmegaSpeed());
        chassis_.SetTargetVyInGimbal((K_NORM * mcu_chassis_data_local.chassis_speed_y + C_NORM) * chassis_.GetMaxOmegaSpeed());
        
        
        /****************************   模式   ****************************/


        // 拨弹盘开关
        if((mcu_chassis_data_local.fn1 && mcu_autoaim_data_local.mode == PC_AUTOAIM_MODE_FIRE) || (mcu_comm_data_local.keyboard.f && mcu_comm_data_local.mouse_lr.mouse_l) ||
          (mcu_comm_data_local.mouse_lr.mouse_r && mcu_comm_data_local.mouse_lr.mouse_l && mcu_autoaim_data_local.mode == PC_AUTOAIM_MODE_FIRE))
        // if(mcu_chassis_data_local.fn1)
        {
            ui_all_flag.reload = 1;

            reload_.SetTargetReloadTorque(MAX_RELORD_TORQUE);
        }
        else
        {
            ui_all_flag.reload = 0;

            reload_.SetTargetReloadTorque(0);
        }

        // 底盘模式
        if(mcu_chassis_data_local.cns == 2 || mcu_comm_data_local.keyboard.shift)
        {
            chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_SPIN);
        }
        else if (mcu_chassis_data_local.cns == 0 || mcu_comm_data_local.keyboard.ctrl)
        {
            float follow_change = gimbal_.GetNowYawRadian();

            if (-PI / 2 <= follow_change && follow_change <= PI / 2) {
                chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_FOLLOW_HEAD);
            } else {
                chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_FOLLOW_BACK);
            }
        }
        else
        {
            chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_NORMAL);
        }

        // 用于ui检测是否开摩擦轮
        if (mcu_comm_data_local.mouse_lr.mouse_r || mcu_comm_data_local.keyboard.f) {
            ui_all_flag.shoot = 1;
        } else {
            ui_all_flag.shoot = 0;
        }

        if (++count > 20)
        {
            // printf("%f\n", gimbal_.GetNowYawRadian());
        }


        osDelay(pdMS_TO_TICKS(1));
    }
}
