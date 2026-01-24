/**
 * @file dvc_remote_dji.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_remote_dr16.h"
#include "app_gimbal.h"

/* Private macros ------------------------------------------------------------*/

#define K_NORM      1.f / 660.f
#define C_NORM      -256.f / 165.f

#define CLAMP(x, min, max)  ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief DjiDR16遥控初始化函数
 * 
 * @param huart uart句柄
 * @param callback_function 回调函数
 * @param rx_buffer_length 接受缓冲区长度
 */
void RemoteDjiDR16::Init(UART_HandleTypeDef *huart, Uart_Callback callback_function, uint16_t rx_buffer_length)
{
    uart_manage_object_->uart_handle = huart;
    uart_manage_object_->callback_function = callback_function;
    uart_manage_object_->rx_buffer_length = rx_buffer_length;
    uart_init(huart, callback_function, rx_buffer_length);

    static const osThreadAttr_t kRemoteTaskAttr = {
        .name = "kRemoteTaskAttr",
        .stack_size = 256,
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(RemoteDjiDR16::TaskEntry, this, &kRemoteTaskAttr);
}

/**
 * @brief DjiDR16遥控
 * 
 * @param argument 
 */
void RemoteDjiDR16::TaskEntry(void *argument)
{
    RemoteDjiDR16 *self = static_cast<RemoteDjiDR16 *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
};

/**
 * @brief DjiDR16清理数据函数
 * 
 */
void RemoteDjiDR16::ClearData()
{
    
}

/**
 * @brief DjiDR16检测在线回调函数
 * 
 */
void RemoteDjiDR16::AlivePeriodElapsedCallback()
{
    // 判断时间段内是否掉线
    if(pre_flag_ == flag_)
    {
        // 断开连接
        remote_dji_alive_status = REMOTE_DJI_STATUS_DISABLE;
        ClearData();
    }
    else
    {
        remote_dji_alive_status = REMOTE_DJI_STATUS_ENABLE;
    }
    pre_flag_ = flag_;
}

/**
 * @brief 
 * 
 * @param current_raw 
 */
void RemoteDjiDR16::Process_Keyboard_Toggle(RemoteDR16Keyboard current_raw)
{
    static uint16_t last_raw_all = 0;
    static RemoteDR16Keyboard toggle_output = {0};

    uint16_t trigger = current_raw.all & (~last_raw_all);

    uint16_t toggle_mask = 0xFFC0; 

    toggle_output.all ^= (trigger & toggle_mask);
    
    uint16_t normal_mask = ~toggle_mask;
    output_.keyboard.all = (toggle_output.all & toggle_mask) | (current_raw.all & normal_mask);

    last_raw_all = current_raw.all;
}

/**
 * @brief DjiDR16任务函数
 * 
 */
void RemoteDjiDR16::Task()
{
    for(;;)
    {
        AlivePeriodElapsedCallback();
        osDelay(pdMS_TO_TICKS(50));     // 请勿修改频率
    }
}

/**
 * @brief DjiDR16转换函数
 * 
 * @param buffer 传入遥控数据
 */
void RemoteDjiDR16::UartRxCpltCallback(uint8_t* buffer)
{
    // 滑动窗口, 判断是否在线
    flag_ += 1;
    
    DataProcess(buffer);
}

/**
 * @brief DjiDR16数据处理函数
 * 
 */
void RemoteDjiDR16::DataProcess(uint8_t* buffer)
{
    
}
