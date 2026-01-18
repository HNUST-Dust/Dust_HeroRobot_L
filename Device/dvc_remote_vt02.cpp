/**
 * @file dvc_remote_vt02.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_remote_vt02.h"

/* Private macros ------------------------------------------------------------*/

#define CLAMP(x, min, max)  ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief VT02遥控初始化函数
 * 
 * @param huart uart句柄
 * @param callback_function 回调函数
 * @param rx_buffer_length 接受缓冲区长度
 */
void RemoteDjiVT02::Init(UART_HandleTypeDef *huart, Uart_Callback callback_function, uint16_t rx_buffer_length)
{
    uart_manage_object_->uart_handle = huart;
    uart_manage_object_->callback_function = callback_function;
    uart_manage_object_->rx_buffer_length = rx_buffer_length;
    uart_init(huart, callback_function, rx_buffer_length);

    static const osThreadAttr_t kRemoteVT02TaskAttr = {
        .name = "kRemoteVT02TaskAttr",
        .stack_size = 256,
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(RemoteDjiVT02::TaskEntry, this, &kRemoteVT02TaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void RemoteDjiVT02::TaskEntry(void *argument)
{
    RemoteDjiVT02 *self = static_cast<RemoteDjiVT02 *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
};

/**
 * @brief VT02清理数据函数
 * 
 */
void RemoteDjiVT02::ClearData()
{
    output_.mouse_x = 0;
    output_.mouse_y = 0;
    output_.mouse_z = 0;

    output_.mouse_pl = REMOTE_VT02_KEY_STATUS_FREE;
    output_.mouse_pr = REMOTE_VT02_KEY_STATUS_FREE;

    output_.keyboard.all = REMOTE_VT02_KEY_STATUS_FREE;
}

/**
 * @brief VT02检测在线回调函数
 * 
 */
void RemoteDjiVT02::AlivePeriodElapsedCallback()
{
    // 判断时间段内是否掉线
    if(pre_flag_ == flag_)
    {
        // 断开连接
        remote_vt02_alive_status = REMOTE_VT02_ALIVE_STATUS_DISABLE;
        ClearData();
    }
    else
    {
        remote_vt02_alive_status = REMOTE_VT02_ALIVE_STATUS_ENABLE;
    }

    pre_flag_ = flag_;
}

void RemoteDjiVT02::Process_Keyboard_Toggle(Keyboard current_raw)
{
    static uint16_t last_raw_all = 0;
    static Keyboard toggle_output = {0};

    uint16_t trigger = current_raw.all & (~last_raw_all);

    uint16_t toggle_mask = 0xFFC0; 

    toggle_output.all ^= (trigger & toggle_mask);
    
    uint16_t normal_mask = ~toggle_mask;
    output_.keyboard.all = (toggle_output.all & toggle_mask) | (current_raw.all & normal_mask);

    last_raw_all = current_raw.all;
}

/**
 * @brief VT02任务函数
 * 
 */
void RemoteDjiVT02::Task()
{
    for(;;)
    {
        AlivePeriodElapsedCallback();
        osDelay(pdMS_TO_TICKS(50));     // 请勿修改频率
    }
}

/**
 * @brief VT02转换函数
 * 
 * @param buffer 传入遥控数据
 */
void RemoteDjiVT02::UartRxCpltCallback(uint8_t* buffer)
{
    // 滑动窗口, 判断是否在线
    flag_ += 1;
    // 读取数据值
    DataProcess(buffer);
}

/**
 * @brief VT02数据处理函数
 * 
 */
void RemoteDjiVT02::DataProcess(uint8_t* buffer)
{
    int16_t dx = (int16_t)((uint16_t)buffer[7] | ((uint16_t)buffer[8] << 8));
    int16_t dy = (int16_t)((uint16_t)buffer[9] | ((uint16_t)buffer[10] << 8));
    int16_t dz = (int16_t)((uint16_t)buffer[11] | ((uint16_t)buffer[12] << 8));

    raw_data_.mouse_x = CLAMP(dx * 20, -32768, 32767);
    raw_data_.mouse_y = CLAMP(dy, -32768, 32767);
    raw_data_.mouse_z = CLAMP(dz, -32768, 32767);

    output_.mouse_x = (int16_t)raw_data_.mouse_x;
    output_.mouse_y = (float)raw_data_.mouse_y / 32767.0f;
    output_.mouse_z = (int16_t)raw_data_.mouse_z;

    raw_data_.mouse_pl = buffer[13];
    raw_data_.mouse_pr = buffer[14];
    raw_data_.keyboard.all = (uint16_t)buffer[15] | ((uint16_t)buffer[16] << 8);
    
    Process_Keyboard_Toggle(raw_data_.keyboard);

    output_.mouse_pl = raw_data_.mouse_pl;
    output_.mouse_pr = raw_data_.mouse_pr;

    // printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", 
    //     output_.keyboard.keycode.w,
    //     output_.keyboard.keycode.s,
    //     output_.keyboard.keycode.a,
    //     output_.keyboard.keycode.d,
    //     output_.keyboard.keycode.shift,
    //     output_.keyboard.keycode.ctrl,
    //     output_.keyboard.keycode.q,
    //     output_.keyboard.keycode.e,
    //     output_.keyboard.keycode.r,
    //     output_.keyboard.keycode.f,
    //     output_.keyboard.keycode.g,
    //     output_.keyboard.keycode.z,
    //     output_.keyboard.keycode.x,
    //     output_.keyboard.keycode.c,
    //     output_.keyboard.keycode.v,
    //     output_.keyboard.keycode.b
    // );

    // printf("%f,%f,%f\n", output_.mouse_x, output_.mouse_y, output_.mouse_z);
}