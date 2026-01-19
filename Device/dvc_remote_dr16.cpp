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

/* Private macros ------------------------------------------------------------*/

#define K_NORM      1.f / 660.f
#define C_NORM      -256.f / 165.f
#define K_PITCH     1.f / 30.f
#define C_PITCH     -512.f / 15.f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief DR16遥控初始化函数
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

    static const osThreadAttr_t kRemoteDR16TaskAttr = {
        .name = "kRemoteDR16TaskAttr",
        .stack_size = 256,
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(RemoteDjiDR16::TaskEntry, this, &kRemoteDR16TaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void RemoteDjiDR16::TaskEntry(void *argument)
{
    RemoteDjiDR16 *self = static_cast<RemoteDjiDR16 *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
};

/**
 * @brief DR16数据处理函数
 * 
 */
void RemoteDjiDR16::DataProcess(uint8_t* rx_data)
{
    /****************************   原始数据    ****************************/


    raw_data_.rc.ch0 =  ((int16_t)rx_data[0]        | ((int16_t)rx_data[1] << 8)) & 0x07FF;
    raw_data_.rc.ch1 = (((int16_t)rx_data[1] >> 3)  | ((int16_t)rx_data[2] << 5)) & 0x07FF;
    raw_data_.rc.ch2 = (((int16_t)rx_data[2] >> 6)  | ((int16_t)rx_data[3] << 2)  | ((int16_t)rx_data[4] << 10)) &  0x07FF;
    raw_data_.rc.ch3 = (((int16_t)rx_data[4] >> 1)  | ((int16_t)rx_data[5] << 7)) & 0x07FF;

    raw_data_.rc.s1 = ((rx_data[5] >> 4) & 0x000C) >> 2;
    raw_data_.rc.s2 = ((rx_data[5] >> 4) & 0x0003);

    raw_data_.mouse.x = ((int16_t)rx_data[6]) | ((int16_t)rx_data[7] << 8);
    raw_data_.mouse.y = ((int16_t)rx_data[8]) | ((int16_t)rx_data[9] << 8);
    raw_data_.mouse.z = ((int16_t)rx_data[10]) | ((int16_t)rx_data[11] << 8);

    raw_data_.mouse.pl = rx_data[12];
    raw_data_.mouse.pr = rx_data[13];

    raw_data_.keyboard.all = (int16_t)rx_data[14];


    /****************************   遥控数据    ****************************/


    output_.remote.pitch = K_PITCH * raw_data_.rc.ch3 + C_PITCH;

    output_.remote.chassis_x  = raw_data_.rc.ch0;
    output_.remote.chassis_y  = raw_data_.rc.ch1;
    output_.remote.rotation   = raw_data_.rc.ch2;

    output_.remote.switch_l = raw_data_.rc.s1;
    output_.remote.switch_r = raw_data_.rc.s2;


    /****************************   键鼠数据    ****************************/

    output_.mouse.mouse_x = raw_data_.mouse.x;
    output_.mouse.mouse_y = raw_data_.mouse.y;
    output_.mouse.mouse_z = raw_data_.mouse.z;

    output_.mouse.press_l = raw_data_.mouse.pl;
    output_.mouse.press_r = raw_data_.mouse.pr;

    output_.keyboard.all = raw_data_.keyboard.all;
}

/**
 * @brief DR16转换函数
 * 
 * @param buffer 传入遥控数据
 */
void RemoteDjiDR16::UartRxCpltCallback(uint8_t* buffer)
{
    // 滑动窗口, 判断是否在线
    flag_ += 1;
    // 读取数据值
    DataProcess(buffer);
}

/**
 * @brief DR16清理数据函数
 * 
 */
void RemoteDjiDR16::ClearData()
{
    output_.remote.pitch = K_PITCH * 1024 + C_PITCH;
    output_.remote.chassis_x = output_.remote.chassis_y = output_.remote.rotation = 1024;
    output_.remote.switch_l = output_.remote.switch_r = 3;
}

/**
 * @brief DR16检测在线回调函数
 * 
 */
void RemoteDjiDR16::AlivePeriodElapsedCallback()
{
    // 判断时间段内是否掉线
    if(pre_flag_ == flag_)
    {
        // 断开连接
        remote_dr16_alive_status = REMOTE_DR16_ALIVE_STATUS_DISABLE;
        ClearData();
    }
    else
    {
        remote_dr16_alive_status = REMOTE_DR16_ALIVE_STATUS_ENABLE;
    }

    pre_flag_ = flag_;
}

/**
 * @brief DR16任务函数
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

