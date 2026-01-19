/**
 * @file dvc_remote_dji.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __DVC_REMOTE_DJI_DR16_H__
#define __DVC_REMOTE_DJI_DR16_H__

/* Includes ------------------------------------------------------------------*/

#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief DR16在线状态枚举
 * 
 */
enum RemoteDR16AliveStatus
{
    REMOTE_DR16_ALIVE_STATUS_DISABLE = 0,
    REMOTE_DR16_ALIVE_STATUS_ENABLE  = 1,
};

/**
 * @brief DR16拨杆状态枚举
 * 
 */
enum RemoteDR16SwitchStatus
{
    SWITCH_UP    = (uint8_t)1,
    SWITCH_MID   = (uint8_t)3,
    SWITCH_DOWN  = (uint8_t)2,
};

/**
 * @brief DR16原始数据结构体
 * 
 */
struct RemoteDR16RawData
{
    struct 
    {
        uint16_t ch0, ch1, ch2, ch3;
        uint8_t s1, s2;
    } rc;

    struct
    {
        int16_t x, y, z;
        uint8_t pl, pr;
    } mouse;

    struct 
    {
        uint16_t all;
    } keyboard;
};

/**
 * @brief DR16输出数据结构体
 * 
 */
struct RemoteDR16OutputData
{
    struct 
    {
        uint8_t switch_l, switch_r;
        float chassis_x, chassis_y;      // x, y, r 采用右手系
        float rotation;
        float pitch;
    } remote;

    struct
    {
        float mouse_x, mouse_y, mouse_z;
        uint8_t press_l, press_r;
    } mouse;

    union
    {
        uint16_t all;
        struct
        {
            uint16_t w : 1, 
                     s : 1,
                     a : 1,
                     d : 1,
                     q : 1, 
                     e : 1;
            uint16_t shift : 1, 
                     ctrl : 1;
            uint16_t reserved : 8;
        } keycode;
    } keyboard;
};

/**
 * @brief DR16遥控器
 * 
 */
class RemoteDjiDR16
{
public:
    // 遥控器输出数据
    RemoteDR16OutputData output_;

    // 遥控器状态
    RemoteDR16AliveStatus remote_dr16_alive_status = REMOTE_DR16_ALIVE_STATUS_DISABLE;

    void Init(UART_HandleTypeDef *huart, Uart_Callback callback_function, uint16_t rx_buffer_length);

    void Task();

    inline float GetLeftX();

    inline float GetLeftY();

    inline float GetRightX();

    inline float GetRightY();

    inline uint8_t GetSwitchL();

    inline uint8_t GetSwitchR();

    void AlivePeriodElapsedCallback();

    void UartRxCpltCallback(uint8_t* buffer);

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数

private:
    // uart管理模块
    UartManageObject* uart_manage_object_;

    // 原始数据
    RemoteDR16RawData raw_data_;

    // 当前时刻flag
    uint32_t flag_ = 0;

    // 前一时刻flag
    uint32_t pre_flag_ = 0;

    // 掉线清理数据函数

    void ClearData();

    // 内部数据处理函数

    void DataProcess(uint8_t* rx_data);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


#endif