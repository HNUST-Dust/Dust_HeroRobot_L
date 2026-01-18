/**
 * @file dvc_remote_vt02.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef __DVC_REMOTE_DJI_VT02_H__
#define __DVC_REMOTE_DJI_VT02_H__

/* Includes ------------------------------------------------------------------*/

#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "alg_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

union MouseLR
{
    uint8_t all;
    struct 
    {
        uint8_t mouse_l : 2;
        uint8_t mouse_r : 2;
        uint8_t reserved : 4;
    } mousecode;
};

union Keyboard
{
    uint16_t all;
    struct
    {
        uint8_t w : 1;
        uint8_t s : 1;
        uint8_t a : 1;
        uint8_t d : 1;
        uint8_t shift : 1;
        uint8_t ctrl : 1;
        uint8_t q : 1;
        uint8_t e : 1;
        uint8_t r : 1;
        uint8_t f : 1;
        uint8_t g : 1;
        uint8_t z : 1;
        uint8_t x : 1;
        uint8_t c : 1;
        uint8_t v : 1;
        uint8_t b : 1;
    } keycode;
};



/**
 * @brief 遥控按键状态
 * 
 */
enum RemoteVT02KeyStatus
{
    REMOTE_VT02_KEY_STATUS_FREE = 0,
    REMOTE_VT02_KEY_STATUS_PRESS,
};

/**
 * @brief 
 * 
 */
enum RemoteVT02AliveStatus
{
    REMOTE_VT02_ALIVE_STATUS_DISABLE = 0,
    REMOTE_VT02_ALIVE_STATUS_ENABLE  = 1,
};


struct RmoteVT02Data
{
    int32_t mouse_x;
    int32_t mouse_y;
    int32_t mouse_z;

    int8_t mouse_pl;
    int8_t mouse_pr;

    Keyboard keyboard;
};


struct RemoteVT02Output
{
    float mouse_x;
    float mouse_y;
    float mouse_z;

    int8_t mouse_pl;
    int8_t mouse_pr;

    Keyboard keyboard;
};


class RemoteDjiVT02
{
public:
    // 遥控器输出数据
    RemoteVT02Output output_;

    // 遥控器状态
    RemoteVT02AliveStatus remote_vt02_alive_status = REMOTE_VT02_ALIVE_STATUS_DISABLE;

    void Init(UART_HandleTypeDef *huart, Uart_Callback callback_function, uint16_t rx_buffer_length);

    void Task();

    void AlivePeriodElapsedCallback();

    void UartRxCpltCallback(uint8_t* buffer);

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数

private:
    // uart管理模块
    UartManageObject* uart_manage_object_;

    // 遥控原始数据
    RmoteVT02Data raw_data_;

    // 当前时刻flag
    uint32_t flag_ = 0;

    // 前一时刻flag
    uint32_t pre_flag_ = 0;

    void Process_Keyboard_Toggle(Keyboard current_raw);

    void ClearData();

    void DataProcess(uint8_t* buffer);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/







#endif 