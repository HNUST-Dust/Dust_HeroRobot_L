/**
 * @file dvc_remote_vt03.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_remote_vt03.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief VT03清理数据函数
 * 
 */
void RemoteDjiVT03::ClearData()
{
    output_.remote.chassis_x = 1024;
    output_.remote.chassis_y = 1024;
    output_.remote.rotation = 1024;

    output_.mouse.mouse_x = 0;
    output_.mouse.mouse_y = 0;
    output_.mouse.mouse_z = 0;

    output_.mouse.mouse_l = REMOTE_KEY_STATUS_FREE;
    output_.mouse.mouse_r = REMOTE_KEY_STATUS_FREE;

    output_.keyboard.all = REMOTE_KEY_STATUS_FREE;
}

/**
 * @brief VT03数据处理函数
 * 
 */
void RemoteDjiVT03::DataProcess(uint8_t* buffer)
{
    // RmoteVT03RawData const* temp = reinterpret_cast<RmoteVT03RawData const*>(buffer);

    // if (temp->start_of_frame_1 != 0xA9 && temp->start_of_frame_2 != 0x53) {
    //     return;
    // }

    // output_.remote.pitch = K_PITCH * temp->channel_3 + C_PITCH;
    // output_.remote.pitch = CLAMP(output_.remote.pitch, MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

    // output_.remote.chassis_y = temp->channel_0;
    // output_.remote.chassis_x = temp->channel_1;
    // output_.remote.rotation = temp->channel_2;
    // output_.remote.thumbwheel = temp->wheel;

    // output_.remote.cns = temp->cns;
    // output_.remote.fn1 = temp->fn_1;
    // output_.remote.fn2 = temp->fn_2;
    // output_.remote.trigger = temp->trigger;
    // output_.remote.pause = temp->pause;
    
    // output_.mouse.mouse_x = (int16_t)temp->mouse_x;
    // output_.mouse.mouse_y = -(float)temp->mouse_y / (float)INT16_MAX;
    // // output_.mouse.mouse_z = (int16_t)temp->mouse_z;

    // output_.mouse.mouse_l = temp->mouse_l;
    // output_.mouse.mouse_r = temp->mouse_r;

    // Process_Keyboard_Toggle(&output_.keyboard, temp->keyboard);
}