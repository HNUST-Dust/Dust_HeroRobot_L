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
#ifndef __DVC_REMOTE_DJI_VT03_H__
#define __DVC_REMOTE_DJI_VT03_H__

/* Includes ------------------------------------------------------------------*/

#include "dvc_remote.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief VT03原始数据结构体
 * 
 */
struct RmoteVT03RawData
{
    uint8_t start_of_frame_1 = 0xA9;
    uint8_t start_of_frame_2 = 0x53;
    uint64_t channel_0 : 11;
    uint64_t channel_1 : 11;
    uint64_t channel_2 : 11;
    uint64_t channel_3 : 11;
    uint64_t cns : 2;
    uint64_t pause : 1;
    uint64_t fn_1 : 1;
    uint64_t fn_2 : 1;
    uint64_t wheel : 11;
    uint64_t trigger : 1;
    uint64_t reserved_1 : 3;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_l : 2;
    uint8_t mouse_r: 2;
    uint8_t mouse_m : 2;
    uint8_t reserved_2 : 2;
    RemoteKeyboard keyboard;
    uint16_t crc16;
};

/**
 * @brief VT03输出数据结构体
 * 
 */
struct RemoteVT03OutputData
{
    struct 
    {
        uint16_t chassis_x, chassis_y;
        uint16_t rotation, thumbwheel;
        float pitch;

        union 
        {
            uint8_t all;
            struct
            {
                uint8_t pause : 1;
                uint8_t cns : 2;
                uint8_t fn1 : 1;
                uint8_t fn2 : 1;
                uint8_t trigger : 1;
                uint8_t reserved : 2;
            };
        };
    } remote;

    RemoteMouse mouse;
    RemoteKeyboard keyboard;
};

/**
 * @brief VT02类
 * 
 */
class RemoteDjiVT03 : public Remote
{
public:
    // 遥控器输出数据
    RemoteVT03OutputData output_;

private:

    void ClearData() override;

    void DataProcess(uint8_t* buffer) override;
};


/* Exported variables --------------------------------------------------------*/


/* Exported function declarations --------------------------------------------*/







#endif 