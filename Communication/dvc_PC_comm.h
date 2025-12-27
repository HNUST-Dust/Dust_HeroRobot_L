/**
 * @file dvc_PC_comm.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef PC_COMM_H
#define PC_COMM_H

/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "ins_task.h"
#include "bsp_dwt.h"
#include "bsp_usb.h"
#include "crc.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief PcComm转换联合体
 * 
 */
union PcConv
{
    uint8_t b[4];
    float f;
};

/**
 * @brief PcComm自瞄模式
 * 
 */
enum PcAutoAimMode : uint8_t
{
    AUTOAIM_MODE_IDIE = 0,
    AUTOAIM_MODE_FIRE,
};

/**
 * @brief PcComm状态
 * 
 */
enum PcAliveState : uint8_t
{
    DEATH_STATE = (uint8_t)0,
    ALIVE_STATE = 1,
};

/**
 * @brief PcComm敌方颜色
 * 
 */
enum PcEnenmyColor : uint8_t
{
    ENENMY_COLOR_RED = 1,
    ENENMY_COLOR_BLUE = 2,
};

#pragma pack(1)

/**
 * @brief PcComm自瞄发送结构体
 * 
 */
struct PCSendAutoAimData
{
    uint8_t start_of_frame = 0xA5;  // 头帧

    float current_yaw;              // 当前yaw轴弧度
    float current_pitch;            // 当前pitch轴弧度

    uint8_t state;                  // 状态 0-死亡 1-存活
    uint8_t autoaim;                // 自瞄 0-关闭 1-开启

    uint8_t enemy_color;            // 地方颜色 1-红色 2-蓝色
    uint8_t reserved;               // 预留字段
    uint8_t padding;                // 填充
    uint16_t crc16;
};

/**
 * @brief PcComm自瞄接收结构体
 * 
 */
struct PCRecvAutoAimData
{
    uint8_t start_of_frame = 0xA5;  // 头帧

    uint8_t fire;                   // 开火 0-关闭 1-开启

    float shoot_yaw;                // 射击yaw轴弧度
    float shoot_pitch;              // 射击pitch轴弧度
    float avg_speed;                // 弹速

    uint8_t food;                   // 弹药数量
};

#pragma pack()

/**
 * @brief PcComm类
 * 
 */
class PcComm
{
public:
    // 发送自瞄数据
    PCSendAutoAimData send_autoaim_data = 
    {
        0xA5,
        0.0f,
        0.0f,
        1,
        0,
        1,
        0,
        0,
        0,
    };
    // 接收自瞄数据
    PCRecvAutoAimData recv_autoaim_data = 
    {
        0xA5,
        0,
        0,
        0,
        0,
        255,
    };
    void Init();

    void Task();

    void Send_Message();

    void RxCpltCallback();

    void UpdataAutoaimData();

private:

    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif
