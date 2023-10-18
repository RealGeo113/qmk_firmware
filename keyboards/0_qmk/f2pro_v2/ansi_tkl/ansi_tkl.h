// Copyright 2023 Persama (@Persama)
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include "quantum.h"

enum custom_keycodes {
    RF_DFU = QK_KB_0,
    LNK_USB,
    LNK_RF,
    LNK_BLE1,
    LNK_BLE2,
    LNK_BLE3,

    MAC_TASK,
    MAC_SEARCH,
    MAC_VOICE,
    MAC_DNT,
    MAC_PRT,
    MAC_PRTA,

    SIDE_VAI,
    SIDE_VAD,
    SIDE_MOD,
    SIDE_HUI,
    SIDE_SPI,
    SIDE_SPD,

    DEV_RESET,
    SLEEP_MODE,
    BAT_SHOW
};

extern uint8_t m_sleep_led;

typedef enum {
    RX_Idle,        // 空闲
    RX_Receiving,   // 接收中
    RX_Done,        // 接收完成
    RX_Fail,        // 接收错误         3
    RX_OV_ERR,      // 接收溢出         4
    RX_SUM_ERR,     // 数据校验错误     5
    RX_CMD_ERR,     // 命令错误         6
    RX_DATA_ERR,    // 数据错误         7
    RX_DATA_OV,     // 数据溢出         8
    RX_FORMAT_ERR,  // 格式错误         9

    TX_OK = 0XE0,  // 发送成功
    TX_DONE,       // 不应答发送完成
    TX_BUSY,       // 发送忙碌
    TX_TIMEOUT,    // 发送超时
    TX_DATA_ERR,   //

} TYPE_RX_STATE;

#define FUNC_VALID_LEN  32

//------------------------------------------------
// RF模块状态
#define RF_IDLE         0     // 空闲
#define RF_PAIRING      1     // 对码中
#define RF_LINKING      2     // 回连中
#define RF_CONNECT      3     // 连接成功
#define RF_DISCONNECT   4     // 断开连接
#define RF_SLEEP        5     // 休眠
#define RF_SNIF         6     // 轻度休眠
#define RF_INVAILD      0XFE  // 无效状态
#define RF_ERR_STATE    0XFF  // 异常状态

//------------------------------------------------
// RF通讯命令
#define CMD_POWER_UP    0XF0  // 上电命令
#define CMD_SLEEP       0XF1  // 休眠/掉电命令
#define CMD_HAND        0XF2  // 握手命令
#define CMD_SNIF        0XF3  // Snif命令
#define CMD_24G_SUSPEND 0XF4  // 2.G休眠
#define CMD_IDLE_EXIT   0XFE  //

#define CMD_RPT_MS      0XE0  // 鼠标报文
#define CMD_RPT_BYTE_KB 0XE1  // 标准键盘报文
#define CMD_RPT_BIT_KB  0XE2  // 全键盘报文
#define CMD_RPT_CONSUME 0XE3  // 消费类报文
#define CMD_RPT_SYS     0XE4  // 系统类包围

#define CMD_SET_LINK    0XC0  // 设置RF通道/回连
#define CMD_SET_CONFIG  0XC1  // 预留
#define CMD_GET_CONFIG  0XC2  // 预留
#define CMD_SET_NAME    0XC3  // 设置蓝牙设备名
#define CMD_GET_NAME    0XC4  // 预留
#define CMD_CLR_DEVICE  0XC5  // 预留
#define CMD_NEW_ADV     0XC7  // 开新广播
#define CMD_RF_STS_SYSC 0XC9  // 同步命令
#define CMD_GO_TEST     0XCF  // 测试模式

#define CMD_RF_DFU      0XB1  // 进入DFU升级

#define CMD_WRITE_DATA  0X80  // 写数据到RF
#define CMD_READ_DATA   0X81  // 读数据

//------------------------------------------------
// RF通道定义
#define LINK_RF_24      0  // 2.4G通道索引
#define LINK_BT_1       1  // 蓝牙1通道
#define LINK_BT_2       2  // 蓝牙2通道
#define LINK_BT_3       3  // 蓝牙3通道
#define LINK_USB        4  // usb

#define UART_MAX_LEN    64
typedef struct
{
    uint8_t RXDState;              // 接收状态：空闲、接收中、完成、错误
    uint8_t RXDLen;                // 接收长度
    uint8_t RXDOverTime;           // 接收超时计数
    uint8_t TXDLenBack;            // 发送长度
    uint8_t TXDOffset;             // 发送指针
    uint8_t TXDBuf[UART_MAX_LEN];  // 发送数据缓存
    uint8_t RXDBuf[UART_MAX_LEN];  // 发送数据缓存
} USART_MGR_STRUCT;

typedef struct
{
    uint8_t link_mode;     // 连接模式
    uint8_t rf_channel;    // 无线通道
    uint8_t ble_channel;   // BLE通道
    uint8_t rf_state;      // 接收长度
    uint8_t rf_charge;     // 充电管理
    uint8_t rf_led;        // 接收超时计数
    uint8_t rf_baterry;    // 发送长度
    uint8_t sys_sw_state;  //
} DEV_INFO_STRUCT;

/** ================================================================================================================================
 *     通用定时器：周期1ms
 * ================================================================================================================================*/
#define DELAY_2MS         2     // 延时2MS
#define DELAY_4MS         4     // 延时4MS
#define DELAY_5MS         5     // 延时5MS
#define DELAY_6MS         6     // 延时6MS
#define DELAY_8MS         8     // 延时8MS
#define DELAY_10MS        10    // 延时10MS
#define DELAY_15MS        15    // 延时15MS
#define DELAY_20MS        20    // 延时20MS
#define DELAY_30MS        30    // 延时50MS
#define DELAY_40MS        40    // 延时40MS
#define DELAY_50MS        50    // 延时50MS
#define DELAY_100MS       100   // 延时100MS
#define DELAY_200MS       200   // 延时200MS
#define DELAY_300MS       300   // 延时300MS
#define DELAY_400MS       400   // 延时400MS
#define DELAY_500MS       500   // 延时500MS
#define DELAY_800MS       800   // 延时800MS
#define DELAY_1SEC        1000  // 延时1SEC
#define DELAY_2SEC        2000  // 延时2SEC
#define DELAY_3SEC        3000  // 延时3SEC
#define DELAY_4SEC        4000  // 延时4SEC
#define DELAY_5SEC        5000  // 延时5SEC

#define SYS_SW_WIN        0xa1
#define SYS_SW_MAC        0xa2

#define RF_LINK_SHOW_TIME 300

#define HOST_USB_TYPE     0
#define HOST_BLE_TYPE     1
#define HOST_RF_TYPE      2

// 休眠时间定义
#if (1)
#define LINK_TIMEOUT     (uint16_t)(100 * 120)  // 2分钟连接超时
#define SLEEP_TIME_DELAY (uint16_t)(100 * 360)  // 6分钟一级休眠
#define POWER_DOWN_DELAY (uint16_t)(24)         // (24+6)分钟深度休眠

#else
#define LINK_TIMEOUT     (100 * 10)  // 30S
#define SLEEP_TIME_DELAY (100 * 10)  // 10S一级休眠
#define POWER_DOWN_DELAY (30)        // 30分钟深度休眠

#endif

// 用于保存于EEPROM数据
typedef struct
{
    uint8_t default_brightness_flag;
    uint8_t ee_side_mode;
    uint8_t ee_side_light;
    uint8_t ee_side_speed;
    uint8_t ee_side_rgb;
    uint8_t ee_side_colour;
    uint8_t retain1;
    uint8_t retain2;
} user_config_t;
