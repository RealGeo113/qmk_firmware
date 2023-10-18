// Copyright 2023 Persama (@Persama)
// SPDX-License-Identifier: GPL-2.0-or-later
#include "ansi_tkl.h"
#include "uart.h"  // qmk uart.h

#define UART_HEAD 0x5A  // 帧头
//------------------------------------------------
// 变量定义
uint8_t func_tab[32] = {0};  // 功能变量数组

extern DEV_INFO_STRUCT dev_info;
USART_MGR_STRUCT Usart_Mgr;

uint8_t disconnect_delay = 0;  // 断联计时器

uint8_t bitkb_report_buf[32];  // bit键盘报告
uint8_t bytekb_report_buf[8];  // byte键盘报告

uint16_t conkb_report;     // consume报告
uint8_t syskb_report_buf;  // sys报告

extern uint8_t host_mode;

extern uint8_t rf_blink_cnt;        // RF指示灯闪烁次数
extern uint16_t rf_link_show_time;  // RF切换显示时间
extern uint16_t rf_linking_time;
extern uint16_t no_act_time;

extern bool f_uart_ack;  // 串口通讯应答

extern bool f_rf_read_data_ok;  // RF数据同步成功
extern bool f_rf_sts_sysc_ok;   // RF数据同步成功
extern bool f_rf_new_adv_ok;    // RF发送新广播成功
extern bool f_rf_reset;         // RF复位标志
extern bool f_send_channel;     // RF切换通道
extern bool f_rf_hand_ok;       // RF同步标志

extern bool f_rf_send_bitkb;    // RF bit键盘发送标志
extern bool f_rf_send_byte;     // RF byte键盘发送标志
extern bool f_rf_send_consume;  // RF consume报告发送标志

extern bool f_dial_sw_init_ok;  // 拨码开关初始化完成标志

extern bool f_goto_sleep;  // 进入休眠标志

void uart_send_report(uint8_t report_type, uint8_t *report_buf, uint8_t report_size);
void UART_Send_Bytes(uint8_t *Buffer, uint32_t Length);
uint8_t get_checksum(uint8_t *buf, uint8_t len);

/**================================================================
 *		RX接收参数复位
================================================================*/
static void UsartMgr_RXD_Reset(void)
{
    // 复位RX状态
    Usart_Mgr.RXDLen      = 0;        // 长度清零
    Usart_Mgr.RXDState    = RX_Idle;  // 空闲状态
    Usart_Mgr.RXDOverTime = 0;        // 计时器清零
}

#define RX_SBYTE Usart_Mgr.RXDBuf[0]  // 接收首字节
#define RX_CMD   Usart_Mgr.RXDBuf[1]  // 接收CMD字节
#define RX_ACK   Usart_Mgr.RXDBuf[2]  // 接收ACK字节
#define RX_LEN   Usart_Mgr.RXDBuf[3]  // 接收LEN字节
#define RX_DAT   Usart_Mgr.RXDBuf[4]  // 接收数据起始字节

uint8_t sync_lost = 0;

/**================================================================
 * 		轮询调用：串口接收数据解析
 *      接收串口应答数据
================================================================*/
void RF_Protocol_Receive(void)
{
    uint8_t i, check_sum = 0;

    // 接收完毕，进行协议处理
    if (Usart_Mgr.RXDState == RX_Done) {
        f_uart_ack = 1;  // 从机应答

        // 校验和判断
        if (Usart_Mgr.RXDLen > 4) {
            for (i = 0; i < RX_LEN; i++)
                check_sum += Usart_Mgr.RXDBuf[4 + i];

            if (check_sum != Usart_Mgr.RXDBuf[4 + i]) {
                Usart_Mgr.RXDState = RX_SUM_ERR;  // 校验和错误
                return;
            }
        } else if (Usart_Mgr.RXDLen == 3) {
            if (Usart_Mgr.RXDBuf[2] == 0xA0)  // 从机应答
            {
                f_uart_ack = 1;
            }
        }

        sync_lost = 0;

        // 处理协议
        switch (RX_CMD) {
            //------------------------
            case CMD_POWER_UP:
                break;  // RF上电完成
            case CMD_SLEEP:
                break;  // RF进入休眠

                //------------------------
                // RF握手应答
            case CMD_HAND: {
                f_rf_hand_ok = 1;
                break;  // 握手/唤醒
            }

                //------------------------
                // 2.4G主动休眠
            case CMD_24G_SUSPEND: {
                f_goto_sleep = 1;
                break;
            }

            //------------------------
            // 回连应答
            case CMD_SET_LINK: {
                break;
            }

            //------------------------
            // 开广播应答 ack
            case CMD_NEW_ADV: {
                f_rf_new_adv_ok = 1;
                break;
            }

            // RF状态同步命令
            case CMD_RF_STS_SYSC: {
                uint8_t static error_cnt = 0;
                if (dev_info.link_mode == Usart_Mgr.RXDBuf[4])  // RF通道
                {
                    error_cnt         = 0;
                    dev_info.rf_state = Usart_Mgr.RXDBuf[5];  // RF连接状态

                    if (dev_info.rf_state == RF_CONNECT) {
                        dev_info.rf_led = Usart_Mgr.RXDBuf[6];  // 系统指示灯(连接状态下才更新)
                    }

                    // 得到电池信息
                    dev_info.rf_charge  = Usart_Mgr.RXDBuf[7];                 // 充电管理
                    dev_info.rf_baterry = Usart_Mgr.RXDBuf[8];                 // 电池电量
                    if (dev_info.rf_charge & 0x01) dev_info.rf_baterry = 100;  // 监测到5V，电量固定为100

                } else {
                    if (dev_info.rf_state != RF_INVAILD) {
                        // 通道不同步,超时再次发送
                        if (error_cnt >= 5) {
                            error_cnt      = 0;
                            f_send_channel = 1;
                        } else {
                            error_cnt++;
                        }
                    }
                }

                f_rf_sts_sysc_ok = 1;
                break;
            }

            // 清除所有主机信息：从机应答
            case CMD_CLR_DEVICE: {
                // f_rf_reset = 1;     // 复位
                // f_clr_rf_info = 1;
                break;
            }

            // 获取RF设备名
            case CMD_GET_NAME:
                break;

            // 设置RF设备名
            case CMD_SET_NAME:
                break;

            //------------------------
            // 标准键盘
            case CMD_RPT_BYTE_KB: {
                f_rf_send_byte = 0;  // 收到应答,清除发送标志
                break;
            }

            // bit键盘
            case CMD_RPT_BIT_KB: {
                f_rf_send_bitkb = 0;  // 收到应答,清除发送标志
                break;
            }

            // 消费类
            case CMD_RPT_CONSUME: {
                f_rf_send_consume = 0;  // 收到应答,清除发送标志
                break;
            }

            // 系统类
            case CMD_RPT_SYS:
                break;

            //------------------------
            // 写数据
            case CMD_WRITE_DATA:
                break;

            // 读数据
            case CMD_READ_DATA: {
                memcpy(func_tab, &Usart_Mgr.RXDBuf[4], 32);

                // 当前通道
                if (func_tab[4] <= LINK_USB) {
                    dev_info.link_mode = func_tab[4];
                }

                // RF通道
                if (func_tab[5] < LINK_USB) {
                    dev_info.rf_channel = func_tab[5];
                }

                // BLE通道
                if ((func_tab[6] <= LINK_BT_3) && (func_tab[6] >= LINK_BT_1)) {
                    dev_info.ble_channel = func_tab[6];
                }

                f_rf_read_data_ok = 1;
                break;
            }

            //------------------------
            // 非法命令
            default:
                Usart_Mgr.RXDState = RX_CMD_ERR;  // 校验和错误
                return;
        }

        UsartMgr_RXD_Reset();
    }
}

/**================================================================
    应用层：发送报告到RF模组

检测到变化时立即发送
否则定期轮询发送
 ================================================================*/
uint16_t host_last_consumer_usage(void);
void uart_send_report_func(void)
{
    static uint32_t interval_timer = 0;

    if (f_dial_sw_init_ok == 0) return;           // 拨码开关未稳定
    if (dev_info.link_mode == LINK_USB) return;   // USB模式
    if (dev_info.rf_state != RF_CONNECT) return;  // 未连接状态

    keyboard_protocol = 1;

    // NKEY键盘得到mods状态
    if (keymap_config.nkro == 1) {
        bitkb_report_buf[0] = get_mods();
    }

    // 检测byte键盘变化
    if ((keymap_config.nkro == 0) && (memcmp(bytekb_report_buf, keyboard_report->raw, 8))) {
        no_act_time             = 0;
        keyboard_report->raw[1] = 0;  // 预留字节强制为0
        memcpy(bytekb_report_buf, keyboard_report->raw, 8);
        uart_send_report(CMD_RPT_BYTE_KB, bytekb_report_buf, 8);
    }
    // 检测bit键盘变化
    else if ((keymap_config.nkro == 1) && (memcmp(bitkb_report_buf, &keyboard_report->nkro.mods, 16))) {
        no_act_time = 0;
        memcpy(&bitkb_report_buf[1], keyboard_report->nkro.bits, 15);
        uart_send_report(CMD_RPT_BIT_KB, bitkb_report_buf, 16);
    }
    // 检测多媒体变化
    else if (host_last_consumer_usage() != conkb_report) {
        conkb_report = host_last_consumer_usage();
        no_act_time  = 0;
        uart_send_report(CMD_RPT_CONSUME, (uint8_t *)(&conkb_report), 2);
    }
    //  定期发送键盘报告以保证报告稳定性
    else if (timer_elapsed32(interval_timer) > 100) {
        interval_timer = timer_read32();  // store time of last refresh

        if (keymap_config.nkro)
            uart_send_report(CMD_RPT_BIT_KB, bitkb_report_buf, 16);
        else
            uart_send_report(CMD_RPT_BYTE_KB, bytekb_report_buf, 8);
    }
}

/**================================================================
 * @brief           发送命令到RF模组
 * @param cmd       命令
 * @param delayms   发送前延时
 ================================================================*/
uint8_t uart_send_cmd(uint8_t cmd, uint8_t wait_ack, uint8_t delayms)
{
    uint8_t i;

    // 延时
    wait_ms(delayms);

    // 清除发送缓存
    memset(&Usart_Mgr.TXDBuf[0], 0, UART_MAX_LEN);

    // 准备发送数据
    Usart_Mgr.TXDBuf[0] = UART_HEAD;  // FLAG
    Usart_Mgr.TXDBuf[1] = cmd;        // CMD
    Usart_Mgr.TXDBuf[2] = 0x00;       // ACK

    switch (cmd) {
        // 上电命令
        case CMD_POWER_UP: {
            Usart_Mgr.TXDBuf[3] = 1;  // len
            Usart_Mgr.TXDBuf[4] = 0;  // dat
            Usart_Mgr.TXDBuf[5] = 0;  // sum
            break;
        }
        // Idle命令，一级休眠
        case CMD_SNIF: {
            Usart_Mgr.TXDBuf[3] = 1;  // len
            Usart_Mgr.TXDBuf[4] = 0;  // dat
            Usart_Mgr.TXDBuf[5] = 0;  // sum
            break;
        }
        // 休眠信号，二级休眠
        case CMD_SLEEP: {
            Usart_Mgr.TXDBuf[3] = 1;  // len
            Usart_Mgr.TXDBuf[4] = 0;  // dat
            Usart_Mgr.TXDBuf[5] = 0;  // sum
            break;
        }
        // 唤醒信号
        case CMD_HAND: {
            Usart_Mgr.TXDBuf[3] = 1;  // len
            Usart_Mgr.TXDBuf[4] = 0;  // dat
            Usart_Mgr.TXDBuf[5] = 0;  // sum
            break;
        }
        //------------------------------------------------
        // 状态同步命令
        case CMD_RF_STS_SYSC: {
            Usart_Mgr.TXDBuf[3] = 1;                   // len
            Usart_Mgr.TXDBuf[4] = dev_info.link_mode;  // dat
            Usart_Mgr.TXDBuf[5] = dev_info.link_mode;  // sum
            break;
        }
        //------------------------------------------------
        // 设置连接通道：USB/2.4G/BT1/BT2/BT3
        case CMD_SET_LINK: {
            dev_info.rf_state   = RF_LINKING;          // 状态为广播配对模式
            Usart_Mgr.TXDBuf[3] = 1;                   // len
            Usart_Mgr.TXDBuf[4] = dev_info.link_mode;  // dat
            Usart_Mgr.TXDBuf[5] = dev_info.link_mode;  // sum

            rf_linking_time  = 0;     // 重新开始计时
            disconnect_delay = 0xff;  // 直接闪烁
            break;
        }
        //------------------------------------------------
        // 强制广播/配对
        case CMD_NEW_ADV: {
            dev_info.rf_state   = RF_PAIRING;              // 状态为广播配对模式
            Usart_Mgr.TXDBuf[3] = 2;                       // len
            Usart_Mgr.TXDBuf[4] = dev_info.link_mode;      // dat 通道
            Usart_Mgr.TXDBuf[5] = 1;                       // 蓝牙协议版本
            Usart_Mgr.TXDBuf[6] = dev_info.link_mode + 1;  // sum

            rf_linking_time  = 0;     // 重新开始计时
            disconnect_delay = 0xff;  // 直接闪烁

            f_rf_new_adv_ok = 0;
            break;
        }
        //------------------------------------------------
        // 清除无线绑定信息设备
        case CMD_CLR_DEVICE: {
            Usart_Mgr.TXDBuf[3] = 1;  // len
            Usart_Mgr.TXDBuf[4] = 0;  // dat
            Usart_Mgr.TXDBuf[5] = 0;  // sum
            break;
        }
        //------------------------------------------------
        // 设置无线参数 ： 设置深度休眠时间
        case CMD_SET_CONFIG: {
            Usart_Mgr.TXDBuf[3] = 1;                 // len
            Usart_Mgr.TXDBuf[4] = POWER_DOWN_DELAY;  // dat
            Usart_Mgr.TXDBuf[5] = POWER_DOWN_DELAY;  // sum
            break;
        }
        //------------------------------------------------
        // 设置蓝牙设备名
        case CMD_SET_NAME: {
            Usart_Mgr.TXDBuf[3]  = 17;                                                       // data len
            Usart_Mgr.TXDBuf[4]  = 1;                                                        // type     0-带尾缀    1-带尾缀
            Usart_Mgr.TXDBuf[5]  = 15;                                                       // data: ble name len
            Usart_Mgr.TXDBuf[6]  = 'N';                                                      // data: ble name
            Usart_Mgr.TXDBuf[7]  = 'u';                                                      // data: ble name
            Usart_Mgr.TXDBuf[8]  = 'P';                                                      // data: ble name
            Usart_Mgr.TXDBuf[9]  = 'h';                                                      // data: ble name
            Usart_Mgr.TXDBuf[10] = 'y';                                                      // data: ble name
            Usart_Mgr.TXDBuf[11] = ' ';                                                      // data: ble name
            Usart_Mgr.TXDBuf[12] = 'A';                                                      // data: ble name
            Usart_Mgr.TXDBuf[13] = 'i';                                                      // data: ble name
            Usart_Mgr.TXDBuf[14] = 'r';                                                      // data: ble name
            Usart_Mgr.TXDBuf[15] = '7';                                                      // data: ble name
            Usart_Mgr.TXDBuf[16] = '5';                                                      // data: ble name
            Usart_Mgr.TXDBuf[17] = ' ';                                                      // data: ble name
            Usart_Mgr.TXDBuf[18] = 'V';                                                      // data: ble name
            Usart_Mgr.TXDBuf[19] = '2';                                                      // data: ble name
            Usart_Mgr.TXDBuf[20] = '-';                                                      // data: ble name
            Usart_Mgr.TXDBuf[21] = get_checksum(Usart_Mgr.TXDBuf + 4, Usart_Mgr.TXDBuf[3]);  // sum
            break;
        }
        //------------------------------------------------
        // 读取RF通道数据
        case CMD_READ_DATA: {
            Usart_Mgr.TXDBuf[3] = 2;               // 有效数据长度
            Usart_Mgr.TXDBuf[4] = 0x00;            // offset偏移
            Usart_Mgr.TXDBuf[5] = FUNC_VALID_LEN;  // 读取数据长度
            Usart_Mgr.TXDBuf[6] = FUNC_VALID_LEN;  // sum
            break;
        }
        //------------------------------------------------
        // 写数据到RF
        case CMD_WRITE_DATA: {
            func_tab[4] = dev_info.link_mode;
            func_tab[5] = dev_info.rf_channel;
            func_tab[6] = dev_info.ble_channel;

            Usart_Mgr.TXDBuf[3] = FUNC_VALID_LEN + 2;
            Usart_Mgr.TXDBuf[4] = 0;               // RF数据起始地址
            Usart_Mgr.TXDBuf[5] = FUNC_VALID_LEN;  // RF数据长度
                                                   // 数据填入buf
            for (i = 0; i < FUNC_VALID_LEN; i++) {
                Usart_Mgr.TXDBuf[6 + i] = func_tab[i];
            }
            // 计算sum
            Usart_Mgr.TXDBuf[6 + i] = get_checksum(func_tab, FUNC_VALID_LEN);
            Usart_Mgr.TXDBuf[6 + i] += 0;
            Usart_Mgr.TXDBuf[6 + i] += FUNC_VALID_LEN;
            break;
        }
        //------------------------------------------------
        // 发送DFU升级命令
        case CMD_RF_DFU: {
            Usart_Mgr.TXDBuf[3] = 1;  // len
            Usart_Mgr.TXDBuf[4] = 0;  // dat
            Usart_Mgr.TXDBuf[5] = 0;  // sum
            break;
        }

        default:
            break;
    }

    f_uart_ack = 0;
    UART_Send_Bytes(Usart_Mgr.TXDBuf, Usart_Mgr.TXDBuf[3] + 5);

    // 等待应答
    if (wait_ack) {
        // 等待应答
        while (wait_ack--) {
            wait_ms(1);
            if (f_uart_ack)
                return TX_OK;
        }
    } else {
        return TX_OK;
    }

    return TX_TIMEOUT;
}

/**================================================================
 * 		RF设备同步处理
 * 处理回连
 * 更新连接状态
================================================================*/
void m_break_all_key(void);
extern host_driver_t *m_host_driver;
void dev_sts_sync(void)
{
    static uint32_t interval_timer = 0;  // 系统切换长按计时

    // 200ms同步间隔
    if (timer_elapsed32(interval_timer) < 200) return;
    interval_timer = timer_read32();  // store time of last refresh

    //------------------------ RF通道重发
    if (f_send_channel) {
        f_send_channel = 0;
        uart_send_cmd(CMD_SET_LINK, 10, 10);
    }

    //------------------------ RF模块复位，重新启动
    if (f_rf_reset) {
        f_rf_reset = 0;

        writePinLow(NRF_RESET_PIN);  // 复位RF
        wait_ms(200);
        writePinHigh(NRF_RESET_PIN);
    }

    //------------------------ USB模式
    if (dev_info.link_mode == LINK_USB) {
        if (host_mode != HOST_USB_TYPE) {
            host_mode = HOST_USB_TYPE;
            host_set_driver(m_host_driver);
            m_break_all_key();  // 释放所有按键
        }
        rf_blink_cnt = 0;
    }
    //------------------------ RF无线模式
    else {
        if (host_mode != HOST_RF_TYPE) {
            host_mode = HOST_RF_TYPE;
            m_break_all_key();  // 释放所有按键
            host_set_driver(0);
        }

        // 非连接状态
        if (dev_info.rf_state != RF_CONNECT) {
            // 增加断联延时1S，解决偶尔断联就闪烁的问题
            if (disconnect_delay >= 5) {
                rf_blink_cnt      = 3;  // 非连接状态闪烁
                rf_link_show_time = 0;  // 开始显示3S
            } else {
                disconnect_delay++;
            }
        }
        // 已连接状态
        else if (dev_info.rf_state == RF_CONNECT) {
            rf_linking_time  = 0;
            disconnect_delay = 0;
            rf_blink_cnt     = 0;  // 连接成功后闪烁0次
        }
    }

    //------------------------
    // 主动状态同步命令
    uart_send_cmd(CMD_RF_STS_SYSC, 1, 1);  // 200ms 发送一次状态同步命令
    wait_ms(1);

    // 如果长时间没有同步应答, 复位RF模组 / USB模式下需要保持OTA升级，不做复位操作
    if (dev_info.link_mode != LINK_USB) {
        if (++sync_lost >= 5)  // 1S
        {
            sync_lost  = 0;
            f_rf_reset = 1;
        }
    }
}

/**================================================================
    RF模组初始化函数

1. 和RF模组进行通讯握手
2. 从RF中读取功能变量
 ================================================================*/
void rf_device_init(void)
{
    uint8_t timeout = 0;
    void uart_receive_pro(void);

    // 等待RF握手应答
    timeout      = 10;
    f_rf_hand_ok = 0;
    while (timeout--) {
        uart_send_cmd(CMD_HAND, 0, 20);  // 握手
        wait_ms(5);
        uart_receive_pro();  // 第一次接收
        uart_receive_pro();  // 第二次处理
        if (f_rf_hand_ok)
            break;
    }

    // 读取RF通道数据
    timeout           = 10;
    f_rf_read_data_ok = 0;
    while (timeout--) {
        uart_send_cmd(CMD_READ_DATA, 0, 20);  // 读取RF数据
        wait_ms(5);
        uart_receive_pro();  // 第一次接收
        uart_receive_pro();  // 第二次处理
        if (f_rf_read_data_ok)
            break;
    }

    // 同步电池数据
    timeout          = 10;
    f_rf_sts_sysc_ok = 0;
    while (timeout--) {
        uart_send_cmd(CMD_RF_STS_SYSC, 0, 20);  // 读取RF数据
        wait_ms(5);
        uart_receive_pro();  // 第一次接收
        uart_receive_pro();  // 第二次处理
        if (f_rf_sts_sysc_ok)
            break;
    }

    // 上电设置蓝牙名称
    uart_send_cmd(CMD_SET_NAME, 10, 20);
}

/**================================================================
    UART
 ================================================================*/

// 连续发送
void UART_Send_Bytes(uint8_t *Buffer, uint32_t Length)
{
    writePinLow(NRF_WAKEUP_PIN);
    wait_us(50);

    uart_transmit(Buffer, Length);

    wait_us(50 + Length * 30);
    writePinHigh(NRF_WAKEUP_PIN);
}

// 计算数组checksum
uint8_t get_checksum(uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t checksum = 0;

    for (i = 0; i < len; i++)
        checksum += *buf++;

    checksum ^= UART_HEAD;

    return checksum;
}

// 串口发送报告
void uart_send_report(uint8_t report_type, uint8_t *report_buf, uint8_t report_size)
{
    Usart_Mgr.TXDBuf[0] = UART_HEAD;    // 帧起始
    Usart_Mgr.TXDBuf[1] = report_type;  // 命令字节
    Usart_Mgr.TXDBuf[2] = 0x01;         // 应答要求字节
    Usart_Mgr.TXDBuf[3] = report_size;  // 数据字段长度/不包括sum

    memcpy(&Usart_Mgr.TXDBuf[4], report_buf, report_size);
    Usart_Mgr.TXDBuf[4 + report_size] = get_checksum(&Usart_Mgr.TXDBuf[4], report_size);

    // 发送总长度：4 + dat_len + 1
    UART_Send_Bytes(&Usart_Mgr.TXDBuf[0], report_size + 5);

    // 延时控制帧间隔
    wait_us(200);
}

/** ================================================================
 *  串口接收处理函数
  ================================================================ */
void uart_receive_pro(void)
{
    static bool rcv_start = false;

    // 查询是否有新的数据
    if (uart_available()) {
        rcv_start = true;  // 开始接收
        while (uart_available()) {
            if (Usart_Mgr.RXDLen >= UART_MAX_LEN) {
                uart_read();
                Usart_Mgr.RXDState = RX_DATA_OV;
            } else {
                Usart_Mgr.RXDBuf[Usart_Mgr.RXDLen++] = uart_read();
            }
        }

    } else if (rcv_start) {
        // 帧结束：一个轮询周期未收到新数据
        rcv_start          = false;
        Usart_Mgr.RXDState = RX_Done;
        Usart_Mgr.RXDLen   = 0;
        RF_Protocol_Receive();
    }
}
