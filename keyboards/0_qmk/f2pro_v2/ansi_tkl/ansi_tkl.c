// Copyright 2023 Persama (@Persama)
// SPDX-License-Identifier: GPL-2.0-or-later
#include "ansi_tkl.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 长按触发时间
#define RF_LONG_PRESS_DELAY   30
#define DEV_RESET_PRESS_DELAY 30

user_config_t user_config;  // 保存在EEPROM中的用户配置信息

DEV_INFO_STRUCT dev_info =
    {
        .rf_baterry = 100,
        .link_mode  = LINK_USB,
        .rf_state   = RF_IDLE,
};

uint16_t rf_linking_time = 0;  // RF回连计时器
uint16_t rf_link_show_time;    // RF切换显示时间
uint8_t rf_blink_cnt;          // RF指示灯闪烁次数
uint16_t no_act_time = 0;      // 无按键超时

uint16_t dev_reset_press_delay = 0;  // 复位按键按下计时器
uint16_t rf_sw_press_delay     = 0;  // RF切换按键按下计时器

uint8_t rf_sw_temp = 0;

// 侧灯变量，掉电保存
extern uint8_t side_mode;    // 模式
extern uint8_t side_light;   // 默认亮度60 (0-100)
extern uint8_t side_speed;   // 速度
extern uint8_t side_rgb;     // rgb标志
extern uint8_t side_colour;  // 颜色

//------------------------------------------------
// 标志位
bool f_uart_ack         = 0;  // 串口通讯应答
bool f_bat_show         = 0;  // 电量临时显示
bool f_bat_hold         = 0;  // 电量常驻显示
bool f_dev_sleep_enable = 1;  // 设备休眠使能
bool f_chg_show         = 1;  // 充电状态显示
bool f_sys_show         = 0;  // 系统状态显示
bool f_sleep_show       = 0;  // 休眠状态显示

bool f_func_save   = 0;  // 数据保存
bool f_usb_offline = 0;  // USB手动离线标志

bool f_rf_read_data_ok = 0;  // RF数据同步成功
bool f_rf_sts_sysc_ok  = 0;  // RF数据同步成功
bool f_rf_new_adv_ok   = 0;  // RF发送新广播成功
bool f_rf_reset        = 0;  // RF复位标志
bool f_send_channel    = 0;  // RF切换通道
bool f_rf_hand_ok      = 0;  // RF同步标志

bool f_rf_send_bitkb   = 0;  // RF bit键盘发送标志
bool f_rf_send_byte    = 0;  // RF byte键盘发送标志
bool f_rf_send_consume = 0;  // RF consume报告发送标志

bool f_dial_sw_init_ok = 0;  // 拨码开关初始化OK
bool f_goto_sleep      = 0;  // 进入休眠标志

bool f_rf_sw_press     = 0;  // RF切换键按下状态
bool f_dev_reset_press = 0;  // 复位按键按下状态

//------------------------------------------------
// 函数声明

void rf_device_init(void);
void m_side_led_show(void);
void dev_sts_sync(void);
void uart_send_report_func(void);
void uart_receive_pro(void);
void Sleep_Handle(void);
uint8_t uart_send_cmd(uint8_t cmd, uint8_t ack_cnt, uint8_t delayms);
void uart_send_report(uint8_t report_type, uint8_t *report_buf, uint8_t report_size);
void m_uart1_init(void);
void m_timer6_init(void);

/** ================================================================
    功能变量保存
 ================================================================*/

void flash_data_manage(void)
{
    //------------------------ 功能变量保存
    if (f_func_save) {
        f_func_save = 0;
        uart_send_cmd(CMD_WRITE_DATA, 20, 30);
    }
}

/** ================================================================
    GPIO初始化
 ================================================================*/
void m_gpio_init(void)
{
    // 初始化RGB LED电源 PC6
    // 关闭RGB LED PC6
    setPinInput(DRIVER_LED_CS_PIN);
    setPinInput(DRIVER_SIDE_CS_PIN);
    setPinOutput(DRIVER_SIDE_PIN);
    writePinLow(DRIVER_SIDE_PIN);

    // RF唤醒引脚配置 PB8
    setPinOutput(NRF_WAKEUP_PIN);
    writePinHigh(NRF_WAKEUP_PIN);

    // 测试引脚 PB5
    setPinOutput(B5);
    writePinLow(B5);

    // RF复位引脚配置 PB4
    setPinOutput(NRF_RESET_PIN);
    writePinLow(NRF_RESET_PIN);
    wait_ms(200);
    writePinHigh(NRF_RESET_PIN);  //

    // 拨码开关检测引脚
    setPinInputHigh(DEV_MODE_PIN);  // PC0
    setPinInputHigh(SYS_MODE_PIN);  // PC1

    // 使能DC升压
    setPinOutput(DC_BOOST_PIN);
    writePinHigh(DC_BOOST_PIN);  //

}

/**================================================================
    应用层：按键长按处理
 ================================================================*/
void long_press_key(void)
{
    static uint32_t long_press_timer = 0;

    // 100ms间隔
    if (timer_elapsed32(long_press_timer) < 100) return;
    long_press_timer = timer_read32();  // store time of last refresh

    //----------------------
    // RF按键长按计时
    if (f_rf_sw_press) {
        rf_sw_press_delay++;
        if (rf_sw_press_delay >= RF_LONG_PRESS_DELAY)  // 3S
        {
            f_rf_sw_press = 0;

            dev_info.link_mode   = rf_sw_temp;
            dev_info.rf_channel  = rf_sw_temp;
            dev_info.ble_channel = rf_sw_temp;

            // 发送强制对码命令
            uint8_t timeout = 5;
            while (timeout--) {
                uart_send_cmd(CMD_NEW_ADV, 0, 1);  // 发送广播命令
                wait_ms(20);                       // 等待ACK
                if (f_rf_new_adv_ok) break;
            }
        }
    } else {
        rf_sw_press_delay = 0;
    }

    //----------------------
    // 复位长按计时
    if (f_dev_reset_press) {
        dev_reset_press_delay++;
        if (dev_reset_press_delay >= DEV_RESET_PRESS_DELAY)  // 3S
        {
            f_dev_reset_press = 0;
            uart_send_cmd(CMD_CLR_DEVICE, 10, 10);  // 发送清除RF命令
            wait_ms(300);                           // 延时等待RF Flash更新完成

            // 如果当前是无线模式，手动复位RF一次
            if (dev_info.link_mode != LINK_USB) {
                writePinLow(NRF_RESET_PIN);  // 复位RF
                wait_ms(200);
                writePinHigh(NRF_RESET_PIN);
            }

            void device_reset_show(void);
            void device_reset_init(void);

            eeconfig_init();      // 复位EEPROM数据
            device_reset_show();  // 恢复出厂设置 闪烁3次
            device_reset_init();  // 恢复出厂设置 初始化

            // 匹配当前波动开关档位
            if (dev_info.sys_sw_state == SYS_SW_MAC) {
                default_layer_set(1 << 0);  // MAC
            } else {
                default_layer_set(1 << 2);  // WIN
            }
        }
    } else {
        dev_reset_press_delay = 0;
    }
}

// 释放所有按键
void m_break_all_key(void)
{
    extern report_keyboard_t *keyboard_report;
    uint8_t report_buf[16];
    bool nkro_temp = keymap_config.nkro;  // 默认使能NKEY

    // clear_keyboard();                                       // 释放所有按键
    // clear_keys_from_report(keyboard_report);                // 清除报告

    keymap_config.nkro = 1;
    memset(keyboard_report, 0, sizeof(report_keyboard_t));  // 清除报告
    host_keyboard_send(keyboard_report);                    // 发送报告
    wait_ms(10);

    keymap_config.nkro = 0;
    memset(keyboard_report, 0, sizeof(report_keyboard_t));  // 清除报告
    host_keyboard_send(keyboard_report);                    // 发送报告
    wait_ms(10);

    keymap_config.nkro = nkro_temp;

    if (dev_info.link_mode != LINK_USB) {
        memset(report_buf, 0, 16);
        uart_send_report(CMD_RPT_BIT_KB, report_buf, 16);
        wait_ms(10);
        uart_send_report(CMD_RPT_BYTE_KB, report_buf, 8);
        wait_ms(10);
    }
}

// bool f_host_keyboard_send_off;

/** ================================================================
    拨码开关检测
 ================================================================*/
#include "usb_main.h"
uint8_t host_mode;
host_driver_t *m_host_driver = 0;

// 切换LINK模式
static void switch_dev_link(uint8_t mode)
{
    if (mode > LINK_USB) return;

    dev_info.link_mode = mode;

    // 某些场景不能直接发送，RF收到命令后会有Flash操作，需要保证电源稳定
    f_send_channel = 1;
    // uart_send_cmd(CMD_SET_LINK, 10, 10);

    m_break_all_key();  // 释放所有按键

    if (mode == LINK_USB) {
        host_mode = HOST_USB_TYPE;
        host_set_driver(m_host_driver);
        rf_link_show_time = 0;  // 显示USB模式

        // 如果之前已经手动关闭USB，需要手动打开
        if (f_usb_offline) {
            f_usb_offline = 0;
            // void protocol_init(void);
            // protocol_init();
            init_usb_driver(&USB_DRIVER);
        }
    } else {
        host_mode = HOST_RF_TYPE;
        host_set_driver(0);
        f_usb_offline = 1;
        void m_deinit_usb_072(void);
        m_deinit_usb_072();
    }
}

// 拨码开关检测
void dial_sw_scan(void)
{
    uint8_t dial_scan               = 0;
    static uint8_t dial_save        = 0xf0;
    static uint8_t debounce         = 0;
    static uint32_t dial_scan_timer = 0;
    static bool f_power_on          = 1;

    // 20ms扫描间隔
    if (timer_elapsed32(dial_scan_timer) < 20) return;
    dial_scan_timer = timer_read32();  // store time of last refresh

    //------------------------ 扫描拨码开关
    setPinInputHigh(DEV_MODE_PIN);  // PC0
    setPinInputHigh(SYS_MODE_PIN);  // PC1

    if (readPin(DEV_MODE_PIN)) dial_scan |= 0X01;
    if (readPin(SYS_MODE_PIN)) dial_scan |= 0X02;

    // 防抖处理
    if (dial_save != dial_scan) {
        no_act_time     = 0;  // 清除动作计时器
        rf_linking_time = 0;  // 清除回连计时器

        dial_save         = dial_scan;
        debounce          = 25;  // 去抖时间500ms
        f_dial_sw_init_ok = 0;   // 稳定后恢复报告输出

        m_break_all_key();  // 释放所有按键
        return;
    } else if (debounce) {
        debounce--;
        return;
    }

    // 得到稳定状态
    f_dial_sw_init_ok = 1;  // 稳定后恢复报告输出
    // f_host_keyboard_send_off = 0;

    // 有线模式/无线模式选择
    if (dial_scan & 0x01) {
        if (dev_info.link_mode != LINK_USB) {
            switch_dev_link(LINK_USB);
        }
    } else {
        if (dev_info.link_mode != dev_info.rf_channel) {
            switch_dev_link(dev_info.rf_channel);
        }
    }

    // WIN/MAC 系统切换
    if (dial_scan & 0x02) {
        if (dev_info.sys_sw_state != SYS_SW_MAC) {
            if (f_power_on)
                f_power_on = 0;  // 上电不显示
            else
                f_sys_show = 1;

            default_layer_set(1 << 0);  // MAC
            dev_info.sys_sw_state = SYS_SW_MAC;
            keymap_config.nkro    = 0;  // mac禁止NKEY
            m_break_all_key();          // 释放所有按键
        }
    } else {
        if (dev_info.sys_sw_state != SYS_SW_WIN) {
            if (f_power_on)
                f_power_on = 0;  // 上电不显示
            else
                f_sys_show = 1;

            default_layer_set(1 << 2);  // WIN
            dev_info.sys_sw_state = SYS_SW_WIN;
            keymap_config.nkro    = 1;  // win使能NKEY
            m_break_all_key();          // 释放所有按键
        }
    }
}

/** ================================================================
    自定义按键
 ================================================================*/
bool process_record_user(uint16_t keycode, keyrecord_t *record)
{
    switch (keycode) {
        case RF_DFU:
            if (record->event.pressed) {
                // 需要在USB模式下
                if (dev_info.link_mode != LINK_USB) return false;

                // 发送DFU命令
                uart_send_cmd(CMD_RF_DFU, 10, 20);
            }
            return false;
        case LNK_USB:
            if (record->event.pressed) {
                m_break_all_key();  // 释放所有按键
            } else {
                dev_info.link_mode = LINK_USB;
                uart_send_cmd(CMD_SET_LINK, 10, 10);
                rf_blink_cnt = 3;
            }
            return false;
        case LNK_RF:
            if (record->event.pressed) {
                // 有线模式不支持切换
                if (dev_info.link_mode != LINK_USB) {
                    rf_sw_temp    = LINK_RF_24;
                    f_rf_sw_press = 1;
                    m_break_all_key();  // 释放所有按键
                }
            } else if (f_rf_sw_press) {
                f_rf_sw_press = 0;
                // 短按处理
                if (rf_sw_press_delay < RF_LONG_PRESS_DELAY) {
                    dev_info.link_mode   = rf_sw_temp;
                    dev_info.rf_channel  = rf_sw_temp;
                    dev_info.ble_channel = rf_sw_temp;

                    uart_send_cmd(CMD_SET_LINK, 10, 20);  // 发送切换命令
                }
            }
            return false;
        case LNK_BLE1:
            if (record->event.pressed) {
                // 有线模式不支持切换
                if (dev_info.link_mode != LINK_USB) {
                    rf_sw_temp    = LINK_BT_1;
                    f_rf_sw_press = 1;
                    m_break_all_key();  // 释放所有按键
                }
            } else if (f_rf_sw_press) {
                f_rf_sw_press = 0;
                // 短按处理
                if (rf_sw_press_delay < RF_LONG_PRESS_DELAY) {
                    dev_info.link_mode   = rf_sw_temp;
                    dev_info.rf_channel  = rf_sw_temp;
                    dev_info.ble_channel = rf_sw_temp;

                    uart_send_cmd(CMD_SET_LINK, 10, 20);  // 发送切换命令
                }
            }
            return false;
        case LNK_BLE2:
            if (record->event.pressed) {
                if (dev_info.link_mode != LINK_USB) {
                    rf_sw_temp    = LINK_BT_2;
                    f_rf_sw_press = 1;
                    m_break_all_key();  // 释放所有按键
                }
            } else if (f_rf_sw_press) {
                f_rf_sw_press = 0;
                // 短按处理
                if (rf_sw_press_delay < RF_LONG_PRESS_DELAY) {
                    dev_info.link_mode   = rf_sw_temp;
                    dev_info.rf_channel  = rf_sw_temp;
                    dev_info.ble_channel = rf_sw_temp;

                    uart_send_cmd(CMD_SET_LINK, 10, 20);  // 发送切换命令
                }
            }
            return false;
        case LNK_BLE3:
            if (record->event.pressed) {
                if (dev_info.link_mode != LINK_USB) {
                    rf_sw_temp    = LINK_BT_3;
                    f_rf_sw_press = 1;
                    m_break_all_key();  // 释放所有按键
                }
            } else if (f_rf_sw_press) {
                f_rf_sw_press = 0;
                // 短按处理
                if (rf_sw_press_delay < RF_LONG_PRESS_DELAY) {
                    dev_info.link_mode   = rf_sw_temp;
                    dev_info.rf_channel  = rf_sw_temp;
                    dev_info.ble_channel = rf_sw_temp;

                    uart_send_cmd(CMD_SET_LINK, 10, 20);  // 发送切换命令
                }
            }
            return false;

        // 任务调度按键
        case MAC_TASK:
            if (record->event.pressed) {
                host_consumer_send(0x029F);
            } else {
                host_consumer_send(0);
            }
            return false;
        // 搜索按键
        case MAC_SEARCH:
            if (record->event.pressed) {
                register_code(KC_LGUI);
                register_code(KC_SPACE);
                uart_send_report_func();
                wait_ms(50);
                unregister_code(KC_LGUI);
                unregister_code(KC_SPACE);
            }
            return false;
        // 语音按键
        case MAC_VOICE:
            if (record->event.pressed) {
                host_consumer_send(0xcf);
            } else {
                host_consumer_send(0);
            }
            return false;
        // 启动台
        case MAC_DNT:
            if (record->event.pressed) {
                host_consumer_send(0x02A0);
            } else {
                host_consumer_send(0);
            }
            return false;
        case MAC_PRT:
            if (record->event.pressed) {
                register_code(KC_LGUI);
                register_code(KC_LSFT);
                register_code(KC_3);
                uart_send_report_func();
                wait_ms(50);
                unregister_code(KC_3);
                unregister_code(KC_LSFT);
                unregister_code(KC_LGUI);
            }
            return false;
        case MAC_PRTA:
            if (record->event.pressed) {
                register_code(KC_LGUI);
                register_code(KC_LSFT);
                register_code(KC_4);
                uart_send_report_func();
                wait_ms(50);
                unregister_code(KC_4);
                unregister_code(KC_LSFT);
                unregister_code(KC_LGUI);
            }
            return false;

            extern void light_speed_contol(uint8_t fast);
            extern void light_level_control(uint8_t brighten);
            extern void side_colour_control(uint8_t dir);
            extern void side_mode_control(uint8_t dir);

        case SIDE_VAI:
            if (record->event.pressed) {
                light_level_control(1);
            }
            return false;
        case SIDE_VAD:
            if (record->event.pressed) {
                light_level_control(0);
            }
            return false;
        case SIDE_MOD:
            if (record->event.pressed) {
                side_mode_control(1);
            }
            return false;
        case SIDE_HUI:
            if (record->event.pressed) {
                side_colour_control(1);
            }
            return false;
        case SIDE_SPI:
            if (record->event.pressed) {
                light_speed_contol(1);
            }
            return false;
        case SIDE_SPD:
            if (record->event.pressed) {
                light_speed_contol(0);
            }
            return false;

        case DEV_RESET:
            if (record->event.pressed) {
                f_dev_reset_press = 1;
                m_break_all_key();  // 释放所有按键
            } else {
                f_dev_reset_press = 0;
            }
            return false;
        case SLEEP_MODE:
            if (record->event.pressed) {
                f_dev_sleep_enable = !f_dev_sleep_enable;
                f_sleep_show       = 1;
            }
            return false;
        case BAT_SHOW:
            if (record->event.pressed) {
                f_bat_hold = !f_bat_hold;
            }
            return false;
        default:
            return true;
    }
}

// ====================================================================
// 计时处理
// ====================================================================

void timer_pro(void)
{
    static uint32_t interval_timer = 0;
    static bool f_first            = true;

    if (f_first) {
        f_first        = false;
        interval_timer = timer_read32();
        m_host_driver  = host_get_driver();
    }

    // 10ms间隔
    if (timer_elapsed32(interval_timer) < 10) {
        return;
    } else if (timer_elapsed32(interval_timer) > 20) {
        interval_timer = timer_read32();
    } else {
        interval_timer += 10;  // store time of last refresh
    }

    // rf成功连接显示时间
    if (rf_link_show_time < RF_LINK_SHOW_TIME)
        rf_link_show_time++;

    // 休眠计时器
    if (no_act_time < 0xffff)
        no_act_time++;

    if (rf_linking_time < 0xffff)
        rf_linking_time++;  // RF广播/回连时间
}

// ====================================================================
// ====================================================================

// 上电后读取EEPROM数据
void m_londing_eeprom_data(void)
{
    eeconfig_read_user_datablock(&user_config);
    if (user_config.default_brightness_flag != 0xA5) {
        // 出厂默认
        rgb_matrix_sethsv(255, 255, RGB_MATRIX_MAXIMUM_BRIGHTNESS - RGB_MATRIX_VAL_STEP * 2);  // 色调 饱和 亮度
        user_config.default_brightness_flag = 0xA5;
        user_config.ee_side_mode            = side_mode;
        user_config.ee_side_light           = side_light;
        user_config.ee_side_speed           = side_speed;
        user_config.ee_side_rgb             = side_rgb;
        user_config.ee_side_colour          = side_colour;
        eeconfig_update_user_datablock(&user_config);  // 保存
    } else {
        side_mode   = user_config.ee_side_mode;
        side_light  = user_config.ee_side_light;
        side_speed  = user_config.ee_side_speed;
        side_rgb    = user_config.ee_side_rgb;
        side_colour = user_config.ee_side_colour;
    }
}

// 初始化调用一次
void keyboard_post_init_user(void)
{
    m_gpio_init();     // GPIO初始化
    m_uart1_init();    // 串口1初始化
    wait_ms(500);      // 等待模组稳定：电池信息获取需要事件
    rf_device_init();  // RF模组初始化

    m_break_all_key();        // 上电后增加一次全键释放
    m_timer6_init();          // 用户定时器初始化
    m_londing_eeprom_data();  // 上电后读取EEPROM数据

    // keymap_config.nkro = 1;  // 默认使能NKEY
}

uint8_t bat_show_delay = 0xff;

// 指示灯处理，关灯后该指示灯函数不再调用
bool rgb_matrix_indicators_user(void)
{
    // //------------------------ 测灯灯效设计
    m_side_led_show();
    return true;
}

// 该函数在所有 QMK 处理结束、开始下一次迭代之前被调用。可以放用户循环处理程序。
void housekeeping_task_user(void)
{
    // //------------------------ 定时器处理
    timer_pro();

    // //------------------------ 串口接收处理
    uart_receive_pro();

    // //------------------------ 串口发送报告
    uart_send_report_func();

    // //------------------------ RF状态同步
    dev_sts_sync();

    // //------------------------ 按键长按处理
    long_press_key();

    // //------------------------ 拨码开关处理
    dial_sw_scan();

    // //------------------------ 数据存储管理
    flash_data_manage();

    // //------------------------ 低功耗
    Sleep_Handle();
}
