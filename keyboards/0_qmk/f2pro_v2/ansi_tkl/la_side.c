// Copyright 2023 Persama (@Persama)
// SPDX-License-Identifier: GPL-2.0-or-later
#include "ansi_tkl.h"
//------------------------------------------------
#include "la_side.h"

// ���
#define SIDE_WAVE        0  // ����
#define SIDE_MIX         1  // ����
#define SIDE_STATIC      2  // ��̬
#define SIDE_BREATH      3  // ����
#define SIDE_OFF         4  // �ر�

#define LIGHT_COLOUR_MAX 8  // ��ɫ
#define SIDE_COLOUR_MAX  8  // ��ɫ
#define LIGHT_SPEED_MAX  4  // �ٶ�

// ��Ч�ٶȲ��
const uint8_t side_speed_table[5][5] = {
    [SIDE_WAVE]   = {10, 14, 20, 28, 38},
    [SIDE_MIX]    = {10, 14, 20, 28, 38},
    [SIDE_STATIC] = {50, 50, 50, 50, 50},
    [SIDE_BREATH] = {10, 14, 20, 28, 38},
    [SIDE_OFF]    = {50, 50, 50, 50, 50},
};

const uint8_t side_light_table[6] = {
    0,
    22,
    34,
    55,
    79,
    106,
};

#define SIDE_LINE 6  // �������
const uint8_t side_led_index_tab[SIDE_LINE][2] = {
    {5, 6},
    {4, 7},
    {3, 8},
    {2, 9},
    {1, 10},
    {0, 11},
};

uint8_t side_mode       = 0;  // ģʽ
uint8_t side_light      = 3;  // Ĭ������60 (0-100)
uint8_t side_speed      = 2;  // �ٶ�
uint8_t side_rgb        = 1;  // rgb��־
uint8_t side_colour     = 0;  // ��ɫ
uint8_t side_play_point = 0;  // ��Ʋ���ָ��

uint8_t side_play_cnt    = 0;  // ʱ�������
uint32_t side_play_timer = 0;

uint8_t r_temp, g_temp, b_temp;  // rgb��ʱ����

extern DEV_INFO_STRUCT dev_info;
extern bool f_bat_hold;
extern user_config_t user_config;

/*======================================================================
Name:		���RGB������ˢ��
======================================================================*/
#define SIDE_LED_NUM 12
LED_TYPE side_leds[SIDE_LED_NUM] = {0};

void side_ws2812_setleds(LED_TYPE *ledarray, uint16_t leds);

void side_rgb_set_color(int index, uint8_t red, uint8_t green, uint8_t blue)
{
    side_leds[index].r = red;
    side_leds[index].g = green;
    side_leds[index].b = blue;
}

bool is_side_rgb_off(void)
{
    for (int i = 0; i < SIDE_LED_NUM; i++) {
        if ((side_leds[i].r != 0) || (side_leds[i].g != 0) || (side_leds[i].b != 0)) {
            return false;
        }
    }
    return true;
}

void side_rgb_refresh(void)
{
    side_ws2812_setleds(side_leds, SIDE_LED_NUM);
}

/*======================================================================
Name:		RGB����/����
======================================================================*/

// RGB����
void suspend_power_down_kb(void)
{
    rgb_matrix_set_suspend_state(true);
}

// RGB����
void suspend_wakeup_init_kb(void)
{
    rgb_matrix_set_suspend_state(false);
}

/*======================================================================
Name:		�������ȿ���
Descr:		0% / 20% / 40% / 60%/ 80% / 100%
======================================================================*/
void light_level_control(uint8_t brighten)
{
    if (brighten)  // ����++
    {
        if (side_light == 5) {
            return;
        } else
            side_light++;
    } else  // ����--
    {
        if (side_light == 0) {
            return;
        } else
            side_light--;
    }
    user_config.ee_side_light = side_light;
    eeconfig_update_user_datablock(&user_config);  // ����
}

/*======================================================================
Name:   �����ٶȿ��� 0,1,2,3,4 (0���, 4����)
======================================================================*/
void light_speed_contol(uint8_t fast)
{
    if ((side_speed) > LIGHT_SPEED_MAX)
        (side_speed) = LIGHT_SPEED_MAX / 2;

    if (fast) {
        if ((side_speed)) side_speed--;  // �ٶȼӿ�
    } else {
        if ((side_speed) < LIGHT_SPEED_MAX) side_speed++;  // �ٶȼ���
    }
    user_config.ee_side_speed = side_speed;
    eeconfig_update_user_datablock(&user_config);  // ����
}

/*======================================================================
Name:		����׼ɫ�л�
Descr:		����Ϊ��Ȼ��������ϰ�/��ɫѭ��
======================================================================*/
void side_colour_control(uint8_t dir)
{
    // ֻ�в���ģʽ�в�ɫģʽ,����ģʽ������ɫ��λ
    if (side_mode != SIDE_WAVE) {
        if (side_rgb) {
            side_rgb    = 0;
            side_colour = 0;
        }
    }

    //-------- ������ɫ����
    if (dir) {  // ˳���л�
        if (side_rgb) {
            side_rgb    = 0;
            side_colour = 0;
        } else {
            side_colour++;
            if (side_colour >= LIGHT_COLOUR_MAX) {
                side_rgb    = 1;
                side_colour = 0;
            }
        }
    } else {  // �����л�
        if (side_rgb) {
            side_rgb    = 0;
            side_colour = LIGHT_COLOUR_MAX - 1;
        } else {
            side_colour--;
            if (side_colour >= LIGHT_COLOUR_MAX) {
                side_rgb    = 1;
                side_colour = 0;
            }
        }
    }
    user_config.ee_side_rgb    = side_rgb;
    user_config.ee_side_colour = side_colour;
    eeconfig_update_user_datablock(&user_config);  // ����
}

/*======================================================================
Name:		���ģʽ�л�
======================================================================*/
void side_mode_control(uint8_t dir)
{
    if (dir) {  // ˳���л�
        side_mode++;
        if (side_mode > SIDE_OFF) {
            side_mode = 0;
        }
    } else {  // �����л�
        if (side_mode > 0) {
            side_mode--;
        } else {
            side_mode = SIDE_OFF;
        }
    }
    side_play_point          = 0;
    user_config.ee_side_mode = side_mode;
    eeconfig_update_user_datablock(&user_config);  // ����
}

/*======================================================================
Name:		�������Ҳ����ɫ
======================================================================*/
// �������RGB
void set_left_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < 6; i++)
        side_rgb_set_color(i, r >> 2, g >> 2, b >> 2);
}

// �����Ҳ�RGB
void set_right_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 6; i < 12; i++)
        side_rgb_set_color(i, r >> 2, g >> 2, b >> 2);
}

/*======================================================================
Name:		ϵͳ�л�ʱ��ʾ3S
======================================================================*/

void sys_sw_led_show(void)
{
    static uint32_t sys_show_timer = 0;
    static bool sys_show_flag      = false;
    extern bool f_sys_show;

    if (f_sys_show) {
        f_sys_show     = false;
        sys_show_timer = timer_read32();  // store time of last refresh
        sys_show_flag  = true;
    }

    if (sys_show_flag) {
        if (dev_info.sys_sw_state == SYS_SW_MAC) {
            r_temp = 0x80;
            g_temp = 0x80;
            b_temp = 0x80;
        } else {
            r_temp = 0x00;
            g_temp = 0x00;
            b_temp = 0x80;
        }
        if ((timer_elapsed32(sys_show_timer) / 500) % 2 == 0) {
            set_right_rgb(r_temp, g_temp, b_temp);
        } else {
            set_right_rgb(0x00, 0x00, 0x00);
        }
        if (timer_elapsed32(sys_show_timer) >= 3000) {
            sys_show_flag = false;
        }
    }
}

/*======================================================================
Name:		�ر�/���Զ�����ʱ��ʾ3S
======================================================================*/
void sleep_sw_led_show(void)
{
    static uint32_t sleep_show_timer = 0;
    static bool sleep_show_flag      = false;
    extern bool f_sleep_show;
    extern bool f_dev_sleep_enable;

    if (f_sleep_show) {
        f_sleep_show     = false;
        sleep_show_timer = timer_read32();  // store time of last refresh
        sleep_show_flag  = true;
    }

    if (sleep_show_flag) {
        if (f_dev_sleep_enable) {
            r_temp = 0x00;
            g_temp = 0x80;
            b_temp = 0x00;
        } else {
            r_temp = 0x80;
            g_temp = 0x00;
            b_temp = 0x00;
        }
        if ((timer_elapsed32(sleep_show_timer) / 500) % 2 == 0) {
            set_right_rgb(r_temp, g_temp, b_temp);
        } else {
            set_right_rgb(0x00, 0x00, 0x00);
        }
        if (timer_elapsed32(sleep_show_timer) >= 3000) {
            sleep_show_flag = false;
        }
    }
}

/** ================================================================
 *  ϵͳָʾ�ƿ���
 ================================================================*/
void sys_led_show(void)
{
    // USB����ģʽ
    if (dev_info.link_mode == LINK_USB) {
        // caps lock led
        if (host_keyboard_led_state().caps_lock) {
            set_left_rgb(0X00, 0x80, 0x80);
        }
    }
    // RF����ģʽ
    else {
        if (dev_info.rf_led & 0x02) {
            set_left_rgb(0X00, 0x80, 0x80);
        }
    }
}

/*======================================================================
Name:		��Ч����ָ�����
Descr:		���Ƶ�Ч���ŷ���/��ʾ���
======================================================================*/
static void light_point_playing(uint8_t trend, uint8_t step, uint8_t len, uint8_t *point)
{
    if (trend) {
        *point += step;
        if (*point >= len) *point -= len;
    } else {
        *point -= step;
        if (*point >= len) *point = len - (255 - *point) - 1;
    }
}

/*======================================================================
Name:		���ȼ���
Descr:		�������������ֵ, ����R/G/B��ɫ����
======================================================================*/
static void count_rgb_light(uint8_t light_temp)
{
    uint16_t temp;

    temp   = (light_temp)*r_temp + r_temp;
    r_temp = temp >> 8;

    temp   = (light_temp)*g_temp + g_temp;
    g_temp = temp >> 8;

    temp   = (light_temp)*b_temp + b_temp;
    b_temp = temp >> 8;
}

/**================================================================================================================================
 * 		�����ƹ����
 ================================================================================================================================*/
// ����
static void side_wave_mode_show(void)
{
    uint8_t play_index;

    //------------------------------
    if (side_play_cnt <= side_speed_table[side_mode][side_speed])
        return;
    else
        side_play_cnt -= side_speed_table[side_mode][side_speed];
    if (side_play_cnt > 20) side_play_cnt = 0;

    //------------------------------
    if (side_rgb)
        light_point_playing(0, 3, FLOW_COLOUR_TAB_LEN, &side_play_point);
    else
        light_point_playing(0, 2, WAVE_TAB_LEN, &side_play_point);

    play_index = side_play_point;
    for (int i = 0; i < SIDE_LINE; i++) {
        if (side_rgb) {
            r_temp = flow_rainbow_colour_tab[play_index][0];
            g_temp = flow_rainbow_colour_tab[play_index][1];
            b_temp = flow_rainbow_colour_tab[play_index][2];

            // �ƶ�
            light_point_playing(1, 16, FLOW_COLOUR_TAB_LEN, &play_index);
        } else {
            r_temp = colour_lib[side_colour][0];
            g_temp = colour_lib[side_colour][1];
            b_temp = colour_lib[side_colour][2];

            // ��ɫ�ƶ�
            light_point_playing(1, 12, WAVE_TAB_LEN, &play_index);
            count_rgb_light(wave_data_tab[play_index]);
        }

        count_rgb_light(side_light_table[side_light]);

        for (int j = 0; j < 2; j++) {
            side_rgb_set_color(side_led_index_tab[i][j], r_temp >> 2, g_temp >> 2, b_temp >> 2);
        }
    }
}

// ����
static void side_spectrum_mode_show(void)
{
    //------------------------------
    if (side_play_cnt <= side_speed_table[side_mode][side_speed])
        return;
    else
        side_play_cnt -= side_speed_table[side_mode][side_speed];
    if (side_play_cnt > 20) side_play_cnt = 0;

    //------------------------------
    light_point_playing(1, 1, FLOW_COLOUR_TAB_LEN, &side_play_point);

    r_temp = flow_rainbow_colour_tab[side_play_point][0];
    g_temp = flow_rainbow_colour_tab[side_play_point][1];
    b_temp = flow_rainbow_colour_tab[side_play_point][2];

    count_rgb_light(side_light_table[side_light]);

    for (int i = 0; i < SIDE_LINE; i++) {
        for (int j = 0; j < 2; j++) {
            side_rgb_set_color(side_led_index_tab[i][j], r_temp >> 2, g_temp >> 2, b_temp >> 2);
        }
    }
}

// ����
static void side_breathe_mode_show(void)
{
    static uint8_t play_point = 0;

    //------------------------------
    if (side_play_cnt <= side_speed_table[side_mode][side_speed])
        return;
    else
        side_play_cnt -= side_speed_table[side_mode][side_speed];
    if (side_play_cnt > 20) side_play_cnt = 0;

    //------------------------------
    light_point_playing(0, 1, BREATHE_TAB_LEN, &play_point);

    // ������ɫ
    // if (side_rgb) {
    if (0) {
        if (play_point == 0) {
            if (++side_play_point >= LIGHT_COLOUR_MAX)
                side_play_point = 0;
        }

        r_temp = colour_lib[side_play_point][0];
        g_temp = colour_lib[side_play_point][1];
        b_temp = colour_lib[side_play_point][2];
    } else {
        r_temp = colour_lib[side_colour][0];
        g_temp = colour_lib[side_colour][1];
        b_temp = colour_lib[side_colour][2];
    }

    count_rgb_light(breathe_data_tab[play_point]);
    count_rgb_light(side_light_table[side_light]);

    for (int i = 0; i < SIDE_LINE; i++) {
        for (int j = 0; j < 2; j++) {
            side_rgb_set_color(side_led_index_tab[i][j], r_temp >> 2, g_temp >> 2, b_temp >> 2);
        }
    }
}

// ��̬
static void side_static_mode_show(void)
{
    uint8_t play_index;

    //------------------------------
    if (side_play_cnt <= side_speed_table[side_mode][side_speed])
        return;
    else
        side_play_cnt -= side_speed_table[side_mode][side_speed];
    if (side_play_cnt > 20) side_play_cnt = 0;

    //------------------------------
    if (side_play_point >= SIDE_COLOUR_MAX) side_play_point = 0;

    for (int i = 0; i < SIDE_LINE; i++) {
        // if (side_rgb)  // ��ɫ����
        if (0) {
            r_temp = flow_rainbow_colour_tab[16 * i][0];
            g_temp = flow_rainbow_colour_tab[16 * i][1];
            b_temp = flow_rainbow_colour_tab[16 * i][2];
            light_point_playing(0, 24, FLOW_COLOUR_TAB_LEN, &play_index);
        } else  // ��ɫ����
        {
            r_temp = colour_lib[side_colour][0];
            g_temp = colour_lib[side_colour][1];
            b_temp = colour_lib[side_colour][2];
        }

        count_rgb_light(side_light_table[side_light]);

        for (int j = 0; j < 2; j++) {
            side_rgb_set_color(side_led_index_tab[i][j], r_temp >> 2, g_temp >> 2, b_temp >> 2);
        }
    }
}

// �ر�
static void side_off_mode_show(void)
{
    //------------------------------
    if (side_play_cnt <= side_speed_table[side_mode][side_speed])
        return;
    else
        side_play_cnt -= side_speed_table[side_mode][side_speed];
    if (side_play_cnt > 20) side_play_cnt = 0;

    //------------------------------
    r_temp = 0x00;
    g_temp = 0x00;
    b_temp = 0x00;

    for (int i = 0; i < SIDE_LINE; i++) {
        for (int j = 0; j < 2; j++) {
            side_rgb_set_color(side_led_index_tab[i][j], r_temp >> 2, g_temp >> 2, b_temp >> 2);
        }
    }
}

/**================================================================================================================================
 * 		RFָʾ�ƿ���
 ================================================================================================================================*/
#define RF_LED_LINK_PERIOD 500
#define RF_LED_PAIR_PERIOD 250

void rf_led_show(void)
{
    static uint32_t rf_blink_timer = 0;
    uint16_t rf_blink_priod        = 0;
    extern bool f_dial_sw_init_ok;
    extern uint8_t rf_blink_cnt;
    extern uint16_t rf_link_show_time;

    // �ȴ����뿪��ʶ�����
    if (!f_dial_sw_init_ok) return;

    if (dev_info.link_mode == LINK_RF_24)  // 2.4G��ɫ
    {
        r_temp = 0x00;
        g_temp = 0x80;
        b_temp = 0x00;
    } else if (dev_info.link_mode == LINK_USB) {
        r_temp = 0x80;  // usb��ɫ
        g_temp = 0x80;
        b_temp = 0x00;
    } else  // ������ɫ
    {
        r_temp = 0x00;
        g_temp = 0x00;
        b_temp = 0x80;
    }

    // ����RFָʾ��
    if (rf_blink_cnt)  // ��˸����
    {
        // ������˸����
        if (dev_info.rf_state == RF_PAIRING)
            rf_blink_priod = RF_LED_PAIR_PERIOD;
        else
            rf_blink_priod = RF_LED_LINK_PERIOD;

        // ������˸��ɫ
        if (timer_elapsed32(rf_blink_timer) < (rf_blink_priod >> 1)) {
        } else {
            r_temp = 0x00;
            g_temp = 0x00;
            b_temp = 0x00;
        }

        if (timer_elapsed32(rf_blink_timer) >= rf_blink_priod) {
            rf_blink_cnt--;
            rf_blink_timer = timer_read32();
        }
    } else if (rf_link_show_time < RF_LINK_SHOW_TIME) {
    } else {
        rf_blink_timer = timer_read32();
        return;
    }

    set_left_rgb(r_temp, g_temp, b_temp);
}

/**================================================================================================================================
 * 		BATָʾ�ƿ���
 ================================================================================================================================*/

uint8_t bat_pwm_buf[6 * 3] = {0};
uint8_t bat_end_led        = 0;
uint8_t bat_r, bat_g, bat_b;

// ������ʾ
void bat_percent_led(uint8_t bat_percent)
{
    if (bat_percent <= 15) {
        bat_end_led = 0;
        bat_r = 0x80, bat_g = 0, bat_b = 0;  // ��ɫ
    } else if (bat_percent <= 20) {
        bat_end_led = 1;
        bat_r = 0x80, bat_g = 0x40, bat_b = 0;  // ��ɫ
    } else if (bat_percent <= 40) {
        bat_end_led = 2;
        bat_r = 0x80, bat_g = 0x40, bat_b = 0;  // ��ɫ
    } else if (bat_percent <= 60) {
        bat_end_led = 3;
        bat_r = 0x80, bat_g = 0x40, bat_b = 0;  // ��ɫ
    } else if (bat_percent <= 80) {
        bat_end_led = 4;
        bat_r = 0x80, bat_g = 0x40, bat_b = 0;  // ��ɫ
    } else if (bat_percent <= 95) {
        bat_end_led = 5;
        bat_r = 0x80, bat_g = 0x40, bat_b = 0;  // ��ɫ
    } else {
        bat_end_led = 5;
        bat_r = 0, bat_g = 0x80, bat_b = 0;  // ��ɫ
    }

    // ��ʾ��Ч�ƹ�
    uint8_t i;
    for (i = 0; i <= bat_end_led; i++)
        side_rgb_set_color(11 - i, bat_r >> 2, bat_g >> 2, bat_b >> 2);

    // ��ʾϨ��ƹ�
    for (; i < 6; i++)
        side_rgb_set_color(11 - i, 0, 0, 0);
}

// ���״ָ̬ʾ
void bat_led_show(void)
{
    static uint8_t play_point       = 0;  // ����Ч��ָ��
    static uint32_t bat_play_timer  = 0;  // ����Ч���ٶȿ���
    static uint32_t power_on_timer  = 0;  // ������ʱ��ʾʱ��
    static uint8_t old_charge_state = 0;
    static bool bat_show_flag       = true;   // ��ʾ����
    static bool bat_show_breath     = false;  // ��������
    static uint32_t usb_5v_debounce = 0;      // 5V���ȥ��
    static uint8_t bat_temp         = 100;

    // �ϵ���ʾ5�룬ǿ�ƺ���5��
    if (power_on_timer == 0) {
        power_on_timer = timer_read32();
    }
    // ������ʾ5S��ر�
    else if (timer_elapsed32(power_on_timer) > 5000) {
        bat_show_flag   = false;
        bat_show_breath = false;
    }

    // USB_5V��������
    if (old_charge_state != dev_info.rf_charge) {
        if (timer_elapsed32(usb_5v_debounce) > 1000) {
            if (((old_charge_state & 0x01) == 0) && ((dev_info.rf_charge & 0x01) != 0)) {
                bat_show_flag   = true;            // ����ʾ
                bat_show_breath = true;            // �򿪺���
                power_on_timer  = timer_read32();  // ˢ��5S��ʾ
            }
            old_charge_state = dev_info.rf_charge;
        }
    } else {
        usb_5v_debounce = timer_read32();
    }

    // ��ǰΪ���״̬
    if (dev_info.rf_charge == 0x03) {
        bat_show_breath = true;  // ������λ
    }

    // ��ѹǿ������
    if (dev_info.rf_baterry < 15) {
        bat_show_flag  = true;
        power_on_timer = timer_read32();
    }

    // ������פ������ʾ��
    if (f_bat_hold || bat_show_flag) {
        // 5V���������£������̶�100
        if (dev_info.rf_charge & 0x01) {
            dev_info.rf_baterry = 100;
        }

        //------------------------------ ���Ч����ʾ
        // ����У���ɫ����
        if (bat_show_breath) {
            if (timer_elapsed32(bat_play_timer) > 10) {
                bat_play_timer = timer_read32();
                light_point_playing(0, 1, BREATHE_TAB_LEN, &play_point);
            }
            r_temp = 0x80;
            g_temp = 0x40;
            b_temp = 0x00;
            count_rgb_light(breathe_data_tab[play_point]);
            set_right_rgb(r_temp, g_temp, b_temp);
        }
        // ����ģʽ��ع���
        else {
            // �ȶ�״̬�Ÿ��µ���ֵ
            if (old_charge_state == dev_info.rf_charge)
                bat_temp = dev_info.rf_baterry;

            //------------------------------ ��ص�����ʾ
            bat_percent_led(bat_temp);
        }
    }
}
/**================================================================================================================================
 * 		��λָʾ�ƿ���
 ================================================================================================================================*/
// ��λָʾ�ƿ���
void rgb_matrix_update_pwm_buffers(void);
void device_reset_show(void)
{
    writePinHigh(DC_BOOST_PIN);  // ����ѹ
    setPinOutput(DRIVER_SIDE_CS_PIN);
    setPinOutput(DRIVER_LED_CS_PIN);
    writePinLow(DRIVER_SIDE_CS_PIN);
    writePinLow(DRIVER_LED_CS_PIN);
    for (int blink_cnt = 0; blink_cnt < 3; blink_cnt++) {
        rgb_matrix_set_color_all(0x10, 0x10, 0x10);
        set_left_rgb(0x40, 0x40, 0x40);
        set_right_rgb(0x40, 0x40, 0x40);
        rgb_matrix_update_pwm_buffers();
        side_rgb_refresh();
        wait_ms(200);

        rgb_matrix_set_color_all(0x00, 0x00, 0x00);
        set_left_rgb(0x00, 0x00, 0x00);
        set_right_rgb(0x00, 0x00, 0x00);
        rgb_matrix_update_pwm_buffers();
        side_rgb_refresh();
        wait_ms(200);
    }
}

void device_reset_init(void)
{
    side_mode       = 0;  // ��ƻָ�Ĭ������
    side_light      = 3;
    side_speed      = 2;
    side_rgb        = 1;
    side_colour     = 0;
    side_play_point = 0;

    side_play_cnt   = 0;
    side_play_timer = timer_read32();  // store time of last refresh

    f_bat_hold = false;  // ��س��Ч��

    // RGB �ָ���������
    rgb_matrix_enable();                                                                   // �ȿ���
    rgb_matrix_mode(RGB_MATRIX_DEFAULT_MODE);                                              // Ĭ��ģʽ
    rgb_matrix_set_speed(255 - RGB_MATRIX_SPD_STEP * 2);                                   // �ٶ��е�
    rgb_matrix_sethsv(255, 255, RGB_MATRIX_MAXIMUM_BRIGHTNESS - RGB_MATRIX_VAL_STEP * 2);  // ɫ�� ���� ����

    // ����Ĭ��
    user_config.default_brightness_flag = 0xA5;
    user_config.ee_side_mode            = side_mode;
    user_config.ee_side_light           = side_light;
    user_config.ee_side_speed           = side_speed;
    user_config.ee_side_rgb             = side_rgb;
    user_config.ee_side_colour          = side_colour;
    eeconfig_update_user_datablock(&user_config);  // ����
}

/** ================================================================
    ��Ƶ�Ч���
 ================================================================*/
void m_side_led_show(void)
{
    static uint32_t side_refresh_time = 0;

    side_play_cnt += timer_elapsed32(side_play_timer);
    side_play_timer = timer_read32();  // store time of last refresh

    switch (side_mode) {
        case SIDE_WAVE:
            side_wave_mode_show();
            break;  // ����
        case SIDE_MIX:
            side_spectrum_mode_show();
            break;  // ����
        case SIDE_BREATH:
            side_breathe_mode_show();
            break;  // ����
        case SIDE_STATIC:
            side_static_mode_show();
            break;  // ����
        case SIDE_OFF:
            side_off_mode_show();
            break;  // �ر�
    }

    //------------------------  // �Ҳ�RGB : �ں���ĺ������ȼ���
    bat_led_show();       // ��س��Ч��
    sleep_sw_led_show();  // �Զ�˯�߿���ָʾ�ƿ���
    sys_sw_led_show();    // WIN/MACָʾ�ƿ���

    //------------------------  // ���RGB
    sys_led_show();  // ϵͳָʾ�ƿ���
    rf_led_show();   // RFָʾ�ƿ���

    //------------------------  // ˢ�²��RGB
    if (timer_elapsed32(side_refresh_time) > 10) {
        side_refresh_time = timer_read32();  // store time of last refresh
        side_rgb_refresh();
    }
}
