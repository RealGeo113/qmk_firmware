// Copyright 2023 Persama (@Persama)
// SPDX-License-Identifier: GPL-2.0-or-later
#include "ansi_tkl.h"
#include "uart.h"  // qmk uart.h

#define UART_HEAD 0x5A  // ֡ͷ
//------------------------------------------------
// ��������
uint8_t func_tab[32] = {0};  // ���ܱ�������

extern DEV_INFO_STRUCT dev_info;
USART_MGR_STRUCT Usart_Mgr;

uint8_t disconnect_delay = 0;  // ������ʱ��

uint8_t bitkb_report_buf[32];  // bit���̱���
uint8_t bytekb_report_buf[8];  // byte���̱���

uint16_t conkb_report;     // consume����
uint8_t syskb_report_buf;  // sys����

extern uint8_t host_mode;

extern uint8_t rf_blink_cnt;        // RFָʾ����˸����
extern uint16_t rf_link_show_time;  // RF�л���ʾʱ��
extern uint16_t rf_linking_time;
extern uint16_t no_act_time;

extern bool f_uart_ack;  // ����ͨѶӦ��

extern bool f_rf_read_data_ok;  // RF����ͬ���ɹ�
extern bool f_rf_sts_sysc_ok;   // RF����ͬ���ɹ�
extern bool f_rf_new_adv_ok;    // RF�����¹㲥�ɹ�
extern bool f_rf_reset;         // RF��λ��־
extern bool f_send_channel;     // RF�л�ͨ��
extern bool f_rf_hand_ok;       // RFͬ����־

extern bool f_rf_send_bitkb;    // RF bit���̷��ͱ�־
extern bool f_rf_send_byte;     // RF byte���̷��ͱ�־
extern bool f_rf_send_consume;  // RF consume���淢�ͱ�־

extern bool f_dial_sw_init_ok;  // ���뿪�س�ʼ����ɱ�־

extern bool f_goto_sleep;  // �������߱�־

void uart_send_report(uint8_t report_type, uint8_t *report_buf, uint8_t report_size);
void UART_Send_Bytes(uint8_t *Buffer, uint32_t Length);
uint8_t get_checksum(uint8_t *buf, uint8_t len);

/**================================================================
 *		RX���ղ�����λ
================================================================*/
static void UsartMgr_RXD_Reset(void)
{
    // ��λRX״̬
    Usart_Mgr.RXDLen      = 0;        // ��������
    Usart_Mgr.RXDState    = RX_Idle;  // ����״̬
    Usart_Mgr.RXDOverTime = 0;        // ��ʱ������
}

#define RX_SBYTE Usart_Mgr.RXDBuf[0]  // �������ֽ�
#define RX_CMD   Usart_Mgr.RXDBuf[1]  // ����CMD�ֽ�
#define RX_ACK   Usart_Mgr.RXDBuf[2]  // ����ACK�ֽ�
#define RX_LEN   Usart_Mgr.RXDBuf[3]  // ����LEN�ֽ�
#define RX_DAT   Usart_Mgr.RXDBuf[4]  // ����������ʼ�ֽ�

uint8_t sync_lost = 0;

/**================================================================
 * 		��ѯ���ã����ڽ������ݽ���
 *      ���մ���Ӧ������
================================================================*/
void RF_Protocol_Receive(void)
{
    uint8_t i, check_sum = 0;

    // ������ϣ�����Э�鴦��
    if (Usart_Mgr.RXDState == RX_Done) {
        f_uart_ack = 1;  // �ӻ�Ӧ��

        // У����ж�
        if (Usart_Mgr.RXDLen > 4) {
            for (i = 0; i < RX_LEN; i++)
                check_sum += Usart_Mgr.RXDBuf[4 + i];

            if (check_sum != Usart_Mgr.RXDBuf[4 + i]) {
                Usart_Mgr.RXDState = RX_SUM_ERR;  // У��ʹ���
                return;
            }
        } else if (Usart_Mgr.RXDLen == 3) {
            if (Usart_Mgr.RXDBuf[2] == 0xA0)  // �ӻ�Ӧ��
            {
                f_uart_ack = 1;
            }
        }

        sync_lost = 0;

        // ����Э��
        switch (RX_CMD) {
            //------------------------
            case CMD_POWER_UP:
                break;  // RF�ϵ����
            case CMD_SLEEP:
                break;  // RF��������

                //------------------------
                // RF����Ӧ��
            case CMD_HAND: {
                f_rf_hand_ok = 1;
                break;  // ����/����
            }

                //------------------------
                // 2.4G��������
            case CMD_24G_SUSPEND: {
                f_goto_sleep = 1;
                break;
            }

            //------------------------
            // ����Ӧ��
            case CMD_SET_LINK: {
                break;
            }

            //------------------------
            // ���㲥Ӧ�� ack
            case CMD_NEW_ADV: {
                f_rf_new_adv_ok = 1;
                break;
            }

            // RF״̬ͬ������
            case CMD_RF_STS_SYSC: {
                uint8_t static error_cnt = 0;
                if (dev_info.link_mode == Usart_Mgr.RXDBuf[4])  // RFͨ��
                {
                    error_cnt         = 0;
                    dev_info.rf_state = Usart_Mgr.RXDBuf[5];  // RF����״̬

                    if (dev_info.rf_state == RF_CONNECT) {
                        dev_info.rf_led = Usart_Mgr.RXDBuf[6];  // ϵͳָʾ��(����״̬�²Ÿ���)
                    }

                    // �õ������Ϣ
                    dev_info.rf_charge  = Usart_Mgr.RXDBuf[7];                 // ������
                    dev_info.rf_baterry = Usart_Mgr.RXDBuf[8];                 // ��ص���
                    if (dev_info.rf_charge & 0x01) dev_info.rf_baterry = 100;  // ��⵽5V�������̶�Ϊ100

                } else {
                    if (dev_info.rf_state != RF_INVAILD) {
                        // ͨ����ͬ��,��ʱ�ٴη���
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

            // �������������Ϣ���ӻ�Ӧ��
            case CMD_CLR_DEVICE: {
                // f_rf_reset = 1;     // ��λ
                // f_clr_rf_info = 1;
                break;
            }

            // ��ȡRF�豸��
            case CMD_GET_NAME:
                break;

            // ����RF�豸��
            case CMD_SET_NAME:
                break;

            //------------------------
            // ��׼����
            case CMD_RPT_BYTE_KB: {
                f_rf_send_byte = 0;  // �յ�Ӧ��,������ͱ�־
                break;
            }

            // bit����
            case CMD_RPT_BIT_KB: {
                f_rf_send_bitkb = 0;  // �յ�Ӧ��,������ͱ�־
                break;
            }

            // ������
            case CMD_RPT_CONSUME: {
                f_rf_send_consume = 0;  // �յ�Ӧ��,������ͱ�־
                break;
            }

            // ϵͳ��
            case CMD_RPT_SYS:
                break;

            //------------------------
            // д����
            case CMD_WRITE_DATA:
                break;

            // ������
            case CMD_READ_DATA: {
                memcpy(func_tab, &Usart_Mgr.RXDBuf[4], 32);

                // ��ǰͨ��
                if (func_tab[4] <= LINK_USB) {
                    dev_info.link_mode = func_tab[4];
                }

                // RFͨ��
                if (func_tab[5] < LINK_USB) {
                    dev_info.rf_channel = func_tab[5];
                }

                // BLEͨ��
                if ((func_tab[6] <= LINK_BT_3) && (func_tab[6] >= LINK_BT_1)) {
                    dev_info.ble_channel = func_tab[6];
                }

                f_rf_read_data_ok = 1;
                break;
            }

            //------------------------
            // �Ƿ�����
            default:
                Usart_Mgr.RXDState = RX_CMD_ERR;  // У��ʹ���
                return;
        }

        UsartMgr_RXD_Reset();
    }
}

/**================================================================
    Ӧ�ò㣺���ͱ��浽RFģ��

��⵽�仯ʱ��������
��������ѯ����
 ================================================================*/
uint16_t host_last_consumer_usage(void);
void uart_send_report_func(void)
{
    static uint32_t interval_timer = 0;

    if (f_dial_sw_init_ok == 0) return;           // ���뿪��δ�ȶ�
    if (dev_info.link_mode == LINK_USB) return;   // USBģʽ
    if (dev_info.rf_state != RF_CONNECT) return;  // δ����״̬

    keyboard_protocol = 1;

    // NKEY���̵õ�mods״̬
    if (keymap_config.nkro == 1) {
        bitkb_report_buf[0] = get_mods();
    }

    // ���byte���̱仯
    if ((keymap_config.nkro == 0) && (memcmp(bytekb_report_buf, keyboard_report->raw, 8))) {
        no_act_time             = 0;
        keyboard_report->raw[1] = 0;  // Ԥ���ֽ�ǿ��Ϊ0
        memcpy(bytekb_report_buf, keyboard_report->raw, 8);
        uart_send_report(CMD_RPT_BYTE_KB, bytekb_report_buf, 8);
    }
    // ���bit���̱仯
    else if ((keymap_config.nkro == 1) && (memcmp(bitkb_report_buf, &keyboard_report->nkro.mods, 16))) {
        no_act_time = 0;
        memcpy(&bitkb_report_buf[1], keyboard_report->nkro.bits, 15);
        uart_send_report(CMD_RPT_BIT_KB, bitkb_report_buf, 16);
    }
    // ����ý��仯
    else if (host_last_consumer_usage() != conkb_report) {
        conkb_report = host_last_consumer_usage();
        no_act_time  = 0;
        uart_send_report(CMD_RPT_CONSUME, (uint8_t *)(&conkb_report), 2);
    }
    //  ���ڷ��ͼ��̱����Ա�֤�����ȶ���
    else if (timer_elapsed32(interval_timer) > 100) {
        interval_timer = timer_read32();  // store time of last refresh

        if (keymap_config.nkro)
            uart_send_report(CMD_RPT_BIT_KB, bitkb_report_buf, 16);
        else
            uart_send_report(CMD_RPT_BYTE_KB, bytekb_report_buf, 8);
    }
}

/**================================================================
 * @brief           �������RFģ��
 * @param cmd       ����
 * @param delayms   ����ǰ��ʱ
 ================================================================*/
uint8_t uart_send_cmd(uint8_t cmd, uint8_t wait_ack, uint8_t delayms)
{
    uint8_t i;

    // ��ʱ
    wait_ms(delayms);

    // ������ͻ���
    memset(&Usart_Mgr.TXDBuf[0], 0, UART_MAX_LEN);

    // ׼����������
    Usart_Mgr.TXDBuf[0] = UART_HEAD;  // FLAG
    Usart_Mgr.TXDBuf[1] = cmd;        // CMD
    Usart_Mgr.TXDBuf[2] = 0x00;       // ACK

    switch (cmd) {
        // �ϵ�����
        case CMD_POWER_UP: {
            Usart_Mgr.TXDBuf[3] = 1;  // len
            Usart_Mgr.TXDBuf[4] = 0;  // dat
            Usart_Mgr.TXDBuf[5] = 0;  // sum
            break;
        }
        // Idle���һ������
        case CMD_SNIF: {
            Usart_Mgr.TXDBuf[3] = 1;  // len
            Usart_Mgr.TXDBuf[4] = 0;  // dat
            Usart_Mgr.TXDBuf[5] = 0;  // sum
            break;
        }
        // �����źţ���������
        case CMD_SLEEP: {
            Usart_Mgr.TXDBuf[3] = 1;  // len
            Usart_Mgr.TXDBuf[4] = 0;  // dat
            Usart_Mgr.TXDBuf[5] = 0;  // sum
            break;
        }
        // �����ź�
        case CMD_HAND: {
            Usart_Mgr.TXDBuf[3] = 1;  // len
            Usart_Mgr.TXDBuf[4] = 0;  // dat
            Usart_Mgr.TXDBuf[5] = 0;  // sum
            break;
        }
        //------------------------------------------------
        // ״̬ͬ������
        case CMD_RF_STS_SYSC: {
            Usart_Mgr.TXDBuf[3] = 1;                   // len
            Usart_Mgr.TXDBuf[4] = dev_info.link_mode;  // dat
            Usart_Mgr.TXDBuf[5] = dev_info.link_mode;  // sum
            break;
        }
        //------------------------------------------------
        // ��������ͨ����USB/2.4G/BT1/BT2/BT3
        case CMD_SET_LINK: {
            dev_info.rf_state   = RF_LINKING;          // ״̬Ϊ�㲥���ģʽ
            Usart_Mgr.TXDBuf[3] = 1;                   // len
            Usart_Mgr.TXDBuf[4] = dev_info.link_mode;  // dat
            Usart_Mgr.TXDBuf[5] = dev_info.link_mode;  // sum

            rf_linking_time  = 0;     // ���¿�ʼ��ʱ
            disconnect_delay = 0xff;  // ֱ����˸
            break;
        }
        //------------------------------------------------
        // ǿ�ƹ㲥/���
        case CMD_NEW_ADV: {
            dev_info.rf_state   = RF_PAIRING;              // ״̬Ϊ�㲥���ģʽ
            Usart_Mgr.TXDBuf[3] = 2;                       // len
            Usart_Mgr.TXDBuf[4] = dev_info.link_mode;      // dat ͨ��
            Usart_Mgr.TXDBuf[5] = 1;                       // ����Э��汾
            Usart_Mgr.TXDBuf[6] = dev_info.link_mode + 1;  // sum

            rf_linking_time  = 0;     // ���¿�ʼ��ʱ
            disconnect_delay = 0xff;  // ֱ����˸

            f_rf_new_adv_ok = 0;
            break;
        }
        //------------------------------------------------
        // ������߰���Ϣ�豸
        case CMD_CLR_DEVICE: {
            Usart_Mgr.TXDBuf[3] = 1;  // len
            Usart_Mgr.TXDBuf[4] = 0;  // dat
            Usart_Mgr.TXDBuf[5] = 0;  // sum
            break;
        }
        //------------------------------------------------
        // �������߲��� �� �����������ʱ��
        case CMD_SET_CONFIG: {
            Usart_Mgr.TXDBuf[3] = 1;                 // len
            Usart_Mgr.TXDBuf[4] = POWER_DOWN_DELAY;  // dat
            Usart_Mgr.TXDBuf[5] = POWER_DOWN_DELAY;  // sum
            break;
        }
        //------------------------------------------------
        // ���������豸��
        case CMD_SET_NAME: {
            Usart_Mgr.TXDBuf[3]  = 17;                                                       // data len
            Usart_Mgr.TXDBuf[4]  = 1;                                                        // type     0-��β׺    1-��β׺
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
        // ��ȡRFͨ������
        case CMD_READ_DATA: {
            Usart_Mgr.TXDBuf[3] = 2;               // ��Ч���ݳ���
            Usart_Mgr.TXDBuf[4] = 0x00;            // offsetƫ��
            Usart_Mgr.TXDBuf[5] = FUNC_VALID_LEN;  // ��ȡ���ݳ���
            Usart_Mgr.TXDBuf[6] = FUNC_VALID_LEN;  // sum
            break;
        }
        //------------------------------------------------
        // д���ݵ�RF
        case CMD_WRITE_DATA: {
            func_tab[4] = dev_info.link_mode;
            func_tab[5] = dev_info.rf_channel;
            func_tab[6] = dev_info.ble_channel;

            Usart_Mgr.TXDBuf[3] = FUNC_VALID_LEN + 2;
            Usart_Mgr.TXDBuf[4] = 0;               // RF������ʼ��ַ
            Usart_Mgr.TXDBuf[5] = FUNC_VALID_LEN;  // RF���ݳ���
                                                   // ��������buf
            for (i = 0; i < FUNC_VALID_LEN; i++) {
                Usart_Mgr.TXDBuf[6 + i] = func_tab[i];
            }
            // ����sum
            Usart_Mgr.TXDBuf[6 + i] = get_checksum(func_tab, FUNC_VALID_LEN);
            Usart_Mgr.TXDBuf[6 + i] += 0;
            Usart_Mgr.TXDBuf[6 + i] += FUNC_VALID_LEN;
            break;
        }
        //------------------------------------------------
        // ����DFU��������
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

    // �ȴ�Ӧ��
    if (wait_ack) {
        // �ȴ�Ӧ��
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
 * 		RF�豸ͬ������
 * �������
 * ��������״̬
================================================================*/
void m_break_all_key(void);
extern host_driver_t *m_host_driver;
void dev_sts_sync(void)
{
    static uint32_t interval_timer = 0;  // ϵͳ�л�������ʱ

    // 200msͬ�����
    if (timer_elapsed32(interval_timer) < 200) return;
    interval_timer = timer_read32();  // store time of last refresh

    //------------------------ RFͨ���ط�
    if (f_send_channel) {
        f_send_channel = 0;
        uart_send_cmd(CMD_SET_LINK, 10, 10);
    }

    //------------------------ RFģ�鸴λ����������
    if (f_rf_reset) {
        f_rf_reset = 0;

        writePinLow(NRF_RESET_PIN);  // ��λRF
        wait_ms(200);
        writePinHigh(NRF_RESET_PIN);
    }

    //------------------------ USBģʽ
    if (dev_info.link_mode == LINK_USB) {
        if (host_mode != HOST_USB_TYPE) {
            host_mode = HOST_USB_TYPE;
            host_set_driver(m_host_driver);
            m_break_all_key();  // �ͷ����а���
        }
        rf_blink_cnt = 0;
    }
    //------------------------ RF����ģʽ
    else {
        if (host_mode != HOST_RF_TYPE) {
            host_mode = HOST_RF_TYPE;
            m_break_all_key();  // �ͷ����а���
            host_set_driver(0);
        }

        // ������״̬
        if (dev_info.rf_state != RF_CONNECT) {
            // ���Ӷ�����ʱ1S�����ż����������˸������
            if (disconnect_delay >= 5) {
                rf_blink_cnt      = 3;  // ������״̬��˸
                rf_link_show_time = 0;  // ��ʼ��ʾ3S
            } else {
                disconnect_delay++;
            }
        }
        // ������״̬
        else if (dev_info.rf_state == RF_CONNECT) {
            rf_linking_time  = 0;
            disconnect_delay = 0;
            rf_blink_cnt     = 0;  // ���ӳɹ�����˸0��
        }
    }

    //------------------------
    // ����״̬ͬ������
    uart_send_cmd(CMD_RF_STS_SYSC, 1, 1);  // 200ms ����һ��״̬ͬ������
    wait_ms(1);

    // �����ʱ��û��ͬ��Ӧ��, ��λRFģ�� / USBģʽ����Ҫ����OTA������������λ����
    if (dev_info.link_mode != LINK_USB) {
        if (++sync_lost >= 5)  // 1S
        {
            sync_lost  = 0;
            f_rf_reset = 1;
        }
    }
}

/**================================================================
    RFģ���ʼ������

1. ��RFģ�����ͨѶ����
2. ��RF�ж�ȡ���ܱ���
 ================================================================*/
void rf_device_init(void)
{
    uint8_t timeout = 0;
    void uart_receive_pro(void);

    // �ȴ�RF����Ӧ��
    timeout      = 10;
    f_rf_hand_ok = 0;
    while (timeout--) {
        uart_send_cmd(CMD_HAND, 0, 20);  // ����
        wait_ms(5);
        uart_receive_pro();  // ��һ�ν���
        uart_receive_pro();  // �ڶ��δ���
        if (f_rf_hand_ok)
            break;
    }

    // ��ȡRFͨ������
    timeout           = 10;
    f_rf_read_data_ok = 0;
    while (timeout--) {
        uart_send_cmd(CMD_READ_DATA, 0, 20);  // ��ȡRF����
        wait_ms(5);
        uart_receive_pro();  // ��һ�ν���
        uart_receive_pro();  // �ڶ��δ���
        if (f_rf_read_data_ok)
            break;
    }

    // ͬ���������
    timeout          = 10;
    f_rf_sts_sysc_ok = 0;
    while (timeout--) {
        uart_send_cmd(CMD_RF_STS_SYSC, 0, 20);  // ��ȡRF����
        wait_ms(5);
        uart_receive_pro();  // ��һ�ν���
        uart_receive_pro();  // �ڶ��δ���
        if (f_rf_sts_sysc_ok)
            break;
    }

    // �ϵ�������������
    uart_send_cmd(CMD_SET_NAME, 10, 20);
}

/**================================================================
    UART
 ================================================================*/

// ��������
void UART_Send_Bytes(uint8_t *Buffer, uint32_t Length)
{
    writePinLow(NRF_WAKEUP_PIN);
    wait_us(50);

    uart_transmit(Buffer, Length);

    wait_us(50 + Length * 30);
    writePinHigh(NRF_WAKEUP_PIN);
}

// ��������checksum
uint8_t get_checksum(uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t checksum = 0;

    for (i = 0; i < len; i++)
        checksum += *buf++;

    checksum ^= UART_HEAD;

    return checksum;
}

// ���ڷ��ͱ���
void uart_send_report(uint8_t report_type, uint8_t *report_buf, uint8_t report_size)
{
    Usart_Mgr.TXDBuf[0] = UART_HEAD;    // ֡��ʼ
    Usart_Mgr.TXDBuf[1] = report_type;  // �����ֽ�
    Usart_Mgr.TXDBuf[2] = 0x01;         // Ӧ��Ҫ���ֽ�
    Usart_Mgr.TXDBuf[3] = report_size;  // �����ֶγ���/������sum

    memcpy(&Usart_Mgr.TXDBuf[4], report_buf, report_size);
    Usart_Mgr.TXDBuf[4 + report_size] = get_checksum(&Usart_Mgr.TXDBuf[4], report_size);

    // �����ܳ��ȣ�4 + dat_len + 1
    UART_Send_Bytes(&Usart_Mgr.TXDBuf[0], report_size + 5);

    // ��ʱ����֡���
    wait_us(200);
}

/** ================================================================
 *  ���ڽ��մ�����
  ================================================================ */
void uart_receive_pro(void)
{
    static bool rcv_start = false;

    // ��ѯ�Ƿ����µ�����
    if (uart_available()) {
        rcv_start = true;  // ��ʼ����
        while (uart_available()) {
            if (Usart_Mgr.RXDLen >= UART_MAX_LEN) {
                uart_read();
                Usart_Mgr.RXDState = RX_DATA_OV;
            } else {
                Usart_Mgr.RXDBuf[Usart_Mgr.RXDLen++] = uart_read();
            }
        }

    } else if (rcv_start) {
        // ֡������һ����ѯ����δ�յ�������
        rcv_start          = false;
        Usart_Mgr.RXDState = RX_Done;
        Usart_Mgr.RXDLen   = 0;
        RF_Protocol_Receive();
    }
}
