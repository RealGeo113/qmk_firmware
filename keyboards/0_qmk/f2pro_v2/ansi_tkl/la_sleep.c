// Copyright 2023 Persama (@Persama)
// SPDX-License-Identifier: GPL-2.0-or-later
#include "ansi_tkl.h"
#include "la_stm32f0xx.h"

//------------------------------------------------
// �ⲿ����

extern DEV_INFO_STRUCT dev_info;

extern bool f_uart_ack;          // ����ͨѶӦ��
extern bool f_dev_sleep_enable;  // �豸����ʹ��

extern bool f_usb_offline;  // USB�ֶ����߱�־

extern bool f_goto_sleep;  // �������߱�־
extern uint16_t rf_linking_time;

extern uint16_t no_act_time;

void exit_sleep(void);
void goto_sleep(void);

uint8_t uart_send_cmd(uint8_t cmd, uint8_t ack_cnt, uint8_t delayms);
bool is_side_rgb_off(void);

/** ================================================================
 * @brief   UART_GPIO ��ת�������õ���+����
 ================================================================*/
void m_uart_gpio_set_low_speed_and_pullup(void)
{
    ((GPIO_TypeDef *)GPIOB)->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7);
    ((GPIO_TypeDef *)GPIOB)->PUPDR |= (GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0);
}

/** ================================================================
 * @brief   �ر�USB
 *
 ================================================================*/
void m_deinit_usb_072(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

#if (0)
    // ����qmk��ر�USB
    void close_usb(void);
    close_usb();
#endif

    // ��λUSB�Ĵ���
    RCC_APB1PeriphResetCmd(RCC_APB1RSTR_USBRST, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1RSTR_USBRST, DISABLE);
    wait_ms(10);

    // �ر�USBʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USBEN, DISABLE);

    // GPIO�ָ�Ϊ����״̬
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init((GPIO_TypeDef *)GPIOA, &GPIO_InitStructure);
}

/** ================================================================
 * @brief   �͹��Ĵ���
 *
 ================================================================*/
#include "hal_usb.h"
#include "usb_main.h"
void idle_enter_sleep(void);
void Sleep_Handle(void)
{
    static uint32_t delay_step_timer = 0;
    static uint8_t usb_suspend_debounce;

    // ɫ����H, Hue����
    // ���Ͷȣ�S,Saturation����
    // ���ȣ�V, Value��
    extern rgb_config_t rgb_matrix_config;
    extern uint8_t host_mode;
    extern uint8_t side_light;  // �������

    // ��̬����
    if (host_mode != HOST_USB_TYPE) {
        // //------------------------ ˯�ߣ����͹���
        if (((rgb_matrix_config.hsv.v == 0) && (side_light == 0)) || (rgb_matrix_config.enable == 0)) {
            idle_enter_sleep();
        }
    }

    // ��ѯ���50ms
    if (timer_elapsed32(delay_step_timer) < 50) return;
    delay_step_timer = timer_read32();  // store time of last refresh

    // ������Ͳ������Ϊ0���ر�RGB����
    if (rgb_matrix_config.enable) {
        if ((rgb_matrix_config.hsv.v == 0) && (is_side_rgb_off())) {
            setPinInput(DRIVER_LED_CS_PIN);
            setPinInput(DRIVER_SIDE_CS_PIN);
            writePinLow(DC_BOOST_PIN);  // �ر���ѹ
        } else {
            writePinHigh(DC_BOOST_PIN);  // ����ѹ
            if (rgb_matrix_config.hsv.v) {
                setPinOutput(DRIVER_LED_CS_PIN);
                writePinLow(DRIVER_LED_CS_PIN);
            } else {
                setPinInput(DRIVER_LED_CS_PIN);
            }
            if (is_side_rgb_off()) {
                setPinInput(DRIVER_SIDE_CS_PIN);
            } else {
                setPinOutput(DRIVER_SIDE_CS_PIN);
                writePinLow(DRIVER_SIDE_CS_PIN);
            }
        }
    } else {
        //------------------------ �ر�RGB LED
        setPinInput(DRIVER_LED_CS_PIN);
        setPinInput(DRIVER_SIDE_CS_PIN);
        writePinLow(DC_BOOST_PIN);  // �ر���ѹ
    }

    // ����USBģʽ���յ�������suspend�ź�
    if ((USB_DRIVER.state == USB_SUSPENDED) && (host_mode == HOST_USB_TYPE))  //   USBģʽ�Ŵ�������
    {
        // USB Suspend �ź�1Sȥ��
        usb_suspend_debounce++;
        if (usb_suspend_debounce >= 20) {
            goto_sleep();
            exit_sleep();
            return;
        }
    } else {
        usb_suspend_debounce = 0;
    }

    static uint32_t rf_disconnect_time = 0;
    // ����ģʽ���ҿ���dev_sleep
    if (f_dev_sleep_enable && (host_mode != HOST_USB_TYPE)) {
        // �㲥 & ������ʱ
        if ((dev_info.rf_state == RF_LINKING) || (dev_info.rf_state == RF_PAIRING)) {
            if (rf_linking_time >= LINK_TIMEOUT)  // ���ӳ�ʱ
            {
                rf_linking_time = 0;

                goto_sleep();
                exit_sleep();
            }
        }
        // �����ް�����ʱ
        else if (dev_info.rf_state == RF_CONNECT) {
            // �ް�����ʱ
            if (no_act_time >= SLEEP_TIME_DELAY)  // ��ʱû����
            {
                goto_sleep();
                exit_sleep();
            }
            rf_disconnect_time = 0;
        }
        // �Ͽ�����
        else if (dev_info.rf_state == RF_DISCONNECT) {
            rf_disconnect_time++;
            if (rf_disconnect_time > 10 * 1000 / 50) {
                goto_sleep();
                exit_sleep();
                rf_disconnect_time = 0;
            }
        }
        // ���߱�־λ
        if (f_goto_sleep) {
            goto_sleep();
            exit_sleep();

            f_goto_sleep = 0;
        }
    }
}

void SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex)
{
    uint32_t tmp = 0x00;

    tmp = ((uint32_t)0x0F) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03));
    SYSCFG->EXTICR[EXTI_PinSourcex >> 0x02] &= ~tmp;
    SYSCFG->EXTICR[EXTI_PinSourcex >> 0x02] |= (((uint32_t)EXTI_PortSourceGPIOx) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03)));
}

#include "hal_lld.h"

#define EXTI_PortSourceGPIOA ((uint8_t)0x00)
#define EXTI_PortSourceGPIOB ((uint8_t)0x01)
#define EXTI_PortSourceGPIOC ((uint8_t)0x02)
#define EXTI_PortSourceGPIOD ((uint8_t)0x03)

/** ========================================================
 * @brief   �豸�˳�����
 *
 ================================================================*/
#include "usb_main.h"
void exit_sleep(void)
{
    // �����ʼ��
    extern void matrix_init_pins(void);
    matrix_init_pins();

    m_uart_gpio_set_low_speed_and_pullup();

    // �ָ�IO����״̬
    setPinInputHigh(DEV_MODE_PIN);  // PC0
    setPinInputHigh(SYS_MODE_PIN);  // PC1

    // setPinInputHigh(ZA_PIN);
    // setPinInputHigh(ZB_PIN);

    setPinOutput(NRF_WAKEUP_PIN);

    // ʹ��DC��ѹ
    setPinOutput(DC_BOOST_PIN);
    writePinHigh(DC_BOOST_PIN);

    // ���³�ʼ��ϵͳʱ��
    stm32_clock_init();

    /* TIM6 ʹ�� */
    TIM_Cmd(TIM6, ENABLE);

    // ���������ʱ
    no_act_time = 0;

    // ����һ�����ֻ���RF
    uart_send_cmd(CMD_HAND, 0, 1);  // ����

    if (dev_info.link_mode == LINK_USB) {
        // USBԶ�̻���
        usbWakeupHost(&USB_DRIVER);
        restart_usb_driver(&USB_DRIVER);
        send_keyboard_report();
    }
}

/** ================================================================
 * @brief   �豸��������
 *
 ================================================================*/
void goto_sleep(void)
{
    //------------------------ ����RF����״̬
    if (dev_info.rf_state == RF_CONNECT)
        uart_send_cmd(CMD_SET_CONFIG, 5, 5);  // ����״̬�����������ʱ��
    else
        uart_send_cmd(CMD_SLEEP, 5, 5);  // ������״ֱ̬�ӽ����������

    //------------------------ ��USBģʽ�¹ر�USB
    if (dev_info.link_mode != LINK_USB) {
        // �ر�USB
        f_usb_offline = 1;

        m_deinit_usb_072();
    }

    // �رն�ʱ��
    TIM_Cmd(TIM6, DISABLE);

    // //------------------------ ���ð�������
    setPinOutput(KCOL_0);
    writePinHigh(KCOL_0);
    setPinOutput(KCOL_1);
    writePinHigh(KCOL_1);
    setPinOutput(KCOL_2);
    writePinHigh(KCOL_2);
    setPinOutput(KCOL_3);
    writePinHigh(KCOL_3);
    setPinOutput(KCOL_4);
    writePinHigh(KCOL_4);
    setPinOutput(KCOL_5);
    writePinHigh(KCOL_5);
    setPinOutput(KCOL_6);
    writePinHigh(KCOL_6);
    setPinOutput(KCOL_7);
    writePinHigh(KCOL_7);
    setPinOutput(KCOL_8);
    writePinHigh(KCOL_8);
    setPinOutput(KCOL_9);
    writePinHigh(KCOL_9);
    setPinOutput(KCOL_10);
    writePinHigh(KCOL_10);
    setPinOutput(KCOL_11);
    writePinHigh(KCOL_11);
    setPinOutput(KCOL_12);
    writePinHigh(KCOL_12);
    setPinOutput(KCOL_13);
    writePinHigh(KCOL_13);
    setPinOutput(KCOL_14);
    writePinHigh(KCOL_14);
    setPinOutput(KCOL_15);
    writePinHigh(KCOL_15);
    setPinOutput(KCOL_16);
    writePinHigh(KCOL_16);

    setPinInputLow(KROW_0);
    setPinInputLow(KROW_1);
    setPinInputLow(KROW_2);
    setPinInputLow(KROW_3);
    setPinInputLow(KROW_4);
    setPinInputLow(KROW_5);

    // KCOL�����ⲿ�ж�
    SYSCFG_EXTILineConfig(EXTI_PORT_R0, EXTI_PIN_R0);
    SYSCFG_EXTILineConfig(EXTI_PORT_R1, EXTI_PIN_R1);
    SYSCFG_EXTILineConfig(EXTI_PORT_R2, EXTI_PIN_R2);
    SYSCFG_EXTILineConfig(EXTI_PORT_R3, EXTI_PIN_R3);
    SYSCFG_EXTILineConfig(EXTI_PORT_R4, EXTI_PIN_R4);
    SYSCFG_EXTILineConfig(EXTI_PORT_R5, EXTI_PIN_R5);

    // ���ò��뿪�ػ���

    // �ⲿ�ж�����
    EXTI_InitTypeDef m_exti;
    EXTI_StructInit(&m_exti);
    m_exti.EXTI_Line    = 0XFFFF;  // GPIO 0-15
    m_exti.EXTI_LineCmd = ENABLE;
    m_exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    m_exti.EXTI_Trigger = EXTI_Trigger_Falling;  //
    EXTI_Init(&m_exti);

    // NVIC���� GPIO�����жϴ�
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel         = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_3_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    // �ر�DC��ѹ
    setPinOutput(DC_BOOST_PIN);
    writePinLow(DC_BOOST_PIN);

    // �ر�RGB LED
    setPinInput(DRIVER_LED_CS_PIN);
    setPinInput(DRIVER_SIDE_CS_PIN);

    // ������ΧIO, ����©��������
    setPinOutput(DEV_MODE_PIN);
    writePinLow(DEV_MODE_PIN);

    setPinOutput(SYS_MODE_PIN);
    writePinLow(SYS_MODE_PIN);

    setPinOutput(A7);
    writePinLow(A7);
    setPinOutput(DRIVER_SIDE_PIN);
    writePinLow(DRIVER_SIDE_PIN);

    // �������� PB5
    setPinOutput(B5);
    writePinHigh(B5);

    setPinOutput(NRF_WAKEUP_PIN);
    writePinHigh(NRF_WAKEUP_PIN);

    // ����STOPģʽ
    // PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);
    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
    // PWR_EnterSleepMode(PWR_SLEEPEntry_WFI);
}

/**
 * @brief  Clears the EXTI's line pending flags.
 * @param  EXTI_Line: specifies the EXTI lines flags to clear.
 *   This parameter can be any combination of EXTI_Linex where x can be (0..19)
 * @retval None
 */
void EXTI_ClearFlag(uint32_t EXTI_Line)
{
    EXTI->PR = EXTI_Line;
}

///////////////////////////////////////////////////////////////////////////////////////////

#define STM32_EXTI_0_1_HANDLER Vector54
OSAL_IRQ_HANDLER(STM32_EXTI_0_1_HANDLER)
{
    EXTI->PR = 0xffff;
}

#define STM32_EXTI_2_3_HANDLER Vector58
OSAL_IRQ_HANDLER(STM32_EXTI_2_3_HANDLER)
{
    EXTI->PR = 0xffff;
}

#define STM32_EXTI_4_15_HANDLER Vector5C
OSAL_IRQ_HANDLER(STM32_EXTI_4_15_HANDLER)
{
    // if (EXTI->PR & EXTI_Line)
    // {

    // }

    EXTI->PR = 0xffff;
}

void EXTI_DeInit(void)
{
    EXTI->IMR  = 0x0F940000;
    EXTI->EMR  = 0x00000000;
    EXTI->RTSR = 0x00000000;
    EXTI->FTSR = 0x00000000;
    EXTI->PR   = 0x006BFFFF;
}

void EXTI_Init(EXTI_InitTypeDef *EXTI_InitStruct)
{
    uint32_t tmp = 0;

    tmp = (uint32_t)EXTI_BASE;

    if (EXTI_InitStruct->EXTI_LineCmd != DISABLE) {
        /* Clear EXTI line configuration */
        EXTI->IMR &= ~EXTI_InitStruct->EXTI_Line;
        EXTI->EMR &= ~EXTI_InitStruct->EXTI_Line;

        tmp += EXTI_InitStruct->EXTI_Mode;

        *(__IO uint32_t *)tmp |= EXTI_InitStruct->EXTI_Line;

        /* Clear Rising Falling edge configuration */
        EXTI->RTSR &= ~EXTI_InitStruct->EXTI_Line;
        EXTI->FTSR &= ~EXTI_InitStruct->EXTI_Line;

        /* Select the trigger for the selected interrupts */
        if (EXTI_InitStruct->EXTI_Trigger == EXTI_Trigger_Rising_Falling) {
            /* Rising Falling edge */
            EXTI->RTSR |= EXTI_InitStruct->EXTI_Line;
            EXTI->FTSR |= EXTI_InitStruct->EXTI_Line;
        } else {
            tmp = (uint32_t)EXTI_BASE;
            tmp += EXTI_InitStruct->EXTI_Trigger;

            *(__IO uint32_t *)tmp |= EXTI_InitStruct->EXTI_Line;
        }
    } else {
        tmp += EXTI_InitStruct->EXTI_Mode;

        /* Disable the selected external lines */
        *(__IO uint32_t *)tmp &= ~EXTI_InitStruct->EXTI_Line;
    }
}

/**
 * @brief  Fills each EXTI_InitStruct member with its reset value.
 * @param  EXTI_InitStruct: pointer to a EXTI_InitTypeDef structure which will
 *         be initialized.
 * @retval None
 */
void EXTI_StructInit(EXTI_InitTypeDef *EXTI_InitStruct)
{
    EXTI_InitStruct->EXTI_Line    = 0;
    EXTI_InitStruct->EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStruct->EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStruct->EXTI_LineCmd = DISABLE;
}

void uart_init(uint32_t baud);  // qmk uart.c
void m_uart1_init(void)
{
    uart_init(460800);  // RFͨѶ���ڳ�ʼ��
    USART_Cmd(USART1, DISABLE);
    // ʹ��9bit ʹ��żУ��
    USART1->CR1 |= USART_CR1_M0 | USART_CR1_PCE;
    USART_Cmd(USART1, ENABLE);

    m_uart_gpio_set_low_speed_and_pullup();
}

void m_timer6_init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    /* TIM6 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    /*  TIM6 �ж�Ƕ�����*/
    NVIC_InitStructure.NVIC_IRQChannel         = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Time ��ʱ���������� */
    TIM_TimeBaseStructure.TIM_Period        = 1000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler     = 48 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    /* TIM6 ʹ�� */
    TIM_Cmd(TIM6, ENABLE);
}

volatile uint8_t idle_sleep_cnt = 0;
OSAL_IRQ_HANDLER(STM32_TIM6_HANDLER)
{
    if (TIM_GetFlagStatus(TIM6, TIM_FLAG_Update) != ST_RESET) {
        TIM_ClearFlag(TIM6, TIM_FLAG_Update);
        idle_sleep_cnt++;
    }
}
extern uint8_t flag_key_change_cnt;
void idle_enter_sleep(void)
{
    TIM6->CNT      = 0;
    idle_sleep_cnt = 0;
    while (idle_sleep_cnt < 1) {
        PWR_EnterSleepMode(PWR_SLEEPEntry_WFI);
    }
}
