// Copyright 2023 Persama (@Persama)
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#define DYNAMIC_KEYMAP_LAYER_COUNT 8 /* 按键矩阵层数（超过4层需要在这里定义） This is 4 by default. */

// This is the size of the EEPROM for the custom VIA-specific data
#define EECONFIG_USER_DATA_SIZE    8  // 定义保存至EEPROM的用户数据

#define NO_USB_STARTUP_CHECK  // 不检查USB连接

#define DEBOUNCE                 2  // Default debounce time is 5 milliseconds

#define DEV_MODE_PIN             C0  // 模式拨码开关
#define SYS_MODE_PIN             C1  // MAC/WIN拨码开关
#define DC_BOOST_PIN             C2  // DC升压
#define NRF_RESET_PIN            B4  // NRF模块复位
#define NRF_WAKEUP_PIN           B8  // NRF帧同步
#define DRIVER_LED_CS_PIN        C6  // RGB CS

#define DRIVER_SIDE_PIN          C8  // 侧灯RGB驱动引脚
#define DRIVER_SIDE_CS_PIN       C9  // 侧灯RGB电源开关

#define SERIAL_DRIVER            SD1
#define SD1_TX_PIN               B6
#define SD1_TX_PAL_MODE          0
#define SD1_RX_PIN               B7
#define SD1_RX_PAL_MODE          0

#define RGB_MATRIX_LED_COUNT     84  // RGB总灯数 (84轴灯+12侧灯)

#define WS2812_PWM_DRIVER        PWMD3  // default: PWMD2
#define WS2812_PWM_CHANNEL       2      // default: 2
#define WS2812_PWM_PAL_MODE      1      // Pin "alternate function", see the respective datasheet for the appropriate values for your MCU. default: 2
// #define WS2812_PWM_COMPLEMENTARY_OUTPUT        // Define for a complementary timer output (TIMx_CHyN); omit for a normal timer output (TIMx_CHy).
#define WS2812_DMA_STREAM        STM32_DMA1_STREAM3  // DMA Stream for TIMx_UP, see the respective reference manual for the appropriate values for your MCU.
#define WS2812_DMA_CHANNEL       3                   // DMA Channel for TIMx_UP, see the respective reference manual for the appropriate values for your MCU.

#define WS2812_PWM_TARGET_PERIOD 200000  // 200kHz PWM frequency

#define RGB_MATRIX_CENTER \
    {                     \
        112, 30           \
    }                                      // 按键中心位置
#define RGB_MATRIX_MAXIMUM_BRIGHTNESS 100  // 最大亮度 (20*5级)
#define RGB_MATRIX_VAL_STEP           20   // 用于亮度等级(最大100)(5级)
#define RGB_MATRIX_SPD_STEP           52   // 用于速度等级(最大256)(5级)

#define RGB_MATRIX_DEFAULT_MODE       RGB_MATRIX_CYCLE_LEFT_RIGHT  // Sets the default mode, if none has been set
#define RGB_DISABLE_WHEN_USB_SUSPENDED                             // turn off effects when suspended

// RGB Matrix Animation modes. Explicitly enabled
// For full list of effects, see:
// https://docs.qmk.fm/#/feature_rgb_matrix?id=rgb-matrix-effects
// #define ENABLE_RGB_MATRIX_ALPHAS_MODS
#define ENABLE_RGB_MATRIX_GRADIENT_UP_DOWN
#define ENABLE_RGB_MATRIX_GRADIENT_LEFT_RIGHT
#define ENABLE_RGB_MATRIX_BREATHING
#define ENABLE_RGB_MATRIX_BAND_SAT
#define ENABLE_RGB_MATRIX_BAND_VAL
#define ENABLE_RGB_MATRIX_BAND_PINWHEEL_SAT
#define ENABLE_RGB_MATRIX_BAND_PINWHEEL_VAL
#define ENABLE_RGB_MATRIX_BAND_SPIRAL_SAT
#define ENABLE_RGB_MATRIX_BAND_SPIRAL_VAL
#define ENABLE_RGB_MATRIX_CYCLE_ALL
#define ENABLE_RGB_MATRIX_CYCLE_LEFT_RIGHT
#define ENABLE_RGB_MATRIX_CYCLE_UP_DOWN
#define ENABLE_RGB_MATRIX_RAINBOW_MOVING_CHEVRON  // Enables RGB_MATRIX_RAINBOW_MOVING_CHEVRON
#define ENABLE_RGB_MATRIX_CYCLE_OUT_IN            // Enables RGB_MATRIX_CYCLE_OUT_IN
#define ENABLE_RGB_MATRIX_CYCLE_OUT_IN_DUAL       // Enables RGB_MATRIX_CYCLE_OUT_IN_DUAL
#define ENABLE_RGB_MATRIX_CYCLE_PINWHEEL          // Enables RGB_MATRIX_CYCLE_PINWHEEL
#define ENABLE_RGB_MATRIX_CYCLE_SPIRAL            // Enables RGB_MATRIX_CYCLE_SPIRAL
#define ENABLE_RGB_MATRIX_DUAL_BEACON             // Enables RGB_MATRIX_DUAL_BEACON
#define ENABLE_RGB_MATRIX_RAINBOW_BEACON          // Enables RGB_MATRIX_RAINBOW_BEACON
#define ENABLE_RGB_MATRIX_RAINBOW_PINWHEELS       // Enables RGB_MATRIX_RAINBOW_PINWHEELS
#define ENABLE_RGB_MATRIX_RAINDROPS               // Enables RGB_MATRIX_RAINDROPS
#define ENABLE_RGB_MATRIX_JELLYBEAN_RAINDROPS     // Enables RGB_MATRIX_JELLYBEAN_RAINDROPS
#define ENABLE_RGB_MATRIX_HUE_BREATHING           // Enables RGB_MATRIX_HUE_BREATHING
#define ENABLE_RGB_MATRIX_HUE_PENDULUM            // Enables RGB_MATRIX_HUE_PENDULUM
#define ENABLE_RGB_MATRIX_HUE_WAVE                // Enables RGB_MATRIX_HUE_WAVE
// #define ENABLE_RGB_MATRIX_PIXEL_FRACTAL           // Enables RGB_MATRIX_PIXEL_FRACTAL
// #define ENABLE_RGB_MATRIX_PIXEL_FLOW              // Enables RGB_MATRIX_PIXEL_FLOW
// #define ENABLE_RGB_MATRIX_PIXEL_RAIN              // Enables RGB_MATRIX_PIXEL_RAIN

#define RGB_MATRIX_FRAMEBUFFER_EFFECTS
#define ENABLE_RGB_MATRIX_TYPING_HEATMAP  // How hot is your WPM!
#define ENABLE_RGB_MATRIX_DIGITAL_RAIN    // That famous computer simulation

#define RGB_MATRIX_KEYPRESSES
#define RGB_MATRIX_KEYRELEASES
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_SIMPLE      // Pulses keys hit to hue & value then fades value out
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE             // Static single hue, pulses keys hit to shifted hue then fades to current hue
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_WIDE        // Hue & value pulse near a single key hit then fades value out
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTIWIDE   // Hue & value pulse near multiple key hits then fades value out
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_CROSS       // Hue & value pulse the same column and row of a single key hit then fades value out
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTICROSS  // Hue & value pulse the same column and row of multiple key hits then fades value out
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_NEXUS       // Hue & value pulse away on the same column and row of a single key hit then fades value out
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTINEXUS  // Hue & value pulse away on the same column and row of multiple key hits then fades value out
#define ENABLE_RGB_MATRIX_SPLASH                     // Full gradient & value pulse away from a single key hit then fades value out
#define ENABLE_RGB_MATRIX_MULTISPLASH                // Full gradient & value pulse away from multiple key hits then fades value out
#define ENABLE_RGB_MATRIX_SOLID_SPLASH               // Hue & value pulse away from a single key hit then fades value out
#define ENABLE_RGB_MATRIX_SOLID_MULTISPLASH          // Hue & value pulse away from multiple key hits then fades value out

// ==========================================================================
// ==========================================================================

//------------------------
// key matrix pins
#define KROW_0        C14
#define KROW_1        C15
#define KROW_2        A0
#define KROW_3        A1
#define KROW_4        A2
#define KROW_5        A3

#define EXTI_PORT_R0  EXTI_PortSourceGPIOC
#define EXTI_PORT_R1  EXTI_PortSourceGPIOC
#define EXTI_PORT_R2  EXTI_PortSourceGPIOA
#define EXTI_PORT_R3  EXTI_PortSourceGPIOA
#define EXTI_PORT_R4  EXTI_PortSourceGPIOA
#define EXTI_PORT_R5  EXTI_PortSourceGPIOA

#define EXTI_PIN_R0   14  // C14
#define EXTI_PIN_R1   15  // C15
#define EXTI_PIN_R2   0   // A0
#define EXTI_PIN_R3   1   // A1
#define EXTI_PIN_R4   2   // A2
#define EXTI_PIN_R5   3   // A3

//------------------------
#define KCOL_0        A4
#define KCOL_1        A5
#define KCOL_2        A6
#define KCOL_3        B9
#define KCOL_4        B0
#define KCOL_5        B1
#define KCOL_6        B10
#define KCOL_7        B11
#define KCOL_8        B12
#define KCOL_9        B13
#define KCOL_10       B14
#define KCOL_11       B15
#define KCOL_12       A8
#define KCOL_13       A9
#define KCOL_14       A10
#define KCOL_15       A15
#define KCOL_16       B3

#define EXTI_PORT_C0  EXTI_PortSourceGPIOA  // A4
#define EXTI_PORT_C1  EXTI_PortSourceGPIOA  // A5
#define EXTI_PORT_C2  EXTI_PortSourceGPIOA  // A6
#define EXTI_PORT_C3  EXTI_PortSourceGPIOB  // B9
#define EXTI_PORT_C4  EXTI_PortSourceGPIOB  // B0
#define EXTI_PORT_C5  EXTI_PortSourceGPIOB  // B1
#define EXTI_PORT_C6  EXTI_PortSourceGPIOB  // B10
#define EXTI_PORT_C7  EXTI_PortSourceGPIOB  // B11
#define EXTI_PORT_C8  EXTI_PortSourceGPIOB  // B12
#define EXTI_PORT_C9  EXTI_PortSourceGPIOB  // B13
#define EXTI_PORT_C10 EXTI_PortSourceGPIOB  // B14
#define EXTI_PORT_C11 EXTI_PortSourceGPIOB  // B15
#define EXTI_PORT_C12 EXTI_PortSourceGPIOA  // A8
#define EXTI_PORT_C13 EXTI_PortSourceGPIOA  // A9
#define EXTI_PORT_C14 EXTI_PortSourceGPIOA  // A10
#define EXTI_PORT_C15 EXTI_PortSourceGPIOA  // A15
#define EXTI_PORT_C16 EXTI_PortSourceGPIOB  // B3

#define EXTI_PIN_C0   4   // A4
#define EXTI_PIN_C1   5   // A5
#define EXTI_PIN_C2   6   // A6
#define EXTI_PIN_C3   9   // B9
#define EXTI_PIN_C4   0   // B0
#define EXTI_PIN_C5   1   // B1
#define EXTI_PIN_C6   10  // B10
#define EXTI_PIN_C7   11  // B11
#define EXTI_PIN_C8   12  // B12
#define EXTI_PIN_C9   13  // B13
#define EXTI_PIN_C10  14  // B14
#define EXTI_PIN_C11  15  // B15
#define EXTI_PIN_C12  8   // A8
#define EXTI_PIN_C13  9   // A9
#define EXTI_PIN_C14  10  // A10
#define EXTI_PIN_C15  15  // A15
#define EXTI_PIN_C16  3   // B3
