#ifndef _TUAN_S3_XIAOZHI_CONFIG_H_
#define _TUAN_S3_XIAOZHI_CONFIG_H_

#include <driver/gpio.h>
#include <driver/i2c_master.h>

// ============================================================================
// AUDIO: INMP441
// ============================================================================

#define AUDIO_I2S_MIC_GPIO_WS      GPIO_NUM_4
#define AUDIO_I2S_MIC_GPIO_SCK     GPIO_NUM_5
#define AUDIO_I2S_MIC_GPIO_DIN     GPIO_NUM_6

// ============================================================================
// AUDIO: MAX98357A
// ============================================================================

#define AUDIO_I2S_SPK_GPIO_DOUT    GPIO_NUM_7
#define AUDIO_I2S_SPK_GPIO_BCLK    GPIO_NUM_15
#define AUDIO_I2S_SPK_GPIO_LRCK    GPIO_NUM_16

#define AUDIO_INPUT_SAMPLE_RATE    16000
#define AUDIO_OUTPUT_SAMPLE_RATE   24000

// ============================================================================
// ST7789 240x240
// ============================================================================

#define DISPLAY_SPI_HOST           SPI2_HOST

#define DISPLAY_SPI_SCK_PIN        GPIO_NUM_9
#define DISPLAY_SPI_MOSI_PIN       GPIO_NUM_10

#define DISPLAY_DC_PIN             GPIO_NUM_8
#define DISPLAY_RST_PIN            GPIO_NUM_18
#define DISPLAY_CS_PIN             GPIO_NUM_NC

#define DISPLAY_WIDTH              240
#define DISPLAY_HEIGHT             240

#define DISPLAY_OFFSET_X           0
#define DISPLAY_OFFSET_Y           0

#define DISPLAY_MIRROR_X           false
#define DISPLAY_MIRROR_Y           false
#define DISPLAY_SWAP_XY            false

#define DISPLAY_INVERT_COLOR       true

#define DISPLAY_SPI_MODE           3

#define DISPLAY_PCLK_HZ            (40 * 1000 * 1000)

// ============================================================================
// BUTTON
// ============================================================================

#define BOOT_BUTTON_GPIO           GPIO_NUM_0

// ============================================================================
// UART ESP32-C6
// ============================================================================

#define C6_UART_PORT               UART_NUM_1

#define C6_UART_RX_PIN             GPIO_NUM_1
#define C6_UART_TX_PIN             GPIO_NUM_2

#define C6_UART_BAUD_RATE          115200
#define C6_UART_RX_BUFFER_SIZE     512

// MOTOR DRV8833 (Robot 2 bánh)
// ============================================================================

#define MOTOR_A_IN1_GPIO     GPIO_NUM_12
#define MOTOR_A_IN2_GPIO     GPIO_NUM_13
#define MOTOR_B_IN3_GPIO     GPIO_NUM_14
#define MOTOR_B_IN4_GPIO     GPIO_NUM_21

#define MOTOR_PWM_FREQ          20000      // 20kHz
#define MOTOR_PWM_RESOLUTION    LEDC_TIMER_10_BIT
#define MOTOR_PWM_MAX_DUTY      1023       // 2^10 - 1
 

#define OBSTACLE_STOP_DISTANCE_MM 80


// ============================================================================
// VL6180X / ToF050C - NEW I2C MASTER DRIVER
// ============================================================================
//
// GPIO40 -> SDA
// GPIO39 -> SCL
// I2C address -> 0x29
//
// IMPORTANT:
//   Khong include <driver/i2c.h> legacy trong file nay.
//   ESP-IDF 6.x dung <driver/i2c_master.h>.
//
// ============================================================================

#define TOF_I2C_SDA_GPIO           GPIO_NUM_40
#define TOF_I2C_SCL_GPIO           GPIO_NUM_39

#define TOF_I2C_PORT               I2C_NUM_0
#define TOF_I2C_FREQ_HZ            100000
#define VL6180X_I2C_ADDRESS        0x29

#endif  // _TUAN_S3_XIAOZHI_CONFIG_H_
