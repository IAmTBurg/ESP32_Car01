#pragma once

//=====================================================================================
//		pca9685.h
//
//
//=====================================================================================
//
//  see https://learn.adafruit.com/16-channel-pwm-servo-driver/overview
// 
//=====================================================================================

#include <cstring>
#include <stdio.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

// ------------------------ generic defines   ---------------------------------------------------------------------

// ------------------------ PCA9685 defines   ---------------------------------------------------------------------
 /* Register address and bit positions/masks for the devices internal registers */
#define MODE1 0x00
#define MODE1_ALLCALL_BIT 0
#define MODE1_SUB3_BIT 1
#define MODE1_SUB2_BIT 2
#define MODE1_SUB1_BIT 3
#define MODE1_SLEEP_BIT 4
#define MODE1_AI_BIT 5
#define MODE1_EXTCLK_BIT 6
#define MODE1_RESTART_BIT 7

#define MODE2 0x01
#define MODE2_OUTNE_BIT 0
#define MODE2_OUTDRV_BIT 2
#define MODE2_OCH_BIT 3
#define MODE2_INVRT_BIT 4
#define OUTNE_LOW 0

#define SUBADR1 0x02
#define SUBADR2 0x03
#define SUBADR3 0x04
#define ALLCALLADR 0x05
#define ALL_LED_ON_L 0xFA
#define ALL_LED_ON_H 0xFB
#define ALL_LED_OFF_L 0xFC
#define ALL_LED_OFF_H 0xFD
#define PRE_SCALE 0xFE
#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

#define PCA9685_FREQ    27000000                    
#define PCA9685_ADDR    0x40                       /* I2C PWM Expansion Module Address (7-bit then shifted left by 1 bit and 0 bit add for write, 1 bit add for read)  */
#define PCA9685_MODE1   0xA0                       /* see PCA9685 datasheet */
#define PCA9685_MODE2   0x04                       /* see PCA9685 datasheet */

namespace PCA9685_NS
{
    class PCA9685_CL
    {
        public:
            uint32_t    _test_value_1;
            uint32_t    _test_value_2;
            uint8_t     _test_data[20];

            uint8_t     _I2C_addr;
            uint8_t     _mode1_byte[1];
            uint8_t     _mode2_byte[1];
            uint8_t     _data_write_buf[254];
            uint32_t    _oscillator_freq;

        public:
            PCA9685_CL(uint8_t I2C_addr);

            esp_err_t   pca_write_mode1(void);
            esp_err_t   pca_read_mode1(void);
            esp_err_t   pca_write_mode2(void);
            esp_err_t   pca_read_mode2(void);
            esp_err_t   pca_init(void);
            esp_err_t   pca_reset(void);
            esp_err_t   pca_sleep(void);
            esp_err_t   pca_wake(void);
            esp_err_t   pca_autoIncrement(bool mode);
            esp_err_t   pca_set_motor_pwm(uint8_t pwm_channel, uint8_t speed);
            esp_err_t   pca_write_motor_array(void);  // write array for all motors
            void        pca_setOscillatorFrequency(uint32_t freq);
            esp_err_t   pca_setPWMFreq(float freq);
    };  // end class PCA9685_CL
} // end namespace PCA9685_NS


