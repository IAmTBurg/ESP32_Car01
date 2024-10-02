#pragma once

//=====================================================================================
//		my_i2c_interface.h
//
//      Pins:
//          SCL - 22
//          SDA - 21
//          VCC - 3.3V rail
//          GND - GND rail
//
//=====================================================================================
//
//  see https://learn.adafruit.com/16-channel-pwm-servo-driver/overview
// 
//=====================================================================================

#include <driver/gpio.h>
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "../include/main.h"

// -----------------  I2C defines --------------------------------------------------------------------------
#define I2C_MASTER_SCL_IO           (gpio_num_t) 22            /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           (gpio_num_t) 21            /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          75000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL (i2c_ack_type_t) 0x0                             /*!< I2C ack value */
#define NACK_VAL (i2c_ack_type_t) 0x1                            /*!< I2C nack value */

#define DATA_LENGTH 6                           /*!< Data buffer length of test buffer */
#define DATA_LENGTH_WR 12                       /*!< Data buffer length of test buffer */
#define DATA_LENGTH_RD 12

namespace MY_I2C_INTERFACE_NS
{
    class MY_I2C_INTERFACE_CL
    {
        friend class PCA9685_CL;
        private:

        public:
            i2c_port_t  _i2c_master_port = I2C_NUM_0; 
            uint8_t     _my_i2c_slave_addr = PCA9685_ADDR; 
            bool        _i2c_master_config = false;
            uint8_t     *my_i2c_data;
            uint8_t     *my_i2c_data_wr;
            uint8_t     *my_i2c_data_rd;

        public:
            MY_I2C_INTERFACE_CL(uint8_t len);

            esp_err_t       my_i2c_init(void);
            esp_err_t       my_i2c_master_init(void);
            esp_err_t       my_i2c_write(uint8_t *my_i2c_data_wr, size_t wr_size);
            esp_err_t       my_i2c_read(uint8_t *my_i2c_data_rd, size_t rd_size);	
            esp_err_t       my_i2c_write_read(uint8_t *my_i2c_data_wr, size_t wr_size, uint8_t *my_i2c_data_rd, size_t rd_size);
            esp_err_t       my_i2c_wrtcmd_write(uint8_t my_i2c_cmd, uint8_t *my_i2c_data_wr, size_t wr_size);
            esp_err_t       my_i2c_wrtcmd_read(uint8_t my_i2c_cmd, uint8_t *my_i2c_data_rd, size_t rd_size);
    };  // end class MY_I2C_INTERFACE_CL
} // end namespace MY_I2C_INTERFACE_NS