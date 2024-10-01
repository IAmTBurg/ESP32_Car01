// ======================================================================================
//		my_i2c_interface.cpp
//
//      Pins:
//          SCL - 22
//          SDA - 21
//          VCC - 3.3V rail
//          GND - GND rail
//
// ======================================================================================

// ======================================================================================
//  see https://learn.adafruit.com/16-channel-pwm-servo-driver/overview
// ======================================================================================

// ===================== Includes ========================================

#include "main.h"
#include "my_i2c_interface.h"
#include "pca9685.h"

namespace MY_I2C_INTERFACE_NS
{

MY_I2C_INTERFACE_CL::MY_I2C_INTERFACE_CL(uint8_t len1)
{
    my_i2c_data =       (uint8_t *) malloc(len1);
    my_i2c_data_wr =    (uint8_t *) malloc(DATA_LENGTH_WR);
    my_i2c_data_rd =    (uint8_t *) malloc(DATA_LENGTH_RD);
}

esp_err_t MY_I2C_INTERFACE_CL::my_i2c_init()
{
    esp_err_t ret{ESP_OK};
    return ret |=  MY_I2C_INTERFACE_CL::my_i2c_master_init();
}

esp_err_t MY_I2C_INTERFACE_CL::my_i2c_master_init(void)
{
    esp_err_t ret{ESP_OK};

    if (!_i2c_master_config)
    {
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master { .clk_speed = I2C_MASTER_FREQ_HZ },
            .clk_flags = 0
        };
        i2c_param_config(_i2c_master_port, &conf);
        ret = i2c_driver_install(_i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
        _i2c_master_config = true;
    }
    return ret;
}

esp_err_t   MY_I2C_INTERFACE_CL::my_i2c_write(uint8_t *my_i2c_data_wr, size_t wr_size)
{
    esp_err_t ret{ESP_OK};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_my_i2c_slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, my_i2c_data_wr, wr_size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t   MY_I2C_INTERFACE_CL::my_i2c_read(uint8_t *my_i2c_data_rd, size_t rd_size)
{
    esp_err_t ret{ESP_OK};
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_my_i2c_slave_addr << 1) | READ_BIT, ACK_CHECK_EN);
    if (rd_size > 1) {
        i2c_master_read(cmd, my_i2c_data_rd, rd_size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, my_i2c_data_rd + rd_size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t   MY_I2C_INTERFACE_CL::my_i2c_write_read(
    uint8_t *my_i2c_data_wr, size_t wr_size, uint8_t *my_i2c_data_rd, size_t rd_size)
{
    esp_err_t ret{ESP_OK};
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_my_i2c_slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, my_i2c_data_wr, wr_size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) { return ret; }
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_my_i2c_slave_addr << 1) | READ_BIT, ACK_CHECK_EN);
    if (rd_size > 1) {
        i2c_master_read(cmd, my_i2c_data_rd, rd_size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, my_i2c_data_rd + rd_size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t   MY_I2C_INTERFACE_CL::my_i2c_wrtcmd_write(uint8_t my_i2c_cmd, uint8_t *my_i2c_data_wr, size_t wr_size)
{
    esp_err_t ret{ESP_OK};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_my_i2c_slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd,(uint8_t &) my_i2c_cmd, ACK_CHECK_EN);
    i2c_master_write(cmd, my_i2c_data_wr, wr_size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t   MY_I2C_INTERFACE_CL::my_i2c_wrtcmd_read(uint8_t my_i2c_cmd, uint8_t *my_i2c_data_rd, size_t rd_size)
{
    esp_err_t ret{ESP_OK};
    ret = my_i2c_write_read(&my_i2c_cmd, 1, my_i2c_data_rd, rd_size);
    return ret;
}
} // end namespace MY_I2C_INTERFACE_NS
