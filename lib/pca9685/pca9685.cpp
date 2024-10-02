// ======================================================================================
//		PCA9685.cpp
// ======================================================================================

// ======================================================================================
//  see https://learn.adafruit.com/16-channel-pwm-servo-driver/overview
// ======================================================================================

// ===================== Includes ========================================

#include "../include/main.h"

using namespace MOTORS_NS;
using namespace PCA9685_NS;
using namespace MY_I2C_INTERFACE_NS;

extern MY_I2C_INTERFACE_NS::MY_I2C_INTERFACE_CL   my_i2c_obj;

namespace PCA9685_NS
{

PCA9685_CL::PCA9685_CL(uint8_t I2C_addr)
{
	// PCA9685_CL::_I2C_addr = I2C_addr;
	_I2C_addr = I2C_addr;
}

esp_err_t PCA9685_CL::pca_write_mode1() 
{
    esp_err_t ret = ESP_OK;
    return ret |= my_i2c_obj.my_i2c_wrtcmd_write(MODE1, _mode1_byte, 1);
}

esp_err_t PCA9685_CL::pca_read_mode1() 
{
    esp_err_t ret = ESP_OK;
    return ret |= my_i2c_obj.my_i2c_wrtcmd_read(MODE1, _mode1_byte, 1);
}

esp_err_t PCA9685_CL::pca_write_mode2() 
{
    esp_err_t ret = ESP_OK;
    return ret |= my_i2c_obj.my_i2c_wrtcmd_write(MODE2, _mode2_byte, 1);
}

esp_err_t PCA9685_CL::pca_read_mode2() 
{
    esp_err_t ret = ESP_OK;
    return ret |= my_i2c_obj.my_i2c_wrtcmd_read(MODE2, _mode2_byte, 1);
}

/* Initialises the I2C interface and puts the device in to a default state */
esp_err_t PCA9685_CL::pca_init()
{
    esp_err_t ret = ESP_OK;
    pca_setPWMFreq(1600);  
    pca_reset();
	pca_autoIncrement(true);
	pca_sleep();
    pca_setOscillatorFrequency(PCA9685_FREQ);
    pca_wake();
    pca_read_mode1();

    return ret;
}

esp_err_t PCA9685_CL::pca_reset() 
{
    esp_err_t ret = ESP_OK;
    _mode1_byte[0] = 0x00 | (0x01 << MODE1_RESTART_BIT) | (0x01 << MODE1_AI_BIT);  // 0xA0
    _mode2_byte[0] = 0x00 | (0x01 << MODE2_OUTDRV_BIT);  // 0x04
    PCA9685_CL::pca_write_mode1();    
    PCA9685_CL::pca_write_mode2();    
    return ret;
}

esp_err_t PCA9685_CL::pca_sleep()  // put to sleep
{
    esp_err_t ret = ESP_OK;
    ret |= PCA9685_CL::pca_read_mode1();
    _mode1_byte[0] |= (0x01 << MODE1_SLEEP_BIT);  // turn on sleep bit
    ret |= PCA9685_CL::pca_write_mode1();
    vTaskDelay(1);     
    return ret;
}

esp_err_t PCA9685_CL::pca_wake()  // wakeup
{
    esp_err_t ret = ESP_OK;
    ret |= PCA9685_CL::pca_read_mode1();
    _mode1_byte[0] &= ~(1 << MODE1_SLEEP_BIT);  // turn off sleep bit
    ret |= PCA9685_CL::pca_write_mode1();
    vTaskDelay(1);      
    return ret;
}

esp_err_t PCA9685_CL::pca_autoIncrement(bool mode)
{
    esp_err_t ret = ESP_OK;
    ret |= PCA9685_CL::pca_read_mode1();
    if (mode == true) 
        _mode1_byte[0] |= (0x01 << MODE1_AI_BIT); 
    else 
        _mode1_byte[0] &= ~(1 << MODE1_AI_BIT);
    ret |= PCA9685_CL::pca_write_mode1();
    vTaskDelay(1);     
    return ret;
}

esp_err_t PCA9685_CL::pca_set_motor_pwm(uint8_t pwm_channel, uint8_t speed)  // speed range 0 to 100
{
    esp_err_t ret = ESP_OK;
    uint8_t     pwm_channel_register = ((pwm_channel - 1) * 4) + 6;  //e.g. channel 1 == pwm_channel_register 6
    uint16_t    convert_speed = 40 * speed;
    uint8_t     reg_value[4];

    reg_value[0] = 0;                               // LEDx_ON_L
    reg_value[1] = 0;                               // LEDx_ON_H
    reg_value[2] = 0x00 | convert_speed;            // LEDx_OFF_L
    reg_value[3] = 0x00 | (convert_speed>>8);       // LEDx_OFF_H

    if (pwm_channel == 1) _test_value_1 = convert_speed;
    else _test_value_2 = convert_speed;
    _test_data[0] = reg_value[2];
    _test_data[1] = reg_value[3];

    return ret |= my_i2c_obj.my_i2c_wrtcmd_write(pwm_channel_register, reg_value, sizeof(reg_value));
}

esp_err_t PCA9685_CL::pca_write_motor_array()   // write array for all motors
{
    esp_err_t ret = ESP_OK;
    uint8_t     pca_register = 6;  // register 6 is start of pwm channels
    uint16_t    convert_speed;
    uint16_t    forward_direction;
    uint16_t    backward_direction;
    
    uint8_t     reg_values[32];

    for ( int i = 0; i < 4; i++ )
    {
        convert_speed = motors_obj.motors_arr[i].speed * 40;  // scale to 0 - 4000 (register max is 4095)
        // if (motors_obj.motors_arr[i].direction >= 0) {forward_direction = 4095; backward_direction = 0; }
        // else {forward_direction = 0; backward_direction = 4095; }
        if (motors_obj.motors_arr[i].direction >= 0) {forward_direction = convert_speed; backward_direction = 0; }
        else {forward_direction = 0; backward_direction = convert_speed; }
        // // ** motors pwm register
        // reg_values[(i*12) + 0] = 0;                              // LEDx_ON_L
        // reg_values[(i*12) + 1] = 0;                              // LEDx_ON_H
        // reg_values[(i*12) + 2] = 0x00 | convert_speed;           // LEDx_OFF_L
        // reg_values[(i*12) + 3] = 0x00 | (convert_speed>>8);      // LEDx_OFF_H
        // ** motors forward register
        reg_values[(i*8) + 0] = 0;                               // LEDx_ON_L
        reg_values[(i*8) + 1] = 0;                               // LEDx_ON_H
        reg_values[(i*8) + 2] = 0x00 | forward_direction;        // LEDx_OFF_L
        reg_values[(i*8) + 3] = 0x00 | (forward_direction>>8);   // LEDx_OFF_H
        // ** motors backward register
        reg_values[(i*8) + 4] = 0;                               // LEDx_ON_L
        reg_values[(i*8) + 5] = 0;                               // LEDx_ON_H
        reg_values[(i*8) + 6] = 0x00 | backward_direction;       // LEDx_OFF_L
        reg_values[(i*8) + 7] = 0x00 | (backward_direction>>8);  // LEDx_OFF_H
    }
    return ret |= my_i2c_obj.my_i2c_wrtcmd_write(pca_register, reg_values, sizeof(reg_values));
}

void PCA9685_CL::pca_setOscillatorFrequency(uint32_t freq) 
{
    _oscillator_freq = freq;
}

/*!
 *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
 *  @param  freq Floating point frequency that we will attempt to match
 */
esp_err_t PCA9685_CL::pca_setPWMFreq(float freq) {
    esp_err_t   ret = ESP_OK;
    uint8_t     oldMode1Val[1];

    // Range output modulation frequency is dependant on oscillator
    if (freq < 1)
        freq = 1;
    if (freq > 3500)
        freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

    float prescaleval = ((_oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN)
        prescaleval = PCA9685_PRESCALE_MIN;
    if (prescaleval > PCA9685_PRESCALE_MAX)
        prescaleval = PCA9685_PRESCALE_MAX;
    uint8_t prescale = (uint8_t)prescaleval;

    ret |= pca_read_mode1();
    oldMode1Val[0] = _mode1_byte[0];

    _mode1_byte[0] |= (0x01 << MODE1_SLEEP_BIT);  // turn on sleep bit
    _mode1_byte[0] &= ~(0x01 << MODE1_RESTART_BIT);  // turn off restart bit
    ret |= pca_write_mode1();
    ret |= my_i2c_obj.my_i2c_wrtcmd_write(PRE_SCALE, &prescale, 1); // set the prescaler
    _mode1_byte[0] = oldMode1Val[0];
    ret |= pca_write_mode1();
    vTaskDelay(5);
    _mode1_byte[0] = _mode1_byte[0] | (0x01 << MODE1_RESTART_BIT) | (0x01 << MODE1_AI_BIT);
    ret |= pca_write_mode1();

    return ret;
}


} // end namespace PCA9685_NS