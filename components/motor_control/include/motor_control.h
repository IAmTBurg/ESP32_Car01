#pragma once

//=====================================================================================
//		MotorControl.h
//
//
//=====================================================================================
// 
// ** Motor A **
// enable1Pin = 33;      
// motorPin_1 = 0x01 on I/O Expander (was 32);
// motorPin_2 = 0x02 on I/O Expander (was 27); 
// 
// ** Motor B **
// enable2Pin = 26; 
// motorPin_1 = 0x04 on I/O Expander (was 25);  
// motorPin_2 = 0x08 on I/O Expander (was 23);
//
// ** I2C GPIO Pins **
//  I2C Master SCL = 22
//  I2C Master SDA = 21
//=====================================================================================

#include <cstring>
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "motor_control_task.h"
#include "motor_control.h"

// -----------------  motor defines --------------------------------------------------------------------------

namespace MOTORS_NS
{
constexpr int8_t M_DEADBAND = 10;
constexpr int8_t M_LF = 1;
constexpr int8_t M_RF = 2;
constexpr int8_t M_LR = 3;
constexpr int8_t M_RR = 4;

enum enum_Wheels_Movement
{
    stop = 0,
    forward,
    backward,
    sideways_left,
    sideways_right,
    diagonal_left,
    diagonal_right,
    piovt_left,
    pivot_right,
    rotate_left,
    rotate_right,
    pivot_sideways_left,
    pivot_sideways_right
};

struct motor_control_t
    {
        // uint8_t pca_pin1;    // pca motor pwm signal
        // uint8_t pca_pin2;    // pca motor fwd signal
        // uint8_t pca_pin3;    // pca motor bck signal
        int16_t speed;       // motor speed (greatest of abs value of right stick up/down or right/left)
        int16_t direction;   // motor Forward or Backward
    };

 class Motors_CL
    {
    private:

    public:
        Motors_CL();

        // ===================== State variables ===================================
        char                motorLogTag[16];
        int                 motorTest_State;

        // ==================== Motor variables ======================================
        motor_control_t             motors_arr[4];
        enum enum_Wheels_Movement   wm;
        // ======================= Functions ============================================
        esp_err_t           motors_CalcMovement( void );
        esp_err_t           motors_move();
    };      // Sbus Motors_CL
} // namespace MOTORS_NS