// ======================================================================================
//		MotorControl.cpp
// ======================================================================================

// ======================================================================================
//
// ======================================================================================

// ===================== Includes ========================================

#include "main.h"
#include "motor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "sbus_component.h"
#include "pca9685.h"

namespace MOTORS_NS
{
using namespace PCA9685_NS;

Motors_CL::Motors_CL()
{   
    strcpy(motorLogTag, "Motors");
    motors_arr[0] = {0, 0};    // motor left front pins (pwm, forward, backward), speed, direction1, direction2, direction3
    motors_arr[1] = {0, 0};    // motor right front pins (pwm, forward, backward), speed, direction1, direction2, direction3
    motors_arr[2] = {0, 0};    // motor left rear pins (pwm, forward, backward), speed, direction1, direction2, direction3
    motors_arr[3] = {0, 0};    // motor right rear pins (pwm, forward, backward), speed, direction1, direction2, direction3
    wm = stop;
}

esp_err_t Motors_CL::motors_CalcMovement( void )
{
    int16_t motors_speed = 0;
    int16_t motors_direction1 = 0;
    int16_t motors_direction2 = 0;
    int16_t motors_direction3 = 0;
    esp_err_t ret{ESP_OK};

    // Convert sbus (1000 - 2000) to speed (-100 to +100)
    if (SBUS_COMPONENT::Sbus::sbusStatus != 0) 
    {   // stop motors
        for (int i = 0; i < 4; i++) 
        {
            motors_arr[i].speed = 0;
            motors_arr[i].direction = 0;
        }
        return ret;
    }
    else
    {   // Convert sbus (1000 - 2000) to Motors_CL (-100 to +100)
        motors_direction1 = (SBUS_COMPONENT::Sbus::speed / 5) - 300;
        motors_direction2 = (SBUS_COMPONENT::Sbus::direction / 5) - 300;
        motors_direction3 = (SBUS_COMPONENT::Sbus::channel4 / 5) - 300;
        // set motors_speed to greatest of absoulte value of direction 1,2, or 3
        motors_speed  = (abs(motors_direction1) > abs(motors_direction2)) ? abs(motors_direction1) : abs(motors_direction2);
        motors_speed  = (motors_speed > abs(motors_direction3)) ? abs(motors_speed) : abs(motors_direction3);
    }
    
    if (abs(motors_direction1) <= M_DEADBAND && abs(motors_direction2) <= M_DEADBAND && abs(motors_direction3) <= M_DEADBAND ) // stop motors
    {   // stop ()  // motors_speed is in deadband < 5
        for (int i = 0; i < 4; i++) 
        {
            motors_arr[i].speed = 0;
            motors_arr[i].direction = 0;
        }
    }

    if (motors_direction1 >= 0 ) // forward
    {
        for (int i = 0; i < 4; i++) motors_arr[i].direction = 100;
    }
    else // backwards
    {
        for (int i = 0; i < 4; i++) motors_arr[i].direction = -100;
    }

    if (motors_speed > 5)  // test for deadband
    {   
        for (int i = 0; i < 4; i++) motors_arr[i].speed = motors_speed;
    }
    else
    {   // stop ()  // motors_speed is in deadband < 5
        for (int i = 0; i < 4; i++) motors_arr[i].speed = 0;
        for (int i = 0; i < 4; i++) motors_arr[i].direction = 0;
    }

    return ret; 
}

esp_err_t Motors_CL::motors_move()  //  adjust motors
{
    esp_err_t ret{ESP_OK};

    // ******************** initiate motor movement ************************************
    return ret |= pca_obj.pca_write_motor_array();
}

} // end namespace MOTORS_NS