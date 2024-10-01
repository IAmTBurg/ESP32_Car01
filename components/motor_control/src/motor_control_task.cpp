// ======================================================================================
//		motor_control_taskcpp
// ======================================================================================

// ======================================================================================
//
// ======================================================================================

// ===================== Includes ===================================

#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "task0.h"
#include "task1.h"
#include "motor_control_task.h"
#include "motor_control.h"
#include "my_i2c_interface.h"
#include "pca9685.h"

using namespace MOTORS_NS;
using namespace PCA9685_NS;
using namespace MY_I2C_INTERFACE_NS;

// ===================== variables ===================================

TaskHandle_t MotorControl1;	

// ===================== Functions ===================================

// MotorControl1Code - control loop
void MotorControl1Code( void * pvParameters)
{
	(void)      pvParameters;
    int         motorTest_State = 0;

    const TickType_t loopTime = 50 / portTICK_PERIOD_MS; // loop time in ms
    const uint32_t  loopsPerSecond = 1000 / loopTime;
    TickType_t      loopLastWakeTime;
    uint32_t        loopCount = 0;

    // *******************************************************************************************************
    //	disableCore1WDT();  // Disable Watchdog timer on core 0 IDLE0 task *************************************
    // *******************************************************************************************************
    ESP_LOGD(TAG, "MotorControl1Code() running on core %i", xPortGetCoreID());

	//------------------------------------
	// initialize main controll loop 0
	//------------------------------------

    // intialize motors movements
    my_i2c_obj.my_i2c_init();
    pca_obj.pca_init();
    motors_obj.motors_CalcMovement();  // Calculate speed and direction from sbus
    motors_obj.motors_move();

    //--------------------------------------------------------------------------
	// loop - main controll loop 0 - runs every CONTROL_LOOP_0_TIME (LOOPS_PER_SEC_0)
	//--------------------------------------------------------------------------

    // Initialise the xLastWakeTime variable with the current time.
    loopLastWakeTime = xTaskGetTickCount();  // Init loopLastWakeTime with current time.

    for(;;)
	{
        loopCount++;
		
        if (loopCount % (loopsPerSecond / 10) == 0) // Every .1 seconds (100ms)
		{
            // include tasks for every .1 seconds here

            motors_obj.motors_CalcMovement();  // Calculate speed and direction from sbus
            motors_obj.motors_move();
		}

		if (loopCount % (loopsPerSecond / 4) == 0) // Every .25 seconds (250ms)
		{
			// include tasks for every .25 seconds here
		}

		if (loopCount % (loopsPerSecond / 2) == 0) // Every .5 seconds (500ms)
		{
            // include tasks for every .5 seconds here
		}

		if (loopCount % (loopsPerSecond * 1) == 0) // Every 1 seconds (1000ms)
		{
            // include tasks for every 1 seconds here

            ESP_LOGD(TAG, "motors_arr[].speed = %i, %i, %i, %i    motors_arr[].direction = %i, %i, %i, %i", 
                    motors_obj.motors_arr[0].speed, motors_obj.motors_arr[1].speed, 
                    motors_obj.motors_arr[2].speed, motors_obj.motors_arr[3].speed, 
                    motors_obj.motors_arr[0].direction, motors_obj.motors_arr[1].direction, 
                    motors_obj.motors_arr[2].direction, motors_obj.motors_arr[3].direction);
        }

		if (loopCount % (loopsPerSecond * 2) == 0) // Every 2 seconds (2000ms)
		{
            // include tasks for every 2 seconds here

            switch (motorTest_State)
            {
                case -2:
                    break;
                case -1:
                    break;
                case 0:
                    // ESP_LOGD(motors.motorLogTag, "A-speed, A-direction, B-speed, B-direction %i,%i,%i,%i", 
                    //     motors.motor_A_speed, motors.motor_A_direction, motors.motor_B_speed, motors.motor_B_direction);
                    // ESP_LOGD(motors.motorLogTag, "_test_data[0] - %i, _test_data[1] - %i, _test_value_1 - %i, _test_value_2 - %i, ", 
                    //     pca_obj._test_data[0], pca_obj._test_data[1], pca_obj._test_value_1, pca_obj._test_value_2);
                    motorTest_State = 0; 
                    break;
                case 1:
                    break;
                case 2:
                    break;
                case 3:
                    break;
                default:
                    motorTest_State = 0;                     
            }
		}

        //----------------------- End of Loop Processing ----------------------//
        vTaskDelayUntil( &loopLastWakeTime, loopTime );
	}
}