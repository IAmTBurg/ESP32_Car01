// ======================================================================================
//		task0.cpp
// ======================================================================================

// ======================================================================================
//
// ======================================================================================

// ===================== Includes ===================================

#include <stdint.h>
#include <stdio.h>
#include "../include/main.h"
#include "task0.h"
#include "task1.h"

// ===================== variables ===================================

TaskHandle_t Task0;		

// ===================== Functions ===================================

// task0Code - control loop 0
void task0Code( void * pvParameters)
{
	(void) pvParameters;

    const TickType_t loopTime = 50 / portTICK_PERIOD_MS; // loop time in ms
    const uint32_t  loopsPerSecond = 1000 / loopTime;
    TickType_t      loopLastWakeTime;
    uint32_t        loopCount = 0;

// *******************************************************************************************************
//	disableCore0WDT();  // Disable Watchdog timer on core 0 IDLE0 task *************************************
// *******************************************************************************************************
    ESP_LOGD(TAG, "task0Code() running on core %i", xPortGetCoreID());

	//------------------------------------
	// initialize main controll loop 0
	//------------------------------------

    // Initialise the xLastWakeTime variable with the current time.
    loopLastWakeTime = xTaskGetTickCount();  // Init loopLastWakeTime with current time.

    //--------------------------------------------------------------------------
	// loop - main controll loop 0 - runs every CONTROL_LOOP_0_TIME (LOOPS_PER_SEC_0)
	//--------------------------------------------------------------------------
  for(;;)
	{
        loopCount++;

		if (loopCount % (loopsPerSecond / 4) == 0) // Every .25 seconds (250ms)
		{
			// include tasks for every .25 seconds here
		}

		if (loopCount % (loopsPerSecond / 2) == 0) // Every .5 seconds (500ms)
		{
            // include tasks for every .5 seconds here
		}

		if (loopCount % (loopsPerSecond / 4) == 0) // Every .25 seconds (2000ms)
		{
            // include tasks for every .25 seconds here
        }

		if (loopCount % (loopsPerSecond * 2) == 0) // Every 2 seconds (2000ms)
		{
            // include tasks for every 2 seconds here
		}

        //----------------------- End of Loop Processing ----------------------//
        vTaskDelayUntil( &loopLastWakeTime, loopTime );
	}
}