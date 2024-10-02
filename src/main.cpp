/*************************************************************************************************************

2022-10-30: Beginning development on ESP32_Car01-IDF_v00.1/ESP32_Car01

**************************************************************************************************************/

//************   #include section   ********************************************************************

#include <stdio.h>
#include <string>
#include <iostream>
#include "esp_timer.h"
#include "class_main.h"
#include "../include/main.h"

//****************** Vars, Constants, and defines ********************************************************************************
using namespace MOTORS_NS;
using namespace PCA9685_NS;
using namespace MY_I2C_INTERFACE_NS;

char                    TAG[16];
char                    mTag[] = "motors";
Main_class              main_obj;  // Instantiate the Main_class class as App (declared in main.h)
MY_I2C_INTERFACE_CL     my_i2c_obj(6);
Motors_CL               motors_obj;   
// Motors_CL               motors(mTag, MOTOR_A_ENA, MOTOR_A_PIN_1, MOTOR_A_PIN_2, MOTOR_B_ENA, MOTOR_B_PIN_1, MOTOR_B_PIN_2);   
PCA9685_CL              pca_obj(PCA9685_ADDR);

constexpr auto          STACK_DEPTH = configMINIMAL_STACK_SIZE * 5;
int64_t                 next_SBUSPrintDebugMsg1 = 0;

//****************** Code ********************************************************************************

extern "C" void app_main(void)
{
    main_obj.setup();
     while (true)
    {
        main_obj.run();
        // Let the watch dog timer (wdt) reset run.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }    
}

void Main_class::setup(void)
{
    strcpy(TAG, "Main_class:");
    ESP_LOGD(TAG,"Running MAIN::setup()..");
    SBUS_COMPONENT::Sbus::Init_SBUS_receive();      	// Intialialize data structure and start UART processing
    
    setupTasks();
    
    // xTaskCreate(gpio_monitor_task, "GPIO Monitor Task", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL );
    xTaskCreate(SBUS_COMPONENT::Sbus::RC_monitor_task, "RC Monitor Task", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL );
}

void Main_class::run(void)
{
    if (next_SBUSPrintDebugMsg1 <= MILLI_SECS) {
        ESP_LOGD(TAG,"Running MAIN::run next_SBUSPrintDebugMsg1...");
        next_SBUSPrintDebugMsg1 = MILLI_SECS + 15000;
        SBUS_COMPONENT::Sbus::SBUSPrintDebugMsg1();
    }
    return;
}