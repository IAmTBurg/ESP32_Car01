#pragma once

// =====================================================================================
//		main.h
//
//
// =====================================================================================
//
// =====================================================================================

#include <esp_log.h>
#include <cstring>
#include <stdio.h>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/i2c.h"
// #include "driver/i2c_master.h"
#include "../include/class_main.h"
#include "../include/main.h"
#include "../include/task0.h"
#include "../include/task1.h"
#include "../include/tasks.h"
#include "../lib/pca9685/pca9685.h"
#include "../lib/motor_control/motor_control.h"
#include "../lib/motor_control/motor_control_task.h"
#include "../lib/sbus_component/sbus_component.h"
#include "../lib/my_i2c_interface/my_i2c_interface.h"


#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE

#define MICRO_SECS (esp_timer_get_time())
#define MILLI_SECS (esp_timer_get_time() / 1000)

#define LED_BUILTIN 2
extern int8_t   BuiltInLED_Status;       // variable status of onboard LED

extern char TAG[16];

extern SBUS_COMPONENT::Sbus                         sbus;
extern MOTORS_NS::Motors_CL                         motors_obj;
extern PCA9685_NS::PCA9685_CL                       pca_obj;
extern MY_I2C_INTERFACE_NS::MY_I2C_INTERFACE_CL     my_i2c_obj;

// void gpio_monitor_task (void *parameters);
