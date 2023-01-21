#pragma once

// =====================================================================================
//		main.h
//
//
// =====================================================================================
//
// =====================================================================================

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#include "esp_log.h"

#include <cstring>
#include <stdio.h>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "tasks.h"
#include "sbus_component.h"
#include "motor_control.h"
#include "my_i2c_interface.h"
#include "pca9685.h"

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
