#pragma once

// =====================================================================================
//		tasks.h
//
//
// =====================================================================================
//
// =====================================================================================

#include "main.h"
#include "task0.h"
#include "task1.h"

// ===================== Functions =================================
void setupTasks();					// FreeRTOS task setup

// ===================== FreeRTOS Tasks ============================
extern TaskHandle_t Task0;							    // runs on core 0
void task0Code( void * pvParameters );
extern TaskHandle_t Task1;							    // runs on core 1
void task1Code( void * pvParameters );
// extern TaskHandle_t Messages1;							// runs on core 1
// void Messages1Code( void * pvParameters );
extern TaskHandle_t MotorControl1;							// runs on core 1
void MotorControl1Code( void * pvParameters );

// ===================== FreeRTOS Queues ==========================
// msgQueue message displays - off loads Serial.Print overhead to core 1
// extern QueueHandle_t 	msgQueue;
// extern int32_t			c0_printMsgNumb;		// variable to use on core 0
// extern int32_t			c1_printMsgNumb;		// variable to use on core 1
// extern int32_t			m1_printMsgNumb;		// variable to use for message task
