//  =====================================================================================
//		tasks.cpp
//
//
//  =====================================================================================
//
//  =====================================================================================

#include "tasks.h"
#include "main.h"
#include "task0.h"
#include "task1.h"
#include "motor_control_task.h"

// setup for FreeRTOS tasks and resources
void setupTasks()
{
    //===================== FreeRTOS queue creation =====================//
	// do this first to make sure queues are available for tasks when they start
	// msgQueue = xQueueCreate(30, sizeof(c0_printMsgNumb)); 	// 30 item queue that can hold msgIDNumb

    //===================== Setup Motor Control =====================//
    // motorSetup();
    // MotorsTest();       // endless loop for initial testing

	//===================== FreeRTOS task creation =====================//
	// Parameters: 1- task function to run, 2- name of task, 3 - stack size for task, 4 - parameter of the task
	// 5 - priority of the task, 6 - task handle, 7 - processing core to pin the task to
	
    
    // start main control loop
	// xTaskCreatePinnedToCore(task0Code, "Task0", 10000, NULL, 20, &Task0, 0);
	xTaskCreatePinnedToCore(task0Code, "Task0", 10000, NULL, 20, &Task0, 0);
	vTaskDelay( 500 );
    // start utility control loop first 
    // xTaskCreatePinnedToCore(task1Code, "Task1", 10000, NULL, 10, &Task1, 1);
    xTaskCreatePinnedToCore(task1Code, "Task1", 10000, NULL, 10, &Task1, 1);
    vTaskDelay( 500 );
    // // start Message processing task - lower priority then utility controller control loop
	// xTaskCreatePinnedToCore(Messages1Code, "Messages1", 10000, NULL, 8, &Messages1, 1);
    // vTaskDelay( 500 );
    // start Motor Control processing task - lower priority then utility controller control loop
	// xTaskCreatePinnedToCore(MotorControl1Code, "MotorControl1", 10000, NULL, 6, &MotorControl1, 1);
	xTaskCreatePinnedToCore(MotorControl1Code, "MotorControl1", 10000, NULL, 6, &MotorControl1, 1);
    vTaskDelay( 500 );

}