#pragma once

/*---------------------------------------------------------------------------

Uses UART2_NUM_2
ESP32 UART2 
    UART2_RX_PIN 16  connect FrSky receiver data line to UART2_RX_PIN
    UART2_TX_PIN 17  note: FrSky receiver does not us ESP32 tx pin

-----------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/

#include <cstring>
#include <stdio.h>
#include <string>
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"

#define DEBUG_SBUS 1    			// set in SBUS_component.h 1 - turn on;  0 - turn off

namespace SBUS_COMPONENT
{
    //*********************************//
    //*** UART2 task handler config ***//
    //*********************************//

    // using UART2 with only rx pin GPIO16, tx not used
    #define UART2_TX_PIN 17
    #define UART2_RX_PIN 16
    #define EX_UART_NUM UART_NUM_2
    #define UART2_RX_BUF_SIZE (130)
    #define UART2_TX_BUF_SIZE (0)
    #define UART2_RD_BUF_SIZE (UART2_BUF_SIZE)

    // ----------------------------------------------------------------    //  class GpioBase definition
    // ----------------------------------------------------------------
    class Sbus
    {
    private:

    public:

        Sbus(void);

        typedef struct // used by USART receive data interrupt service routine (ISR) to manage and verify the receipt of an S.BUS packet
        {
            uint8_t		lock;					// notify IRQ that the structure is locked
            uint8_t  	newPacketReceived;      // 1 - set when last data byte of packet received by interrupt routine, 0 - set by processing routine when copied to myRxBuf for use outside of the ISR (disable interrupts during copy)
            uint8_t  	state;                  // 0 - not currently processing a packet; 1 - currently receiving and filling packet buffer
            uint32_t 	msLastByteReceived;   	// millis() of the last byte received
            uint32_t 	msStartByte;          	// millis() of the start byte received in last packet
            uint8_t  	rxBuf[25];              // receive packet data buffer
            uint8_t  	pointRxBuf;             // pointer to last data byte received in rxBuf_x array
        } USART_rx_struct;
        static volatile USART_rx_struct 	SBUS_receive_data;	    // data structure for a SBUS packet.

        // SBUS off / Motor Direction / Speed variables
        static uint16_t     sbusStatus;			// channels[0] is the SBUS status
        static uint16_t     throttle;			// channels[3] is the throttle.  
        static uint16_t     direction;			// channels[1] is the right stick horizontal direction (right stick left and right).   
        static uint16_t     channel4;			// channels[4] is the left stick horizontal direction (left stick left and right).   
        static uint16_t     speed;			    // channels[2] is the Speed (channel 2 is right stick up and down and is centered).  

        static void         Init_SBUS_receive(void);
        static void         ProcessUARTDataReceived(void * pvParameters );
        static int16_t      ProcessSBUSData(void); 
        static void         SBusToChannels(void); 
        static void         SBUSPrintDebugMsg1(void);
        static void         RC_monitor_task (void *parameters);

    };      // Sbus Class
} // namespace SBUS_COMPONENT

/* variables ------------------------------------------------------------------*/

extern              SBUS_COMPONENT::Sbus sbus;
extern 				uint32_t uart2ChrCnt;
extern 				uint16_t channels[19];           // The array of channel values derived by processing myRxBuf (S.BUS data)
extern          	uint8_t  dataByte;
extern uint16_t		rcProcessSBUSData[3];

#if DEBUG_SBUS
extern uint16_t  tmpDB1;
extern uint16_t  sbusDebugCntrs[4];              // counters used for debugging
#endif // #if DEBUG_SBUS
