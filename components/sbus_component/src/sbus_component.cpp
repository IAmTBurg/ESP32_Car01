#include <stdio.h>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/projdefs.h>
#include <freertos/queue.h>
#include <driver/uart.h>
#include "sbus_component.h"

#define MICRO_SECS (esp_timer_get_time())
#define MILLI_SECS (esp_timer_get_time() / 1000)
    
namespace SBUS_COMPONENT
{
    //*****************************//
    //*** Class Objects         ***//
    //*****************************//

    SBUS_COMPONENT::Sbus sbus;

    //*****************************//
    //*** Variables             ***//
    //*****************************//

    TaskHandle_t sbusRx;							// runs on core 1
    static QueueHandle_t uart2_event_queue;

    char                SbusLogTag[16];
    uint16_t            rcProcessSBUSData[3];
    uint16_t            SBUS_COMPONENT::Sbus::sbusStatus = 0;			// channels[0] is the SBUS status
    uint16_t            SBUS_COMPONENT::Sbus::throttle = 0;			    // channels[3] is the throttle (left stick up / down).  
    uint16_t            SBUS_COMPONENT::Sbus::direction = 0;			// channels[1] is the right stick horizontal direction (right stick left and right).   
    uint16_t            SBUS_COMPONENT::Sbus::channel4 = 0;			    // channels[4] is the left stick horizontal direction (left stick left and right).   
    uint16_t            SBUS_COMPONENT::Sbus::speed = 0;			    // channels[2] is the Speed (channel 2 is right stick up and down and is centered).  
    
    volatile SBUS_COMPONENT::Sbus::USART_rx_struct     SBUS_COMPONENT::Sbus::SBUS_receive_data;	

    uint32_t 					uart2ChrCnt;
    uint8_t 					myRxBuf[25];        // working buffer of S.BUS data received, when a full-validated packet is received by the USART ISR, it is copied here for use outside of the ISR to avoid conflicts with new packet reception.
    uint16_t  					channels[19];       // The array of channel values derived by processing myRxBuf (S.BUS data)
    uint8_t                     dataByte;
    
    #if DEBUG_SBUS              // set in SBUS.h 1 - turn on;  0 - turn off
    uint16_t                    tmpDB1;
    uint16_t                    sbusDebugCntrs[4];      // counters used for debugging
    #endif // #if DEBUG_SBUS

    //****************************************//
    //*** Sbus class functions             ***//
    //****************************************//
    
    // *** Sbus class initialization
    Sbus::Sbus(void)
    {
        strcpy(SbusLogTag, "Sbus");
    }

    // ** Initialize the packet data structure
    void Sbus::Init_SBUS_receive(void) 
    {
        int rc;
        // using UART2 with only rx pin GPIO16, tx not used

        ESP_LOGD(SbusLogTag,"\nSbus::Init_SBUS_receive()...\n");

        // Configure parameters for UART2 driver
        uart_config_t uart_config = {
            .baud_rate = 100000,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_EVEN,
            .stop_bits = UART_STOP_BITS_2,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 40,
            .use_ref_tick = true,
        };

        ESP_ERROR_CHECK(uart_param_config(EX_UART_NUM, &uart_config));

        //Set UART pins (using UART2 default pins ie no changes.)
        ESP_ERROR_CHECK(uart_set_pin(EX_UART_NUM, UART2_TX_PIN, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        
        //Install UART driver, and get the queue.
        ESP_ERROR_CHECK(uart_driver_install(EX_UART_NUM, UART2_RX_BUF_SIZE * 2, UART2_TX_BUF_SIZE * 2, 20, &uart2_event_queue, 0));

        //Create a task to handler UART event from ISR
        rc = xTaskCreatePinnedToCore(SBUS_COMPONENT::Sbus::ProcessUARTDataReceived, "ProcessUARTDataReceived", 10000, NULL, 15, &sbusRx, 1);
        if (rc != pdPASS )
        {
            ESP_LOGD(SbusLogTag,"******************  xTaskCreatePinnedToCore() rc - %x", rc);
        }

        ESP_LOGD(SbusLogTag,"****************** Inti_SBUS_receive() complete **********************\n");
        // SBUS off / Motor Direction / Speed variables
        sbusStatus = 20;			// channels[0] is the SBUS status
        throttle = 1500;			// channels[3] is the throttle.  
        direction = 1500;			// channels[1] is the right stick right/left direction. 
        channel4 = 1500;			// channels[4] is the left stick right/left direction. 
        speed = 1500;			    // channels[2] is the right/left Speed (channel 2 is right stick up and down and is centered). 
    }

    // ************************************************************************************************** //
    // ** ProcessUARTDataReceived() - check for receipt of SBUS Uart data.                             ** //
    //        Verify timing of beginning of SBUS packet and valid packet stream.                       ** //
    // Note: When sbus data is received on serial2, it is stored in a buffer.                          ** //
    //       This routine checks for data in the buffer.  											   ** //
    //       This routine is not triggered by an interupt, it must be called                           ** //
    // ************************************************************************************************** //
    void Sbus::ProcessUARTDataReceived(void * pvParameters ) 
    {
        (void)          pvParameters;

        uint32_t        current_msCntr = 0;

        uart_event_t    event;
        uint8_t*        dtmp = (uint8_t*) malloc(UART2_RX_BUF_SIZE);
        uint8_t         dptr;

        ESP_LOGD(SbusLogTag,"****************  Begin ProcessUARTDataReceived() ***************************\n");
        for(;;) 
        {
            #if DEBUG_SBUS
            sbusDebugCntrs[3]++;
            #endif // #if DEBUG_SBUS

            //Waiting for UART event.
            if(xQueueReceive(uart2_event_queue, (void * )&event, (portTickType)portTICK_PERIOD_MS * 1000)) 
            {
                if (event.type != UART_DATA) continue;
            }

            #if DEBUG_SBUS
            sbusDebugCntrs[2]++;
            #endif // #if DEBUG_SBUS
            bzero(dtmp, UART2_RX_BUF_SIZE);
            dptr = 0;
            uart_read_bytes(EX_UART_NUM, dtmp, event.size, 1000);
            if (event.size < 1) continue;
            for(int nDataBytes = 1; nDataBytes <= event.size; nDataBytes++)
            {
                dataByte = dtmp[dptr];
                dptr++;
                current_msCntr = MILLI_SECS;
                uart2ChrCnt++;

                if (SBUS_receive_data.lock == 1) // packet structure is locked, abort this packet and wait for the next packet.
                {  
                    SBUS_receive_data.msLastByteReceived = current_msCntr;
                    SBUS_receive_data.pointRxBuf = 0;
                    SBUS_receive_data.state = 0;
                    SBUS_receive_data.newPacketReceived = 0;
                    continue; 
                }
                if (SBUS_receive_data.state == 0) // currently not processing a packet, expect the start byte 0x0F
                {  
                    // expect 0x0F start of packet and should be at least 4 ms ideal between SBUS packets
                    if ((dataByte != 0x0F) || (SBUS_receive_data.msLastByteReceived > (esp_timer_get_time()/1000-2) ) ) 
                    { 
                        // discard packet

                        #if DEBUG_SBUS
                        sbusDebugCntrs[1]++;
                        #endif // #if DEBUG_SBUS

                        continue;
                    }
            
                    // okay, valid first data byte of packet
                    SBUS_receive_data.msLastByteReceived = current_msCntr;
                    SBUS_receive_data.msStartByte = current_msCntr;
                    SBUS_receive_data.rxBuf[0] = dataByte;
                    SBUS_receive_data.pointRxBuf = 1;
                    SBUS_receive_data.state = 1;                    // 1 - processing a packet
                    SBUS_receive_data.newPacketReceived = 0;		// in new packet now, just in case, reset new packet flag (1 is flag for packet received)
                    continue;
                }  // first byte of packet

                // middle or end of packet.
                SBUS_receive_data.msLastByteReceived = current_msCntr;
                SBUS_receive_data.rxBuf[SBUS_receive_data.pointRxBuf++] = dataByte;
            
                if (SBUS_receive_data.pointRxBuf == 25) // last byte in packet
                {
                    // last data byte received
                    SBUS_receive_data.pointRxBuf = 0;
                    SBUS_receive_data.state = 0;
                    SBUS_receive_data.newPacketReceived = 1;

                    #if DEBUG_SBUS
                    sbusDebugCntrs[0]++;
                    #endif // #if DEBUG_SBUS
                }
            } // end for(int xBytes = 1; xBytes <= event.size; xBytes++)
        }   // end for(;;) 
        free(dtmp);
        dtmp = NULL;
        vTaskDelete(NULL);
    }

    // ************************************************************************************************** //
    // ** ProcessSBUSData() - converts the incoming SBUS packet to individual channel values.          ** //
    //        This is not done for every packet, just when called and then against the latest packet.  ** //
    // ************************************************************************************************** //
    int16_t Sbus::ProcessSBUSData(void)  	// returns: 0 = new packet, 1 = no new packet received yet, 2 = USART and IRQ busy receiving a packet (3 ms).
    {
        if (SBUS_receive_data.newPacketReceived != 1) return 1;		// no new packet yet
        if (SBUS_receive_data.state == 1) return 2;		// busy receiving packet

        SBUS_receive_data.lock = 1;		// notify ProcessUARTDataReceived() that the data structure is locked
        for (int i=0; i<= 24; i++)
        {
            myRxBuf[i] = SBUS_receive_data.rxBuf[i];   	// we have a new packet, copy to myRxBuf for use outside of the USART ISR.
        }
        SBUS_receive_data.newPacketReceived = 0;		// reset the new packet flag
        SBUS_receive_data.lock = 0;						// unlock the data structure
        SBusToChannels();                               // convert S.BUS data to channel data
        return 0;
    }

    void Sbus::SBusToChannels(void) 
    {
        // The S.BUS has two flags, failsafe activated and frame lost
        channels[0]  = (uint16_t) myRxBuf[23] & 0x0C;     // failsafe activated (0x08), frame lost - equivalent to red LED on receiver (0x04) 

        // Each channel (1-16) is encoded into 11 bits on S.BUS.  The encoding is somewhat strange and messy to decode.
        // By observation with a FrSky Taranis X9D-Plus transmitter and a FrSky XM receiver, channel low value is 172, mid value is 992, and high value is 1811
        // Note: FrSky Taranis X9D-Plus transmitter does not support digital channels 17-18.  
        // FYI: to normalize the values to a range 0-100 then y = (x - 172) * (100 / (1811 - 172)); or for range 0-1000 then y = (x - 172) * (1000 / (1811 - 172))
        channels[1]  = (uint16_t)((myRxBuf[1]    | myRxBuf[2]<<8)                   & 0x07FF);
        channels[2]  = (uint16_t)((myRxBuf[2]>>3 | myRxBuf[3]<<5)                   & 0x07FF); 
        channels[3]  = (uint16_t)((myRxBuf[3]>>6 | myRxBuf[4]<<2 | myRxBuf[5]<<10)  & 0x07FF); 
        channels[4]  = (uint16_t)((myRxBuf[5]>>1 | myRxBuf[6]<<7)                   & 0x07FF); 
        channels[5]  = (uint16_t)((myRxBuf[6]>>4 | myRxBuf[7]<<4)                   & 0x07FF); 
        channels[6]  = (uint16_t)((myRxBuf[7]>>7 | myRxBuf[8]<<1 | myRxBuf[9]<<9)   & 0x07FF); 
        channels[7]  = (uint16_t)((myRxBuf[9]>>2 | myRxBuf[10]<<6)                  & 0x07FF); 
        channels[8]  = (uint16_t)((myRxBuf[10]>>5| myRxBuf[11]<<3)                  & 0x07FF); 
        channels[9]  = (uint16_t)((myRxBuf[12]   | myRxBuf[13]<<8)                  & 0x07FF); 
        channels[10] = (uint16_t)((myRxBuf[13]>>3| myRxBuf[14]<<5)                  & 0x07FF); 
        channels[11] = (uint16_t)((myRxBuf[14]>>6| myRxBuf[15]<<2| myRxBuf[16]<<10) & 0x07FF); 
        channels[12] = (uint16_t)((myRxBuf[16]>>1| myRxBuf[17]<<7)                  & 0x07FF); 
        channels[13] = (uint16_t)((myRxBuf[17]>>4| myRxBuf[18]<<4)                  & 0x07FF); 
        channels[14] = (uint16_t)((myRxBuf[18]>>7| myRxBuf[19]<<1| myRxBuf[20]<<9)  & 0x07FF); 
        channels[15] = (uint16_t)((myRxBuf[20]>>2| myRxBuf[21]<<6)                  & 0x07FF); 
        channels[16] = (uint16_t)((myRxBuf[21]>>5| myRxBuf[22]<<3)                  & 0x07FF); 

        // Channels 17-18 are digital channels (on/off) and are encode on S.BUS using single bits.
        if (myRxBuf[23] & 0x01) channels[17] = 1811; else channels[17] = 172;
        if (myRxBuf[23] & 0x02) channels[18] = 1811; else channels[18] = 172;

        // **************************************************************************************** //
        // ***  Rescale channel value from 172 - 1811 to 1000 - 2000 range                      *** //
        // **************************************************************************************** //
        for (int8_t i = 1; i <= 18; i++) 
        {
            // channels[i] = ((channels[i] - 172.0) * (1000.0 / (1811.0 - 172.0)) + 1000.0);
            channels[i] = (uint16_t) ((((float) channels[i] - 172.0) * 0.610128) + 1000.0);
            if ((channels[i] > 1995) && (channels[i] < 2005)) channels[i] = 2000;			// if it is close to 2000, then set it to 2000
        }

        // set throttle to channel 3 with 1400 - 1600 to 1500 to create a dead band throttle at center 
        if ((channels[3] >= 1400) && (channels[3] <= 1600)) Sbus::throttle = 1500;
        else Sbus::throttle = channels[3];

        // set speed to channel 2 with 1450 - 1550 to 1500 to create a dead band speed at center stick
        if ((channels[2] >= 1450) && (channels[2] <= 1550)) Sbus::speed = 1500;
        else Sbus::speed = channels[2];

        // set direction to channel 1 with 1400 - 1600 to 1500 to create a dead band direction at center 
        if ((channels[1] >= 1400) && (channels[1] <= 1600)) Sbus::direction = 1500;
        else Sbus::direction = channels[1];

        // set direction to channel 1 with 1400 - 1600 to 1500 to create a dead band direction at center 
        if ((channels[4] >= 1400) && (channels[4] <= 1600)) Sbus::channel4 = 1500;
        else Sbus::channel4 = channels[4];


        sbusStatus = channels[0];			// channels[0] is the SBUS status
        if ((sbusStatus == 0) && (channels[1] < 100)) sbusStatus = 99;
    }

    void Sbus::RC_monitor_task (void *parameters)
    {
                // #define configTICK_RATE_HZ                              ( CONFIG_FREERTOS_HZ )
                // CONFIG_FREERTOS_HZ=1000 default is 100, changed to 1000 (1 ms) TJB 2022-11-02 in sdkconfig
                // #define portTICK_PERIOD_MS              ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
        const TickType_t loopTime = 50 / portTICK_PERIOD_MS; // loop time in ms
        const uint32_t  loopsPerSecond = 1000 / loopTime;
        TickType_t      loopLastWakeTime;
        uint32_t        loopCount = 0;
        int16_t         rc1;

        ESP_LOGI(SbusLogTag, "Begin the RC_monitor_task - Rtos task");

        // Initialise the xLastWakeTime variable with the current time.
        loopLastWakeTime = xTaskGetTickCount();  // Init loopLastWakeTime with current time.
        for (;;)
        {
            loopCount++;
            // ESP_LOGI(SbusLogTag, "1 - For loop RC_monitor time %i %i %i %i ", 
            //     xTaskGetTickCount(), loopLastWakeTime, portTICK_PERIOD_MS, loopTime);

            if (loopCount % (loopsPerSecond / 4) == 0) // Every .25 seconds (250ms)
            {
                // include tasks for every .25 seconds here
                rc1 = ProcessSBUSData();
                rcProcessSBUSData[rc1]++;

                // ESP_LOGI(SbusLogTag, "2 - RC_monitor time %i %i %i", loopCount, loopsPerSecond, loopTime);
            }

            // ESP_LOGI(SbusLogTag, "2 - For loop RC_monitor time %i %i %i %i", 
            //     xTaskGetTickCount(), loopLastWakeTime, portTICK_PERIOD_MS, loopTime);

            
            //----------------------- End of Loop Processing ----------------------//
            vTaskDelayUntil( &loopLastWakeTime, loopTime );
        }  // end of for(;;)
    };

    void Sbus::SBUSPrintDebugMsg1(void)
    {
        char str[200];
        char strtmp[200];
        ESP_LOGI(SbusLogTag,"                           ");
        strcpy(str, "");
        sprintf(strtmp,"sbusStatus %i - ", sbusStatus);
        strcat(str,strtmp);
        for (int i = 0; i < 18; i++) 
        {
            // ESP_LOGI(SbusLogTag,"****************  Begin ProcessUARTDataReceived() ***************************");
            sprintf(strtmp,"%i ", channels[i]);
            strcat(str,strtmp);
            // Serial.print(channels[i]); Serial.print(" ");
        }
        ESP_LOGI(SbusLogTag, "%s", str);
        strcpy(str, "");

        // return;         // return here to only print channels

        for (int i = 0; i < 25; i++) 
        {
            sprintf(strtmp, "%i ", SBUS_receive_data.rxBuf[i]);
            strcat(str,strtmp);
        }
        // Serial.println();
        ESP_LOGI(SbusLogTag, "%s", str);
        strcpy(str, "");

        sprintf(str,"  uart2ChrCnt: %i  dataByte: %i", uart2ChrCnt, dataByte);
        ESP_LOGI(SbusLogTag, "%s", str);
        strcpy(str, "");

        #if DEBUG_SBUS

        sprintf(str, "  tmpDB1: %i", tmpDB1);
        ESP_LOGI(SbusLogTag, "%s", str);
        strcpy(str, "");

        #endif // #if DEBUG_SBUS

        sprintf(str, "  rcProcessSBUSData: rc0 - %i rc1 - %i rc2 - %i",
            rcProcessSBUSData[0], rcProcessSBUSData[1], rcProcessSBUSData[2]);
        ESP_LOGI(SbusLogTag, "%s", str);
        strcpy(str, "");

        #if DEBUG_SBUS

        sprintf(str, "  sbusDebugCntrs: cntr0 - %i cntr1 - %i  cntr2 - %i  cntr3 - %i", 
            sbusDebugCntrs[0], sbusDebugCntrs[1], sbusDebugCntrs[2], sbusDebugCntrs[3]);
        ESP_LOGI(SbusLogTag, "%s", str);
        strcpy(str, "");

        #endif // #if DEBUG_SBUS
    }

} // Namespace SBUS_COMPONENT