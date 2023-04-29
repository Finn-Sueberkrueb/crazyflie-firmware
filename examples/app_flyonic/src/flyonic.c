/**
 * Finn Süberkrüb - Crazyflie@rpex.de
 *
 * flyonic.c - App layer application for the Flyonic project
 */

#include "FreeRTOS.h"
#include "stm32fxxx.h"
#include "config.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "exti.h"
#include "semphr.h"
#include "nvicconf.h"
#include "timers.h"   // Software timer related API prototypes. 
#include "uart2.h"

#include "system.h"
#include "stabilizer_types.h"
#include "stabilizer.h"

#include "app.h"
#include "task.h"
#include "deck.h"
#include "deck_core.h"

#define DEBUG_MODULE "Flyonic"
#include "debug.h"
#include <inttypes.h>

//#define SEND_LOOP

#if defined(SEND_LOOP)
#define sendState_PERIOD_MS pdMS_TO_TICKS( 2 )
static void vSendStateCallback( TimerHandle_t xTimer );
static uint8_t uartTxBufferState[sizeof(t_externalState)];
#endif


t_extrenalActuator *reciveActionPtr;



#if defined(SEND_LOOP)
// function is called when the timer is expired
static void vSendStateCallback( TimerHandle_t xTimer )
{
  
    //memset(&uartTxBufferState, '+', 3);
    t_externalState *sendStatePtr = (t_externalState *)(((uint8_t*)&uartTxBufferState));
    getCrazyflieState(sendStatePtr);

    // TODO: delete
    sendStatePtr->timestamp = usecTimestamp();
    
    uart2SendDataDmaBlocking(sizeof(t_externalState), (uint8_t *)(&uartTxBufferState));
    //DEBUG_PRINT("send frame: %u\n", sendStatePtr->frame );

}
#endif



static void ReciveUARTActuator(void *arg)
{
  char byte1 = '0';
  char byte2 = '0';
  char byte3 = '0';

  for(;;) {
    // Wait for start!
    byte1 = '0';
    byte2 = '0';
    byte3 = '0';
    do
    {
      byte1 = byte2;
      byte2 = byte3;
      uart2GetDataWithTimeout(1, &byte3, M2T(200));
    } while ((byte1 != 'E') && (byte2 != 'S') && (byte3 != 'P'));

    if((byte1 == 'E') && (byte2 == 'S') && (byte3 == 'P')){
      //DEBUG_PRINT("found ESP");

      uart2GetData(sizeof(t_extrenalActuator), (uint8_t*)reciveActionPtr);

        // calculate LRC
    	//char *send_data_ptr = (char *)&reciveActionPtr;
        uint8_t LRC = 0;
        for (uint16_t i = 0; i < (sizeof(t_extrenalActuator) - 1); i++) {
            LRC ^= *((uint8_t *)reciveActionPtr + i);
        }

        //uint8_t crc;
        //uart2GetData(1, &crc);
        //ASSERT(crc == calcCrc(&uartRxp));

        // only use data if LRC is correct
        if (LRC == reciveActionPtr->LRC) {

            //uint64_t LastExternalLatency;
            // TODO: accept only if timestamp is not older than 5ms
            if((reciveActionPtr->status == 1) && (reciveActionPtr->timestamp >= usecTimestamp() - 5000)) {
                // 1 = external controll active
                systemSetExternalControl(true);
                setExternelMotorThrustUncapped(reciveActionPtr->motor_1, reciveActionPtr->motor_2, reciveActionPtr->motor_3, reciveActionPtr->motor_4, reciveActionPtr->frame);
                CalculateExternalLatency(reciveActionPtr->timestamp);
            } else {
                // external controll inactive
                //systemSetExternalControl(false);
                //CalculateExternalLatency(reciveActionPtr->timestamp);

            }

            //DEBUG_PRINT("time: %"PRId64"\n", reciveActionPtr->timestamp);
            //DEBUG_PRINT("status: %d\n", reciveActionPtr->status);
            //DEBUG_PRINT("recived frame: %u\n", reciveActionPtr->frame);
            //DEBUG_PRINT("motor1: %u\n", reciveActionPtr->motor_1);
            //DEBUG_PRINT("motor2: %u\n", reciveActionPtr->motor_2);
            //DEBUG_PRINT("motor3: %u\n", reciveActionPtr->motor_3);
            //DEBUG_PRINT("motor4: %u\n", reciveActionPtr->motor_4);
            //DEBUG_PRINT("%.3f\n", LastExternalLatency/1000.0);
        }
    }
  }
}


void appInit() {

  // initialize UART with baud rate
  uart2Init(2500000);

  reciveActionPtr = malloc(sizeof(t_extrenalActuator));

#if defined(SEND_LOOP)
  TimerHandle_t xSendStateSoftwareTimer = NULL;
  
  // Create a software timer for sending the state
  xSendStateSoftwareTimer = xTimerCreate(( const char * ) "SendStateTimer", // Time name
                                        sendState_PERIOD_MS, // timer call period in ms 
                                        pdTRUE, //  periodic timer, so xAutoReload = pdTRUE
                                        ( void * ) 0, // timer ID
                                        vSendStateCallback // callback name
                                        );

  // Start the created timer.  A block time of zero
  xTimerStart( xSendStateSoftwareTimer, 0 );
#endif

  xTaskCreate(ReciveUARTActuator, "ReciveUARTActuator", 2048, NULL, configMAX_PRIORITIES, NULL);


  //handshakeInterruptInitxSemaphoreGive(spiFree);
  DEBUG_PRINT("APP init finished\n");
  }