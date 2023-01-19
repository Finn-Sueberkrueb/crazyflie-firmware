/**
 * Finn Süberkrüb - Crazyflie@rpex.de
 *
 * flyonic.c - App layer application for the Flyonic project
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "system.h"
#include "stabilizer_types.h"
#include "stabilizer.h"

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "deck_core.h"

#define DEBUG_MODULE "Flyonic"
#include "debug.h"
#include <inttypes.h>


// TODO: define Buffer size
#define BUFFER_SIZE sizeof(t_externalState)
#define EPS32_CS_PIN DECK_GPIO_IO4
#define GPIO_HANDSHAKE DECK_GPIO_IO1



//static t_externalState sendState;
//static t_extrenalActuator reciveAction;
//static t_externalState reciveAction;


static uint8_t spiTxBufferState[sizeof(t_externalState)];
static uint8_t spiRxBufferState[sizeof(t_externalState)];
static uint8_t spiTxBufferActor[sizeof(t_extrenalActuator)];
static uint8_t spiRxBufferActor[sizeof(t_extrenalActuator)];
// TODO: increase SPI speed
//static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ; // SPI_BAUDRATE_21MHZ



void appMain() {

  pinMode(EPS32_CS_PIN, INPUT);
  spiBeginSlave();

  memset(&spiTxBufferState, 0x06, sizeof(t_externalState));
  memset(&spiRxBufferState, 0x05, sizeof(t_externalState));

  memset(&spiTxBufferActor, 0x02, sizeof(t_extrenalActuator));
  memset(&spiRxBufferActor, 0x03, sizeof(t_extrenalActuator));
  
  
  //t_externalState *sendStatePtr = malloc(sizeof(t_externalState));
  //t_extrenalActuator *reciveActionPtr = malloc(sizeof(t_extrenalActuator));
  //t_extrenalActuator *reciveActionPtr = (t_extrenalActuator *)(&spiRxBuffer + 1);
  
  //int64_t last_latency = 0;



  pinMode(GPIO_HANDSHAKE, OUTPUT);     // Set Handshake line
  digitalWrite(GPIO_HANDSHAKE, LOW);  // Handshake LOW (no data for master available)


  spiBeginTransactionSlave();

  uint64_t LastExternalLatency;
  
  //DEBUG_PRINT("send buffer size: %d\n", BUFFER_SIZE);
  DEBUG_PRINT("SPI for ESP32 sarted\n");
  // TODO: put in task
  while(1) {

    //memset(&spiTxBufferState, 0x06, sizeof(t_externalState));
    t_externalState *sendStatePtr = (t_externalState *)((uint8_t*)&spiTxBufferState);
    getCrazyflieState(sendStatePtr);

    //DEBUG_PRINT("spiTxBufferState 0-7: %d %d %d %d %d %d %d %d\n", spiTxBufferState[0], spiTxBufferState[1], spiTxBufferState[2], spiTxBufferState[3], spiTxBufferState[4], spiTxBufferState[5], spiTxBufferState[6], spiTxBufferState[7]);
    spiSendSlave(sizeof(t_externalState), (uint8_t *)(&spiTxBufferState));

    //spiExchangeSlave(sizeof(t_externalState), (uint8_t *)(&spiTxBuffer), (uint8_t *)(&spiRxBuffer));





    //DEBUG_PRINT("done send to CF\n");
    //uint64_t outTimestamp_2 = usecTimestamp();
    //vTaskDelay(10 / portTICK_PERIOD_MS);

    //memset(&spiRxBufferActor, 0x03, sizeof(t_extrenalActuator));

    
    bool result = spiReciveSlave(sizeof(t_extrenalActuator), (uint8_t *)(&spiRxBufferActor), 20);//(10 / portTICK_PERIOD_MS));
    //uint64_t LastExternalLatency_2 = usecTimestamp() - outTimestamp_2;

    //DEBUG_PRINT("latency for wait: %.6f ms sucess = %d , wait %ld \n", LastExternalLatency_2/1000.0, result, (10000 / portTICK_PERIOD_MS));
    


    if(result){
    //DEBUG_PRINT("spiRxBufferActor 0-7: %d %d %d %d %d %d %d %d\n", spiRxBufferActor[0], spiRxBufferActor[1], spiRxBufferActor[2], spiRxBufferActor[3], spiRxBufferActor[4], spiRxBufferActor[5], spiRxBufferActor[6], spiRxBufferActor[7]);

    //}
    
    

    //spiExchangeSlave(BUFFER_SIZE, (uint8_t *)(&spiTxBuffer), (uint8_t *)(&spiRxBuffer));
    //DEBUG_PRINT("recived 0-7: %d %d %d %d %d %d %d %d\n", spiRxBuffer[0], spiRxBuffer[1], spiRxBuffer[2], spiRxBuffer[3], spiRxBuffer[4], spiRxBuffer[5], spiRxBuffer[6], spiRxBuffer[7]);
    //memcpy(&spiTxBuffer, ((uint8_t *)&spiRxBuffer + 1), BUFFER_SIZE);



    //t_externalState *sendStatePtr = (t_externalState *)((uint8_t*)&spiTxBuffer);



    //DEBUG_PRINT("send 7-13: %d %d %d %d %d %d %d\n", spiTxBuffer[7], spiTxBuffer[8], spiTxBuffer[9], spiTxBuffer[10], spiTxBuffer[11], spiTxBuffer[12], spiTxBuffer[13]);


    //portMAX_DELAY
    //0x00fffffUL
    //spiExchangeSlave(sizeof(t_externalState), (uint8_t *)(&spiTxBuffer), (uint8_t *)(&spiRxBuffer), false, 10/portTICK_PERIOD_MS);
    //spiExchangeSlave(sizeof(t_extrenalActuator), (uint8_t *)(&spiTxBufferActor), (uint8_t *)(&spiRxBufferActor), false, 10/portTICK_PERIOD_MS);

 //   if(true) {
      //DEBUG_PRINT("recived new actuator\n");

      //spiExchangeSlave(BUFFER_SIZE, (uint8_t *)(sendStatePtr), (uint8_t *)(reciveActionPtr));
      //DEBUG_PRINT("recived 0-7: %d %d %d %d %d %d %d\n", spiRxBuffer[0], spiRxBuffer[8], spiRxBuffer[9], spiRxBuffer[10], spiRxBuffer[11], spiRxBuffer[12], spiRxBuffer[13]);

      t_extrenalActuator *reciveActionPtr = (t_extrenalActuator *)((uint8_t*)&spiRxBufferActor + 1);

      // TODO: this bit is somehow transmittet in the SPI connection to much.

      //DEBUG_PRINT("time: %"PRId64"\n", reciveActionPtr->timestamp);
      //DEBUG_PRINT("status: %d\n", reciveActionPtr->status);
      //DEBUG_PRINT("frame: %u\n", reciveActionPtr->frame);
      //DEBUG_PRINT("motor1: %u\n", reciveActionPtr->motor_1);
      //DEBUG_PRINT("motor2: %u\n", reciveActionPtr->motor_2);
      //DEBUG_PRINT("motor3: %u\n", reciveActionPtr->motor_3);
      //DEBUG_PRINT("motor4: %u\n", reciveActionPtr->motor_4);
    
      //uint64_t latency = usecTimestamp() - recive_action->timestamp;
      //lost frames = ???? recive_action->frame;


      if(reciveActionPtr->status == 1) {
        // 1 = external controll active
        systemSetExternalControl(true);
        setExternelMotorThrustUncapped(reciveActionPtr->motor_1, reciveActionPtr->motor_2, reciveActionPtr->motor_3, reciveActionPtr->motor_4, reciveActionPtr->frame);
        LastExternalLatency = CalculateExternalLatency(reciveActionPtr->timestamp);
      } else {
        // external controll inactive
        systemSetExternalControl(false);
        LastExternalLatency = CalculateExternalLatency(reciveActionPtr->timestamp);

      }

      DEBUG_PRINT("%.3f\n", LastExternalLatency/1000.0);

      



   } else {
      DEBUG_PRINT("timeout\n");
      //DEBUG_PRINT("-1\n");
    }


    float DT = 10.0;
    float time_to_wait_in_ms = DT - (usecTimestamp() - sendStatePtr->timestamp)/1000;

    if(time_to_wait_in_ms < DT){
      vTaskDelay(time_to_wait_in_ms / portTICK_PERIOD_MS);
    }

  }
  spiEndTransaction();
}

