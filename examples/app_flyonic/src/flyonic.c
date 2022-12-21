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

#include "stabilizer.h"
#include "stabilizer_types.h"

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"

#define DEBUG_MODULE "Flyonic"
#include "debug.h"
#include <inttypes.h>


#define BUFFER_SIZE 56
#define EPS32_CS_PIN DECK_GPIO_IO4
#define EPS32_HANDSHAKE_PIN DECK_GPIO_IO3




static t_externalState sendState;
//static t_extrenalActuator reciveAction;
static t_externalState reciveAction;


  //t_externalState *reciveActionPtr = malloc(sizeof(t_externalState));
//static uint8_t spiTxBuffer[BUFFER_SIZE];
//static uint8_t spiRxBuffer[BUFFER_SIZE];
static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ; // SPI_BAUDRATE_21MHZ



void appMain() {

  pinMode(EPS32_CS_PIN, OUTPUT);
  pinMode(EPS32_HANDSHAKE_PIN, INPUT);
  spiBegin();

  //memset(&spiTxBuffer, 0x02, BUFFER_SIZE);
  //memset(&spiRxBuffer, 0x03, BUFFER_SIZE);

  bool send_next = true;
  uint64_t startetime = usecTimestamp();


  



  spiBeginTransaction(spiSpeed);
  DEBUG_PRINT("SPI for ESP32 sarted\n");
  while(1) {
    

    if(send_next){
      startetime = usecTimestamp();
      sendState.timestamp = startetime;

      send_next = false;
    }



    //DEBUG_PRINT("send: %d %d\n", spiTxBuffer[0], spiTxBuffer[1]);


    if(digitalRead(EPS32_HANDSHAKE_PIN) == HIGH){
      DEBUG_PRINT("Handshake high\n");
    } else {
      DEBUG_PRINT("Handshake low\n");
    }

    
    digitalWrite(EPS32_CS_PIN, LOW);
    spiExchange(sizeof(t_externalState), (uint8_t *)(&sendState), (uint8_t *)(&reciveAction));
    digitalWrite(EPS32_CS_PIN, HIGH);


    vTaskDelay(M2T(1));

    //itoa(sendState.timestamp,spiTxBuffer,2);
    //DEBUG_PRINT("%s (send)\n", spiTxBuffer); 

    //itoa(reciveAction.timestamp,spiRxBuffer,2);
    //DEBUG_PRINT("%s (recived)\n", spiRxBuffer);


    if(reciveAction.timestamp == startetime){
      uint64_t stoptime = usecTimestamp(); 
      DEBUG_PRINT("latency in ms %.3f (send)\n", (stoptime - reciveAction.timestamp)/1000.0); 
      DEBUG_PRINT("latency in ms %.3f\n", (stoptime - startetime)/1000.0);
      send_next = true;
      vTaskDelay(M2T(500));
    }

    //DEBUG_PRINT("recived: %d %d\n", spiRxBuffer[0], spiRxBuffer[1]);
  }

  spiEndTransaction();
}

