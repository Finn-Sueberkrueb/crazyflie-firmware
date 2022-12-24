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
#include "deck_core.h"

#define DEBUG_MODULE "Flyonic"
#include "debug.h"
#include <inttypes.h>


#define BUFFER_SIZE 56
#define EPS32_CS_PIN DECK_GPIO_IO4




//static t_externalState sendState;
//static t_extrenalActuator reciveAction;
//static t_externalState reciveAction;


  //t_externalState *reciveActionPtr = malloc(sizeof(t_externalState));
static uint8_t spiTxBuffer[BUFFER_SIZE];
static uint8_t spiRxBuffer[BUFFER_SIZE];
static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ; // SPI_BAUDRATE_21MHZ



void appMain() {

  pinMode(EPS32_CS_PIN, INPUT);
  spiBeginSlave();

  memset(&spiTxBuffer, 0x02, BUFFER_SIZE);
  memset(&spiRxBuffer, 0x03, BUFFER_SIZE);


  spiBeginTransactionSlave(spiSpeed);
  DEBUG_PRINT("SPI for ESP32 sarted\n");
  while(1) {
    
    
    //spiExchangeSlave(sizeof(t_externalState), (uint8_t *)(&sendState), (uint8_t *)(&reciveAction));
    spiExchangeSlave(BUFFER_SIZE, (uint8_t *)(&spiTxBuffer), (uint8_t *)(&spiRxBuffer));
    DEBUG_PRINT("recived: %d %d %d %d\n", spiRxBuffer[0], spiRxBuffer[1], spiRxBuffer[2], spiRxBuffer[3]);
    DEBUG_PRINT("send: %d %d %d %d\n", spiTxBuffer[0], spiTxBuffer[1], spiTxBuffer[2], spiTxBuffer[3]);
    
    //spiTxBuffer
    memcpy(&spiTxBuffer, &spiRxBuffer, BUFFER_SIZE);
    

  }

  spiEndTransaction();
}

