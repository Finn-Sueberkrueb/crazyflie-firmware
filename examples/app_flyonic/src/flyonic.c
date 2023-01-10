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



//static t_externalState sendState;
//static t_extrenalActuator reciveAction;
//static t_externalState reciveAction;


static uint8_t spiTxBuffer[BUFFER_SIZE];
static uint8_t spiRxBuffer[BUFFER_SIZE];
// TODO: increase SPI speed
//static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ; // SPI_BAUDRATE_21MHZ



void appMain() {

  pinMode(EPS32_CS_PIN, INPUT);
  spiBeginSlave();

  memset(&spiTxBuffer, 0x02, BUFFER_SIZE);
  memset(&spiRxBuffer, 0x03, BUFFER_SIZE);
  
  //t_externalState *sendStatePtr = malloc(sizeof(t_externalState));
  //t_extrenalActuator *reciveActionPtr = malloc(sizeof(t_extrenalActuator));
  //t_extrenalActuator *reciveActionPtr = (t_extrenalActuator *)(&spiRxBuffer + 1);
  
  //int64_t last_latency = 0;

  spiBeginTransactionSlave();
  DEBUG_PRINT("SPI for ESP32 sarted\n");

  DEBUG_PRINT("send buffer size: %d\n", BUFFER_SIZE);
  while(1) {
    
    

    //spiExchangeSlave(BUFFER_SIZE, (uint8_t *)(&spiTxBuffer), (uint8_t *)(&spiRxBuffer));
    //DEBUG_PRINT("recived 0-7: %d %d %d %d %d %d %d %d\n", spiRxBuffer[0], spiRxBuffer[1], spiRxBuffer[2], spiRxBuffer[3], spiRxBuffer[4], spiRxBuffer[5], spiRxBuffer[6], spiRxBuffer[7]);
    //memcpy(&spiTxBuffer, ((uint8_t *)&spiRxBuffer + 1), BUFFER_SIZE);



    t_externalState *sendStatePtr = (t_externalState *)((uint8_t*)&spiTxBuffer);

    getCrazyflieState(sendStatePtr);
    /*
    sendStatePtr->timestamp = 1;
    sendStatePtr->pos_x = 2.0;
    sendStatePtr->pos_y = 3.0;
    sendStatePtr->pos_z = 4.0;
    sendStatePtr->vel_x = 5.0;
    sendStatePtr->vel_y = 6.0;
    sendStatePtr->vel_z = 7.0;
    sendStatePtr->acc_x = 8.0;
    sendStatePtr->acc_y = 9.0;
    sendStatePtr->acc_z = 10.0;
    sendStatePtr->q_1 = 11.0;
    sendStatePtr->rot_x = 12.0;
    sendStatePtr->rot_y = 13.0;
    sendStatePtr->rot_z = 14.0;
    sendStatePtr->rot_vel_x = 15.0;
    sendStatePtr->rot_vel_y = 16.0;
    sendStatePtr->rot_vel_z = 17.0;
    sendStatePtr->rot_acc_x = 18.0;
    sendStatePtr->rot_acc_y = 19.0;
    sendStatePtr->rot_acc_z = 20.0;
    sendStatePtr->latency = 21.0;
    sendStatePtr->motor_1 = 7;
    sendStatePtr->motor_2 = 6;
    sendStatePtr->motor_3 = 5;
    sendStatePtr->motor_4 = 4;
    sendStatePtr->frame = 3;
    sendStatePtr->lased_actuator_frame = 2;
    sendStatePtr->status = 1;
    */

    //DEBUG_PRINT("send 7-13: %d %d %d %d %d %d %d\n", spiTxBuffer[7], spiTxBuffer[8], spiTxBuffer[9], spiTxBuffer[10], spiTxBuffer[11], spiTxBuffer[12], spiTxBuffer[13]);

    spiExchangeSlave(BUFFER_SIZE, (uint8_t *)(&spiTxBuffer), (uint8_t *)(&spiRxBuffer));

    //spiExchangeSlave(BUFFER_SIZE, (uint8_t *)(sendStatePtr), (uint8_t *)(reciveActionPtr));
    //DEBUG_PRINT("recived 0-7: %d %d %d %d %d %d %d\n", spiRxBuffer[0], spiRxBuffer[8], spiRxBuffer[9], spiRxBuffer[10], spiRxBuffer[11], spiRxBuffer[12], spiRxBuffer[13]);

    t_extrenalActuator *reciveActionPtr = (t_extrenalActuator *)((uint8_t*)&spiRxBuffer);

    //reciveActionPtr->timestamp = 

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
      setExternelMotorThrustUncapped(reciveActionPtr->motor_1, reciveActionPtr->motor_2, reciveActionPtr->motor_3, reciveActionPtr->motor_4);
    } else {
      // external controll inactive
      systemSetExternalControl(false);
    }
    

  }

  spiEndTransaction();
}

