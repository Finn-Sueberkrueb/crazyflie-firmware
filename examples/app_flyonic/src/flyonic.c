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

static SemaphoreHandle_t pickupActuatorPackage;
static SemaphoreHandle_t spiFree;
#define handshake_TASK_PRIORITY 2
#define sendState_PERIOD_MS pdMS_TO_TICKS( 1000 )
static void vSendStateCallback( TimerHandle_t xTimer );


// TODO: define Buffer size
#define BUFFER_SIZE sizeof(t_externalState)
#define EPS32_CS_PIN DECK_GPIO_IO4
#define GPIO_HANDSHAKE DECK_GPIO_IO1



//  #define EXTI_PortSource EXTI_PortSourceGPIOC
//  #define EXTI_PinSource 	EXTI_PinSource11
//  #define EXTI_LineN 		  EXTI_Line11


static uint8_t spiTxBufferState[sizeof(t_externalState)];
static uint8_t spiRxBufferState[sizeof(t_externalState)];
static uint8_t spiTxBufferActor[sizeof(t_extrenalActuator)];
static uint8_t spiRxBufferActor[sizeof(t_extrenalActuator)];


// TODO: increase SPI speed
static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ; // SPI_BAUDRATE_21MHZ


/* Interrupt service routine, call the interrupt callback */
void __attribute__((used)) EXTI8_Callback(void)
{

  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR(pickupActuatorPackage, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }

  
}

static void handshake_task(void* arg)
{
    for(;;) {
        if(xSemaphoreTake(pickupActuatorPackage, portMAX_DELAY) == pdTRUE) {
          xSemaphoreTake(spiFree, portMAX_DELAY);

          //spiReciveMaster(sizeof(t_extrenalActuator), (uint8_t *)(&spiRxBufferActor));
          spiExchange(sizeof(t_extrenalActuator), (uint8_t *)(&spiRxBufferActor), (uint8_t *)(&spiRxBufferActor));


          //spiExchange(sizeof(t_extrenalActuator), (uint8_t *)(&spiTxBufferActor), (uint8_t *)(&spiRxBufferActor));

          t_extrenalActuator *reciveActionPtr = (t_extrenalActuator *)((uint8_t*)&spiRxBufferActor);

          //DEBUG_PRINT("time: %"PRId64"\n", reciveActionPtr->timestamp);
          //DEBUG_PRINT("status: %d\n", reciveActionPtr->status);
          DEBUG_PRINT("recived frame: %u\n", reciveActionPtr->frame);
          //DEBUG_PRINT("motor1: %u\n", reciveActionPtr->motor_1);
          //DEBUG_PRINT("motor2: %u\n", reciveActionPtr->motor_2);
          //DEBUG_PRINT("motor3: %u\n", reciveActionPtr->motor_3);
          //DEBUG_PRINT("motor4: %u\n", reciveActionPtr->motor_4);
          
          //uint64_t latency = usecTimestamp() - recive_action->timestamp;
          //lost frames = ???? recive_action->frame;

          uint64_t LastExternalLatency;
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

          //DEBUG_PRINT("%.3f - %u - %u\n", LastExternalLatency/1000.0, reciveActionPtr->frame, sendStatePtr->frame);
          DEBUG_PRINT("%.3f\n", LastExternalLatency/1000.0);
          xSemaphoreGive(spiFree);
        }
    }
}




static void handshakeInterruptInit(void)
{

  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  // Enable the interrupt on PB8 = deck IO1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //GPIO_HANDSHAKE 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);

  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  portDISABLE_INTERRUPTS();
  EXTI_Init(&EXTI_InitStructure);
  EXTI_ClearITPendingBit(EXTI_Line8);
  portENABLE_INTERRUPTS();
}


// function is called when the timer is expired
static void vSendStateCallback( TimerHandle_t xTimer )
{
  
    xSemaphoreTake(spiFree, portMAX_DELAY);
    t_externalState *sendStatePtr = (t_externalState *)(((uint8_t*)&spiTxBufferState));
    getCrazyflieState(sendStatePtr);
    //DEBUG_PRINT("done send to CF\n");
    


    //bool result = spiSendThanReciveSlave(sizeof(t_externalState), (uint8_t *)(&spiTxBufferState), sizeof(t_extrenalActuator)+1, (uint8_t *)(&spiRxBufferActor), DT);


    

    //DEBUG_PRINT("spiTxBufferState 0-7: %d %d %d %d %d %d %d %d\n", spiTxBufferState[0], spiTxBufferState[1], spiTxBufferState[2], spiTxBufferState[3], spiTxBufferState[4], spiTxBufferState[5], spiTxBufferState[6], spiTxBufferState[7]);
    //spiSendSlave(sizeof(t_externalState), (uint8_t *)(&spiTxBufferState));
    //bool spiReciveMaster(size_t length_rx, uint8_t * data_rx);


    DEBUG_PRINT("send frame: %u\n", sendStatePtr->frame );
    //spiSendMaster(sizeof(t_externalState), (uint8_t *)(&spiTxBufferState));
    spiExchange(sizeof(t_externalState), (uint8_t *)(&spiTxBufferState), (uint8_t *)(&spiTxBufferState));
    
    DEBUG_PRINT("send\n");
    xSemaphoreGive(spiFree);

}



void appInit() {

  handshakeInterruptInit();

  pickupActuatorPackage = xSemaphoreCreateBinary();
  spiFree = xSemaphoreCreateBinary();

  spiBegin();

  memset(&spiTxBufferState, 0x06, sizeof(t_externalState));
  memset(&spiRxBufferState, 0x05, sizeof(t_externalState));
  memset(&spiTxBufferActor, 0x02, sizeof(t_extrenalActuator));
  memset(&spiRxBufferActor, 0x03, sizeof(t_extrenalActuator));
  

  spiBeginTransaction(spiSpeed);



  xTaskCreate(handshake_task,   // task function
              "ESP_handshake", // Task name
              2*configMINIMAL_STACK_SIZE,
              NULL, 
              handshake_TASK_PRIORITY, // priority 5 = higest
              NULL );



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

  xSemaphoreGive(spiFree);
  DEBUG_PRINT("APP init finished\n");
  }