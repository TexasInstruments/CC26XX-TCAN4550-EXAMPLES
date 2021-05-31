/*
 * Copyright (c) 2015-2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== can_task.c ========
 */
#include <string.h>
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>

/* Example/Board Header files */
#include "ti_drivers_config.h"
#include "bsp_spi.h"
#include "can_task.h"
#include "can_hw.h"
#include "can_hw_config.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define CAN_MSG_1_EVT                         Event_Id_00

#define CAN_ALL_EVENTS                        (CAN_MSG_1_EVT)

// Task configuration
#define CAN_TASK_PRIORITY            1
#define CAN_TASK_STACK_SIZE          1024

uint8_t canTaskStack[CAN_TASK_STACK_SIZE];
Task_Struct canTask;

Event_Struct canEvent;
static Event_Handle canEventHandle;

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8_t  msg_rx_Data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t  msg_rx_DLC;
uint32_t msg_rx_ID;

uint8_t msg_tx_1_Data[8] = {1, 2, 3, 4, 5, 6, 7, 8};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void CAN_taskFxn(UArg a0, UArg a1);
static void TCAN4550_irqHandler(void);

/***********************************************************
  Function: CAN_createTasks
 */
void CAN_createTasks(void)
{
    Task_Params taskParams;
    Event_Params eventParam;

    Event_Params_init(&eventParam);
    Event_construct(&canEvent, &eventParam);
    canEventHandle = Event_handle(&canEvent);

    // Configure tasks receive
    Task_Params_init(&taskParams);
    taskParams.stack = canTaskStack;
    taskParams.stackSize = CAN_TASK_STACK_SIZE;
    taskParams.priority = CAN_TASK_PRIORITY;
    Task_construct(&canTask, CAN_taskFxn, &taskParams, NULL);
}

/***********************************************************
  Function: CAN_taskFxn
 */
void CAN_taskFxn(UArg a0, UArg a1)
{
    uint32_t storeEvents = 0;
    uint32_t events;

    /* Call driver init functions */
    GPIO_init();
    SPI_init();
    bspSpiOpen();

    /* TCAN4550 IRQ init */
    GPIO_setCallback(Board_TCAN4550_IRQ, (GPIO_CallbackFxn) TCAN4550_irqHandler);
    GPIO_clearInt(Board_TCAN4550_IRQ);
    GPIO_enableInt(Board_TCAN4550_IRQ);

    /* Initialize CAN Stack */
    Init_CAN();

    while(1)
    {
        events = Event_pend(canEventHandle, Event_Id_NONE, CAN_ALL_EVENTS,
                            10000);
        if (events)
        {
            if (events & CAN_MSG_1_EVT)
            {
                storeEvents ^= CAN_MSG_1_EVT;
            }
        }

        msg_tx_1_Data[0] = (uint8_t)storeEvents;
        HW_Tx_Msg(0x123, 0x08, msg_tx_1_Data);
        GPIO_toggle(CONFIG_GPIO_LED_0);
    }
}

/*********************************************************************
 * @fn      CAN_msg_1_Handler
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void CAN_msg_1_Handler(void)
{
    Event_post(canEventHandle, CAN_MSG_1_EVT);
}

/***********************************************************
  Function: TCAN4550_irqHandler
 */
static void TCAN4550_irqHandler(void)
{
    HW_ISR_Rx(&msg_rx_ID, &msg_rx_Data[0], &msg_rx_DLC);

    if (msg_rx_ID == CAN_RX_ID_MSG)
    {
        CAN_msg_1_Handler();
        GPIO_toggle(CONFIG_GPIO_LED_1);
    }

    GPIO_clearInt(Board_TCAN4550_IRQ);
}
