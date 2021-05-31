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
 
//#############################################################################
//
//! \file   can_hw.c
//!
//! \brief  CAN HW Initialization API's
//!         
//
//  Group:          CMCU
//  Target Device:  CC26xx
//
//  (C) Copyright 2019, Texas Instruments, Inc.
//#############################################################################

// File Includes
#ifndef USING_DRIVERLIB
#include <ti/sysbios/hal/Hwi.h>
#else
#include <driverlib/sys_ctrl.h>
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "can_task.h"
#include "can_hw.h"
#include "TCAN4550.h"

// Defines
#define CAN_DLC_MAX                      0x08
#define TCAN4550_MESSAGE_BUFFERS         64

// Initialize to 0 or you'll get garbage
TCAN4x5x_MCAN_RX_Header MsgHeader = {0};
uint8_t numBytes = 0;
uint8_t dataPayload[64] = {0};

// Prototype Declarations
static void    TCAN4550_SetConfiguration(void);
static void    TCAN4550_SetRxTxMessageBuffers(uint32_t CAN_msg_ID,
                                              CAN_ID_Type_t CAN_ID_type);
static void    TCAN4550_Start(void);

// Local Function Declarations
//****************************************************************************
// Name: Init_CAN
// Details:
//****************************************************************************
void Init_CAN(void)
{
   // Intialize all control registers
   TCAN4550_SetConfiguration();
   TCAN4550_Start();
}

//****************************************************************************
// Name: TCAN4550_SetConfiguration
// Details:
//****************************************************************************
static void TCAN4550_SetConfiguration( void )
{
    uint8_t tx_param_size;
    uint8_t rx_param_size;

    // Clear any SPI ERR flags that might be set as a result of our pin mux changing during MCU startup
    TCAN4x5x_Device_ClearSPIERR();

    /* Step one attempt to clear all interrupts */
    /* Initialize to 0 to all bits are set to 0 */
    TCAN4x5x_Device_Interrupt_Enable dev_ie = {0};

    /* Disable all non-MCAN related interrupts for simplicity */
    TCAN4x5x_Device_ConfigureInterruptEnable(&dev_ie);

    /* Setup a new MCAN IR object for easy interrupt checking */
    TCAN4x5x_Device_Interrupts dev_ir = {0};

    /* Request that the struct be updated with current DEVICE (not MCAN) interrupt values */
    TCAN4x5x_Device_ReadInterrupts(&dev_ir);

    /* If the Power On interrupt flag is set */
    if (dev_ir.PWRON)
    {
        /* Clear it because if it's not cleared within ~4 minutes, it goes to sleep */
        TCAN4x5x_Device_ClearInterrupts(&dev_ir);
    }

    /* Configure the CAN bus speeds */
    /* 500k arbitration with a 40 MHz crystal (40E6 / (57 + 22 + 1 (Sync)) = 500E3) */
    TCAN4x5x_MCAN_Nominal_Timing_Simple TCANNomTiming = {0};
    TCANNomTiming.NominalBitRatePrescaler = 2;
    TCANNomTiming.NominalTqBeforeSamplePoint = 32;
    TCANNomTiming.NominalTqAfterSamplePoint = 8;

    /* 2 Mbps CAN FD with a 40 MHz crystal (40E6 / (15 + 5) = 2E6) */
    TCAN4x5x_MCAN_Data_Timing_Simple TCANDataTiming = {0};
    TCANDataTiming.DataBitRatePrescaler = 1;
    TCANDataTiming.DataTqBeforeSamplePoint = 15;
    TCANDataTiming.DataTqAfterSamplePoint = 5;

    /* Configure the MCAN core settings */
    /* Remember to initialize to 0, or you'll get random garbage! */
    TCAN4x5x_MCAN_CCCR_Config cccrConfig = {0};

    /* CAN FD mode enable */
    cccrConfig.FDOE = 0;

    /* CAN FD Bit rate switch enable*/
    cccrConfig.BRSE = 1;

    /* Configure the default CAN packet filtering settings */
    TCAN4x5x_MCAN_Global_Filter_Configuration gfc = {0};

    /* Reject remote frames (TCAN4x5x doesn't support this) */
    gfc.RRFE = 1;

    /* Reject remote frames (TCAN4x5x doesn't support this) */
    gfc.RRFS = 1;

    /* Default behavior if incoming message doesn't match a filter is to accept into RXFIO0 for extended ID messages (29 bit IDs) */
    gfc.ANFE = TCAN4x5x_GFC_REJECT; //TCAN4x5x_GFC_ACCEPT_INTO_RXFIFO0;

    /* Default behavior if incoming message doesn't match a filter is to accept into RXFIO0 for standard ID messages (11 bit IDs) */
    gfc.ANFS = TCAN4x5x_GFC_REJECT; //TCAN4x5x_GFC_ACCEPT_INTO_RXFIFO0;

    /* ************************************************************************
     * In the next configuration block, we will set the MCAN core up to have:
     *   - 1 SID filter element
     *   - 1 XID Filter element
     *   - RX FIFO 0 elements -> depends on CAN_rx_msg_table
     *   - RX FIFO 0 supports data payloads up to 64 bytes
     *   - RX FIFO 1 and RX Buffer will not have any elements, but we still set
     *   their data payload sizes, even though it's not required
     *   - No TX Event FIFOs
     *   - TX buffers supporting up to 64 bytes of data payload -> depends on CAN_tx_msg_table
     */
    /* get the number of messages from the can_config.c table */
    rx_param_size = 1;
    tx_param_size = 1;

    TCAN4x5x_MRAM_Config MRAMConfiguration = {0};
    // Standard ID number of elements, you MUST have a filter written to MRAM for each element defined
    MRAMConfiguration.SIDNumElements = 1;
    // Extended ID number of elements, you MUST have a filter written to MRAM for each element defined
    MRAMConfiguration.XIDNumElements = 1;
    // RX0 Number of elements
    MRAMConfiguration.Rx0NumElements = rx_param_size;
    // RX0 data payload size (Use the defines)
    MRAMConfiguration.Rx0ElementSize = MRAM_64_Byte_Data;
    // RX1 number of elements
    MRAMConfiguration.Rx1NumElements = 0;
    // RX1 data payload size (use the defines)
    MRAMConfiguration.Rx1ElementSize = MRAM_64_Byte_Data;
    // RX buffer number of elements
    MRAMConfiguration.RxBufNumElements = 0;
    // RX buffer data payload size (use the defines)
    MRAMConfiguration.RxBufElementSize = MRAM_64_Byte_Data;
    // TX Event FIFO number of elements
    MRAMConfiguration.TxEventFIFONumElements = 0;
    // TX buffer number of elements
    MRAMConfiguration.TxBufferNumElements = tx_param_size;
    // TX buffer data payload size (use the defines)
    MRAMConfiguration.TxBufferElementSize = MRAM_64_Byte_Data;

    /* Configure the MCAN core with the settings above, the changes in this block are write protected registers,
     * so it makes the most sense to do them all at once, so we only unlock and lock once                             */
    // Start by making protected registers accessible
    TCAN4x5x_MCAN_EnableProtectedRegisters();
    // Enable FD mode and Bit rate switching
    TCAN4x5x_MCAN_ConfigureCCCRRegister(&cccrConfig);
    // Configure the global filter configuration (Default CAN message behavior)
    TCAN4x5x_MCAN_ConfigureGlobalFilter(&gfc);
    // Setup nominal/arbitration bit timing
    TCAN4x5x_MCAN_ConfigureNominalTiming_Simple(&TCANNomTiming);
    // Setup CAN FD timing
    TCAN4x5x_MCAN_ConfigureDataTiming_Simple(&TCANDataTiming);
    // Clear all of MRAM (Writes 0's to all of it)
    TCAN4x5x_MRAM_Clear();
    // Set up the applicable registers related to MRAM configuration
    TCAN4x5x_MRAM_Configure(&MRAMConfiguration);
    // Disable protected write and take device out of INIT mode
    TCAN4x5x_MCAN_DisableProtectedRegisters();

    /* Set the interrupts we want to enable for MCAN */
    /* Remember to initialize to 0, or you'll get random garbage! */
    TCAN4x5x_MCAN_Interrupt_Enable mcan_ie = {0};

    /* RX FIFO 0 new message interrupt enable */
    mcan_ie.RF0NE = 1;

    /* Enable the appropriate registers */
    TCAN4x5x_MCAN_ConfigureInterruptEnable(&mcan_ie);

    //Initialize receive and transmit buffers
    TCAN4550_SetRxTxMessageBuffers(CAN_RX_ID_MSG, CAN_RX_ID_MSG_TYPE);

    /* Configure the TCAN4550 Non-CAN-related functions */
    // Remember to initialize to 0, or you'll get random garbage!
    TCAN4x5x_DEV_CONFIG devConfig = {0};
    // Keep Sleep Wake Error Enabled (it's a disable bit, not an enable)
    devConfig.SWE_DIS = 0;
    // Not requesting a software reset
    devConfig.DEVICE_RESET = 0;
    // Watchdog disabled
    devConfig.WD_EN = 0;
    // Mirror INH function (default)
    devConfig.nWKRQ_CONFIG = 0;
    // INH enabled (default)
    devConfig.INH_DIS = 0;
    // MCAN nINT 1 (default)
    devConfig.GPIO1_GPO_CONFIG = TCAN4x5x_DEV_CONFIG_GPO1_MCAN_INT1;
    // Failsafe disabled (default)
    devConfig.FAIL_SAFE_EN = 0;
    // GPIO set as GPO (Default)
    devConfig.GPIO1_CONFIG = TCAN4x5x_DEV_CONFIG_GPIO1_CONFIG_GPO;
    // Watchdog set an interrupt (default)
    devConfig.WD_ACTION = TCAN4x5x_DEV_CONFIG_WDT_ACTION_nINT;
    // Don't reset the watchdog
    devConfig.WD_BIT_RESET = 0;
    // Set nWKRQ to internal voltage rail (default)
    devConfig.nWKRQ_VOLTAGE = 0;
    // GPO2 has no behavior (default)
    devConfig.GPO2_CONFIG = TCAN4x5x_DEV_CONFIG_GPO2_NO_ACTION;
    // Input crystal is a 40 MHz crystal (default)
    devConfig.CLK_REF = 1;
    // Wake pin can be triggered by either edge (default)
    devConfig.WAKE_CONFIG = TCAN4x5x_DEV_CONFIG_WAKE_BOTH_EDGES;
    // Configure the device with the above configuration
    TCAN4x5x_Device_Configure(&devConfig);
}

//****************************************************************************
// Name: TCAN4550_SetRxTxMessageBuffers
// Details:
//****************************************************************************
static void TCAN4550_SetRxTxMessageBuffers(uint32_t CAN_msg_ID,
                                           CAN_ID_Type_t CAN_ID_type)
{
    TCAN4x5x_MCAN_SID_Filter SID_ID = {0};
    TCAN4x5x_MCAN_XID_Filter XID_ID = {0};
    uint32_t id_32;

    id_32  = CAN_msg_ID;

    if ( CAN_ID_type == EXTENDED_ID )
    {
        // EFT
        XID_ID.EFT = TCAN4x5x_XID_EFT_CLASSIC;
        // EFEC
        XID_ID.EFEC = TCAN4x5x_XID_EFEC_PRIORITYSTORERX0;
        // EFID1 (Classic mode filter)
        XID_ID.EFID1 = id_32;
        // EFID2 (Classic mode mask)
        XID_ID.EFID2 = 0x1FFFFFFF;
        // Write to the MRAM
        TCAN4x5x_MCAN_WriteXIDFilter(0, &XID_ID);
    }
    else
    {
        // SFT: Standard filter type. Configured as a classic filter
        SID_ID.SFT = TCAN4x5x_SID_SFT_CLASSIC;
        // Standard filter element configuration, store it in RX fifo 0 as a priority message
        SID_ID.SFEC = TCAN4x5x_SID_SFEC_PRIORITYSTORERX0;
        // SFID1 (Classic mode Filter)
        SID_ID.SFID1 = id_32;
        // SFID2 (Classic mode Mask)
        SID_ID.SFID2 = 0x7FF;
        // Write to the MRAM
        TCAN4x5x_MCAN_WriteSIDFilter(0, &SID_ID);
    }
}

//****************************************************************************
// Name: TCAN4550_Start
// Details:
//****************************************************************************
static void TCAN4550_Start(void )
{
    /* Todo: Change to TCAN45x0_MODE_STANDBY */
    // Set to normal mode, since configuration is done. This line turns on the transceiver
    TCAN4x5x_Device_SetMode(TCAN4x5x_DEVICE_MODE_NORMAL);

    /* Resets all MCAN interrupts (does NOT include any SPIERR interrupts) */
    TCAN4x5x_MCAN_ClearInterruptsAll();
}

//****************************************************************************
// Name: TCAN4550_enterCS_HW
// Details: This function enters the critical section by disabling HWI
//****************************************************************************
uint32_t TCAN4550_enterCS_HW( void )
{
#if defined(USING_DRIVERLIB)
  return (!IntMasterDisable());
#else /* Default to TIRTOS */
  return (Hwi_disable());
#endif
}
//****************************************************************************
// Name: TCAN4550_exitCS_HW
// Details: This function exits the critical section by restoring HWI
//****************************************************************************
void TCAN4550_exitCS_HW( uint32_t key )
{
#if defined(USING_DRIVERLIB)
  if (key)
  {
    (void) IntMasterEnable();
  }
#else /* Default to TIRTOS */
  Hwi_restore(key);
#endif
}

//****************************************************************************
// Name: HW_Tx_Msg
// Details:
//****************************************************************************
uint8_t HW_Tx_Msg(uint32_t CAN_msg_ID, uint8_t CAN_msg_DLC,
                  uint8_t *CAN_msg_Buffer)
{
   uint8_t data_cnt;
   TCAN4x5x_MCAN_TX_Header header = {0};
   uint8_t data[8];
   uint32_t key;

   // Load DLC into transmit message buffer
   if (CAN_msg_DLC > CAN_DLC_MAX)
   {
       CAN_msg_DLC = CAN_DLC_MAX;
   }

   // Set the DLC to be equal to or less than the data payload
   header.DLC = CAN_msg_DLC;
   header.ID = CAN_msg_ID;
   header.FDF = 0;                                 // CAN FD frame enabled
   header.BRS = 0;                                 // Bit rate switch enabled
   header.EFC = 0;
   header.MM = 0;
   header.RTR = 0;
   header.XTD = 0;                                 // We are not using an extended ID in this example
   header.ESI = 0;                                 // Error state indicator

   // Load data bytes into transmit message buffer
   for(data_cnt = 0; data_cnt < CAN_msg_DLC; data_cnt++)
   {
       data[data_cnt] =
               CAN_msg_Buffer[data_cnt];
   }

   // Start critical section
   key = TCAN4550_enterCS_HW();

   // This line writes the data and header to TX FIFO
   TCAN4x5x_MCAN_WriteTXBuffer(0, &header, data);
   // Request that TX Buffer be transmitted
   TCAN4x5x_MCAN_TransmitBufferContents(0 );

   // End critical section
   TCAN4550_exitCS_HW(key);

   // Success
   return (1);
}

//****************************************************************************
// Name: HW_ISR_Rx
// Details:
//****************************************************************************
void HW_ISR_Rx(uint32_t *msg_ID,
               uint8_t *msg_Payload,
               uint8_t *msg_DLC)
{
    uint8_t data_byte;

   // Setup a new MCAN IR object for easy interrupt checking
   TCAN4x5x_MCAN_Interrupts mcan_ir = {0};

   // Read the interrupt register
   TCAN4x5x_MCAN_ReadInterrupts(&mcan_ir);

   // If a new message in RX FIFO 0
   if (mcan_ir.RF0N) {

       // Clear the interrupt bit to release the INT pin.
       // Clear any of the interrupt bits that are set.
       TCAN4x5x_MCAN_ClearInterrupts(&mcan_ir);

       // This will read the next element in the RX FIFO 0
       numBytes = TCAN4x5x_MCAN_ReadNextFIFO(RXFIFO0, &MsgHeader, dataPayload);

       *msg_DLC = MsgHeader.DLC;
       *msg_ID = MsgHeader.ID;

       for(data_byte = 0; data_byte < MsgHeader.DLC; data_byte++)
       {
           msg_Payload[data_byte] = dataPayload[data_byte];
       }
   }
}

//****************************************************************************
// Name: ISR_Tx
// Details:
//****************************************************************************
void ISR_Tx(void)
{
}

//****************************************************************************
// Name: ISR_Error
// Details:
//****************************************************************************
void ISR_Error(void)
{
}

//****************************************************************************
// Name: Check_Error_State
// Details:
//****************************************************************************
void Check_Error_State(void)
{
}

//****************************************************************************
// Name: ISR_Wakeup
// Details:
//****************************************************************************
void ISR_Wakeup(void)
{
}

//****************************************************************************
// Name: Interrupt_Enable_Rx
// Details:
//****************************************************************************
void Interrupt_Enable_Rx(void)
{
}

//****************************************************************************
// Name: Interrupt_Enable_Tx
// Details:
//****************************************************************************
void Interrupt_Enable_Tx(void)
{
}

//****************************************************************************
// Name: Interrupt_Enable_Error
// Details:
//****************************************************************************
void Interrupt_Enable_Error(void)
{
}

//****************************************************************************
// Name: Interrupt_Enable_Wakeup
// Details:
//****************************************************************************
void Interrupt_Enable_Wakeup(void)
{
}

//****************************************************************************
// Name: Interrupt_Disable_Rx
// Details:
//****************************************************************************
void Interrupt_Disable_Rx(void)
{
}

//****************************************************************************
// Name: Interrupt_Disable_Tx
// Details:
//****************************************************************************
void Interrupt_Disable_Tx(void)
{
}

//****************************************************************************
// Name: Interrupt_Disable_Error
// Details:
//****************************************************************************
void Interrupt_Disable_Error(void)
{
}

//****************************************************************************
// Name: Interrupt_Disable_Wakeup
// Details:
//****************************************************************************
void Interrupt_Disable_Wakeup(void)
{
}

//****************************************************************************
// Name: Recover_From_Busoff
// Details:
//****************************************************************************
void Recover_From_Busoff(void)
{
}
