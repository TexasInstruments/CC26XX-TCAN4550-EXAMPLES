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

#ifndef CAN_HW_H
#define CAN_HW_H
//#############################################################################
//
//! \file   can_hw.h
//!
//! \brief  CAN HW  
//!         
//
//  Group:          CMCU
//  Target Device:  CC26xx
//
//  (C) Copyright 2019, Texas Instruments, Inc.
//#############################################################################

// File Includes
#include "can_hw_config.h"

// Defines
#define CAN_RECEIVE_BUFFER_LOCKED                                    0xFA
#define CAN_INVALID_DLC                                              0xFB

// Types

typedef enum
{
   EXTENDED_ID,
   STANDARD_ID
} CAN_ID_Type_t;

// Constants

// Exported Variables

// Exported Functions
extern void Init_CAN(void);
extern void ISR_Error(void);
extern void Interrupt_Enable_Rx(void);
extern void Interrupt_Enable_Tx(void);
extern void Interrupt_Enable_Error(void);
extern void Interrupt_Enable_Wakeup(void);
extern void Interrupt_Disable_Rx(void);
extern void Interrupt_Disable_Tx(void);
extern void Interrupt_Disable_Error(void);
extern void Interrupt_Disable_Wakeup(void);
extern uint8_t HW_Tx_Msg(uint32_t CAN_msg_ID, uint8_t CAN_msg_DLC,
                         uint8_t *CAN_msg_Buffer);
#endif
