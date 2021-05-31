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

#ifndef CAN_HW_CONFIG_H
#define CAN_HW_CONFIG_H
//#############################################################################
//
//! \file   can_hw_config.h
//!
//! \brief  CAN HW header file
//!
//
//  Group:          CMCU
//  Target Device:  CC26xx
//
//  (C) Copyright 2019, Texas Instruments, Inc.
//#############################################################################

// File Includes

// Defines

// Types

// Constants

// Global Variables

// Prototype Declarations
extern void HW_ISR_Rx(uint32_t *msg_ID,
                      uint8_t *msg_Payload,
                      uint8_t *msg_DLC);
extern void ISR_Tx(void);
extern void ISR_Wakeup(void);
extern void ISR_Error(void);
extern void TCAN4x5x_TX_Error_Passive_Flag_Clear(void);
extern void TCAN4x5x_RX_Error_Passive_Flag_Clear(void);
extern void TCAN4x5x_Busoff_Flag_Clear(void);
#endif
