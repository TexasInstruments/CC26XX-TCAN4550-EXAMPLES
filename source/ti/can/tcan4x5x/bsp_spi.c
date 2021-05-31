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
 *  ======== bsp_spi.c ========
 */
 
/*******************************************************************************
 *                                INCLUDES
 ******************************************************************************/
#ifdef USING_DRIVERLIB
#include <driverlib/ssi.h>
#include <driverlib/gpio.h>
#include <driverlib/prcm.h>
#include <driverlib/ioc.h>
#include <driverlib/rom.h>
#include <driverlib/ssi.h>
#include "board_config.h"
#else
#include <string.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/GPIO.h>
#ifdef USE_SYSCFG
#include "ti_drivers_config.h"
#else
#include "Board.h"
#endif
#endif

#include "bsp_spi.h"

/*******************************************************************************
 *                                SPI SETUP
 ******************************************************************************/
#ifdef USING_DRIVERLIB
#define BLS_SPI_BASE       SSI0_BASE
#define BLS_CPU_FREQ       48000000ul
#define SPI_BIT_RATE       8000000ul
#else
SPI_Handle spiHandle = NULL;
#endif

//  Write to an SPI device
int bspSpiWrite(const uint8_t *buf, size_t length)
{   
#ifdef USING_DRIVERLIB
    while (length > 0)
    {
      uint32_t ul;

      SSIDataPut(BLS_SPI_BASE, *buf);
      SSIDataGet(BLS_SPI_BASE, &ul);
      length--;
      buf++;
    }

    return (0);
#else
    SPI_Transaction masterTransaction;
    bool success;

    masterTransaction.count = length;
    masterTransaction.txBuf = (void*)buf;
    masterTransaction.arg = NULL;
    masterTransaction.rxBuf = NULL;

    success = SPI_transfer(spiHandle, &masterTransaction);

    return (success ? 0 : -1);
#endif
}

//  Read from an SPI device
int bspSpiRead(uint8_t *buf, size_t length)
{
#ifdef USING_DRIVERLIB
    while (length > 0)
    {
      uint32_t ul;

      if (!SSIDataPutNonBlocking(BLS_SPI_BASE, 0))
      {
        /* Error */
        return -1;
      }

      SSIDataGet(BLS_SPI_BASE, &ul);
      *buf = (uint8_t) ul;
      length--;
      buf++;
    }

    return 0;
#else
    SPI_Transaction masterTransaction;
    bool success;

    masterTransaction.count = length;
    masterTransaction.txBuf = NULL;
    masterTransaction.arg = NULL;
    masterTransaction.rxBuf = buf;

    success = SPI_transfer(spiHandle, &masterTransaction);

    return (success ? 0 : -1);
#endif
}

//  Write and read from an SPI device
int bspSpiWriteRead(uint8_t *buf, uint8_t wlen, uint8_t rlen)
{   
#ifdef USING_DRIVERLIB
    return(0);
#else
    SPI_Transaction masterTransaction;
    bool success;

    masterTransaction.count = wlen + rlen;
    masterTransaction.txBuf = buf;
    masterTransaction.arg = NULL;
    masterTransaction.rxBuf = buf;

    success = SPI_transfer(spiHandle, &masterTransaction);
    if (success)
    {   
        memcpy(buf,buf+wlen,rlen);
    }

    return (success ? 0 : -1);
#endif
}

//  Initialize the SPI communication
void bspSpiOpen(void)
{   
#ifdef USING_DRIVERLIB
    /* GPIO power && SPI power domain */
    PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH | PRCM_DOMAIN_SERIAL);
    while (PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH | PRCM_DOMAIN_SERIAL)
           != PRCM_DOMAIN_POWER_ON);

    /* GPIO power */
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
    PRCMLoadSet();
    while (!PRCMLoadGet());

    /* SPI power */
    PRCMPeripheralRunEnable(PRCM_PERIPH_SSI0);
    PRCMLoadSet();
    while (!PRCMLoadGet());

    /* SPI configuration */
    SSIIntDisable(BLS_SPI_BASE, SSI_RXOR | SSI_RXFF | SSI_RXTO | SSI_TXFF);
    SSIIntClear(BLS_SPI_BASE, SSI_RXOR | SSI_RXTO);
    SSIConfigSetExpClk(BLS_SPI_BASE,
                       BLS_CPU_FREQ, /* CPU rate */
                       SSI_FRF_MOTO_MODE_0, /* frame format */
                       SSI_MODE_MASTER, /* mode */
                       SPI_BIT_RATE, /* bit rate */
                       8); /* data size */
    IOCPinTypeSsiMaster(BLS_SPI_BASE, BSP_SPI_MISO, BSP_SPI_MOSI,
                        IOID_UNUSED, BSP_SPI_CLK);

    SSIEnable(BLS_SPI_BASE);

    /* CS */
    IOCPinTypeGpioOutput( BSP_SPI_CS );
    GPIO_writeDio(BSP_SPI_CS, 1);
#else
    SPI_Params spiParams;

    if (spiHandle == NULL)
    {   
        /*  Configure SPI as master, 12MHz bit rate*/
        SPI_Params_init(&spiParams);
        spiParams.bitRate = 12000000;
        spiParams.frameFormat  = SPI_POL0_PHA0;
        spiParams.mode = SPI_MASTER;
        spiParams.transferMode = SPI_MODE_BLOCKING;

        /* Attempt to open SPI. */
        spiHandle = SPI_open(Board_SPI0, &spiParams);

        if (spiHandle == NULL)
        {   
            while(1)
            {   
                // wait here forever
            }
        }
    }
#endif
}

/* See bsp_spi.h file for description */
void bspSpiCSSelect(bool select)
{
    /* set the CS low to start the transaction */
    if (select)
    {
#ifdef USING_DRIVERLIB
        GPIO_writeDio(BSP_SPI_CS, 0);
#else
        GPIO_write(Board_GPIO_TCAN4550_CS, 0);
#endif
    }
    else
    {
#ifdef USING_DRIVERLIB
        GPIO_writeDio(BSP_SPI_CS, 1);
#else
        GPIO_write(Board_GPIO_TCAN4550_CS, 1);
#endif
    }
}

/* See bsp_spi.h file for description */
void bspSpiFlush(void)
{   
}


/* See bsp_spi.h file for description */
void bspSpiClose(void)
{   
#ifdef USING_DRIVERLIB
    // Power down SSI0
    ROM_PRCMPeripheralRunDisable(PRCM_PERIPH_SSI0);
    PRCMLoadSet();
    while (!PRCMLoadGet());
#else
    if (spiHandle != NULL)
    {   
        SPI_close(spiHandle);
        spiHandle = NULL;
    }
#endif
}
