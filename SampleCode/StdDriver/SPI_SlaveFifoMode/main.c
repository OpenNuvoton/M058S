/**************************************************************************//**
 * @file     main.c
 * @version  V3.0
 * $Revision: 3 $
 * $Date: 15/02/06 10:22a $
 * @brief
 *           Configure SPI0 as Slave mode and demonstrate how to communicate with an off-chip SPI Master device with FIFO mode.
 *           This sample code needs to work with SPI_MasterFifoMode sample code.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M058S.h"


#define TEST_COUNT 16

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);
extern char GetChar(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    volatile uint32_t u32TxDataCount, u32RxDataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------+\n");
    printf("|           SPI Slave Mode Sample Code                |\n");
    printf("+-----------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a slave.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for SPI0:\n");
    printf("    SPISS (P0.4)\n    SPICLK (P0.7)\n");
    printf("    MISO (P0.6)\n    MOSI (P0.5)\n\n");
    printf("SPI controller will enable FIFO mode and transfer %d data to a off-chip master device.\n", TEST_COUNT);
    printf("In the meanwhile the SPI controller will receive %d data from the off-chip master device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);

    for(u32TxDataCount = 0; u32TxDataCount < TEST_COUNT; u32TxDataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32TxDataCount] = 0x00AA0000 + u32TxDataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32TxDataCount] = 0;
    }

    u32TxDataCount = 0;
    u32RxDataCount = 0;
    printf("Press any key if the master device configuration is ready.\n");
    GetChar();
    printf("\n");

    /* Set TX FIFO threshold and enable FIFO mode. */
    SPI_EnableFIFO(SPI0, 2, 2);

    /* Access TX and RX FIFO */
    while(u32RxDataCount < TEST_COUNT)
    {
        /* Check TX FULL flag and TX data count */
        if((SPI_GET_TX_FIFO_FULL_FLAG(SPI0) == 0) && (u32TxDataCount < TEST_COUNT))
            SPI_WRITE_TX0(SPI0, g_au32SourceData[u32TxDataCount++]); /* Write to TX FIFO */
        /* Check RX EMPTY flag */
        if(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
            g_au32DestinationData[u32RxDataCount++] = SPI_READ_RX0(SPI0); /* Read RX FIFO */
    }

    /* Print the received data */
    printf("Received data:\n");
    for(u32RxDataCount = 0; u32RxDataCount < TEST_COUNT; u32RxDataCount++)
    {
        printf("%d:\t0x%X\n", u32RxDataCount, g_au32DestinationData[u32RxDataCount]);
    }
    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Reset SPI0 */
    SPI_Close(SPI0);
    while(1);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable external 12 MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Switch HCLK clock source to HXT and set HCLK divider to 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT, CLK_CLKDIV_HCLK(1));

    /* Select HXT as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Select HCLK as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL1_SPI0_S_HCLK, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD | SYS_MFP_P31_TXD);

    /* Setup SPI0 multi-function pins */
    SYS->P0_MFP &= ~(SYS_MFP_P04_Msk | SYS_MFP_P05_Msk | SYS_MFP_P06_Msk | SYS_MFP_P07_Msk);
    SYS->P0_MFP |= (SYS_MFP_P04_SPISS | SYS_MFP_P05_MOSI | SYS_MFP_P06_MISO | SYS_MFP_P07_SPICLK);
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI0 as a low level active device. */
    SPI_Open(SPI0, SPI_SLAVE, SPI_MODE_0, 32, 0);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


