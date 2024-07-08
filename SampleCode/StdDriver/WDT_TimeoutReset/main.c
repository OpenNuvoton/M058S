/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/09/14 4:06p $
 * @brief
 *           Show how to generate time-out reset system event while WDT time-out reset delay period expired.
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M058S.h"

#define PLL_CLOCK           50000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8IsWDTTimeoutINT = 0;


/**
 * @brief       IRQ Handler for WDT and WWDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WDT and WWDT, declared in startup_M058S.s.
 */
void WDT_IRQHandler(void)
{
    if(WDT_GET_TIMEOUT_INT_FLAG() == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();

        g_u8IsWDTTimeoutINT = 1;

        printf("WDT time-out interrupt occurred.\n");
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz and LIRC 10KHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC10K_EN_Msk);

    /* Waiting for external XTAL and LIRC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk | CLK_CLKSTATUS_OSC10K_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDT_S_LIRC, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD, TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD | SYS_MFP_P31_TXD);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    if(WDT_GET_RESET_FLAG() == 1)
    {
        /* Use P0.0 to check time-out period time */
        GPIO_SetMode(P0, BIT0, GPIO_PMD_OUTPUT);
        P00 = 1;
        P00 = 0;

        WDT_CLEAR_RESET_FLAG();
        printf("\n*** WDT time-out reset occurred ***\n");
        while(1);
    }

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------+\n");
    printf("|    WDT Time-out Reset Sample Code    |\n");
    printf("+--------------------------------------+\n\n");

    printf("# WDT Settings:\n");
    printf("  Clock source is 10 kHz; Enable interrupt; Time-out interval is 2^14 * WDT clock.\n");
    printf("# When WDT start counting, system will generate a WDT time-out interrupt after 1.6384 ~ 1.7408 s.\n");
    printf("  Measure P0.0 low period to check time-out interval and do not reload WDT counter.\n\n");

    /* Use P0.0 to check time-out period time */
    GPIO_SetMode(P0, BIT0, GPIO_PMD_OUTPUT);
    P00 = 1;
    P00 = 0;

    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Enable WDT time-out reset function and select time-out interval to 2^14 * WDT clock then start WDT counting */
    g_u8IsWDTTimeoutINT = 0;
    WDT_Open(WDT_TIMEOUT_2POW14, 0, TRUE, FALSE);

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    /* Check if WDT time-out interrupt occurred */
    while(g_u8IsWDTTimeoutINT == 0);

    P00 = 1;

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
