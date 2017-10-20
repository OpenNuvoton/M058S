/******************************************************************************
 * @file     LDROM_main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/01/13 2:55p $
 * @brief    FMC LDROM IAP sample program for MINI51 series MCU
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M058S.h"



#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    CLK->PLLCON = PLLCON_SETTING;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_PLL;

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_HXT;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD and TXD  */
    SYS->P3_MFP = SYS_MFP_P30_RXD | SYS_MFP_P31_TXD;

}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}




int main()
{
    /* Unlock protected register */
    SYS_UnlockReg();

    SYS_Init();
    UART_Init();

    printf("\n\n");
    printf("M058S FMC IAP Sample Code [LDROM code]\n");

    /* Enable FMC ISP function */
    FMC_Open();

    printf("\n\nPress any key to branch to APROM...\n");
    getchar();

    printf("\n\nChange VECMAP and branch to LDROM...\n");
    UART_WAIT_TX_EMPTY(UART0);
    
    /* Set Boot selection bit for rebooting to LDROM */
    FMC->ISPCON &= ~FMC_ISPCON_BS_Msk;
    
    /* Software reset to boot to APROM */
    NVIC_SystemReset();

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
