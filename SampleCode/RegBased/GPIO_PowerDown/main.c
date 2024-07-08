/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 6 $
 * $Date: 15/07/13 4:13p $
 * @brief    Show how to wake up system from Power-down mode by GPIO interrupt.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M058S.h"


#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(UART0)
        if(--u32TimeOutCnt == 0) break;

    /* Set the processor is deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled and Power-down entry condition */
    CLK->PWRCON |= (CLK_PWRCON_PWR_DOWN_EN_Msk | CLK_PWRCON_PD_WU_INT_EN_Msk);

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();

}

/**
 * @brief       Port0/Port1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port0/Port1 default IRQ, declared in startup_M058S.s.
 */
void GPIOP0P1_IRQHandler(void)
{
    /* To check if P1.3 interrupt occurred */
    if(P1->ISRC & BIT3)
    {
        P1->ISRC = BIT3;
        printf("P1.3 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORT0, PORT1 interrupts */
        P0->ISRC = P0->ISRC;
        P1->ISRC = P1->ISRC;
        printf("Un-expected interrupts.\n");
    }
}

/**
 * @brief       Port6/Port7 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port6/Port7 default IRQ, declared in startup_M058S.s.
 */
void GPIOP6P7_IRQHandler(void)
{
    /* To check if P6.2 interrupt occurred */
    if(P6->ISRC & BIT2)
    {
        P6->ISRC = BIT2;
        printf("P6.2 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORT6, PORT7 interrupts */
        P6->ISRC = P6->ISRC;
        P7->ISRC = P7->ISRC;
        printf("Un-expected interrupts.\n");
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | CLK_CLKDIV_HCLK(1);

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware. */
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART_S_Msk)) | CLK_CLKSEL1_UART_S_PLL;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD | SYS_MFP_P31_TXD);

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PLL_CLOCK, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+---------------------------------------------------------------+\n");
    printf("|    GPIO Power-Down and Wake-up by P1.3 or P6.2 Sample Code    |\n");
    printf("+---------------------------------------------------------------+\n\n");

    /* Configure P1.3 as Input mode and enable interrupt by rising edge trigger */
    P1->PMD = (P1->PMD & (~GPIO_PMD_PMD3_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD3_Pos);
    P1->IMD |= (GPIO_IMD_EDGE << 3);
    P1->IEN |= (BIT3 << GPIO_IEN_IR_EN_Pos);
    NVIC_EnableIRQ(GPIO_P0P1_IRQn);

    /* Configure P6.2 as Input mode and enable interrupt by falling edge trigger */
    P6->PMD = (P6->PMD & (~GPIO_PMD_PMD2_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD2_Pos);
    P6->IMD |= (GPIO_IMD_EDGE << 2);
    P6->IEN |= (BIT2 << GPIO_IEN_IF_EN_Pos);
    NVIC_EnableIRQ(GPIO_P6P7_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO->DBNCECON = (GPIO_DBNCECON_ICLK_ON_Msk | GPIO_DBCLKSRC_LIRC | GPIO_DBCLKSEL_1024);
    P1->DBEN |= BIT3;
    P6->DBEN |= BIT2;

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Waiting for P1.3 rising-edge or P6.2 falling-edge interrupt event */
    while(1)
    {
        printf("Enter to Power-Down ......\n");

        /* Enter to Power-down mode */
        PowerDownFunction();

        printf("System waken-up done.\n\n");
    }

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
