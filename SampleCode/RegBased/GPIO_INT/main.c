/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 5 $
 * $Date: 15/07/13 4:13p $
 * @brief    Show the usage of GPIO interrupt function.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M058S.h"


#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000


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
 * @brief       Port2/Port3/Port4 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port2/Port3/Port4 default IRQ, declared in startup_M058S.s.
 */
void GPIOP2P3P4_IRQHandler(void)
{
    /* To check if P4.5 interrupt occurred */
    if(P4->ISRC & BIT5)
    {
        P4->ISRC = BIT5;
        printf("P4.5 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORT2, PORT3 and PORT4 interrupts */
        P2->ISRC = P2->ISRC;
        P3->ISRC = P3->ISRC;
        P4->ISRC = P4->ISRC;
        printf("Un-expected interrupts.\n");
    }
}

/**
 * @brief       Port5 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port5 default IRQ, declared in startup_M058S.s.
 */
void GPIOP5_IRQHandler(void)
{
    /* To check if P5.2 interrupt occurred */
    if(P5->ISRC & BIT2)
    {
        P5->ISRC = BIT2;
        printf("P5.2 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORT5 interrupts */
        P5->ISRC = P5->ISRC;
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
    /* To check if P6.1 interrupt occurred */
    if(P6->ISRC & BIT1)
    {
        P6->ISRC = BIT1;
        printf("P6.1 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORT6 and PORT7 interrupts */
        P6->ISRC = P6->ISRC;
        P7->ISRC = P7->ISRC;
        printf("Un-expected interrupts.\n");
    }
}


void SYS_Init(void)
{
	uint32_t u32TimeOutCnt;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    u32TimeOutCnt = __HIRC;
	while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | CLK_CLKDIV_HCLK(1);

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware. */
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    u32TimeOutCnt = __HIRC;
	while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    u32TimeOutCnt = __HIRC;
	while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk))
		if(--u32TimeOutCnt == 0) break;
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
    printf("+------------------------------------------------------+\n");
    printf("|    GPIO P1.3/P4.5/P5.2/P6.1 Interrupt Sample Code    |\n");
    printf("+------------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("P1.3/P4.5/P5.2/P6.1 are used to test interrupt ......\n");

    /* Configure P1.3 as Input mode and enable interrupt by rising edge trigger */
    P1->PMD = (P1->PMD & (~GPIO_PMD_PMD3_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD3_Pos);
    P1->IMD |= (GPIO_IMD_EDGE << 3);
    P1->IEN |= (BIT3 << GPIO_IEN_IR_EN_Pos);
    NVIC_EnableIRQ(GPIO_P0P1_IRQn);

    /* Configure P4.5 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
    P4->PMD = (P4->PMD & (~GPIO_PMD_PMD5_Msk)) | (GPIO_PMD_QUASI << GPIO_PMD_PMD5_Pos);
    P4->IMD |= (GPIO_IMD_EDGE << 5);
    P4->IEN |= (BIT5 << GPIO_IEN_IF_EN_Pos);
    NVIC_EnableIRQ(GPIO_P2P3P4_IRQn);

    /* Configure P5.2 as Input mode and enable interrupt by rising and falling edge trigger */
    P5->PMD = (P5->PMD & (~GPIO_PMD_PMD2_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD2_Pos);
    P5->IMD |= (GPIO_IMD_EDGE << 2);
    P5->IEN |= (BIT2 << GPIO_IEN_IR_EN_Pos) | (BIT2 << GPIO_IEN_IF_EN_Pos);
    NVIC_EnableIRQ(GPIO_P5_IRQn);

    /* Configure P6.1 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
    P6->PMD = (P6->PMD & (~GPIO_PMD_PMD1_Msk)) | (GPIO_PMD_QUASI << GPIO_PMD_PMD1_Pos);
    P6->IMD |= (GPIO_IMD_EDGE << 1);
    P6->IEN |= (BIT1 << GPIO_IEN_IF_EN_Pos);
    NVIC_EnableIRQ(GPIO_P6P7_IRQn);

    /* Waiting for interrupts */
    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
