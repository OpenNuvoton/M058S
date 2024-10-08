/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 5 $
 * $Date: 15/02/06 10:22a $
 * @brief    Show the usage of GPIO interrupt function.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M058S.h"


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
    if(GPIO_GET_INT_FLAG(P1, BIT3))
    {
        GPIO_CLR_INT_FLAG(P1, BIT3);
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
    if(GPIO_GET_INT_FLAG(P4, BIT5))
    {
        GPIO_CLR_INT_FLAG(P4, BIT5);
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
    if(GPIO_GET_INT_FLAG(P5, BIT2))
    {
        GPIO_CLR_INT_FLAG(P5, BIT2);
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
    if(GPIO_GET_INT_FLAG(P6, BIT1))
    {
        GPIO_CLR_INT_FLAG(P6, BIT1);
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
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));

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
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
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
    GPIO_SetMode(P1, BIT3, GPIO_PMD_INPUT);
    GPIO_EnableInt(P1, 3, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPIO_P0P1_IRQn);

    /*  Configure P4.5 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
    GPIO_SetMode(P4, BIT5, GPIO_PMD_QUASI);
    GPIO_EnableInt(P4, 5, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPIO_P2P3P4_IRQn);

    /*  Configure P5.2 as Input mode and enable interrupt by rising and falling edge trigger */
    GPIO_SetMode(P5, BIT2, GPIO_PMD_INPUT);
    GPIO_EnableInt(P5, 2, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(GPIO_P5_IRQn);

    /*  Configure P6.1 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
    GPIO_SetMode(P6, BIT1, GPIO_PMD_QUASI);
    GPIO_EnableInt(P6, 1, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPIO_P6P7_IRQn);

    /* Waiting for interrupts */
    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
