/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 5 $
 * $Date: 15/02/06 10:22a $
 * @brief    Show how to set GPIO pin mode and use pin data input/output control.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M058S.h"


#define PLL_CLOCK           50000000
extern char GetChar(void);

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
    int32_t i32Err;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------------------------+\n");
    printf("|    P1.2(Output) to P4.1(Input) and P5.1(Output) to P6.2(Input) Sample Code    |\n");
    printf("+-------------------------------------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin                              */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("  >> Please connect P1.2 and P4.1 << \n");
    printf("  >> Please connect P5.1 and P6.2 << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    GetChar();

    /* Configure P1.2 as Output mode and P4.1 as Input mode */
    GPIO_SetMode(P1, BIT2, GPIO_PMD_OUTPUT);
    GPIO_SetMode(P4, BIT1, GPIO_PMD_INPUT);

    i32Err = 0;
    printf("GPIO P1.2(output mode) connect to P4.1(input mode) ......");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    P12 = 0;
    if(P41 != 0)
    {
        i32Err = 1;
    }

    P12 = 1;
    if(P41 != 1)
    {
        i32Err = 1;
    }

    if(i32Err)
    {
        printf("  [FAIL].\n");
    }
    else
    {
        printf("  [OK].\n");
    }

    /* Configure P1.2 and P4.1 to default Quasi-bidirectional mode */
    GPIO_SetMode(P1, BIT2, GPIO_PMD_QUASI);
    GPIO_SetMode(P4, BIT1, GPIO_PMD_QUASI);

    if(i32Err != 0)
        while(1);

    /* Configure P5.1 as Output mode and P6.2 as Input mode then close it */
    GPIO_SetMode(P5, BIT1, GPIO_PMD_OUTPUT);
    GPIO_SetMode(P6, BIT2, GPIO_PMD_INPUT);

    printf("GPIO P5.1(output mode) connect to P6.2(input mode) ......");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    P51 = 0;
    if(P62 != 0)
    {
        i32Err = 1;
    }

    P51 = 1;
    if(P62 != 1)
    {
        i32Err = 1;
    }

    if(i32Err)
    {
        printf("  [FAIL].\n");
    }
    else
    {
        printf("  [OK].\n");
    }

    /* Configure P5.1 amd P6.2 to default Quasi-bidirectional mode */
    GPIO_SetMode(P5, BIT1, GPIO_PMD_QUASI);
    GPIO_SetMode(P6, BIT2, GPIO_PMD_QUASI);

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
