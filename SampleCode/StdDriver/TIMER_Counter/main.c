/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/09/14 4:04p $
 * @brief    Implement timer1 event counter function to count the external input event.
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
volatile uint32_t g_au32TMRINTCount[4] = {0};


/*---------------------------------------------------------------------------------------------------------*/
/*  Create Counter Source by GPIO PORT2                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void GeneratePORT2Counter(uint32_t u32Pin, uint32_t u32Counts)
{
    while(u32Counts--)
    {
        GPIO_PIN_DATA(2, u32Pin) = 1;
        GPIO_PIN_DATA(2, u32Pin) = 0;
    }
}

/**
 * @brief       Timer1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer1 default IRQ, declared in startup_M058S.s.
 */
void TMR1_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);

        g_au32TMRINTCount[1]++;
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

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HCLK, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD, TXD and T1 */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk | SYS_MFP_P35_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD | SYS_MFP_P31_TXD | SYS_MFP_P35_T1);
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
    volatile uint32_t u32InitCount;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------+\n");
    printf("|    Timer1 External Counter Input Sample Code    |\n");
    printf("+-------------------------------------------------+\n\n");

    printf("# Timer Settings:\n");
    printf("  Timer1: Clock source is HCLK(50 MHz); Continuous counting mode; Interrupt enable;\n");
    printf("          External counter input enable; TCMP is 56789.\n");
    printf("# Connect P2.0 to T1 pin and pull P2.0 High/Low as T1 counter input source.\n\n");

    /* Configure P2.0 as GPIO output pin and pull pin status to Low first */
    GPIO_SetMode(P2, BIT0, GPIO_PMD_OUTPUT);
    P20 = 0;

    /* Initial Timer1 default setting */
    TIMER_Open(TIMER1, TIMER_CONTINUOUS_MODE, 1);

    /* Configure Timer1 setting for external counter input function */
    TIMER_SELECT_TOUT_PIN(TIMER1, TIMER_TOUT_PIN_FROM_TX_PIN);
    TIMER_SET_PRESCALE_VALUE(TIMER1, 0);
    TIMER_SET_CMP_VALUE(TIMER1, 56789);
    TIMER_EnableEventCounter(TIMER1, TIMER_COUNTER_FALLING_EDGE);
    TIMER_EnableInt(TIMER1);

    /* Enable Timer1 NVIC */
    NVIC_EnableIRQ(TMR1_IRQn);

    /* Clear Timer1 interrupt counts to 0 */
    g_au32TMRINTCount[1] = 0;

    /* Start Timer1 counting */
    TIMER_Start(TIMER1);

    /* To check if TDR of Timer1 must be 0 as default value */
    if(TIMER_GetCounter(TIMER1) != 0)
    {
        printf("Default counter value is not 0. (%d)\n", TIMER_GetCounter(TIMER1));
        goto lexit;
    }

    /* To generate one counter event to T1 pin */
    GeneratePORT2Counter(0, 1);

    /* To check if TDR of Timer1 must be 1 */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(TIMER_GetCounter(TIMER1) == 0)
        if(--u32TimeOutCnt == 0) break;
    if(TIMER_GetCounter(TIMER1) != 1)
    {
        printf("Get unexpected counter value. (%d)\n", TIMER_GetCounter(TIMER1));
        goto lexit;
    }

    /* To generate remains counts to T1 pin */
    GeneratePORT2Counter(0, (56789 - 1));

    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(1)
    {
        if((g_au32TMRINTCount[1] == 1) && (TIMER_GetCounter(TIMER1) == 56789))
        {
            printf("Timer1 external counter input function ... PASS.\n");
            break;
        }

        if(--u32TimeOutCnt == 0)
        {
            printf("Timer1 external counter input function ... FAIL.\n");
            break;
        }
    }

lexit:

    /* Stop Timer1 counting */
    TIMER_Close(TIMER1);

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
