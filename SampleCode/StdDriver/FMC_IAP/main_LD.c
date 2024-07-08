/**************************************************************************//**
 * @file     main_LD.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/02/06 10:22a $
 * @brief    M058S Series Flash Memory Controller Driver Sample Code on LDROM
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M058S.h"

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000

#define KEY_ADDR            0x20000FFC  /* The location of signature */
#define SIGNATURE           0x21557899  /* The signature word is used by AP code to check if simple LD is finished */

#define CONFIG0_TEST_CODE   0x0F9000FF

#define FUN_TBL_BASE        0x00100E00

int32_t IAP_Func0(int32_t n);
int32_t IAP_Func1(int32_t n);
int32_t IAP_Func2(int32_t n);
int32_t IAP_Func3(int32_t n);

#if defined ( __ICCARM__ )
# pragma location = "FunTblSection" /* The location of FunTblSection is defined in FMC_IAP_LD.icf file. */
__root const uint32_t g_funcTable[4] =
{
    (uint32_t)IAP_Func0, (uint32_t)IAP_Func1, (uint32_t)IAP_Func2, (uint32_t)IAP_Func3
} ;
#else
#if defined(__GNUC_LD_IAP__)
const uint32_t __attribute__((section (".IAPFunTable"))) g_funcTable[4] =
{
    (uint32_t)IAP_Func0, (uint32_t)IAP_Func1, (uint32_t)IAP_Func2, (uint32_t)IAP_Func3
};
#else
const uint32_t * __attribute__((section(".ARM.__at_0x00100E00"))) g_funcTable[4] =
{
    (uint32_t *)IAP_Func0, (uint32_t *)IAP_Func1, (uint32_t *)IAP_Func2, (uint32_t *)IAP_Func3
};
#endif
#endif

void ProcessHardFault(void){}
	
void SysTickDelay(uint32_t us)
{
    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
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
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD , TXD and CKO */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk  | SYS_MFP_P36_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD | SYS_MFP_P31_TXD | SYS_MFP_P36_CKO);
}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}

int32_t IAP_Func0(int32_t n)
{
    int32_t i;

    for(i = 0; i < n; i++)
    {
        printf("Hello IAP0! #%d\n", i);
    }

    return n;
}

int32_t IAP_Func1(int32_t n)
{
    int32_t i;

    for(i = 0; i < n; i++)
    {
        printf("Hello IAP1! #%d\n", i);
    }

    return n;
}
int32_t IAP_Func2(int32_t n)
{
    int32_t i;

    for(i = 0; i < n; i++)
    {
        printf("Hello IAP2! #%d\n", i);
    }

    return n;
}
int32_t IAP_Func3(int32_t n)
{
    int32_t i;

    for(i = 0; i < n; i++)
    {
        printf("Hello IAP3! #%d\n", i);
    }

    return n;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t i;

    /* Unlock protected registers for ISP function */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*
        This is a simple sample code for LDROM in new IAP mode.
        The base address is 0x100000.
        The base address for function table is defined by FUN_TBL_BASE.
    */

    printf("+------------------------------------------------------------------+\n");
    printf("|    M05xx Flash Memory Controller Driver Sample Code for LDROM    |\n");
    printf("+------------------------------------------------------------------+\n");

    printf("\nCPU @ %dHz\n\n", SystemCoreClock);

    // Delay 3 seconds
    for(i = 0; i < 30; i++)
    {
        printf(".");
        SysTickDelay(10000);
    }

    while(SYS->PDID)__WFI();
}




