/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 15/07/13 4:13p $
 * @brief
 *           Transmit and receive data in UART IrDA mode.
 *           This sample code needs to work with UART_IrDA_Master.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M058S.h"

#define PLLCON_SETTING  CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK       50000000


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
int32_t main(void);
void IrDA_FunctionRxTest(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  IrDA Function Receive Test                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionRxTest()
{
    uint8_t u8InChar = 0xFF;

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     Pin Configure                                           |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|      ______                                    _______      |\n");
    printf("|     |      |                                  |       |     |\n");
    printf("|     |Master|--- TXD(P3.1)        RXD(P3.0) ---|Slave  |     |\n");
    printf("|     |      |                                  |       |     |\n");
    printf("|     |______|                                  |_______|     |\n");
    printf("|                                                             |\n");
    printf("+-------------------------------------------------------------+\n");

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     IrDA Function Test                                      |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Please enable semihosted to show messages on debug session.|\n");
    printf("|  Keil users must define DEBUG_ENABLE_SEMIHOST in both C/C++ |\n");
    printf("|  and Asm preprocessor symbols.                              |\n");
    printf("|  IAR users must define DEBUG_ENABLE_SEMIHOST in both C/C++  |\n");
    printf("|  Compiler and Assembler preprocessor symbols.               |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Description :                                              |\n");
    printf("|    The sample code needs two boards. One is Master and      |\n");
    printf("|    the other is slave.  Master will send data based on      |\n");
    printf("|    terminal input and Slave will printf received data on    |\n");
    printf("|    terminal screen.                                         |\n");
    printf("+-------------------------------------------------------------+\n");

    /*
        The IrDA sample code needs two module board to execute and enables semihosted.
        Set the master board is IrDA tx Mode and the other is IrDA rx mode.
        Inputing char on terminal will be sent to the UART0 Tx of master by IrDA mode.
        Slave will print received char from UART0 Rx.
        Note that IrDA mode is ONLY used when baud rate equation is selected mode 0.
    */

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     IrDA Function Mode Test (Slave)                         |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("| 1). Polling RDA_Flag to check data input though UART0       |\n");
    printf("| 2). If received data is '0', the program will exit.         |\n");
    printf("|     Otherwise, print received data on terminal              |\n");
    printf("+-------------------------------------------------------------+\n\n");

    /* In IrDA Mode, Baud Rate configuration must be used MODE0 */
    UART0->BAUD = UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HXT, 57600);

    /* IrDA Function Enable */
    UART0->FUN_SEL = UART_FUNC_SEL_IrDA;

    /* Set IrDA Rx mode */
    UART0->IRCR &= ~UART_IRCR_TX_SELECT_Msk;
    UART0->IRCR |= UART_IRCR_INV_RX_Msk;     //Rx signal is inversed

    printf("Waiting...\n");

    /* Use polling method to wait master data */
    do
    {
        if(UART0->ISR & UART_ISR_RDA_IF_Msk)
        {
            u8InChar = UART0->RBR;;
            printf("   Input: %c \n", u8InChar);
        }
    }
    while(u8InChar != '0');

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
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART_S_Msk)) | CLK_CLKSEL1_UART_S_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD | SYS_MFP_P31_TXD);

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
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

    /* Init UART0 for testing */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\n\nUART Sample Program\n");

    /* UART sample IrDA Slave function */
    IrDA_FunctionRxTest();

    while(1);

}
