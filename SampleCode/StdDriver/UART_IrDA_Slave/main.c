/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/02/06 10:22a $
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

#define PLL_CLOCK   50000000


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

    /* Set IrDA Rx Mode, Baud Rate configuration must be used MODE0 */
    UART_SelectIrDAMode(UART0, 57600, UART_IRCR_RX_SELECT);

    printf("Waiting...\n");

    /* Use polling method to wait master data */
    do
    {
        if(UART_IS_RX_READY(UART0))
        {
            u8InChar = UART_READ(UART0);
            printf("   Input: %c \n", u8InChar);
        }
    }
    while(u8InChar != '0');

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
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
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

