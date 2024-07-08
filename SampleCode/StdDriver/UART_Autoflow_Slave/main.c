/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 5 $
 * $Date: 15/02/06 10:22a $
 * @brief
 *           Transmit and receive data with auto flow control.
 *           This sample code needs to work with UART_Autoflow_Master.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M058S.h"


#define PLL_CLOCK 50000000

# if defined ( __GNUC__ )
#define RXBUFSIZE 128
#else
#define RXBUFSIZE 1024
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8SendData[12] = {0};
uint8_t g_u8RecData[RXBUFSIZE]  = {0};
volatile int32_t g_i32pointer = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
int32_t main(void);
void AutoFlow_FunctionRxTest(void);


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

    /* Set P0 multi-function pins for UART0 RTS */
    SYS->P0_MFP = (SYS->P0_MFP & (~SYS_MFP_P03_Msk)) | SYS_MFP_P03_RTS;

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

    /* UART auto flow sample slave function */
    AutoFlow_FunctionRxTest();

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    volatile uint32_t u32IntSts = UART0->ISR;;

    /* Rx Ready or Time-out INT */
    if(UART_GET_INT_FLAG(UART0, UART_ISR_RDA_INT_Msk) ||  UART_GET_INT_FLAG(UART0, UART_ISR_TOUT_INT_Msk))
    {
        /* Handle received data */
        g_u8RecData[g_i32pointer] = UART_READ(UART0);
        g_i32pointer++;
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Slave)                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionRxTest()
{
    uint32_t u32i, u32Err = 0;

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     Pin Configure                                           |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     _______                                    _______      |\n");
    printf("|    |       |                                  |       |     |\n");
    printf("|    |Master |--- TXD(P3.1) <====> RXD(P3.0) ---| Slave |     |\n");
    printf("|    |       |--- CTS(P0.2) <====> RTS(P0.3) ---|       |     |\n");
    printf("|    |_______|                                  |_______|     |\n");
    printf("|                                                             |\n");
    printf("+-------------------------------------------------------------+\n");

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test (Slave)                        |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Please enable semihosted to show messages on debug session.|\n");
    printf("|  Keil users must define DEBUG_ENABLE_SEMIHOST in both C/C++ |\n");
    printf("|  and Asm preprocessor symbols.                              |\n");
    printf("|  IAR users must define DEBUG_ENABLE_SEMIHOST in both C/C++  |\n");
    printf("|  Compiler and Assembler preprocessor symbols.               |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Description :                                              |\n");
    printf("|    The sample code needs two boards. One is Master and the  |\n");
    printf("|    is slave. Master will send 1k bytes data to slave.       |\n");
    printf("|    Slave will check if received data is correct after       |\n");
    printf("|    getting 1k bytes data.                                   |\n");
    printf("|    Press any key to start...                                |\n");
    printf("+-------------------------------------------------------------+\n");
    GetChar();

    /* Enable RTS and CTS autoflow control */
    UART_EnableFlowCtrl(UART0);

    /* Set RTS Trigger Level as 8 bytes */
    UART0->FCR = (UART0->FCR & (~UART_FCR_RTS_TRI_LEV_Msk)) | UART_FCR_RTS_TRI_LEV_8BYTES;

    /* Enable RDA\RLS\RTO Interrupt */
    UART_ENABLE_INT(UART0, (UART_IER_RDA_IEN_Msk | UART_IER_RLS_IEN_Msk | UART_IER_RTO_IEN_Msk));

    /* Set RX Trigger Level as 8 bytes */
    UART0->FCR = (UART0->FCR & (~UART_FCR_RFITL_Msk)) | UART_FCR_RFITL_8BYTES;

    /* Set Timeout time 0x3E bit-time and time-out counter enable */
    UART_SetTimeoutCnt(UART0, 0x3E);

    /* Enable UART0 IRQ */
    NVIC_EnableIRQ(UART0_IRQn);

    printf("\n Starting to receive data...\n");

    /* Wait for receive 1k bytes data */
    while(g_i32pointer < RXBUFSIZE);

    /* Compare Data */
    for(u32i = 0; u32i < RXBUFSIZE; u32i++)
    {
        if(g_u8RecData[u32i] != (u32i & 0xFF))
        {
            u32Err = 1;
            break;
        }
    }

    if( u32Err )
        printf("Compare Data Failed\n");
    else
        printf("\n Receive OK & Check OK\n");

    /* Disable UART0 IRQ */
    NVIC_DisableIRQ(UART0_IRQn);

    /* Disable RDA\RLS\RTO Interrupt */
    UART_DISABLE_INT(UART0, (UART_IER_RDA_IEN_Msk | UART_IER_RLS_IEN_Msk | UART_IER_RTO_IEN_Msk));

}
