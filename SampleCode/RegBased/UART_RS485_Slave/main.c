/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 6 $
 * $Date: 15/07/13 4:13p $
 * @brief
 *           Transmit and receive data in UART RS485 mode.
 *           This sample code needs to work with UART_RS485_Master.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M058S.h"


#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000


#define RXBUFSIZE           128

#define IS_USE_RS485NMM     0      //1:Select NMM_Mode , 0:Select AAD_Mode
#define MATCH_ADDRSS        0xC0


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
int32_t main(void);
void RS485_HANDLE(void);
void RS485_9bitModeSlave(void);

volatile int32_t r_pointer = 0;
uint8_t g_u8RecData[RXBUFSIZE] = {0};


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    RS485_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* RS485 Callback function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_HANDLE()
{
    volatile uint32_t addr = 0;
    volatile uint32_t u32IntSts = UART0->ISR;

    /* RLS INT & RDA INT */  //For RS485 Detect Address
    if((u32IntSts & UART_ISR_RLS_INT_Msk) && (u32IntSts & UART_ISR_RDA_INT_Msk))
    {
        if(UART0->FSR & UART_FSR_RS485_ADD_DETF_Msk)    /* ADD_IF, RS485 mode */
        {
            addr = UART0->RBR;
            UART0->FSR = UART_FSR_RS485_ADD_DETF_Msk;   /* clear ADD_IF flag */

#if (IS_USE_RS485NMM ==1) //RS485_NMM

            /* if address match, enable RX to receive data, otherwise to disable RX */
            /* In NMM mode, user can decide multi-address filter */
            /* In AAD mode, only one address can set */
            if(addr == MATCH_ADDRSS)
            {
                UART0->FCR &= ~UART_FCR_RX_DIS_Msk;   /* Enable RS485 RX */
            }
            else
            {
                UART0->FCR |= UART_FCR_RX_DIS_Msk;    /* Disable RS485 RX */
                UART0->FCR |= UART_FCR_RFR_Msk;       /* Clear data from RX FIFO */
            }
#endif
        }
    }
    else if((u32IntSts & UART_ISR_RDA_INT_Msk) || (u32IntSts & UART_ISR_TOUT_INT_Msk))  /* Rx Ready or Time-out INT */
    {
        /* Handle received data */
        g_u8RecData[r_pointer++] = UART0->RBR;
    }

    else if(u32IntSts & UART_ISR_BUF_ERR_INT_Msk)     /* Buffer Error INT */
    {
        printf("\nBuffer Error...\n");
        UART0->FSR = (UART_FSR_RX_OVER_IF_Msk | UART_FSR_TX_OVER_IF_Msk);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Receive Test  (IS_USE_RS485NMM: 0:AAD  1:NMM)                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeSlave()
{
    uint32_t i, u32Err = 0;

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     Pin Configure                                           |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     _______                                    _______      |\n");
    printf("|    |       |                                  |       |     |\n");
    printf("|    |Master |--- TXD(P3.1)        RXD(P3.0) ---| Slave |     |\n");
    printf("|    |       |--- RTS(P0.3)        RTS(P0.3) ---|       |     |\n");
    printf("|    |_______|                                  |_______|     |\n");
    printf("|                                                             |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Please enable semihosted to show messages on debug session.|\n");
    printf("|  Keil users must define DEBUG_ENABLE_SEMIHOST in both C/C++ |\n");
    printf("|  and Asm preprocessor symbols.                              |\n");
    printf("|  IAR users must define DEBUG_ENABLE_SEMIHOST in both C/C++  |\n");
    printf("|  Compiler and Assembler preprocessor symbols.               |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|            RS485  Function Test (9-bit Slave)               |\n");
    printf("+-------------------------------------------------------------+\n");

    /*
        The sample code is used to test RS485 9-bit mode.
        It needs two Module test board and enables semihosted to complete the test.

        Master:
            1.Set AUD mode and HW will control RTS pin. LEV_RTS is set to '0'.
            2.Master will send four different address with 10 bytes data to test Slave.
            3.Address bytes : the parity bit should be '1'. (Set UA_LCR = 0x2B)
            4.Data bytes : the parity bit should be '0'. (Set UA_LCR = 0x3B)
            5.RTS pin is low in idle state. When master is sending,
              RTS pin will be pull high.

        Slave:
            1.Set AAD and AUD mode firstly. LEV_RTS is set to '0'.
            2.The received byte, parity bit is '1' , is considered "ADDRESS".
            3.The received byte, parity bit is '0' , is considered "DATA".  (Default)
            4.AAD: The slave will ignore any data until ADDRESS match ADDR_MATCH value.
              When RLS and RDA interrupt is happened, it means the ADDRESS is received.
              Check if RS485_ADD_DETF is set and read UA_RBR to clear ADDRESS stored in RX FIFO.

              NMM: The slave will ignore data byte until disable RX_DIS.
              When RLS and RDA interrupt is happened, it means the ADDRESS is received.
              Check the ADDRESS is match or not by user in UART_IRQHandler.
              If the ADDRESS is match, clear RX_DIS bit to receive data byte.
              If the ADDRESS is not match, set RX_DIS bit to avoid data byte stored in FIFO.

        Note: User can measure transmitted data waveform on TXD and RXD pin.
              RTS pin is used for RS485 transceiver to control transmission direction.
              RTS pin is low in idle state. When master is sending data, RTS pin will be pull high.
              The connection to RS485 transceiver is as following figure for reference.
               __________     ___________      ___________      __________
              |          |   |           |    |           |    |          |
              |Master    |   |RS485      |    |RS485      |    |Slave     |
              | UART_TXD |---|Transceiver|<==>|Transceiver|----| UART_RXD |
              | UART_nRTS|---|           |    |           |----| UART_nRTS|
              |__________|   |___________|    |___________|    |__________|
    */

    /* Select UART RS485 function mode */
    UART0->FUN_SEL = UART_FUNC_SEL_RS485;

    /* Set Data Format, only need parity enable whenever parity ODD/EVEN */
    UART0->LCR = (UART_WORD_LEN_8 | UART_PARITY_EVEN | UART_STOP_BIT_1);

    /* Set RTS pin active level as high level active */
    UART0->MCR = (UART0->MCR & (~UART_MCR_LEV_RTS_Msk)) | UART_RTS_IS_HIGH_LEV_ACTIVE;

#if(IS_USE_RS485NMM == 1)

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|    Normal Multidrop Operation Mode                          |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.              |\n");
    printf("| Only Address %x data can receive                            |\n", MATCH_ADDRSS);
    printf("+-------------------------------------------------------------+\n");

    /* Set RX_DIS enable before set RS485-NMM mode */
    UART0->FCR |= UART_FCR_RX_DIS_Msk;

    /* Set RS485-NMM Mode, RS485 address detection enable */
    UART0->ALT_CSR = UART_ALT_CSR_RS485_NMM_Msk    |
                     UART_ALT_CSR_RS485_ADD_EN_Msk |
                     UART_ALT_CSR_RS485_AUD_Msk;

#else
    printf("+-------------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.              |\n");
    printf("|    Auto Address Match Operation Mode                        |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|Only Address %x, data can receive                            |\n", MATCH_ADDRSS);
    printf("+-------------------------------------------------------------+\n");

    /* Set RS485-AAD Mode, address match is 0xC0 and RS485 address detection enable */
    UART0->ALT_CSR = UART_ALT_CSR_RS485_AAD_Msk    |
                     UART_ALT_CSR_RS485_ADD_EN_Msk |
                     UART_ALT_CSR_RS485_AUD_Msk    |
                     ((uint32_t)(MATCH_ADDRSS) << UART_ALT_CSR_ADDR_MATCH_Pos) ;

#endif

    /* Enable RDA\RLS\Time-out Interrupt */
    UART0->IER |= UART_IER_RDA_IEN_Msk | UART_IER_RLS_IEN_Msk | UART_IER_RTO_IEN_Msk;
    NVIC_EnableIRQ(UART0_IRQn);

    r_pointer = 0;

    /* Check Rx empty, otherwise read Rx */
    printf("Starting to recevice %d bytes data...\n", RXBUFSIZE);

    while(r_pointer < RXBUFSIZE);

    /* Compare Data */
    for(i = 0; i < RXBUFSIZE; i++)
    {
        if(g_u8RecData[i] != (i & 0xFF))
        {
            u32Err = 1;
            break;
        }
    }

    if( u32Err )
        printf("Compare Data Failed\n");
    else
        printf("\n Receive OK & Check OK\n");

    /* Flush Rx FIFO */
    UART0->FCR |= UART_FCR_RFR_Msk;

    /* Disable RDA\RLS\Time-out Interrupt */
    UART_DISABLE_INT(UART0, (UART_IER_RDA_IEN_Msk | UART_IER_RLS_IEN_Msk | UART_IER_RTO_IEN_Msk));
    NVIC_DisableIRQ(UART0_IRQn);

    printf("\nEnd test\n");

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

    /* Set P0 multi-function pins for UART0 RTS */
    SYS->P0_MFP = (SYS->P0_MFP & (~SYS_MFP_P03_Msk)) | SYS_MFP_P03_RTS;

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

    /* UART RS485 sample slave function */
    RS485_9bitModeSlave();

    while(1);

}
