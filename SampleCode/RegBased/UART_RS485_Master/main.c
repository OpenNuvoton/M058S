/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/07/13 4:13p $
 * @brief
 *           Transmit and receive data in UART RS485 mode.
 *           This sample code needs to work with UART_RS485_Slave.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M058S.h"


#define PLLCON_SETTING  CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK       50000000

#define RXBUFSIZE       128

#define MATCH_ADDRSS    0xC0
#define UNMATCH_ADDRSS  0xB1



/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
int32_t main(void);
void RS485_SendAddressByte(uint8_t u8data);
void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
void RS485_9bitModeMaster(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Control  (Address Byte: Parity Bit =1 , Data Byte:Parity Bit =0)                        */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_SendAddressByte(uint8_t u8data)
{
    uint32_t u32TimeOutCnt;

    /* Set UART parity as MARK */
    UART0->LCR = (UART_WORD_LEN_8 | UART_PARITY_MARK | UART_STOP_BIT_1);

    /* Send data */
    UART0->THR = u8data;

    /* Wait Tx empty */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(UART0->FSR & UART_FSR_TE_FLAG_Msk))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for UART Tx empty time-out!\n");
            break;
        }
    }
}

void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t u32Count, u32TimeOutCnt;

    /* Set UART parity as SPACE */
    UART0->LCR = (UART_WORD_LEN_8 | UART_PARITY_SPACE | UART_STOP_BIT_1);

    /* Send data */
    for(u32Count = 0; u32Count != u32WriteBytes; u32Count++)
    {
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(!(UART0->FSR & UART_FSR_TE_FLAG_Msk))    /* Wait Tx empty */
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for UART Tx empty time-out!\n");
                return;
            }
        }

        UART0->THR = pu8TxBuf[u32Count]; /* Send UART Data from buffer */
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeMaster()
{
    int32_t i32;
    uint8_t g_u8SendData[RXBUFSIZE] = {0};

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


    /*
        The sample code is used to test RS485 9-bit mode.
        It needs two Module test board and enables semihosted to complete the test.
        Master:
            1.Set AUD mode and HW will control RTS pin. LEV_RTS is set to '0'.
            2.Master will send four different address with 10 bytes data to test Slave.
            3.Address bytes : the parity bit should be '1'. (Set UA_LCR = 0x2B)
            4.Data bytes : the parity bit should be '0'. (Set UA_LCR = 0x3B)
            5.RTS pin is low in idle state. When master is sending, RTS pin will be pull high.

        Slave:
            1.Set AAD and AUD mode firstly. LEV_RTS is set to '0'.
            2.The received byte, parity bit is '1' , is considered "ADDRESS".
            3.The received byte, parity bit is '0' , is considered "DATA".  (Default)
            4.AAD: The slave will ignore any data until ADDRESS match ADDR_MATCH value.
              When RLS and RDA interrupt is happened, it means the ADDRESS is received.
              Check if RS485_ADD_DETF is set and read UA_RBR to clear ADDRESS stored in RX FIFO.

              NMM: The slave will ignore data byte until disable RX_DIS.
              When RLS and RDA interrupt is happened,it means the ADDRESS is received.
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

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|            RS485  Function Test (9-bit Master)              |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("| The function will send address with 128 data bytes to test  |\n");
    printf("| RS485 9-bit mode. Please connect TX/RX to another board and |\n");
    printf("| wait its ready to receive.                                  |\n");
    printf("| Press any key to start...                                   |\n");
    printf("+-------------------------------------------------------------+\n\n");
    GetChar();

    /* Select UART RS485 function mode */
    UART0->FUN_SEL = UART_FUNC_SEL_RS485;

    /* Set RS485-Master as AUD mode */
    /* Enable AUD mode to HW control RTS pin automatically */
    /* You also can use GPIO to control RTS pin for replacing AUD mode */
    UART0->ALT_CSR = UART_ALT_CSR_RS485_AUD_Msk;

    /* Set RTS pin active level as high level active */
    UART0->MCR = (UART0->MCR & (~UART_MCR_LEV_RTS_Msk)) | UART_RTS_IS_HIGH_LEV_ACTIVE;

    /* Set TX delay time */
    UART0->TOR = 0x2000;

    /* Prepare Data to transmit */
    for(i32 = 0; i32 < RXBUFSIZE; i32++)
    {
        g_u8SendData[i32] = i32 & 0xFF;
    }

    /* Send address and data for test */
    printf("Send Address 0x%x and data 0~127\n", MATCH_ADDRSS);
    RS485_SendAddressByte(MATCH_ADDRSS);
    RS485_SendDataByte(g_u8SendData, RXBUFSIZE);

    printf("Transfer Done\n");

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

    /* UART RS485 sample master function */
    RS485_9bitModeMaster();

    while(1);

}
