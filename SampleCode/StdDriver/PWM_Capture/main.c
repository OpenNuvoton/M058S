/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/02/06 10:22a $
 * @brief    Capture the PWMA Channel 1 waveform by PWMA Channel 2.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M058S.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000

extern char GetChar(void);
/**
 * @brief       PWMA IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWMA interrupt event
 */
void PWMA_IRQHandler(void)
{
    uint32_t u32CapIntFlag1;

    /* Handle PWMA Capture function */
    u32CapIntFlag1 = PWMA->CCR2;

    /* PWMA channel 2 Capture interrupt */
    if(u32CapIntFlag1 & PWM_CCR2_CAPIF2_Msk)
    {
        PWMA->CCR2 &= (PWM_CCR_MASK | PWM_CCR2_CAPIF2_Msk);
    }
}

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* u32Count[4] : Keep the internal counter value when input signal rising / falling     */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
void CalPeriodTime(PWM_T *PWM, uint32_t u32Ch)
{
    uint16_t u32Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeroid, u16LowPeroid, u16TotalPeroid;

    /* Clear Capture Falling Indicator (Time A) */
    PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH);

    /* Wait for Capture Falling Indicator  */
    while(PWM_GetCaptureIntFlag(PWM, u32Ch) < 2);

    /* Clear Capture Falling Indicator (Time B)*/
    PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH);

    u32i = 0;

    while(u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        while(PWM_GetCaptureIntFlag(PWM, u32Ch) < 2);

        /* Clear Capture Falling and Rising Indicator */
        PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH | PWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Falling Latch Counter Data */
        u32Count[u32i++] = PWM_GET_CAPTURE_FALLING_DATA(PWM, u32Ch);

        /* Wait for Capture Rising Indicator */
        while(PWM_GetCaptureIntFlag(PWM, u32Ch) < 2);

        /* Clear Capture Rising Indicator */
        PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Rising Latch Counter Data */
        u32Count[u32i++] = PWM_GET_CAPTURE_RISING_DATA(PWM, u32Ch);
    }

    u16RisingTime = u32Count[1];

    u16FallingTime = u32Count[0];

    u16HighPeroid = u32Count[1] - u32Count[2];

    u16LowPeroid = 0x10000 - u32Count[1];

    u16TotalPeroid = 0x10000 - u32Count[2];

    printf("\nPWM generate: \nHigh Period=7199 ~ 7201, Low Period=16799 ~ 16801, Total Period=23999 ~ 24001\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeroid, u16LowPeroid, u16TotalPeroid);
    if((u16HighPeroid < 7199) || (u16HighPeroid > 7201) || (u16LowPeroid < 16799) || (u16LowPeroid > 16801) || (u16TotalPeroid < 23999) || (u16TotalPeroid > 24001))
        printf("Capture Test Fail!!\n");
    else
        printf("Capture Test Pass!!\n");
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for IRC22M clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLK_N_Msk;

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC22M_EN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCON = PLLCON_SETTING;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & (CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk | CLK_CLKSTATUS_OSC22M_STB_Msk)));

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_HCLK_DIV2 | CLK_CLKSEL0_HCLK_S_PLL;

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_PWM01_EN_Msk | CLK_APBCLK_PWM23_EN_Msk;

    /* IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_PLL;

    /* IP clock source */
    CLK->CLKSEL1 |= CLK_CLKSEL1_PWM01_S_HXT | CLK_CLKSEL1_PWM23_S_HXT;

    /* Reset PWMA channel0~channel3 */
    SYS->IPRSTC2 = SYS_IPRSTC2_PWM03_RST_Msk;
    SYS->IPRSTC2 = 0;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART RXD and TXD  */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD | SYS_MFP_P31_TXD);
    /* Set P2 multi-function pins for PWMA Channel 1 and 2 */
    SYS->P2_MFP &= ~(SYS_MFP_P21_Msk | SYS_MFP_P22_Msk);
    SYS->P2_MFP |= (SYS_MFP_P21_PWM1 | SYS_MFP_P22_PWM2);
}


void UART0_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PLL_CLOCK, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}



/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWMA channel 2 to capture\n  the signal from PWMA channel 1.\n");
    printf("  I/O configuration:\n");
    printf("    PWM1(P2.1 PWMA channel 1) <--> PWM2(P2.2 PWMA channel 2)\n\n");
    printf("Use PWMA Channel 2(P2.2) to capture the PWMA Channel 1(P2.1) Waveform\n");

    while(1)
    {
        printf("Press any key to start PWM Capture Test\n");
        GetChar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the PWMA Channel 1 as PWM output function.                                               */
        /*--------------------------------------------------------------------------------------*/

        /* Assume PWM output frequency is 250Hz and duty ratio is 30%, user can calculate PWM settings by follows.
           duty ratio = (CMR+1)/(CNR+1)
           cycle time = CNR+1
           High level = CMR+1
           PWM clock source frequency = __HXT = 12000000
           (CNR+1) = PWM clock source frequency/prescaler/clock source divider/PWM output frequency
                   = 12000000/2/1/250 = 24000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 23999
           duty ratio = 30% ==> (CMR+1)/(CNR+1) = 30%
           CMR = 7199
           Prescale value is 1 : prescaler= 2
           Clock divider is PWM_CSR_DIV1 : clock divider =1
        */

        /* set PWMA channel 1 output configuration */
        PWM_ConfigOutputChannel(PWMA, PWM_CH1, 250, 30);

        /* Enable PWM Output path for PWMA channel 1 */
        PWM_EnableOutput(PWMA, 0x2);

        /* Enable Timer for PWMA channel 1 */
        PWM_Start(PWMA, 0x2);

        /*--------------------------------------------------------------------------------------*/
        /* Set the PWMA channel 2  for capture function                                         */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = __HXT = 12000000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/clock source divider/minimum input frequency
                   = 12000000/2/1/250 = 24000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)
        */

        /* set PWMA channel 2 capture configuration */
        PWM_ConfigCaptureChannel(PWMA, PWM_CH2, 166, 0);

        /* Enable capture falling edge interrupt for PWMA channel 2 */
        PWM_EnableCaptureInt(PWMA, PWM_CH2, PWM_CAPTURE_INT_FALLING_LATCH);

        /* Enable PWMA NVIC interrupt */
        NVIC_EnableIRQ((IRQn_Type)(PWMA_IRQn));

        /* Enable Timer for PWMA channel 2  */
        PWM_Start(PWMA, 0x4);

        /* Enable Capture Function for PWMA channel 2 */
        PWM_EnableCapture(PWMA, 0x4);

        /* Wait until PWMA channel 2 Timer start to count */
        while(PWMA->PDR2 == 0);

        /* Capture the Input Waveform Data */
        CalPeriodTime(PWMA, PWM_CH2);
        /*------------------------------------------------------------------------------------------------------*/
        /* Stop PWMA channel 1 (Recommended procedure method 1)                                                 */
        /* Set PWM Timer loaded value(CNR) as 0. When PWM internal counter(PDR) reaches to 0, disable PWM Timer */
        /*------------------------------------------------------------------------------------------------------*/

        /* Set PWMA channel 1 loaded value as 0 */
        PWM_Stop(PWMA, 0x2);

        /* Wait until PWMA channel 1 Timer Stop */
        while(PWMA->PDR1 != 0);

        /* Disable Timer for PWMA channel 1 */
        PWM_ForceStop(PWMA, 0x2);

        /* Disable PWM Output path for PWMA channel 1 */
        PWM_DisableOutput(PWMA, 0x2);

        /*------------------------------------------------------------------------------------------------------*/
        /* Stop PWMA channel 2 (Recommended procedure method 1)                                                 */
        /* Set PWM Timer loaded value(CNR) as 0. When PWM internal counter(PDR) reaches to 0, disable PWM Timer */
        /*------------------------------------------------------------------------------------------------------*/

        /* Disable PWMA NVIC */
        NVIC_DisableIRQ((IRQn_Type)(PWMA_IRQn));

        /* Set loaded value as 0 for PWMA channel 2 */
        PWM_Stop(PWMA, 0x4);

        /* Wait until PWMA channel 2 current counter reach to 0 */
        while(PWMA->PDR2 != 0);

        /* Disable Timer for PWMA channel 2 */
        PWM_ForceStop(PWMA, 0x4);

        /* Disable Capture Function and Capture Input path for  PWMA channel 2*/
        PWM_DisableCapture(PWMA, 0x4);

        /* Clear Capture Interrupt flag for PWMA channel 2*/
        PWM_ClearCaptureIntFlag(PWMA, PWM_CH2, PWM_CAPTURE_INT_FALLING_LATCH);
    }
}




