/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    A project template for M251 MCU.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/

#include <stdio.h>
#include "scs0009_servo_driver.h"
#include "NuMicro.h"

#define DIR_PIN_HIGH (PA0 = 1) //TX
#define DIR_PIN_LOW  (PA0 = 0) //RX


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    CLK_EnableSysTick(CLK_CLKSEL0_STCLKSEL_HCLK, SystemCoreClock / 1000);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

void delay_ms(uint32_t ms)
{
    while(ms > 0)
    {
        CLK_SysTickDelay(1000); //delay in us, 1000us = 1ms
        ms--;
    }
}

void uart_tx(const uint8_t *data, uint32_t length)
{
    DIR_PIN_HIGH; // Set direction to TX

    for (size_t i = 0; i < length; i++)
    {
        UART_WRITE(UART0, data[i]);     // Send a byte
    }

    while (!UART_IS_TX_EMPTY(UART0));

    DIR_PIN_LOW; // Set direction back to RX
}

size_t uart_rx(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms){
    DIR_PIN_LOW; // Set direction to RX

    size_t count = 0;

    for (uint32_t i = 0; i < timeout_ms*1000; i++) {
        if (!UART_GET_RX_EMPTY(UART0)) {
            buffer[count++] = UART_READ(UART0);
            if (count >= buffer_size) {
                break; // Buffer full
            }
        }
    }
    return count;
}

/*
 * This is a template project for M251 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */
int main()
{
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT); // Set PA.0 as output for direction control
    DIR_PIN_LOW; // Set initial direction to RX

    scs_init(uart_tx, uart_rx);

    SCS_Status_t status;
    bool success = scs_get_status(1, &status);

    if (success) {
        // Success! Received valid header, matching ID, and correct checksum
        printf("Success! Pos: %d, Speed: %d, Temp: %d C, Volts: %d V\n", 
               status.position, status.speed, status.temperature, status.voltage);
    } else {
        // Communication failed (Check baud rate, wiring, common ground, or DIR pin timing)
        printf("Error: Failed to read servo status!\n");
    }

    scs_set_limits(1, 100, 900);

    scs_set_position(1, 512);
    delay_ms(1000);

    scs_set_position(1, 300);
    delay_ms(1000);

    scs_get_status(1, &status);
    printf("New Position: %d\n", status.position);

    /* Got no where to go, just loop forever */
    while (1);
}


/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
