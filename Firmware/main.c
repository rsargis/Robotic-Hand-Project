/**************************************************************************//**
 * @file     main.c
 * @brief    Dedicated UART1 pin configuration for Feetech SCS Servo Bus
 ****************************************************************************/

#include <stdio.h>
#include "scs0009_servo_driver.h"
#include "NuMicro.h"

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 (Debug VCOM) and UART1 (Servo Bus) Module Clocks */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART clock sources */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    SystemCoreClockUpdate();

    /* System Default VCOM Multi-Function Pin setup for Serial Monitor */
    Uart0DefaultMPF();

    /* Set PB.2 -> UART1_RXD and PB.3 -> UART1_TXD */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk)) | 
                    (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void delay_ms(uint32_t ms)
{
    while(ms > 0)
    {
        CLK_SysTickDelay(1000); // 1000us = 1ms
        ms--;
    }
}

void servo_uart_tx(const uint8_t *data, size_t length) 
{
    // Clear any residual bytes before sending
    while (!UART_GET_RX_EMPTY(UART1)) {
        volatile uint8_t dummy = UART_READ(UART1);
        (void)dummy;
    }

    // Transmit packet
    for (size_t i = 0; i < length; i++)
    {
        while (UART_IS_TX_FULL(UART1));
        UART_WRITE(UART1, data[i]);
    }
    while (!UART_IS_TX_EMPTY(UART1));
}

size_t servo_uart_rx(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms)
{
    size_t count = 0;
    uint32_t timeout_us = timeout_ms * 1000;

    for (uint32_t t = 0; t < timeout_us; t += 10) 
    {
        while (!UART_GET_RX_EMPTY(UART1)) 
        {
            buffer[count++] = UART_READ(UART1);
            if (count >= buffer_size) {
                return count;
            }
        }
        CLK_SysTickDelay(10); // Wait 10 us between checks
    }
    return count;
}

void debug_dump_registers(uint8_t servo_id)
{
    uint8_t buf[2];

    int res = scs_read_reg(servo_id, SCS_REG_MIN_ANGLE_LIMIT, 2, buf);
    if (res == SCS_OK) {
        uint16_t min_limit = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
        printf("[servo %u] Min Limit: %u\n", servo_id, min_limit);
    } else {
        printf("[servo %u] Failed to read min limit (error code: %d)\n", servo_id, res);
    }

    res = scs_read_reg(servo_id, SCS_REG_MAX_ANGLE_LIMIT, 2, buf);
    if (res == SCS_OK) {
        uint16_t max_limit = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
        printf("[servo %u] Max Limit: %u\n", servo_id, max_limit);
    } else {
        printf("[servo %u] Failed to read max limit (error code: %d)\n", servo_id, res);
    }

    res = scs_read_reg(servo_id, SCS_REG_TORQUE_ENABLE, 1, buf);
    if (res == SCS_OK) {
        printf("[servo %u] Torque Enable: %u\n", servo_id, buf[0]);
    } else {
        printf("[servo %u] Failed to read torque enable (error code: %d)\n", servo_id, res);
    }
}

int main(void)
{
    SYS_Init();

    UART_Open(UART0, 115200);
    UART_Open(UART1, 1000000); // 1 Mbps

    scs_init(servo_uart_tx, servo_uart_rx);

    printf("\n=== Testing SCS0009 Servo Movement ===\n");

    // Assign ID 1
    scs_set_id(0xFE, 1);
    delay_ms(100);

    // Set non-zero position limits to force Servo/Positional Mode
    scs_set_limits(1, 100, 900);
    delay_ms(100);
    debug_dump_registers(1);

    // Enable torque
    scs_set_torque(1, true);
    delay_ms(100);
    debug_dump_registers(1);

    scs_clear_speed(1);

    while (1) {
        printf("Moving to 100\n");
        scs_set_position_time(1, 100, 1000);
        delay_ms(2000);

        SCS_Status_t status;
        int status_res = scs_get_status(1, &status);
        if (status_res == SCS_OK) {
            printf("  -> present position: %u\n", status.position);
        } else {
            printf("  -> status read failed (error code: %d)\n", status_res);
        }

        printf("Moving to 200\n");
        scs_set_position_time(1, 900, 1000);
        delay_ms(2000);

        status_res = scs_get_status(1, &status);
        if (status_res == SCS_OK) {
            printf("  -> present position: %u\n", status.position);
        } else {
            printf("  -> status read failed (error code: %d)\n", status_res);
        }
    }
}