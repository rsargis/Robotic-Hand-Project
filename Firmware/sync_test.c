/*Test for sync motor movement (single finger)*/

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

    printf("\n=== Testing SCS0009 Sync Servo Movement ===\n");

    int ID1 = 7;
    int ID2 = 8;

    SCS_SyncTarget_t targets0[2];
    SCS_SyncTarget_t targets1[2];
    SCS_SyncTarget_t targets2[2];

    targets0[0].id = ID1;
    targets0[0].position = 255;
    targets0[0].time = 500;

    targets0[1].id = ID2;
    targets0[1].position = 767;
    targets0[1].time = 500;

    targets1[0].id = ID1;
    targets1[0].position = 767;
    targets1[0].time = 500;

    targets1[1].id = ID2;
    targets1[1].position = 255;
    targets1[1].time = 500;

    targets2[0].id = ID1;
    targets2[0].position = 511;
    targets2[0].time = 500;

    targets2[1].id = ID2;
    targets2[1].position = 511;
    targets2[1].time = 500;

    // Set non-zero position limits to force Servo/Positional Mode
    scs_set_limits(ID1, 255, 767);
    delay_ms(100);
    debug_dump_registers(ID1);

    // Enable torque
    scs_set_torque(ID1, true);
    delay_ms(100);
    debug_dump_registers(ID1);

     // Enable torque
    scs_set_torque(ID2, true);
    delay_ms(100);
    debug_dump_registers(ID2);

    scs_clear_speed(ID1);
    scs_clear_speed(ID2);

    int count = 0;
    while (1){
        printf("Moving to first position\n");
        scs_sync_write_position_time(targets0, 2);
        delay_ms(1000);
        printf("Moving to second position\n");
        scs_sync_write_position_time(targets1, 2);
        delay_ms(1000);
        printf("Moving to third position\n");
        scs_sync_write_position_time(targets2, 2);
        delay_ms(1000);
        count++;
        if (count >= 5){
            printf("Reset to neutral position\n");
            break;
        }
    }
}