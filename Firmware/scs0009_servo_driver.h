/* SCS0009 Servo Driver header file */
#ifndef SCS0009_SERVO_DRIVER_H
#define SCS0009_SERVO_DRIVER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

//Instructions
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_REG_ACTION 0x05
#define INST_SYNC_READ 0x82
#define INST_SYNC_WRITE 0x83

//Register Values
#define SCS_REG_MIN_ANGLE_LIMIT   0x09
#define SCS_REG_MAX_ANGLE_LIMIT   0x0B
#define SCS_REG_TORQUE_ENABLE     0x28
#define SCS_REG_GOAL_POSITION     0x2A
#define SCS_REG_GOAL_TIME         0x2C
#define SCS_REG_GOAL_SPEED        0x2E
#define SCS_REG_LOCK              0x30
#define SCS_REG_PRESENT_POSITION  0x38
#define SCS_REG_PRESENT_SPEED     0x3A
#define SCS_REG_PRESENT_VOLTAGE   0x3E
#define SCS_REG_PRESENT_TEMP      0x3F

//Function pointer for UART transmissions
typedef void (*scs_uart_tx_fn) (const uint8_t *data, size_t length);
// Typedef for UART RX callback: returns the number of bytes actually read within timeout_ms
typedef size_t (*scs_uart_rx_fn)(uint8_t *buffer, size_t length, uint32_t timeout_ms);

//Status struct
typedef struct{
    uint16_t position;
    uint16_t speed;
    uint8_t voltage;
    uint8_t temperature;
} SCS_Status_t;

typedef struct{
    uint8_t id;
    uint16_t position;
    uint16_t time;
} SCS_SyncTarget_t;

// Function prototypes for SCS0009 servo control
void scs_clear_speed(uint8_t servo_id);
void scs_init(scs_uart_tx_fn tx_func, scs_uart_rx_fn rx_func);
void scs_set_id(uint8_t current_id, uint8_t new_id);
void scs_set_torque(uint8_t servo_id, bool enable);
void scs_set_position(uint8_t servo_id, uint16_t position);
void scs_set_position_time(uint8_t servo_id, uint16_t position, uint16_t time_ms);

// Return status codes for read/status operations
#define SCS_OK                  0
#define SCS_ERR_INVALID_ARG     1
#define SCS_ERR_TIMEOUT         2
#define SCS_ERR_HEADER_NOT_FOUND 3
#define SCS_ERR_SERVO_ERROR     4
#define SCS_ERR_CHECKSUM        5

int scs_get_status(uint8_t servo_id, SCS_Status_t *status);
int scs_read_reg(uint8_t servo_id, uint8_t start_addr, uint8_t length, uint8_t *out_buf);
void scs_set_limits(uint8_t servo_id, uint16_t min_limit, uint16_t max_limit);
void scs_sync_write_position(SCS_SyncTarget_t *targets, size_t count);
void scs_sync_write_position_time(SCS_SyncTarget_t *targets, size_t count);

#endif // SCS0009_SERVO_DRIVER_H