/* SCS0009 Servo Driver 
Functions for driving individual SECS0009 servos
*/

#include "scs0009_servo_driver.h"

static scs_uart_tx_fn g_tx_func = NULL;
static scs_uart_rx_fn g_rx_func = NULL;

static uint8_t calculate_checksum(uint8_t id, uint8_t length, uint8_t instruction, const uint8_t *params, uint8_t param_len) {
    uint32_t sum = id + length + instruction;
    for (uint8_t i = 0; i < param_len; i++) {
        sum += params[i];
    }
    return (uint8_t)(~sum & 0xFF);
}

void scs_init(scs_uart_tx_fn tx_func, scs_uart_rx_fn rx_func){
    g_tx_func = tx_func;
    g_rx_func = rx_func;
}

/*
void scs_set_torque(uint8_t servo_id, bool enable){

}
*/

void scs_set_position(uint8_t servo_id, uint16_t position){
    if (g_tx_func == NULL) return;

    uint8_t packet[9];
    uint8_t length = 5;

    uint8_t params[3];
    params[0] = SCS_REG_GOAL_POSITION;
    params[1] = (uint8_t)((position >> 8) & 0xFF); // High byte
    params[2] = (uint8_t)(position & 0xFF); // Low byte

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servo_id;
    packet[3] = length;
    packet[4] = INST_WRITE;
    packet[5] = params[0];
    packet[6] = params[1];
    packet[7] = params[2];
    packet[8] = calculate_checksum(servo_id, length, INST_WRITE, params, 3);

    g_tx_func(packet, sizeof(packet));
}

bool scs_get_status(uint8_t servo_id, SCS_Status_t *status){
    if (g_tx_func == NULL || status == NULL || g_rx_func == NULL) return false;

    uint8_t tx_packet[8];
    uint8_t length = 4;
    uint8_t params[2] = { SCS_REG_PRESENT_POSITION, 0x08 }; // Address 0x38, Read 8 Bytes

    tx_packet[0] = 0xFF;
    tx_packet[1] = 0xFF;
    tx_packet[2] = servo_id;
    tx_packet[3] = length;
    tx_packet[4] = INST_READ;
    tx_packet[5] = params[0];
    tx_packet[6] = params[1];
    tx_packet[7] = calculate_checksum(servo_id, length, INST_READ, params, 2);

    g_tx_func(tx_packet, sizeof(tx_packet));

    uint8_t rx_buffer[14];
    size_t bytes_received = g_rx_func(rx_buffer, sizeof(rx_buffer), 10);

    if (bytes_received < sizeof(rx_buffer)) {
        return false; //timout or missing bytes
    }

    if (rx_buffer[0] != 0xFF || rx_buffer[1] != 0xFF || rx_buffer[2] != servo_id) {
        return false; // Invalid header or mismatched Servo ID
    }

    uint8_t error_code = rx_buffer[4];
    if (error_code != 0x00) {
        return false; // Servo reported an internal error
    }

    uint32_t checksum_calc = rx_buffer[2] + rx_buffer[3] + rx_buffer[4];
    for (size_t i = 5; i < 13; i++) {
        checksum_calc += rx_buffer[i];
    }
    uint8_t checksum_received = rx_buffer[13];
    if ((uint8_t)(~checksum_calc & 0xFF) != checksum_received) {
        return false; // Checksum mismatch
    }

    // Parse the received data
    status->position = (((uint16_t)rx_buffer[5] << 8) | (rx_buffer[6]));
    status->speed = (((uint16_t)rx_buffer[7] << 8) | (rx_buffer[8]));
    status->voltage = rx_buffer[11];
    status->temperature = rx_buffer[12];

    return true;
}

void scs_set_limits(uint8_t servo_id, uint16_t min_limit, uint16_t max_limit){
    if (g_tx_func == NULL) return;

    uint8_t packet[11];
    uint8_t length = 7;

    uint8_t params[5];
    params[0] = SCS_REG_MIN_ANGLE_LIMIT;
    params[1] = (uint8_t)((min_limit >> 8) & 0xFF); // High byte
    params[2] = (uint8_t)(min_limit & 0xFF); // Low byte
    params[3] = (uint8_t)((max_limit >> 8) & 0xFF); // High byte
    params[4] = (uint8_t)(max_limit & 0xFF); // Low byte

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servo_id;
    packet[3] = length;
    packet[4] = INST_WRITE;
    packet[5] = params[0];
    packet[6] = params[1];
    packet[7] = params[2];
    packet[8] = params[3];
    packet[9] = params[4];
    packet[10] = calculate_checksum(servo_id, length, INST_WRITE, params, 5);

    g_tx_func(packet, sizeof(packet));
}