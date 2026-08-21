/* SCS0009 Servo Driver 
   Functions for driving individual SCS0009 servos
*/

#include "scs0009_servo_driver.h"

static scs_uart_tx_fn g_tx_func = NULL;
static scs_uart_rx_fn g_rx_func = NULL;

void scs_clear_speed(uint8_t servo_id) {
    if (g_tx_func == NULL) return;

    uint8_t packet[9];
    uint8_t length = 5;
    // Big-endian: High byte first, then low byte
    uint8_t params[3] = { SCS_REG_GOAL_SPEED, 0x00, 0x00 };

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servo_id;
    packet[3] = length;
    packet[4] = INST_WRITE;
    packet[5] = params[0];
    packet[6] = params[1];
    packet[7] = params[2];
    packet[8] = ~((servo_id + length + INST_WRITE + params[0] + params[1] + params[2]) & 0xFF);

    g_tx_func(packet, sizeof(packet));
}

static uint8_t calculate_checksum(uint8_t id, uint8_t length, uint8_t instruction, const uint8_t *params, uint8_t param_len) {
    uint32_t sum = id + length + instruction;
    for (uint8_t i = 0; i < param_len; i++) {
        sum += params[i];
    }
    return (uint8_t)(~sum & 0xFF);
}

static void scs_delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 12000; j++) {
            // Empty loop (~1ms at 48 MHz)
        }
    }
}

void scs_init(scs_uart_tx_fn tx_func, scs_uart_rx_fn rx_func) {
    g_tx_func = tx_func;
    g_rx_func = rx_func;
}

void scs_set_id(uint8_t current_id, uint8_t new_id) {
    if (g_tx_func == NULL) return;

    uint8_t unlock_pkt[8] = { 0xFF, 0xFF, current_id, 4, INST_WRITE, SCS_REG_LOCK, 0x00, 0 };
    uint8_t u_params[2] = { SCS_REG_LOCK, 0x00 };
    unlock_pkt[7] = calculate_checksum(current_id, 4, INST_WRITE, u_params, 2);
    g_tx_func(unlock_pkt, 8);
    scs_delay_ms(50);

    uint8_t id_pkt[8] = { 0xFF, 0xFF, current_id, 4, INST_WRITE, 0x05, new_id, 0 };
    uint8_t id_params[2] = { 0x05, new_id };
    id_pkt[7] = calculate_checksum(current_id, 4, INST_WRITE, id_params, 2);
    g_tx_func(id_pkt, 8);
    scs_delay_ms(50);

    uint8_t lock_pkt[8] = { 0xFF, 0xFF, new_id, 4, INST_WRITE, SCS_REG_LOCK, 0x01, 0 };
    uint8_t l_params[2] = { SCS_REG_LOCK, 0x01 };
    lock_pkt[7] = calculate_checksum(new_id, 4, INST_WRITE, l_params, 2);
    g_tx_func(lock_pkt, 8);
    scs_delay_ms(50);
}

void scs_set_torque(uint8_t servo_id, bool enable) {
    if (g_tx_func == NULL) return;

    uint8_t packet[8];
    uint8_t length = 4;
    uint8_t params[2] = { SCS_REG_TORQUE_ENABLE, enable ? 0x01 : 0x00 };

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servo_id;
    packet[3] = length;
    packet[4] = INST_WRITE;
    packet[5] = params[0];
    packet[6] = params[1];
    packet[7] = calculate_checksum(servo_id, length, INST_WRITE, params, 2);

    g_tx_func(packet, sizeof(packet));
}

void scs_set_position(uint8_t servo_id, uint16_t position) {
    if (g_tx_func == NULL) return;

    uint8_t packet[9];
    uint8_t length = 5;

    // Big-Endian format: High byte first, then low byte
    uint8_t params[3];
    params[0] = SCS_REG_GOAL_POSITION;
    params[1] = (uint8_t)((position >> 8) & 0xFF);
    params[2] = (uint8_t)(position & 0xFF);

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

void scs_set_position_time(uint8_t servo_id, uint16_t position, uint16_t time_ms) {
    if (g_tx_func == NULL) return;

    uint8_t packet[11];
    uint8_t length = 7;

    // Big-Endian format for both position and time
    uint8_t params[5];
    params[0] = SCS_REG_GOAL_POSITION;
    params[1] = (uint8_t)((position >> 8) & 0xFF);
    params[2] = (uint8_t)(position & 0xFF);
    params[3] = (uint8_t)((time_ms >> 8) & 0xFF);
    params[4] = (uint8_t)(time_ms & 0xFF);

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servo_id;
    packet[3] = length;
    packet[4] = INST_WRITE;
    for (int i = 0; i < 5; i++) packet[5 + i] = params[i];
    packet[10] = calculate_checksum(servo_id, length, INST_WRITE, params, 5);

    g_tx_func(packet, sizeof(packet));
}

int scs_get_status(uint8_t servo_id, SCS_Status_t *status) {
    if (g_tx_func == NULL || status == NULL || g_rx_func == NULL) {
        return SCS_ERR_INVALID_ARG;
    }

    uint8_t tx_packet[8];
    uint8_t length = 4;
    uint8_t params[2] = { SCS_REG_PRESENT_POSITION, 0x08 };

    tx_packet[0] = 0xFF;
    tx_packet[1] = 0xFF;
    tx_packet[2] = servo_id;
    tx_packet[3] = length;
    tx_packet[4] = INST_READ;
    tx_packet[5] = params[0];
    tx_packet[6] = params[1];
    tx_packet[7] = calculate_checksum(servo_id, length, INST_READ, params, 2);

    g_tx_func(tx_packet, sizeof(tx_packet));

    uint8_t rx_raw[32];
    size_t bytes_received = g_rx_func(rx_raw, sizeof(rx_raw), 50);

    if (bytes_received < 6) {
        return SCS_ERR_TIMEOUT;
    }

    uint8_t *rx_buffer = NULL;
    for (size_t i = 0; i <= bytes_received - 6; i++) {
        if (rx_raw[i] == 0xFF && rx_raw[i+1] == 0xFF && (rx_raw[i+2] == servo_id || servo_id == 0xFE)) {
            rx_buffer = &rx_raw[i];
            break;
        }
    }

    if (rx_buffer == NULL) {
        return SCS_ERR_HEADER_NOT_FOUND;
    }

    uint8_t pkt_len = rx_buffer[3];
    uint8_t error_code = rx_buffer[4];
    if (error_code != 0x00) {
        return SCS_ERR_SERVO_ERROR;
    }

    uint32_t checksum_calc = 0;
    for (uint8_t i = 2; i <= (2 + pkt_len); i++) {
        checksum_calc += rx_buffer[i];
    }
    
    if ((uint8_t)(~checksum_calc & 0xFF) != rx_buffer[3 + pkt_len]) {
        return SCS_ERR_CHECKSUM;
    }

    // Big-Endian Unpacking: rx_buffer[5] is High byte, rx_buffer[6] is Low byte
    uint16_t raw_pos = (uint16_t)(((uint16_t)rx_buffer[5] << 8) | rx_buffer[6]);
    status->position = raw_pos & 0x03FF; 
    status->speed    = (uint16_t)(((uint16_t)rx_buffer[7] << 8) | rx_buffer[8]);
    status->voltage  = rx_buffer[11];
    status->temperature = rx_buffer[12];

    return SCS_OK;
}

int scs_read_reg(uint8_t servo_id, uint8_t start_addr, uint8_t length, uint8_t *out_buf) {
    if (g_tx_func == NULL || g_rx_func == NULL || out_buf == NULL || length == 0) {
        return SCS_ERR_INVALID_ARG;
    }

    uint8_t tx_packet[8];
    uint8_t pkt_length = 4;
    uint8_t params[2] = { start_addr, length };

    tx_packet[0] = 0xFF;
    tx_packet[1] = 0xFF;
    tx_packet[2] = servo_id;
    tx_packet[3] = pkt_length;
    tx_packet[4] = INST_READ;
    tx_packet[5] = params[0];
    tx_packet[6] = params[1];
    tx_packet[7] = calculate_checksum(servo_id, pkt_length, INST_READ, params, 2);

    g_tx_func(tx_packet, sizeof(tx_packet));

    uint8_t rx_raw[32];
    size_t bytes_received = g_rx_func(rx_raw, sizeof(rx_raw), 50);

    if (bytes_received < 6) {
        return SCS_ERR_TIMEOUT;
    }

    uint8_t *rx_buffer = NULL;
    for (size_t i = 0; i <= bytes_received - 6; i++) {
        if (rx_raw[i] == 0xFF && rx_raw[i+1] == 0xFF && (rx_raw[i+2] == servo_id || servo_id == 0xFE)) {
            rx_buffer = &rx_raw[i];
            break;
        }
    }

    if (rx_buffer == NULL) {
        return SCS_ERR_HEADER_NOT_FOUND;
    }

    uint8_t resp_len = rx_buffer[3];
    uint8_t error_code = rx_buffer[4];
    if (error_code != 0x00) {
        return SCS_ERR_SERVO_ERROR;
    }

    uint32_t checksum_calc = 0;
    for (uint8_t i = 2; i <= (2 + resp_len); i++) {
        checksum_calc += rx_buffer[i];
    }

    if ((uint8_t)(~checksum_calc & 0xFF) != rx_buffer[3 + resp_len]) {
        return SCS_ERR_CHECKSUM;
    }

    uint8_t data_len = resp_len - 2;
    if (data_len > length) data_len = length;

    for (uint8_t i = 0; i < data_len; i++) {
        out_buf[i] = rx_buffer[5 + i];
    }

    return SCS_OK;
}

void scs_set_limits(uint8_t servo_id, uint16_t min_limit, uint16_t max_limit) {
    if (g_tx_func == NULL) return;

    // 1. Unlock EEPROM
    uint8_t unlock_params[2] = { SCS_REG_LOCK, 0x00 };
    uint8_t unlock_pkt[8] = { 0xFF, 0xFF, servo_id, 4, INST_WRITE, SCS_REG_LOCK, 0x00, 
                             calculate_checksum(servo_id, 4, INST_WRITE, unlock_params, 2) };
    g_tx_func(unlock_pkt, 8);
    scs_delay_ms(50);

    // 2. Write Limits (Big-Endian: High byte first for min and max limits)
    uint8_t length = 7;
    uint8_t params[5] = {
        SCS_REG_MIN_ANGLE_LIMIT,
        (uint8_t)((min_limit >> 8) & 0xFF),
        (uint8_t)(min_limit & 0xFF),
        (uint8_t)((max_limit >> 8) & 0xFF),
        (uint8_t)(max_limit & 0xFF)
    };
    
    uint8_t packet[11] = {
        0xFF, 0xFF, servo_id, length, INST_WRITE,
        params[0], params[1], params[2], params[3], params[4],
        calculate_checksum(servo_id, length, INST_WRITE, params, 5)
    };
    g_tx_func(packet, 11); 
    scs_delay_ms(50);

    // 3. Lock EEPROM
    uint8_t lock_params[2] = { SCS_REG_LOCK, 0x01 };
    uint8_t lock_pkt[8] = { 0xFF, 0xFF, servo_id, 4, INST_WRITE, SCS_REG_LOCK, 0x01, 
                           calculate_checksum(servo_id, 4, INST_WRITE, lock_params, 2) };
    g_tx_func(lock_pkt, 8);
    scs_delay_ms(50);
}

void scs_sync_write_position(SCS_SyncTarget_t *targets, size_t count) {
    if (g_tx_func == NULL || targets == NULL || count == 0) return;

    const uint8_t start_address = SCS_REG_GOAL_POSITION;
    const uint8_t data_len_per_servo = 2;

    uint16_t param_len = 2 + (count * (1 + data_len_per_servo));
    uint8_t length_field = (uint8_t)(param_len + 2);
    uint16_t total_packet_size = 5 + param_len + 1;

    uint8_t packet[128];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFE;
    packet[3] = length_field;
    packet[4] = INST_SYNC_WRITE;

    uint8_t params[120];
    params[0] = start_address;
    params[1] = data_len_per_servo;

    uint16_t param_index = 2;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t id   = targets[i].id;
        uint16_t pos = targets[i].position;

        params[param_index++] = id;
        params[param_index++] = (uint8_t)((pos >> 8) & 0xFF); // High byte
        params[param_index++] = (uint8_t)(pos & 0xFF);        // Low byte
    }

    for (uint16_t i = 0; i < param_len; i++) {
        packet[5 + i] = params[i];
    }

    packet[total_packet_size - 1] = calculate_checksum(0xFE, length_field, INST_SYNC_WRITE, params, (uint8_t)param_len);

    g_tx_func(packet, total_packet_size);
}

void scs_sync_write_position_time(SCS_SyncTarget_t *targets, size_t count) {
    if (g_tx_func == NULL || targets == NULL || count == 0) return;

    const uint8_t start_address = SCS_REG_GOAL_POSITION;
    const uint8_t data_len_per_servo = 4;

    uint16_t param_len = 2 + (count * (1 + data_len_per_servo));
    uint8_t length_field = (uint8_t)(param_len + 2);
    uint16_t total_packet_size = 5 + param_len + 1;

    uint8_t packet[128];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFE;
    packet[3] = length_field;
    packet[4] = INST_SYNC_WRITE;

    uint8_t params[120];
    params[0] = start_address;
    params[1] = data_len_per_servo;

    uint8_t param_index = 2;
    for (size_t i = 0; i < count; i++) {
        uint16_t pos = targets[i].position;
        uint16_t time = targets[i].time;

        params[param_index++] = targets[i].id;
        params[param_index++] = (uint8_t)((pos >> 8) & 0xFF);
        params[param_index++] = (uint8_t)(pos & 0xFF);
        params[param_index++] = (uint8_t)((time >> 8) & 0xFF);
        params[param_index++] = (uint8_t)(time & 0xFF);
    }

    for (uint16_t i = 0; i < param_len; i++) {
        packet[5 + i] = params[i];
    }

    packet[total_packet_size - 1] = calculate_checksum(0xFE, length_field, INST_SYNC_WRITE, params, (uint8_t)param_len);

    g_tx_func(packet, total_packet_size);
}