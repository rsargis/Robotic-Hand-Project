#include "motion_control.h"
#include "scs0009_servo_driver.h"
#include <stdlib.h>
#include <math.h>

const hand_pose_t POSE_OPEN_HAND = { .angle =  {767, 255, 767, 255, 767, 255, 767, 255}};
const hand_pose_t POSE_CLOSED_HAND = { .angle = {255, 767, 255, 767, 255, 767, 255, 767} };
const hand_pose_t POSE_NEUTRAL = { .angle = {511, 511, 511, 511, 511, 511, 511, 511} };
const hand_pose_t POSE_MIDDLE = { .angle = {255, 767, 767, 255, 255, 767, 255, 767} };
const hand_pose_t POSE_OK = { .angle = {255, 767, 767, 255, 767, 255, 255, 767} };
const hand_pose_t POSE_VICTORY = { .angle = {600, 300, 700, 400, 255, 767, 255, 767} };

hand_pose_t current_pose;
float speed;

uint16_t clamp_position(uint16_t pos) {
    if (pos < SERVO_MIN_POS) return SERVO_MIN_POS;
    if (pos > SERVO_MAX_POS) return SERVO_MAX_POS;
    return pos;
}

void clamp_pose(hand_pose_t *input) {
    for (size_t i = 0; i < NUM_DOF; i++) {
        input->angle[i] = clamp_position(input->angle[i]);
    }
}

static uint32_t calculate_duration(const hand_pose_t *start, const hand_pose_t *target, float speed_factor) {
    if (speed_factor == 0) return 0;

    uint16_t max_delta = 0;
    for (size_t i = 0; i < NUM_DOF; i++) {
        uint16_t diff = abs((int)target->angle[i] - (int)start->angle[i]);
        if (diff > max_delta) {
            max_delta = diff;
        }
    }
    //max speed should be 100 ticks in 100ms
    // Convert raw position units to duration in milliseconds based on speed
    // (Assuming ~0.29 degrees per raw tick for 0-1023 scale over ~300 deg range)
    return (max_delta < 50) ? 50 : max_delta*speed_factor; // Minimum 50ms guard
}

void motion_init(const uint8_t servo_ids[NUM_DOF]) {
        scs_set_limits(0xFE, SERVO_MIN_POS, SERVO_MAX_POS);
        scs_set_torque(0xFE, true);
        scs_set_position(0xFE, POSE_NEUTRAL.angle[0]); // Move all servos to neutral
        scs_clear_speed(0xFE);
        speed = 1;

        current_pose = POSE_NEUTRAL;
}

void motion_set_pose(const hand_pose_t *pose) {
    SCS_SyncTarget_t targets[NUM_DOF];

    const hand_pose_t curr = current_pose;

    uint32_t max_time = calculate_duration(&curr, pose, speed);

    for (int i = 1; i <= NUM_DOF; i++){
        targets[i-1].id = i;
        targets[i-1].position = pose->angle[i-1];
        targets[i-1].time = max_time;
    }

    scs_sync_write_position_time(targets, NUM_DOF);

    current_pose = *pose;
}

void update_speed(float new_speed) {
    if (new_speed < 0.5) {
        speed = 0.5; // Minimum speed factor
    } else {
        speed = new_speed;
    }
}