#include "motion_control.h"
#include "scs0009_servo_driver.h"
#include <stdlib.h>
#include <math.h>

const hand_pose_t POSE_OPEN_HAND = { .angle = {200, 200, 200, 200, 200, 200, 200, 200} };
const hand_pose_t POSE_CLOSED_HAND = { .angle = {800, 800, 800, 800, 800, 800, 800, 800} };
const hand_pose_t POSE_NEUTRAL = { .angle = {511, 511, 511, 511, 511, 511, 511, 511} };

hand_pose_t current_pose;

static uint16_t clamp_position(uint16_t pos) {
    if (pos < SERVO_MIN_POS) return SERVO_MIN_POS;
    if (pos > SERVO_MAX_POS) return SERVO_MAX_POS;
    return pos;
}

static uint32_t calculate_duration(const hand_pose_t *start, const hand_pose_t *target, uint32_t speed_deg_per_sec) {
    if (speed_deg_per_sec == 0) return 0;

    uint16_t max_delta = 0;
    for (size_t i = 0; i < NUM_DOF; i++) {
        uint16_t diff = abs((int)target->angle[i] - (int)start->angle[i]);
        if (diff > max_delta) {
            max_delta = diff;
        }
    }

    // Convert raw position units to duration in milliseconds based on speed
    // (Assuming ~0.29 degrees per raw tick for 0-1023 scale over ~300 deg range)
    uint32_t duration_ms = (max_delta * 1000) / speed_deg_per_sec;
    return (duration_ms < 50) ? 50 : duration_ms; // Minimum 50ms guard
}

void motion_init(const uint8_t servo_ids[NUM_DOF]) {
        scs_set_limits(0xFE, SERVO_MIN_POS, SERVO_MAX_POS);
        scs_set_torque(0xFE, true);
        scs_set_position(0xFE, POSE_NEUTRAL.angle[0]); // Move all servos to neutral
        scs_clear_speed(0xFE);

        current_pose = POSE_NEUTRAL;
}

void motion_set_pose(const hand_pose_t *pose) {
    SCS_SyncTarget_t targets[NUM_DOF];

    const hand_pose_t curr = current_pose;

    uint32_t max_time = calculate_duration(&curr, pose, 150);

    for (int i = 0; i < NUM_DOF; i++){
        targets[i].id = i;
        targets[i].position = pose->angle[i];
        targets[i].time = max_time;
    }

    scs_sync_write_position_time(targets, NUM_DOF);

    current_pose = *pose;
}