/* Motion Control Functions for Hand*/
#ifndef HAND_CONTROLLER_H
#define HAND_CONTROLLER_H

#include "scs0009_servo_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define NUM_DOF 8

#define SERVO_MIN_POS  100
#define SERVO_MAX_POS  900

typedef struct {
    int16_t angle[NUM_DOF];
} hand_pose_t;

extern const hand_pose_t POSE_OPEN_HAND;
extern const hand_pose_t POSE_CLOSED_HAND;
extern const hand_pose_t POSE_NEUTRAL;


void motion_init(const uint8_t servo_ids[NUM_DOF]);
void motion_set_pose(const hand_pose_t *pose);
void motion_interpolate(const hand_pose_t *from,
                        const hand_pose_t *to,
                        uint16_t duration_ms);

#endif