#ifndef ARTEMIS_TYPES_H
#define ARTEMIS_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ARTEMIS_MAX_LINE_SENSORS 8U

typedef struct {
    uint32_t sequence_id;
    float sim_time_s;
    float yaw_deg;
    uint8_t digital_values[ARTEMIS_MAX_LINE_SENSORS];
    size_t digital_count;
    float rear_left_total_ticks;
    float rear_right_total_ticks;
    float forward_distance_cm;
} artemis_observation_t;

typedef struct {
    uint32_t sequence_id;
    float velocity;
    float turn;
    float rear_left_target_speed;
    float rear_right_target_speed;
    bool completed;
} artemis_control_command_t;

#endif
