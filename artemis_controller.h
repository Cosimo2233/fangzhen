#ifndef ARTEMIS_CONTROLLER_H
#define ARTEMIS_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "artemis_types.h"

/* 循迹 PID 实现模式。具体通过 artemis_config.h 的 ARTEMIS_LINE_PID_MODE 选择。 */
typedef enum {
    ARTEMIS_LINE_CONTROLLER_ORIGINAL,
    ARTEMIS_LINE_CONTROLLER_FUZZY
} artemis_line_controller_mode_t;

/* 循迹控制器内部状态：保存当前线误差、上一帧误差和积分项。 */
typedef struct {
    artemis_line_controller_mode_t mode;
    float line_error;
    float previous_line_error;
    float integral;
    float previous_control_error;
} artemis_line_controller_t;

/* 航向控制器内部状态：用于 yaw PID、误差滤波和稳定判定。 */
typedef struct {
    float integral;
    float previous_error;
    float filtered_error;
    uint16_t stable_counter;
} artemis_yaw_controller_t;

/* task3 任务状态机的运行时状态。 */
typedef struct {
    artemis_line_controller_t line_controller;
    artemis_yaw_controller_t yaw_controller;
    float base_yaw_deg;
    float action_started_at_s;
    float distance_started_at_cm;
    uint16_t confirm_count;
    uint16_t lap_count;
    uint8_t action_index;
    bool base_yaw_valid;
    bool action_started;
    bool line_seen;
} artemis_mission_t;

void artemis_line_controller_init(
    artemis_line_controller_t *controller,
    artemis_line_controller_mode_t mode);
void artemis_line_controller_reset(artemis_line_controller_t *controller);
bool artemis_line_controller_scan(
    artemis_line_controller_t *controller,
    const uint8_t *digital_values,
    size_t digital_count);
float artemis_line_controller_compute_turn(artemis_line_controller_t *controller, float velocity);

void artemis_yaw_controller_reset(artemis_yaw_controller_t *controller);
float artemis_yaw_controller_compute(
    artemis_yaw_controller_t *controller,
    float current_yaw_deg,
    float target_yaw_deg,
    bool *stable);

void artemis_mission_reset(artemis_mission_t *mission);
artemis_control_command_t artemis_mission_step(
    artemis_mission_t *mission,
    const artemis_observation_t *observation);

#endif
