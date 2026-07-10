#include "artemis_controller.h"

#include <math.h>
#include <string.h>

#include "artemis_config.h"

typedef enum {
    ACTION_DRIVE_UNTIL_LINE,
    ACTION_TRACK_UNTIL_LOST,
    ACTION_TURN_SETTLE,
    ACTION_FINISH
} action_kind_t;

typedef struct {
    action_kind_t kind;
    float yaw_offset_deg;
    float velocity;
    float duration_s;
    float max_duration_s;
} task_action_t;

static const task_action_t task3_actions[] = {
    {ACTION_DRIVE_UNTIL_LINE, -38.659808254090095f, 7.0f, 0.0f, 5.0f},
    {ACTION_TRACK_UNTIL_LOST, 0.0f, 7.0f, 0.0f, 8.0f},
    {ACTION_TURN_SETTLE, -141.3401917459099f, 0.0f, 1.2f, 0.0f},
    {ACTION_DRIVE_UNTIL_LINE, -141.3401917459099f, 7.0f, 0.0f, 5.0f},
    {ACTION_TRACK_UNTIL_LOST, 0.0f, 7.0f, 0.0f, 8.0f},
    {ACTION_FINISH, 0.0f, 0.0f, 0.0f, 0.0f},
};

static const float line_weights[ARTEMIS_MAX_LINE_SENSORS] = {
    -4.0f, -3.0f, -2.0f, -1.0f, 1.0f, 2.0f, 3.0f, 4.0f};

static float clamp_float(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static float wrap_error_deg(float angle_deg)
{
    while (angle_deg <= -180.0f) {
        angle_deg += 360.0f;
    }
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

static float wrap_target_deg(float angle_deg)
{
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

static bool any_line(const artemis_observation_t *observation)
{
    size_t index;
    const size_t count = observation->digital_count < ARTEMIS_MAX_LINE_SENSORS
        ? observation->digital_count
        : ARTEMIS_MAX_LINE_SENSORS;
    for (index = 0U; index < count; index++) {
        if (observation->digital_values[index] != 0U) {
            return true;
        }
    }
    return false;
}

void artemis_line_controller_init(
    artemis_line_controller_t *controller,
    artemis_line_controller_mode_t mode)
{
    memset(controller, 0, sizeof(*controller));
    controller->mode = mode;
}

void artemis_line_controller_reset(artemis_line_controller_t *controller)
{
    const artemis_line_controller_mode_t mode = controller->mode;
    memset(controller, 0, sizeof(*controller));
    controller->mode = mode;
}

bool artemis_line_controller_scan(
    artemis_line_controller_t *controller,
    const uint8_t *digital_values,
    size_t digital_count)
{
    size_t index;
    size_t detected_count = 0U;
    float weighted_sum = 0.0f;
    const size_t count = digital_count < ARTEMIS_MAX_LINE_SENSORS
        ? digital_count
        : ARTEMIS_MAX_LINE_SENSORS;

    for (index = 0U; index < count; index++) {
        if (digital_values[index] != 0U) {
            weighted_sum += line_weights[index];
            detected_count++;
        }
    }
    if (detected_count == 0U) {
        return false;
    }
    controller->line_error = weighted_sum / (float) detected_count;
    return true;
}

static float compute_original_turn(artemis_line_controller_t *controller, float velocity)
{
    const float control_error = -controller->line_error;
    const float derivative = control_error - controller->previous_control_error;
    const float calibration = 150.0f / (velocity + 1.0f);
    float pid_value;

    controller->integral += control_error;
    pid_value = 25.0f * control_error + 3.5f * derivative;
    controller->previous_line_error = controller->line_error;
    controller->previous_control_error = control_error;
    return pid_value / calibration;
}

static float compute_fuzzy_turn(artemis_line_controller_t *controller, float velocity)
{
    const float delta_error = controller->line_error - controller->previous_line_error;
    const float control_error = -controller->line_error;
    const float derivative = control_error - controller->previous_control_error;
    const float error_level = fminf(fabsf(controller->line_error) / 4.0f, 1.0f);
    const float delta_level = fminf(fabsf(delta_error) / 4.0f, 1.0f);
    const float centered = fmaxf(0.0f, 1.0f - error_level * 2.0f);
    const float steady = fmaxf(0.0f, 1.0f - delta_level * 2.0f);
    const float kp = 18.0f + 20.0f * error_level + 4.0f * delta_level;
    const float kd = 1.5f + 2.0f * error_level + 8.0f * delta_level;
    const float ki = 0.02f * centered * steady;
    const float calibration = 150.0f / (velocity + 1.0f);
    float pid_value;

    controller->integral += control_error;
    if (ki == 0.0f) {
        controller->integral = 0.0f;
    } else {
        controller->integral = clamp_float(controller->integral, -32.0f, 32.0f);
    }
    pid_value = ki * controller->integral + kp * control_error + kd * derivative;
    controller->previous_line_error = controller->line_error;
    controller->previous_control_error = control_error;
    return pid_value / calibration;
}

float artemis_line_controller_compute_turn(artemis_line_controller_t *controller, float velocity)
{
    if (controller->mode == ARTEMIS_LINE_CONTROLLER_FUZZY) {
        return compute_fuzzy_turn(controller, velocity);
    }
    return compute_original_turn(controller, velocity);
}

void artemis_yaw_controller_reset(artemis_yaw_controller_t *controller)
{
    memset(controller, 0, sizeof(*controller));
}

float artemis_yaw_controller_compute(
    artemis_yaw_controller_t *controller,
    float current_yaw_deg,
    float target_yaw_deg,
    bool *stable)
{
    const float raw_error = wrap_error_deg(target_yaw_deg - current_yaw_deg);
    float pid_output;

    controller->filtered_error =
        0.3f * controller->filtered_error + 0.7f * raw_error;
    controller->integral += controller->filtered_error;
    pid_output = 0.3f * controller->filtered_error +
        0.015f * (controller->filtered_error - controller->previous_error);
    controller->previous_error = controller->filtered_error;
    if (fabsf(controller->filtered_error) < 2.0f) {
        if (controller->stable_counter < UINT16_MAX) {
            controller->stable_counter++;
        }
    } else {
        controller->stable_counter = 0U;
    }
    *stable = controller->stable_counter >= 10U;
    return -pid_output / 4.5f;
}

static artemis_control_command_t stop_command(uint32_t sequence_id, bool completed)
{
    artemis_control_command_t command;
    memset(&command, 0, sizeof(command));
    command.sequence_id = sequence_id;
    command.completed = completed;
    return command;
}

static artemis_control_command_t yaw_command(
    artemis_mission_t *mission,
    const artemis_observation_t *observation,
    float target_yaw_deg,
    float velocity)
{
    artemis_control_command_t command;
    bool stable = false;

    memset(&command, 0, sizeof(command));
    command.sequence_id = observation->sequence_id;
    command.velocity = velocity;
    command.turn = artemis_yaw_controller_compute(
        &mission->yaw_controller, observation->yaw_deg, target_yaw_deg, &stable);
    command.rear_left_target_speed = velocity + command.turn;
    command.rear_right_target_speed = velocity - command.turn;
    if (stable && (velocity == 0.0f)) {
        command.rear_left_target_speed = 0.0f;
        command.rear_right_target_speed = 0.0f;
    }
    return command;
}

static artemis_control_command_t track_command(
    artemis_mission_t *mission,
    const artemis_observation_t *observation,
    float velocity)
{
    artemis_control_command_t command;
    memset(&command, 0, sizeof(command));
    artemis_line_controller_scan(
        &mission->line_controller, observation->digital_values, observation->digital_count);
    command.sequence_id = observation->sequence_id;
    command.velocity = velocity;
    command.turn = artemis_line_controller_compute_turn(&mission->line_controller, velocity);
    command.rear_left_target_speed = velocity - command.turn;
    command.rear_right_target_speed = velocity + command.turn;
    return command;
}

static void enter_action(artemis_mission_t *mission, const artemis_observation_t *observation)
{
    mission->action_started_at_s = observation->sim_time_s;
    mission->distance_started_at_cm = observation->forward_distance_cm;
    mission->confirm_count = 0U;
    mission->line_seen = false;
    mission->action_started = true;
    artemis_line_controller_reset(&mission->line_controller);
    mission->yaw_controller.stable_counter = 0U;
}

static void advance_action(artemis_mission_t *mission)
{
    mission->action_index++;
    mission->action_started = false;
    mission->confirm_count = 0U;
    mission->line_seen = false;
}

static bool action_completed(
    artemis_mission_t *mission,
    const task_action_t *action,
    const artemis_observation_t *observation)
{
    const float elapsed = observation->sim_time_s - mission->action_started_at_s;
    const bool line_detected = any_line(observation);

    if (action->kind == ACTION_TURN_SETTLE) {
        return elapsed >= action->duration_s;
    }
    if (action->kind == ACTION_DRIVE_UNTIL_LINE) {
        mission->confirm_count = line_detected ? (uint16_t) (mission->confirm_count + 1U) : 0U;
        return (mission->confirm_count >= ARTEMIS_LINE_CONFIRM_FRAMES) ||
            ((action->max_duration_s > 0.0f) && (elapsed >= action->max_duration_s));
    }
    if (action->kind == ACTION_TRACK_UNTIL_LOST) {
        if (line_detected) {
            mission->line_seen = true;
            mission->confirm_count = 0U;
        } else if (mission->confirm_count < UINT16_MAX) {
            mission->confirm_count++;
        }
        return (mission->line_seen && (mission->confirm_count >= ARTEMIS_LINE_LOSS_FRAMES)) ||
            ((action->max_duration_s > 0.0f) && (elapsed >= action->max_duration_s));
    }
    return false;
}

void artemis_mission_reset(artemis_mission_t *mission)
{
    const artemis_line_controller_mode_t mode =
        ARTEMIS_LINE_PID_MODE == ARTEMIS_LINE_PID_FUZZY
        ? ARTEMIS_LINE_CONTROLLER_FUZZY
        : ARTEMIS_LINE_CONTROLLER_ORIGINAL;
    memset(mission, 0, sizeof(*mission));
    artemis_line_controller_init(&mission->line_controller, mode);
    artemis_yaw_controller_reset(&mission->yaw_controller);
}

artemis_control_command_t artemis_mission_step(
    artemis_mission_t *mission,
    const artemis_observation_t *observation)
{
    const size_t action_count = sizeof(task3_actions) / sizeof(task3_actions[0]);

    if (!mission->base_yaw_valid) {
        mission->base_yaw_deg = observation->yaw_deg;
        mission->base_yaw_valid = true;
    }
    while (mission->action_index < action_count) {
        const task_action_t *action = &task3_actions[mission->action_index];
        artemis_control_command_t command;

        if (action->kind == ACTION_FINISH) {
            return stop_command(observation->sequence_id, true);
        }
        if (!mission->action_started) {
            enter_action(mission, observation);
        }
        if (action->kind == ACTION_TRACK_UNTIL_LOST) {
            command = track_command(mission, observation, action->velocity);
        } else {
            const float target_yaw =
                wrap_target_deg(mission->base_yaw_deg + action->yaw_offset_deg);
            command = yaw_command(
                mission,
                observation,
                target_yaw,
                action->kind == ACTION_DRIVE_UNTIL_LINE ? action->velocity : 0.0f);
        }
        if (!action_completed(mission, action, observation)) {
            return command;
        }
        advance_action(mission);
    }
    return stop_command(observation->sequence_id, true);
}
