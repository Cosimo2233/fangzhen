#include "artemis_controller.h"

#include <math.h>
#include <string.h>

#include "artemis_config.h"
#include "artemis_runtime_params.h"

typedef enum {
    ACTION_DRIVE_DISTANCE,
    ACTION_DRIVE_UNTIL_LINE,
    ACTION_TRACK_UNTIL_LOST,
    ACTION_TURN_SETTLE,
    ACTION_FINISH
} action_kind_t;

typedef struct {
    action_kind_t kind;
    float yaw_offset_deg;
    float velocity;
    float min_distance_cm;
    float duration_s;
    float max_duration_s;
} task_action_t;

/*
 * task3 鍔ㄤ綔琛ㄣ€備慨鏀逛换鍔℃祦绋嬫椂浼樺厛鏀硅繖閲岋細
 * 1. 鎸?yaw_offset_deg 琛岄┒鐩村埌瑙佺嚎
 * 2. 寰抗鐩村埌涓㈢嚎
 * 3. 杞悜绋冲畾
 * 4. 鍐嶆鎵剧嚎
 * 5. 鍐嶆寰抗
 * 6. 瀹屾垚浠诲姟
 */
static const task_action_t task3_actions[] = {
    {ACTION_DRIVE_UNTIL_LINE, ARTEMIS_TASK_ENTRY_C_YAW_DEG, ARTEMIS_TASK_DRIVE_VELOCITY, 0.0f, 0.0f, ARTEMIS_TASK_DRIVE_MAX_S},
    {ACTION_TRACK_UNTIL_LOST, 0.0f, ARTEMIS_TASK_TRACK_VELOCITY, 0.0f, 0.0f, ARTEMIS_TASK_TRACK_MAX_S},
    {ACTION_TURN_SETTLE, ARTEMIS_TASK_ENTRY_D_YAW_DEG, 0.0f, 0.0f, ARTEMIS_TASK_TURN_SETTLE_S, 0.0f},
    {ACTION_DRIVE_UNTIL_LINE, ARTEMIS_TASK_ENTRY_D_YAW_DEG, ARTEMIS_TASK_DRIVE_VELOCITY, 0.0f, 0.0f, ARTEMIS_TASK_DRIVE_MAX_S},
    {ACTION_TRACK_UNTIL_LOST, 0.0f, ARTEMIS_TASK_TRACK_VELOCITY, 0.0f, 0.0f, ARTEMIS_TASK_TRACK_MAX_S},
    {ACTION_FINISH, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
};

static const task_action_t task2_actions[] = {
    {ACTION_TURN_SETTLE, ARTEMIS_TASK2_AB_YAW_DEG, 0.0f, 0.0f, ARTEMIS_TASK2_ALIGN_MIN_S, ARTEMIS_TASK2_ALIGN_MAX_S},
    {ACTION_DRIVE_UNTIL_LINE, ARTEMIS_TASK2_AB_YAW_DEG, ARTEMIS_TASK_DRIVE_VELOCITY, ARTEMIS_TASK2_STRAIGHT_MIN_CM, 0.0f, ARTEMIS_TASK_DRIVE_MAX_S},
    {ACTION_TRACK_UNTIL_LOST, 0.0f, ARTEMIS_TASK_TRACK_VELOCITY, ARTEMIS_TASK2_ARC_MIN_CM, 0.0f, ARTEMIS_TASK_TRACK_MAX_S},
    {ACTION_TURN_SETTLE, ARTEMIS_TASK2_CD_YAW_DEG, 0.0f, 0.0f, ARTEMIS_TASK2_ALIGN_MIN_S, ARTEMIS_TASK2_ALIGN_MAX_S},
    {ACTION_DRIVE_UNTIL_LINE, ARTEMIS_TASK2_CD_YAW_DEG, ARTEMIS_TASK_DRIVE_VELOCITY, ARTEMIS_TASK2_STRAIGHT_MIN_CM, 0.0f, ARTEMIS_TASK_DRIVE_MAX_S},
    {ACTION_TRACK_UNTIL_LOST, 0.0f, ARTEMIS_TASK_TRACK_VELOCITY, ARTEMIS_TASK2_ARC_MIN_CM, 0.0f, ARTEMIS_TASK_TRACK_MAX_S},
    {ACTION_FINISH, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
};

/* 8 璺贰绾夸紶鎰熷櫒鐨勬í鍚戞潈閲嶏紝宸︿晶涓鸿礋锛屽彸渚т负姝ｏ紝涓棿娌℃湁 0 鏉冮噸銆?*/
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
    /* yaw 璇樊褰掍竴鍖栧埌 (-180, 180]锛岄伩鍏?359 搴﹀拰 0 搴﹂檮杩戣烦鍙樸€?*/
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

static artemis_line_controller_mode_t configured_line_mode(void)
{
    return artemis_runtime_params_get()->line_pid_mode == ARTEMIS_LINE_PID_FUZZY
        ? ARTEMIS_LINE_CONTROLLER_FUZZY
        : ARTEMIS_LINE_CONTROLLER_ORIGINAL;
}

static const task_action_t *selected_actions(
    artemis_task_id_t task_id,
    size_t *action_count)
{
    if (task_id == ARTEMIS_TASK_ID_2) {
        *action_count = sizeof(task2_actions) / sizeof(task2_actions[0]);
        return task2_actions;
    }
    *action_count = sizeof(task3_actions) / sizeof(task3_actions[0]);
    return task3_actions;
}

static task_action_t configured_action(artemis_task_id_t task_id, uint8_t action_index)
{
    size_t action_count = 0U;
    const task_action_t *actions = selected_actions(task_id, &action_count);
    task_action_t action = actions[action_index];
    const artemis_runtime_params_t *params = artemis_runtime_params_get();

    (void) action_count;
    if (task_id == ARTEMIS_TASK_ID_3) {
        switch (action_index) {
            case 0U:
                action.yaw_offset_deg = params->task_entry_c_yaw_deg;
                action.velocity = params->task_drive_velocity;
                action.max_duration_s = params->task_drive_max_s;
                break;
            case 1U:
            case 4U:
                action.velocity = params->task_track_velocity;
                action.max_duration_s = params->task_track_max_s;
                break;
            case 2U:
                action.yaw_offset_deg = params->task_entry_d_yaw_deg;
                action.duration_s = params->task_turn_settle_s;
                break;
            case 3U:
                action.yaw_offset_deg = params->task_entry_d_yaw_deg;
                action.velocity = params->task_drive_velocity;
                action.max_duration_s = params->task_drive_max_s;
                break;
            default:
                break;
        }
        return action;
    }

    switch (action_index) {
        case 0U:
            action.yaw_offset_deg = params->task2_ab_yaw_deg;
            break;
        case 1U:
            action.yaw_offset_deg = params->task2_ab_yaw_deg;
            action.min_distance_cm = params->task2_straight_min_cm;
            action.velocity = params->task_drive_velocity;
            action.max_duration_s = params->task_drive_max_s;
            break;
        case 2U:
        case 5U:
            action.min_distance_cm = params->task2_arc_min_cm;
            action.velocity = params->task_track_velocity;
            action.max_duration_s = params->task_track_max_s;
            break;
        case 3U:
            action.yaw_offset_deg = params->task2_cd_yaw_deg;
            break;
        case 4U:
            action.yaw_offset_deg = params->task2_cd_yaw_deg;
            action.min_distance_cm = params->task2_straight_min_cm;
            action.velocity = params->task_drive_velocity;
            action.max_duration_s = params->task_drive_max_s;
            break;
        default:
            break;
    }
    return action;
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

    /* 鎶婃墍鏈夊帇鍒伴粦绾跨殑浼犳劅鍣ㄦ潈閲嶅彇骞冲潎锛屼綔涓哄綋鍓嶄綅缃宸€?*/
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
    /* 鏅€氬惊杩?PID锛氬弬鏁伴泦涓湪 artemis_config.h锛屼究浜庣幇鍦鸿皟鍙傘€?*/
    const artemis_runtime_params_t *params = artemis_runtime_params_get();
    const float control_error = -controller->line_error;
    const float derivative = control_error - controller->previous_control_error;
    const float calibration = params->line_output_base_scale / (velocity + 1.0f);
    float pid_value;

    controller->integral += control_error;
    pid_value = params->line_original_ki * controller->integral +
        params->line_original_kp * control_error +
        params->line_original_kd * derivative;
    controller->previous_line_error = controller->line_error;
    controller->previous_control_error = control_error;
    return pid_value / calibration;
}

static float compute_fuzzy_turn(artemis_line_controller_t *controller, float velocity)
{
    /* 妯＄硦 PID锛氭牴鎹宸拰璇樊鍙樺寲閲忓湪绾胯皟鏁?kp/ki/kd锛屽弬鏁拌 artemis_config.h銆?*/
    const artemis_runtime_params_t *params = artemis_runtime_params_get();
    const float delta_error = controller->line_error - controller->previous_line_error;
    const float control_error = -controller->line_error;
    const float derivative = control_error - controller->previous_control_error;
    const float error_level =
        fminf(fabsf(controller->line_error) / params->line_fuzzy_error_normalizer, 1.0f);
    const float delta_level =
        fminf(fabsf(delta_error) / params->line_fuzzy_delta_normalizer, 1.0f);
    const float centered =
        fmaxf(0.0f, 1.0f - error_level * params->line_fuzzy_center_width);
    const float steady =
        fmaxf(0.0f, 1.0f - delta_level * params->line_fuzzy_steady_width);
    const float kp = params->line_fuzzy_kp_base +
        params->line_fuzzy_kp_error_gain * error_level +
        params->line_fuzzy_kp_delta_gain * delta_level;
    const float kd = params->line_fuzzy_kd_base +
        params->line_fuzzy_kd_error_gain * error_level +
        params->line_fuzzy_kd_delta_gain * delta_level;
    const float ki = params->line_fuzzy_ki_base * centered * steady;
    const float calibration = params->line_output_base_scale / (velocity + 1.0f);
    float pid_value;

    controller->integral += control_error;
    if (ki == 0.0f) {
        controller->integral = 0.0f;
    } else {
        controller->integral = clamp_float(
            controller->integral,
            -params->line_fuzzy_integral_limit,
            params->line_fuzzy_integral_limit);
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
    const artemis_runtime_params_t *params = artemis_runtime_params_get();
    const float raw_error = wrap_error_deg(target_yaw_deg - current_yaw_deg);
    float pid_output;

    /* 鑸悜 PID 鍏堟护娉㈣宸紝鍑忓皯浠跨湡 yaw 鎶栧姩瀵艰嚧鐨勮疆閫熻烦鍙樸€?*/
    controller->filtered_error =
        params->yaw_error_filter_previous * controller->filtered_error +
        params->yaw_error_filter_current * raw_error;
    controller->integral += controller->filtered_error;
    pid_output = params->yaw_kp * controller->filtered_error +
        params->yaw_ki * controller->integral +
        params->yaw_kd * (controller->filtered_error - controller->previous_error);
    controller->previous_error = controller->filtered_error;
    if (fabsf(controller->filtered_error) < params->yaw_stable_error_deg) {
        if (controller->stable_counter < UINT16_MAX) {
            controller->stable_counter++;
        }
    } else {
        controller->stable_counter = 0U;
    }
    *stable = controller->stable_counter >= ARTEMIS_YAW_STABLE_FRAMES;
    return -pid_output / params->yaw_turn_scale;
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
    /* 姣忎釜鍔ㄤ綔杩涘叆鏃惰褰曡捣濮嬫椂闂村拰璺濈锛屽苟娓呯┖瑙佺嚎/涓㈢嚎璁℃暟銆?*/
    mission->action_started_at_s = observation->sim_time_s;
    mission->distance_started_at_cm = observation->forward_distance_cm;
    mission->confirm_count = 0U;
    mission->line_seen = false;
    mission->action_started = true;
    artemis_line_controller_reset(&mission->line_controller);
    artemis_yaw_controller_reset(&mission->yaw_controller);
}

static void advance_action(artemis_mission_t *mission)
{
    mission->action_index++;
    mission->action_started = false;
    mission->confirm_count = 0U;
    mission->line_seen = false;
}

static uint32_t configured_repeat_count(const artemis_mission_t *mission)
{
    return artemis_mission_get_task(mission) == ARTEMIS_TASK_ID_2
        ? (uint32_t) ARTEMIS_TASK2_REPEAT_COUNT
        : (uint32_t) ARTEMIS_TASK3_REPEAT_COUNT;
}

static bool mission_should_repeat(artemis_mission_t *mission)
{
    const uint32_t repeat_count = configured_repeat_count(mission);

    if (repeat_count == 0U) {
        if (mission->lap_count < UINT16_MAX) {
            mission->lap_count++;
        }
        return true;
    }
    if ((uint32_t) mission->lap_count + 1U < repeat_count) {
        mission->lap_count++;
        return true;
    }
    return false;
}

static void restart_mission_cycle(artemis_mission_t *mission)
{
    mission->action_index = 0U;
    mission->action_started = false;
    mission->confirm_count = 0U;
    mission->line_seen = false;
    artemis_line_controller_reset(&mission->line_controller);
    artemis_yaw_controller_reset(&mission->yaw_controller);
}

static bool action_completed(
    artemis_mission_t *mission,
    const task_action_t *action,
    const artemis_observation_t *observation)
{
    const float elapsed = observation->sim_time_s - mission->action_started_at_s;
    const float distance_cm = observation->forward_distance_cm - mission->distance_started_at_cm;
    const bool line_detected = any_line(observation);

    if (action->kind == ACTION_TURN_SETTLE) {
        if ((action->duration_s <= 0.0f) && (action->max_duration_s <= 0.0f)) {
            return true;
        }
        return ((elapsed >= action->duration_s) &&
                (mission->yaw_controller.stable_counter >= ARTEMIS_YAW_STABLE_FRAMES)) ||
            ((action->max_duration_s > 0.0f) && (elapsed >= action->max_duration_s));
    }
    if (action->kind == ACTION_DRIVE_DISTANCE) {
        return (distance_cm >= action->min_distance_cm) ||
            ((action->max_duration_s > 0.0f) && (elapsed >= action->max_duration_s));
    }
    if (action->kind == ACTION_DRIVE_UNTIL_LINE) {
        /* 鎵剧嚎闃舵瑕佹眰杩炵画澶氬抚瑙佺嚎锛岄伩鍏嶅崟甯у櫔澹板垏鎹㈠姩浣溿€?*/
        mission->confirm_count =
            (line_detected && (distance_cm >= action->min_distance_cm))
                ? (uint16_t) (mission->confirm_count + 1U)
                : 0U;
        return (mission->confirm_count >= ARTEMIS_LINE_CONFIRM_FRAMES) ||
            ((action->max_duration_s > 0.0f) && (elapsed >= action->max_duration_s));
    }
    if (action->kind == ACTION_TRACK_UNTIL_LOST) {
        /* 寰抗闃舵蹇呴』鍏堣杩囩嚎锛屽啀鐢ㄨ繛缁涪绾垮抚鏁板垽瀹氱寮€榛戠嚎銆?*/
        if (line_detected) {
            mission->line_seen = true;
            mission->confirm_count = 0U;
        } else if (mission->confirm_count < UINT16_MAX) {
            mission->confirm_count++;
        }
        return (mission->line_seen &&
                (distance_cm >= action->min_distance_cm) &&
                (mission->confirm_count >= ARTEMIS_LINE_LOSS_FRAMES)) ||
            ((action->max_duration_s > 0.0f) && (elapsed >= action->max_duration_s));
    }
    return false;
}

void artemis_mission_reset(artemis_mission_t *mission)
{
    /* PID 妯″紡鍦ㄧ紪璇戞湡鐢?artemis_config.h 閫夋嫨銆?*/
    memset(mission, 0, sizeof(*mission));
    mission->task_id = ARTEMIS_TASK_ID_3;
    artemis_line_controller_init(&mission->line_controller, configured_line_mode());
    artemis_yaw_controller_reset(&mission->yaw_controller);
}

void artemis_mission_set_task(artemis_mission_t *mission, artemis_task_id_t task_id)
{
    if ((task_id != ARTEMIS_TASK_ID_2) && (task_id != ARTEMIS_TASK_ID_3)) {
        task_id = ARTEMIS_TASK_ID_3;
    }
    mission->task_id = task_id;
}

artemis_task_id_t artemis_mission_get_task(const artemis_mission_t *mission)
{
    if ((mission->task_id != ARTEMIS_TASK_ID_2) && (mission->task_id != ARTEMIS_TASK_ID_3)) {
        return ARTEMIS_TASK_ID_3;
    }
    return mission->task_id;
}

artemis_control_command_t artemis_mission_step(
    artemis_mission_t *mission,
    const artemis_observation_t *observation)
{
    size_t action_count = 0U;
    const artemis_task_id_t task_id = artemis_mission_get_task(mission);

    mission->line_controller.mode = configured_line_mode();
    (void) selected_actions(task_id, &action_count);

    /* 绗竴甯ц娴嬬殑 yaw 浣滀负浠诲姟鍩哄噯瑙掞紝鍔ㄤ綔琛ㄩ噷鐨勮搴︽槸鐩稿鍋忕Щ銆?*/
    if (!mission->base_yaw_valid) {
        mission->base_yaw_deg = observation->yaw_deg;
        mission->base_yaw_valid = true;
    }
    while (mission->action_index < action_count) {
        const task_action_t action = configured_action(task_id, mission->action_index);
        artemis_control_command_t command;

        if (action.kind == ACTION_FINISH) {
            if (mission_should_repeat(mission)) {
                restart_mission_cycle(mission);
                continue;
            }
            return stop_command(observation->sequence_id, true);
        }
        if (!mission->action_started) {
            enter_action(mission, observation);
        }
        if (action.kind == ACTION_TRACK_UNTIL_LOST) {
            command = track_command(mission, observation, action.velocity);
        } else {
            const float target_yaw =
                wrap_target_deg(mission->base_yaw_deg + action.yaw_offset_deg);
            command = yaw_command(
                mission,
                observation,
                target_yaw,
                (action.kind == ACTION_DRIVE_UNTIL_LINE) ||
                (action.kind == ACTION_DRIVE_DISTANCE) ? action.velocity : 0.0f);
        }
        if (!action_completed(mission, &action, observation)) {
            return command;
        }
        advance_action(mission);
    }
    return stop_command(observation->sequence_id, true);
}
