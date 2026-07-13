#ifndef ARTEMIS_RUNTIME_PARAMS_H
#define ARTEMIS_RUNTIME_PARAMS_H

#include <stdbool.h>
#include <stdint.h>

/* 运行时可调参数。默认值来自 artemis_config.h，串口 PARAM 命令只修改 RAM。 */
typedef struct {
    uint8_t line_pid_mode;
    float yaw_kp;
    float yaw_ki;
    float yaw_kd;
    float yaw_error_filter_previous;
    float yaw_error_filter_current;
    float yaw_turn_scale;
    float yaw_stable_error_deg;
    float line_original_kp;
    float line_original_ki;
    float line_original_kd;
    float line_output_base_scale;
    float line_fuzzy_error_normalizer;
    float line_fuzzy_delta_normalizer;
    float line_fuzzy_center_width;
    float line_fuzzy_steady_width;
    float line_fuzzy_kp_base;
    float line_fuzzy_kp_error_gain;
    float line_fuzzy_kp_delta_gain;
    float line_fuzzy_kd_base;
    float line_fuzzy_kd_error_gain;
    float line_fuzzy_kd_delta_gain;
    float line_fuzzy_ki_base;
    float line_fuzzy_integral_limit;
    float task_drive_velocity;
    float task_track_velocity;
    float task_entry_c_yaw_deg;
    float task_entry_d_yaw_deg;
    float task_turn_settle_s;
    float task_drive_max_s;
    float task_track_max_s;
    float task2_ab_yaw_deg;
    float task2_cd_yaw_deg;
    float task2_straight_min_cm;
    float task2_arc_min_cm;
} artemis_runtime_params_t;

const artemis_runtime_params_t *artemis_runtime_params_get(void);
void artemis_runtime_params_reset(void);
bool artemis_runtime_param_set(const char *name, float value);

#endif
