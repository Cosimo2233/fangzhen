#include "artemis_runtime_params.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "artemis_config.h"

static const artemis_runtime_params_t default_params = {
    ARTEMIS_LINE_PID_MODE,
    ARTEMIS_YAW_PID_KP,
    ARTEMIS_YAW_PID_KI,
    ARTEMIS_YAW_PID_KD,
    ARTEMIS_YAW_ERROR_FILTER_PREVIOUS,
    ARTEMIS_YAW_ERROR_FILTER_CURRENT,
    ARTEMIS_YAW_TURN_SCALE,
    ARTEMIS_YAW_STABLE_ERROR_DEG,
    ARTEMIS_LINE_ORIGINAL_KP,
    ARTEMIS_LINE_ORIGINAL_KI,
    ARTEMIS_LINE_ORIGINAL_KD,
    ARTEMIS_LINE_OUTPUT_BASE_SCALE,
    ARTEMIS_LINE_FUZZY_ERROR_NORMALIZER,
    ARTEMIS_LINE_FUZZY_DELTA_NORMALIZER,
    ARTEMIS_LINE_FUZZY_CENTER_WIDTH,
    ARTEMIS_LINE_FUZZY_STEADY_WIDTH,
    ARTEMIS_LINE_FUZZY_KP_BASE,
    ARTEMIS_LINE_FUZZY_KP_ERROR_GAIN,
    ARTEMIS_LINE_FUZZY_KP_DELTA_GAIN,
    ARTEMIS_LINE_FUZZY_KD_BASE,
    ARTEMIS_LINE_FUZZY_KD_ERROR_GAIN,
    ARTEMIS_LINE_FUZZY_KD_DELTA_GAIN,
    ARTEMIS_LINE_FUZZY_KI_BASE,
    ARTEMIS_LINE_FUZZY_INTEGRAL_LIMIT,
};

static artemis_runtime_params_t runtime_params = {
    ARTEMIS_LINE_PID_MODE,
    ARTEMIS_YAW_PID_KP,
    ARTEMIS_YAW_PID_KI,
    ARTEMIS_YAW_PID_KD,
    ARTEMIS_YAW_ERROR_FILTER_PREVIOUS,
    ARTEMIS_YAW_ERROR_FILTER_CURRENT,
    ARTEMIS_YAW_TURN_SCALE,
    ARTEMIS_YAW_STABLE_ERROR_DEG,
    ARTEMIS_LINE_ORIGINAL_KP,
    ARTEMIS_LINE_ORIGINAL_KI,
    ARTEMIS_LINE_ORIGINAL_KD,
    ARTEMIS_LINE_OUTPUT_BASE_SCALE,
    ARTEMIS_LINE_FUZZY_ERROR_NORMALIZER,
    ARTEMIS_LINE_FUZZY_DELTA_NORMALIZER,
    ARTEMIS_LINE_FUZZY_CENTER_WIDTH,
    ARTEMIS_LINE_FUZZY_STEADY_WIDTH,
    ARTEMIS_LINE_FUZZY_KP_BASE,
    ARTEMIS_LINE_FUZZY_KP_ERROR_GAIN,
    ARTEMIS_LINE_FUZZY_KP_DELTA_GAIN,
    ARTEMIS_LINE_FUZZY_KD_BASE,
    ARTEMIS_LINE_FUZZY_KD_ERROR_GAIN,
    ARTEMIS_LINE_FUZZY_KD_DELTA_GAIN,
    ARTEMIS_LINE_FUZZY_KI_BASE,
    ARTEMIS_LINE_FUZZY_INTEGRAL_LIMIT,
};

typedef struct {
    const char *name;
    float *slot;
} float_param_entry_t;

const artemis_runtime_params_t *artemis_runtime_params_get(void)
{
    return &runtime_params;
}

void artemis_runtime_params_reset(void)
{
    runtime_params = default_params;
}

static bool set_line_mode(float value)
{
    if (!isfinite(value)) {
        return false;
    }
    runtime_params.line_pid_mode =
        value >= 0.5f ? ARTEMIS_LINE_PID_FUZZY : ARTEMIS_LINE_PID_ORIGINAL;
    return true;
}

bool artemis_runtime_param_set(const char *name, float value)
{
    size_t index;
    const float_param_entry_t float_params[] = {
        {"yaw_kp", &runtime_params.yaw_kp},
        {"yaw_ki", &runtime_params.yaw_ki},
        {"yaw_kd", &runtime_params.yaw_kd},
        {"yaw_filter_previous", &runtime_params.yaw_error_filter_previous},
        {"yaw_filter_current", &runtime_params.yaw_error_filter_current},
        {"yaw_turn_scale", &runtime_params.yaw_turn_scale},
        {"yaw_stable_error_deg", &runtime_params.yaw_stable_error_deg},
        {"line_kp", &runtime_params.line_original_kp},
        {"line_ki", &runtime_params.line_original_ki},
        {"line_kd", &runtime_params.line_original_kd},
        {"line_scale", &runtime_params.line_output_base_scale},
        {"fuzzy_error_normalizer", &runtime_params.line_fuzzy_error_normalizer},
        {"fuzzy_delta_normalizer", &runtime_params.line_fuzzy_delta_normalizer},
        {"fuzzy_center_width", &runtime_params.line_fuzzy_center_width},
        {"fuzzy_steady_width", &runtime_params.line_fuzzy_steady_width},
        {"fuzzy_kp_base", &runtime_params.line_fuzzy_kp_base},
        {"fuzzy_kp_error_gain", &runtime_params.line_fuzzy_kp_error_gain},
        {"fuzzy_kp_delta_gain", &runtime_params.line_fuzzy_kp_delta_gain},
        {"fuzzy_kd_base", &runtime_params.line_fuzzy_kd_base},
        {"fuzzy_kd_error_gain", &runtime_params.line_fuzzy_kd_error_gain},
        {"fuzzy_kd_delta_gain", &runtime_params.line_fuzzy_kd_delta_gain},
        {"fuzzy_ki_base", &runtime_params.line_fuzzy_ki_base},
        {"fuzzy_integral_limit", &runtime_params.line_fuzzy_integral_limit},
    };

    if ((name == NULL) || !isfinite(value)) {
        return false;
    }
    if (strcmp(name, "line_mode") == 0) {
        return set_line_mode(value);
    }
    for (index = 0U; index < sizeof(float_params) / sizeof(float_params[0]); index++) {
        if (strcmp(name, float_params[index].name) == 0) {
            *float_params[index].slot = value;
            return true;
        }
    }
    return false;
}
