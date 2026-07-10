#include "artemis_protocol.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "artemis_config.h"

typedef struct {
    const char *start;
    size_t length;
} token_view_t;

static const char *skip_space(const char *cursor)
{
    while ((*cursor != '\0') && isspace((unsigned char) *cursor)) {
        cursor++;
    }
    return cursor;
}

static bool next_token(const char **cursor, token_view_t *token)
{
    const char *start = skip_space(*cursor);
    const char *end = start;

    if (*start == '\0') {
        *cursor = start;
        return false;
    }
    while ((*end != '\0') && !isspace((unsigned char) *end)) {
        end++;
    }
    token->start = start;
    token->length = (size_t) (end - start);
    *cursor = end;
    return true;
}

static bool token_equals(token_view_t token, const char *text)
{
    const size_t text_length = strlen(text);
    return (token.length == text_length) && (strncmp(token.start, text, text_length) == 0);
}

static bool split_key_value(token_view_t token, token_view_t *key, token_view_t *value)
{
    const char *separator = memchr(token.start, '=', token.length);
    if ((separator == NULL) || (separator == token.start)) {
        return false;
    }
    key->start = token.start;
    key->length = (size_t) (separator - token.start);
    value->start = separator + 1;
    value->length = token.length - key->length - 1U;
    return true;
}

static bool copy_token(char *destination, size_t destination_size, token_view_t value)
{
    if ((destination_size == 0U) || (value.length >= destination_size)) {
        return false;
    }
    memcpy(destination, value.start, value.length);
    destination[value.length] = '\0';
    return true;
}

static bool parse_float_token(token_view_t value, float *parsed)
{
    char text[48];
    char *end = NULL;
    float result;

    if (!copy_token(text, sizeof(text), value) || (value.length == 0U)) {
        return false;
    }
    errno = 0;
    result = strtof(text, &end);
    if ((errno == ERANGE) || (end == text) || (*end != '\0') || !isfinite(result)) {
        return false;
    }
    *parsed = result;
    return true;
}

static bool parse_u32_token(token_view_t value, uint32_t *parsed)
{
    char text[24];
    char *end = NULL;
    unsigned long result;

    if (!copy_token(text, sizeof(text), value) || (value.length == 0U) || (value.start[0] == '-')) {
        return false;
    }
    errno = 0;
    result = strtoul(text, &end, 10);
    if ((errno == ERANGE) || (end == text) || (*end != '\0') || (result > UINT32_MAX)) {
        return false;
    }
    *parsed = (uint32_t) result;
    return true;
}

static bool parse_bool_token(token_view_t value, bool *parsed)
{
    if (token_equals(value, "1")) {
        *parsed = true;
        return true;
    }
    if (token_equals(value, "0")) {
        *parsed = false;
        return true;
    }
    return false;
}

static bool parse_digital_token(token_view_t value, artemis_observation_t *observation)
{
    size_t index;

    if (value.length == 0U) {
        return false;
    }
    memset(observation->digital_values, 0, sizeof(observation->digital_values));
    observation->digital_count =
        value.length < ARTEMIS_MAX_LINE_SENSORS ? value.length : ARTEMIS_MAX_LINE_SENSORS;
    for (index = 0U; index < value.length; index++) {
        const char bit = value.start[index];
        if ((bit != '0') && (bit != '1')) {
            return false;
        }
        if (index < ARTEMIS_MAX_LINE_SENSORS) {
            observation->digital_values[index] = (uint8_t) (bit == '1');
        }
    }
    return true;
}

static bool parse_observation_fields(const char *cursor, artemis_observation_t *observation)
{
    enum {
        SEEN_SEQUENCE = 1U << 0,
        SEEN_TIME = 1U << 1,
        SEEN_YAW = 1U << 2,
        SEEN_DIGITAL = 1U << 3,
        SEEN_LEFT_TICKS = 1U << 4,
        SEEN_RIGHT_TICKS = 1U << 5,
        SEEN_ALL = (1U << 6) - 1U
    };
    unsigned int seen = 0U;
    token_view_t token;

    memset(observation, 0, sizeof(*observation));
    while (next_token(&cursor, &token)) {
        token_view_t key;
        token_view_t value;
        if (!split_key_value(token, &key, &value)) {
            return false;
        }
        if (token_equals(key, "seq")) {
            if (!parse_u32_token(value, &observation->sequence_id)) {
                return false;
            }
            seen |= SEEN_SEQUENCE;
        } else if (token_equals(key, "t")) {
            if (!parse_float_token(value, &observation->sim_time_s)) {
                return false;
            }
            seen |= SEEN_TIME;
        } else if (token_equals(key, "yaw")) {
            if (!parse_float_token(value, &observation->yaw_deg)) {
                return false;
            }
            seen |= SEEN_YAW;
        } else if (token_equals(key, "dig")) {
            if (!parse_digital_token(value, observation)) {
                return false;
            }
            seen |= SEEN_DIGITAL;
        } else if (token_equals(key, "enc_tl")) {
            if (!parse_float_token(value, &observation->rear_left_total_ticks)) {
                return false;
            }
            seen |= SEEN_LEFT_TICKS;
        } else if (token_equals(key, "enc_tr")) {
            if (!parse_float_token(value, &observation->rear_right_total_ticks)) {
                return false;
            }
            seen |= SEEN_RIGHT_TICKS;
        }
    }
    if (seen != SEEN_ALL) {
        return false;
    }
    observation->forward_distance_cm =
        0.5f * (observation->rear_left_total_ticks + observation->rear_right_total_ticks) /
        ARTEMIS_ENCODER_TICKS_PER_CM;
    return true;
}

static bool parse_finished_fields(const char *cursor, artemis_finished_t *finished)
{
    enum {
        SEEN_REASON = 1U << 0,
        SEEN_GOAL = 1U << 1,
        SEEN_ELAPSED = 1U << 2,
        SEEN_ALL = (1U << 3) - 1U
    };
    unsigned int seen = 0U;
    token_view_t token;

    memset(finished, 0, sizeof(*finished));
    while (next_token(&cursor, &token)) {
        token_view_t key;
        token_view_t value;
        if (!split_key_value(token, &key, &value)) {
            return false;
        }
        if (token_equals(key, "reason")) {
            if (!copy_token(finished->reason, sizeof(finished->reason), value)) {
                return false;
            }
            seen |= SEEN_REASON;
        } else if (token_equals(key, "reached_goal")) {
            if (!parse_bool_token(value, &finished->reached_goal)) {
                return false;
            }
            seen |= SEEN_GOAL;
        } else if (token_equals(key, "elapsed_time_s")) {
            if (!parse_float_token(value, &finished->elapsed_time_s)) {
                return false;
            }
            seen |= SEEN_ELAPSED;
        }
    }
    return seen == SEEN_ALL;
}

static bool parse_error_fields(const char *cursor, char *message, size_t message_size)
{
    token_view_t token;
    while (next_token(&cursor, &token)) {
        token_view_t key;
        token_view_t value;
        if (!split_key_value(token, &key, &value)) {
            return false;
        }
        if (token_equals(key, "message")) {
            return copy_token(message, message_size, value);
        }
    }
    return false;
}

static int format_fixed6(char *buffer, size_t buffer_size, float value)
{
    float scaled_float;
    int32_t scaled;
    uint32_t magnitude;
    uint32_t integer_part;
    uint32_t fractional_part;
    const char *sign = "";

    if (!isfinite(value)) {
        return -1;
    }
    scaled_float = value * 1000000.0f;
    if ((scaled_float > (float) INT32_MAX) || (scaled_float < (float) INT32_MIN)) {
        return -1;
    }
    scaled = (int32_t) (scaled_float >= 0.0f ? scaled_float + 0.5f : scaled_float - 0.5f);
    if (scaled < 0) {
        sign = "-";
        magnitude = (uint32_t) -scaled;
    } else {
        magnitude = (uint32_t) scaled;
    }
    integer_part = magnitude / 1000000U;
    fractional_part = magnitude % 1000000U;
    return snprintf(
        buffer,
        buffer_size,
        "%s%lu.%06lu",
        sign,
        (unsigned long) integer_part,
        (unsigned long) fractional_part);
}

bool artemis_protocol_parse_response(const char *line, artemis_response_t *response)
{
    const char *cursor = line;
    token_view_t prefix;

    if ((line == NULL) || (response == NULL)) {
        return false;
    }
    memset(response, 0, sizeof(*response));
    if (!next_token(&cursor, &prefix)) {
        return false;
    }
    if (token_equals(prefix, "STARTED")) {
        response->type = ARTEMIS_RESPONSE_STARTED;
        return parse_observation_fields(cursor, &response->observation);
    }
    if (token_equals(prefix, "OBS")) {
        response->type = ARTEMIS_RESPONSE_OBSERVATION;
        return parse_observation_fields(cursor, &response->observation);
    }
    if (token_equals(prefix, "FINISHED")) {
        response->type = ARTEMIS_RESPONSE_FINISHED;
        return parse_finished_fields(cursor, &response->finished);
    }
    if (token_equals(prefix, "ERR")) {
        response->type = ARTEMIS_RESPONSE_ERROR;
        return parse_error_fields(cursor, response->error_message, sizeof(response->error_message));
    }
    return false;
}

int artemis_protocol_format_start(char *buffer, size_t buffer_size)
{
    static const char start_command[] =
        "START max_time_s=120 control_period_s=0.02 initial_progress_index=0\n";

    if (buffer_size < sizeof(start_command)) {
        return (int) (sizeof(start_command) - 1U);
    }
    memcpy(buffer, start_command, sizeof(start_command));
    return (int) (sizeof(start_command) - 1U);
}

int artemis_protocol_format_step(
    char *buffer,
    size_t buffer_size,
    const artemis_control_command_t *command)
{
    char left[24];
    char right[24];
    int left_length;
    int right_length;

    if (command == NULL) {
        return -1;
    }
    left_length = format_fixed6(left, sizeof(left), command->rear_left_target_speed);
    right_length = format_fixed6(right, sizeof(right), command->rear_right_target_speed);
    if ((left_length < 0) || ((size_t) left_length >= sizeof(left)) ||
        (right_length < 0) || ((size_t) right_length >= sizeof(right))) {
        return -1;
    }
    return snprintf(
        buffer,
        buffer_size,
        "STEP seq=%lu left=%s right=%s\n",
        (unsigned long) command->sequence_id,
        left,
        right);
}

int artemis_protocol_format_stop(char *buffer, size_t buffer_size, const char *reason)
{
    const char *safe_reason = (reason == NULL) || (*reason == '\0') ? "mcu_stop" : reason;
    return snprintf(buffer, buffer_size, "STOP reason=%s\n", safe_reason);
}
