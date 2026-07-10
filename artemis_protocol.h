#ifndef ARTEMIS_PROTOCOL_H
#define ARTEMIS_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>

#include "artemis_types.h"

typedef enum {
    ARTEMIS_RESPONSE_STARTED,
    ARTEMIS_RESPONSE_OBSERVATION,
    ARTEMIS_RESPONSE_FINISHED,
    ARTEMIS_RESPONSE_ERROR
} artemis_response_type_t;

typedef struct {
    char reason[64];
    bool reached_goal;
    float elapsed_time_s;
} artemis_finished_t;

typedef struct {
    artemis_response_type_t type;
    artemis_observation_t observation;
    artemis_finished_t finished;
    char error_message[128];
} artemis_response_t;

bool artemis_protocol_parse_response(const char *line, artemis_response_t *response);
int artemis_protocol_format_start(char *buffer, size_t buffer_size);
int artemis_protocol_format_step(
    char *buffer,
    size_t buffer_size,
    const artemis_control_command_t *command);
int artemis_protocol_format_stop(char *buffer, size_t buffer_size, const char *reason);

#endif
