#ifndef ARTEMIS_PROTOCOL_H
#define ARTEMIS_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>

#include "artemis_types.h"

/* 桥接软件发回 MCU 的四类响应。 */
typedef enum {
    ARTEMIS_RESPONSE_STARTED,
    ARTEMIS_RESPONSE_OBSERVATION,
    ARTEMIS_RESPONSE_FINISHED,
    ARTEMIS_RESPONSE_ERROR
} artemis_response_type_t;

/* FINISHED 响应携带的任务结束信息。 */
typedef struct {
    char reason[64];
    bool reached_goal;
    float elapsed_time_s;
} artemis_finished_t;

/* 协议解析结果。不同 type 只使用对应的联合含义字段。 */
typedef struct {
    artemis_response_type_t type;
    artemis_observation_t observation;
    artemis_finished_t finished;
    char error_message[128];
} artemis_response_t;

typedef enum {
    ARTEMIS_CONFIG_COMMAND_SET_PARAM,
    ARTEMIS_CONFIG_COMMAND_RESET_PARAMS
} artemis_config_command_type_t;

typedef struct {
    artemis_config_command_type_t type;
    char name[40];
    float value;
} artemis_config_command_t;

/* 解析 STARTED/OBS/FINISHED/ERR，字段按 key=value 读取，不依赖顺序。 */
bool artemis_protocol_parse_response(const char *line, artemis_response_t *response);
bool artemis_protocol_is_config_command(const char *line);
bool artemis_protocol_parse_config_command(
    const char *line,
    artemis_config_command_t *command);
/* 格式化 MCU -> 桥接软件的 START 命令。 */
int artemis_protocol_format_start(char *buffer, size_t buffer_size);
/* 格式化 STEP 命令。速度用定点方式输出，避免 Keil printf 浮点支持问题。 */
int artemis_protocol_format_step(
    char *buffer,
    size_t buffer_size,
    const artemis_control_command_t *command);
/* 格式化 STOP 命令。 */
int artemis_protocol_format_stop(char *buffer, size_t buffer_size, const char *reason);

#endif
