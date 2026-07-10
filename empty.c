#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#include "artemis_config.h"
#include "artemis_controller.h"
#include "artemis_protocol.h"
#include "line_indicator.h"
#include "uart_link.h"

typedef enum {
    APP_RETRY_START,
    APP_WAIT_STARTED,
    APP_WAIT_OBSERVATION,
    APP_WAIT_FINISHED,
    APP_DONE
} app_state_t;

static volatile uint32_t system_millis;
static app_state_t app_state;
static uint32_t app_deadline_ms;
static artemis_mission_t mission;
static line_indicator_t line_indicator;
static bool led_output_on;

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t) (now_ms - deadline_ms) >= 0;
}

static bool send_formatted(int length, const char *buffer, size_t buffer_size)
{
    if ((length < 0) || ((size_t) length >= buffer_size)) {
        return false;
    }
    uart_link_write(buffer);
    return true;
}

static void reset_runtime(uint32_t now_ms)
{
    artemis_mission_reset(&mission);
    line_indicator_reset(&line_indicator);
    led_output_on = false;
    DL_GPIO_clearPins(LED_PORT, LED_PIN_0_PIN);
    app_state = APP_RETRY_START;
    app_deadline_ms = now_ms + ARTEMIS_RETRY_DELAY_MS;
}

static void start_session(uint32_t now_ms)
{
    char output[ARTEMIS_UART_TX_BUFFER_SIZE];
    const int length = artemis_protocol_format_start(output, sizeof(output));

    artemis_mission_reset(&mission);
    line_indicator_reset(&line_indicator);
    DL_GPIO_clearPins(LED_PORT, LED_PIN_0_PIN);
    led_output_on = false;
    if (!send_formatted(length, output, sizeof(output))) {
        reset_runtime(now_ms);
        return;
    }
    app_state = APP_WAIT_STARTED;
    app_deadline_ms = now_ms + ARTEMIS_RESPONSE_TIMEOUT_MS;
}

static void send_stop(uint32_t now_ms)
{
    char output[ARTEMIS_UART_TX_BUFFER_SIZE];
    const int length =
        artemis_protocol_format_stop(output, sizeof(output), "task_completed");

    if (!send_formatted(length, output, sizeof(output))) {
        app_state = APP_DONE;
        return;
    }
    app_state = APP_WAIT_FINISHED;
    app_deadline_ms = now_ms + ARTEMIS_RESPONSE_TIMEOUT_MS;
}

static void handle_observation(const artemis_observation_t *observation, uint32_t now_ms)
{
    char output[ARTEMIS_UART_TX_BUFFER_SIZE];
    const artemis_control_command_t command = artemis_mission_step(&mission, observation);
    int length;

    line_indicator_update(
        &line_indicator,
        observation->digital_values,
        observation->digital_count,
        now_ms);
    if (command.completed) {
        send_stop(now_ms);
        return;
    }
    length = artemis_protocol_format_step(output, sizeof(output), &command);
    if (!send_formatted(length, output, sizeof(output))) {
        reset_runtime(now_ms);
        return;
    }
    app_state = APP_WAIT_OBSERVATION;
    app_deadline_ms = now_ms + ARTEMIS_RESPONSE_TIMEOUT_MS;
}

static void handle_response(const artemis_response_t *response, uint32_t now_ms)
{
    if (app_state == APP_DONE) {
        return;
    }
    switch (response->type) {
        case ARTEMIS_RESPONSE_STARTED:
            artemis_mission_reset(&mission);
            handle_observation(&response->observation, now_ms);
            break;
        case ARTEMIS_RESPONSE_OBSERVATION:
            if (app_state == APP_WAIT_OBSERVATION) {
                handle_observation(&response->observation, now_ms);
            } else {
                reset_runtime(now_ms);
            }
            break;
        case ARTEMIS_RESPONSE_FINISHED:
            app_state = APP_DONE;
            break;
        case ARTEMIS_RESPONSE_ERROR:
        default:
            reset_runtime(now_ms);
            break;
    }
}

static void update_led(uint32_t now_ms)
{
    line_indicator_tick(&line_indicator, now_ms);
    if (line_indicator_is_on(&line_indicator) == led_output_on) {
        return;
    }
    led_output_on = line_indicator_is_on(&line_indicator);
    if (led_output_on) {
        DL_GPIO_setPins(LED_PORT, LED_PIN_0_PIN);
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN_0_PIN);
    }
}

int main(void)
{
    char input[ARTEMIS_UART_LINE_BUFFER_SIZE];
    artemis_response_t response;

    SYSCFG_DL_init();
    uart_link_init();
    if (SysTick_Config(CPUCLK_FREQ / 1000U) != 0U) {
        while (1) {
            __WFI();
        }
    }
    artemis_mission_reset(&mission);
    line_indicator_reset(&line_indicator);
    app_state = APP_RETRY_START;
    app_deadline_ms = system_millis;

    while (1) {
        const uint32_t now_ms = system_millis;

        update_led(now_ms);
        if (uart_link_read_line(input, sizeof(input))) {
            if (artemis_protocol_parse_response(input, &response)) {
                handle_response(&response, now_ms);
            } else {
                reset_runtime(now_ms);
            }
        }
        if ((app_state == APP_RETRY_START) && time_reached(now_ms, app_deadline_ms)) {
            start_session(now_ms);
        } else if (((app_state == APP_WAIT_STARTED) ||
                    (app_state == APP_WAIT_OBSERVATION)) &&
                   time_reached(now_ms, app_deadline_ms)) {
            start_session(now_ms);
        } else if ((app_state == APP_WAIT_FINISHED) &&
                   time_reached(now_ms, app_deadline_ms)) {
            app_state = APP_DONE;
        }
        __WFI();
    }
}

void SysTick_Handler(void)
{
    system_millis++;
}
