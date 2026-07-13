#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#include "artemis_config.h"
#include "artemis_controller.h"
#include "artemis_protocol.h"
#include "artemis_runtime_params.h"
#include "artemis_ui.h"
#include "line_indicator.h"
#include "uart_link.h"

typedef enum {
    APP_MENU,
    APP_RETRY_START,
    APP_WAIT_STARTED,
    APP_WAIT_OBSERVATION,
    APP_WAIT_FINISHED,
    APP_DONE
} app_state_t;

/* 主循环状态。大缓冲放在静态区，避免 MSPM0 小栈被协议解析调用链压爆。 */
static volatile uint32_t system_millis;
static app_state_t app_state;
static uint32_t app_deadline_ms;
static artemis_mission_t mission;
static line_indicator_t line_indicator;
static bool led_output_on;
static bool track_exit_led_fired;
static uint8_t led_action_index;
static char app_input_buffer[ARTEMIS_UART_LINE_BUFFER_SIZE];
static char app_tx_buffer[ARTEMIS_UART_TX_BUFFER_SIZE];
static artemis_response_t app_response;
static artemis_config_command_t app_config_command;
static artemis_task_id_t selected_task = ARTEMIS_TASK_ID_2;

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t) (now_ms - deadline_ms) >= 0;
}

static bool send_formatted(int length, const char *buffer, size_t buffer_size)
{
    /* snprintf 返回值 >= buffer_size 表示输出被截断，不能发送半条协议。 */
    if ((length < 0) || ((size_t) length >= buffer_size)) {
        return false;
    }
    uart_link_write(buffer);
    return true;
}

static void reset_runtime(uint32_t now_ms)
{
    /* 仅用于握手阶段或发送失败后的重新开始。正式任务启动后不再反复 START。 */
    artemis_mission_reset(&mission);
    artemis_mission_set_task(&mission, selected_task);
    line_indicator_reset(&line_indicator);
    track_exit_led_fired = false;
    led_action_index = 0U;
    led_output_on = false;
    DL_GPIO_clearPins(LED_PORT, LED_PIN_0_PIN);
    app_state = APP_RETRY_START;
    app_deadline_ms = now_ms + ARTEMIS_RETRY_DELAY_MS;
}

static void stop_after_started_fault(void)
{
    /* 握手成功后的异常先静默停机，避免把 Mudri 会话反复拉回起点。 */
    app_state = APP_DONE;
    artemis_ui_show(selected_task, ARTEMIS_UI_STATE_ERROR);
}

static void start_session(uint32_t now_ms)
{
    /* 上电或握手超时后发送 START，请桥接软件启动一次仿真会话。 */
    const int length = artemis_protocol_format_start(app_tx_buffer, sizeof(app_tx_buffer));

    artemis_mission_reset(&mission);
    artemis_mission_set_task(&mission, selected_task);
    line_indicator_reset(&line_indicator);
    track_exit_led_fired = false;
    led_action_index = 0U;
    DL_GPIO_clearPins(LED_PORT, LED_PIN_0_PIN);
    led_output_on = false;
    if (!send_formatted(length, app_tx_buffer, sizeof(app_tx_buffer))) {
        reset_runtime(now_ms);
        return;
    }
    app_state = APP_WAIT_STARTED;
    app_deadline_ms = now_ms + ARTEMIS_START_RESPONSE_TIMEOUT_MS;
    artemis_ui_show(selected_task, ARTEMIS_UI_STATE_STARTING);
}

static void send_stop(uint32_t now_ms)
{
    /* task3 完成后显式通知桥接软件停止会话。 */
    const int length =
        artemis_protocol_format_stop(app_tx_buffer, sizeof(app_tx_buffer), "task_completed");

    if (!send_formatted(length, app_tx_buffer, sizeof(app_tx_buffer))) {
        app_state = APP_DONE;
        return;
    }
    app_state = APP_WAIT_FINISHED;
    app_deadline_ms = now_ms + ARTEMIS_RESPONSE_TIMEOUT_MS;
}

static bool observation_has_line(const artemis_observation_t *observation)
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

static void update_line_indicator_from_mission(
    uint8_t previous_action_index,
    const artemis_observation_t *observation,
    uint32_t now_ms)
{
    const bool has_line = observation_has_line(observation);
    const bool current_is_track =
        (mission.action_index == 1U) || (mission.action_index == 4U);
    const float track_elapsed_s = observation->sim_time_s - mission.action_started_at_s;

    if (mission.action_index != led_action_index) {
        led_action_index = mission.action_index;
        if (current_is_track) {
            track_exit_led_fired = false;
        }
    }

    /*
     * PA14 只跟随任务状态机相关事件：
     * 0/3 找线动作结束并且当前确实见线，表示进入 C/D 点巡线；
     * 1/4 巡线稳定一段时间后，连续全 0 达到较短阈值时，提前提示已经出线；
     * 若提前提示没有触发，巡线动作正式结束时再补一次。
     */
    if ((mission.action_index == previous_action_index) && current_is_track &&
        mission.line_seen && !has_line && !track_exit_led_fired &&
        (track_elapsed_s >= ARTEMIS_LED_TRACK_EXIT_MIN_TRACK_S) &&
        (mission.confirm_count >= ARTEMIS_LED_TRACK_EXIT_CONFIRM_FRAMES)) {
        track_exit_led_fired = true;
        line_indicator_notify_edge(&line_indicator, now_ms);
        return;
    }
    if (mission.action_index == previous_action_index) {
        return;
    }

    if (((previous_action_index == 0U) || (previous_action_index == 3U)) && has_line) {
        line_indicator_notify_edge(&line_indicator, now_ms);
    } else if (((previous_action_index == 1U) || (previous_action_index == 4U)) &&
               !has_line && !track_exit_led_fired) {
        track_exit_led_fired = true;
        line_indicator_notify_edge(&line_indicator, now_ms);
    }
}
static void handle_observation(const artemis_observation_t *observation, uint32_t now_ms)
{
    /* 每收到 STARTED/OBS 中的一帧观测，就推进一次任务并回一条 STEP。 */
    const uint8_t previous_action_index = mission.action_index;
    const artemis_control_command_t command = artemis_mission_step(&mission, observation);
    int length;

    update_line_indicator_from_mission(previous_action_index, observation, now_ms);
    if (command.completed) {
        send_stop(now_ms);
        return;
    }
    length = artemis_protocol_format_step(app_tx_buffer, sizeof(app_tx_buffer), &command);
    if (!send_formatted(length, app_tx_buffer, sizeof(app_tx_buffer))) {
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
            /* STARTED 自带第一帧观测，握手成功后立即进入正式任务。 */
            if (app_state == APP_WAIT_STARTED) {
                artemis_mission_reset(&mission);
                artemis_mission_set_task(&mission, selected_task);
                artemis_ui_show(selected_task, ARTEMIS_UI_STATE_RUNNING);
                handle_observation(&response->observation, now_ms);
            }
            break;
        case ARTEMIS_RESPONSE_OBSERVATION:
            /* 正式任务阶段只接受 OBS，收到一帧才发送下一条 STEP。 */
            if (app_state == APP_WAIT_OBSERVATION) {
                handle_observation(&response->observation, now_ms);
            } else if (app_state == APP_WAIT_STARTED) {
                reset_runtime(now_ms);
            }
            break;
        case ARTEMIS_RESPONSE_FINISHED:
            app_state = APP_DONE;
            artemis_ui_show(selected_task, ARTEMIS_UI_STATE_DONE);
            break;
        case ARTEMIS_RESPONSE_ERROR:
        default:
            if (app_state == APP_WAIT_STARTED) {
                reset_runtime(now_ms);
            } else {
                stop_after_started_fault();
            }
            break;
    }
}

static void handle_config_command(const artemis_config_command_t *command)
{
    if (command->type == ARTEMIS_CONFIG_COMMAND_RESET_PARAMS) {
        artemis_runtime_params_reset();
    } else {
        (void) artemis_runtime_param_set(command->name, command->value);
    }
}

static void update_led(uint32_t now_ms)
{
    /* PA14 LED 闪烁由 SysTick 时间驱动，不阻塞串口和控制计算。 */
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

static void toggle_selected_task(void)
{
    selected_task = selected_task == ARTEMIS_TASK_ID_2
        ? ARTEMIS_TASK_ID_3
        : ARTEMIS_TASK_ID_2;
    artemis_mission_set_task(&mission, selected_task);
    artemis_ui_show(selected_task, ARTEMIS_UI_STATE_MENU);
}

static void handle_ui(uint32_t now_ms)
{
    const artemis_ui_event_t event = artemis_ui_poll(now_ms);

    if ((app_state != APP_MENU) && (app_state != APP_DONE)) {
        return;
    }
    if (event.key0_pressed) {
        toggle_selected_task();
    }
    if (event.key1_pressed) {
        app_state = APP_RETRY_START;
        app_deadline_ms = now_ms;
        artemis_ui_show(selected_task, ARTEMIS_UI_STATE_STARTING);
    }
}

int main(void)
{
    /* 初始化硬件、串口、1ms SysTick，然后进入无 RTOS 主循环。 */
    SYSCFG_DL_init();
    uart_link_init();
    if (SysTick_Config(CPUCLK_FREQ / 1000U) != 0U) {
        while (1) {
            __WFI();
        }
    }
    artemis_mission_reset(&mission);
    artemis_mission_set_task(&mission, selected_task);
    line_indicator_reset(&line_indicator);
    app_state = APP_MENU;
    app_deadline_ms = 0U;
    artemis_ui_init();
    artemis_ui_show(selected_task, ARTEMIS_UI_STATE_MENU);

    while (1) {
        const uint32_t now_ms = system_millis;

        update_led(now_ms);
        handle_ui(now_ms);
        /* 先处理桥接软件响应，再检查状态机超时。 */
        if (uart_link_read_line(app_input_buffer, sizeof(app_input_buffer))) {
            if (artemis_protocol_parse_config_command(app_input_buffer, &app_config_command)) {
                handle_config_command(&app_config_command);
            } else if (artemis_protocol_is_config_command(app_input_buffer)) {
                /* 调参命令格式错误时忽略，不打断正在运行的仿真会话。 */
            } else if (artemis_protocol_parse_response(app_input_buffer, &app_response)) {
                if ((app_state != APP_MENU) && (app_state != APP_DONE)) {
                    handle_response(&app_response, now_ms);
                }
            } else if (app_state == APP_WAIT_STARTED) {
                reset_runtime(now_ms);
            } else if ((app_state != APP_MENU) && (app_state != APP_DONE)) {
                stop_after_started_fault();
            }
        }
        if ((app_state == APP_RETRY_START) && time_reached(now_ms, app_deadline_ms)) {
            start_session(now_ms);
        } else if ((app_state == APP_WAIT_STARTED) && time_reached(now_ms, app_deadline_ms)) {
            /* 只有握手阶段会重发 START，避免正式任务中反复重启仿真。 */
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

