#include "line_indicator.h"

#include <limits.h>
#include <string.h>

#include "artemis_config.h"
#include "artemis_types.h"

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t) (now_ms - deadline_ms) >= 0;
}

typedef enum {
    LINE_SAMPLE_ABSENT,
    LINE_SAMPLE_PRESENT,
    LINE_SAMPLE_UNCERTAIN
} line_sample_t;

static uint8_t active_sensor_count(const uint8_t *digital_values, size_t digital_count)
{
    size_t index;
    uint8_t active_count = 0U;
    const size_t count = digital_count < ARTEMIS_MAX_LINE_SENSORS
        ? digital_count
        : ARTEMIS_MAX_LINE_SENSORS;

    for (index = 0U; index < count; index++) {
        if (digital_values[index] != 0U) {
            active_count++;
        }
    }
    return active_count;
}

static line_sample_t classify_line_sample(const uint8_t *digital_values, size_t digital_count)
{
    const uint8_t active_count = active_sensor_count(digital_values, digital_count);

    /*
     * 带滞回的黑线检测：
     * 单个传感器为 1 常见于边缘抖动，不作为入线；
     * 稳定离线必须全部传感器为 0，再由连续帧数确认。
     */
    if (active_count >= ARTEMIS_LED_ENTER_MIN_ACTIVE_SENSORS) {
        return LINE_SAMPLE_PRESENT;
    }
    if (active_count <= ARTEMIS_LED_EXIT_MAX_ACTIVE_SENSORS) {
        return LINE_SAMPLE_ABSENT;
    }
    return LINE_SAMPLE_UNCERTAIN;
}

static void request_pulse(line_indicator_t *indicator, uint32_t now_ms)
{
    /* 如果正在闪烁，新边沿只排队，保证两次闪烁之间有 100ms 灭灯间隔。 */
    if (indicator->phase == LINE_INDICATOR_IDLE) {
        indicator->phase = LINE_INDICATOR_ON;
        indicator->deadline_ms = now_ms + ARTEMIS_LED_ON_MS;
    } else if (indicator->pending_pulses < UCHAR_MAX) {
        indicator->pending_pulses++;
    }
}

void line_indicator_notify_edge(line_indicator_t *indicator, uint32_t now_ms)
{
    request_pulse(indicator, now_ms);
}

void line_indicator_reset(line_indicator_t *indicator)
{
    memset(indicator, 0, sizeof(*indicator));
    indicator->phase = LINE_INDICATOR_IDLE;
}

void line_indicator_update(
    line_indicator_t *indicator,
    const uint8_t *digital_values,
    size_t digital_count,
    uint32_t now_ms)
{
    const line_sample_t sample = classify_line_sample(digital_values, digital_count);
    const bool present = sample == LINE_SAMPLE_PRESENT;
    const uint16_t confirm_frames = present
        ? (uint16_t) ARTEMIS_LED_ENTER_CONFIRM_FRAMES
        : (uint16_t) ARTEMIS_LED_EXIT_CONFIRM_FRAMES;

    if (sample == LINE_SAMPLE_UNCERTAIN) {
        indicator->candidate_line_present = indicator->stable_line_present;
        indicator->candidate_frames = 0U;
        return;
    }
    if (present == indicator->stable_line_present) {
        indicator->candidate_line_present = present;
        indicator->candidate_frames = 0U;
        return;
    }
    if (present != indicator->candidate_line_present) {
        indicator->candidate_line_present = present;
        indicator->candidate_frames = 1U;
    } else if (indicator->candidate_frames < UINT16_MAX) {
        indicator->candidate_frames++;
    }
    if (indicator->candidate_frames >= confirm_frames) {
        indicator->stable_line_present = present;
        indicator->candidate_frames = 0U;
        request_pulse(indicator, now_ms);
    }
}

void line_indicator_tick(line_indicator_t *indicator, uint32_t now_ms)
{
    /* 非阻塞计时，主循环每次调用一次，不暂停串口接收和控制计算。 */
    while ((indicator->phase != LINE_INDICATOR_IDLE) &&
           time_reached(now_ms, indicator->deadline_ms)) {
        if (indicator->phase == LINE_INDICATOR_ON) {
            if (indicator->pending_pulses == 0U) {
                indicator->phase = LINE_INDICATOR_IDLE;
            } else {
                indicator->pending_pulses--;
                indicator->phase = LINE_INDICATOR_GAP;
                indicator->deadline_ms += ARTEMIS_LED_GAP_MS;
            }
        } else {
            indicator->phase = LINE_INDICATOR_ON;
            indicator->deadline_ms += ARTEMIS_LED_ON_MS;
        }
    }
}

bool line_indicator_is_on(const line_indicator_t *indicator)
{
    return indicator->phase == LINE_INDICATOR_ON;
}
