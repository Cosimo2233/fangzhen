#include "line_indicator.h"

#include <limits.h>
#include <string.h>

#include "artemis_config.h"
#include "artemis_types.h"

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t) (now_ms - deadline_ms) >= 0;
}

static bool detect_line(const uint8_t *digital_values, size_t digital_count)
{
    size_t index;
    const size_t count = digital_count < ARTEMIS_MAX_LINE_SENSORS
        ? digital_count
        : ARTEMIS_MAX_LINE_SENSORS;
    /* 任意一个巡线位为 1 就认为当前传感器接触黑线。 */
    for (index = 0U; index < count; index++) {
        if (digital_values[index] != 0U) {
            return true;
        }
    }
    return false;
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
    const bool present = detect_line(digital_values, digital_count);

    /* 接触/离开都要求连续 ARTEMIS_LED_CONFIRM_FRAMES 帧确认，过滤单帧抖动。 */
    if (present == indicator->stable_line_present) {
        indicator->candidate_line_present = present;
        indicator->candidate_frames = 0U;
        return;
    }
    if (present != indicator->candidate_line_present) {
        indicator->candidate_line_present = present;
        indicator->candidate_frames = 1U;
    } else if (indicator->candidate_frames < UCHAR_MAX) {
        indicator->candidate_frames++;
    }
    if (indicator->candidate_frames >= ARTEMIS_LED_CONFIRM_FRAMES) {
        indicator->stable_line_present = present;
        indicator->candidate_frames = 0U;
        request_pulse(indicator, now_ms);
    }
}

void line_indicator_tick(line_indicator_t *indicator, uint32_t now_ms)
{
    /* 非阻塞计时。主循环每次调用一次，不暂停串口接收和控制计算。 */
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
