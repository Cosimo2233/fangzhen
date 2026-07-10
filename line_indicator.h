#ifndef LINE_INDICATOR_H
#define LINE_INDICATOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    LINE_INDICATOR_IDLE,
    LINE_INDICATOR_ON,
    LINE_INDICATOR_GAP
} line_indicator_phase_t;

/* PA14 黑线边沿提示灯状态。pending_pulses 用来排队重叠闪烁事件。 */
typedef struct {
    uint32_t deadline_ms;
    uint8_t candidate_frames;
    uint8_t pending_pulses;
    line_indicator_phase_t phase;
    bool stable_line_present;
    bool candidate_line_present;
} line_indicator_t;

void line_indicator_reset(line_indicator_t *indicator);
void line_indicator_update(
    line_indicator_t *indicator,
    const uint8_t *digital_values,
    size_t digital_count,
    uint32_t now_ms);
void line_indicator_tick(line_indicator_t *indicator, uint32_t now_ms);
bool line_indicator_is_on(const line_indicator_t *indicator);

#endif
