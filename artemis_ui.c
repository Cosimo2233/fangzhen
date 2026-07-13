#include "artemis_ui.h"

#include <stdio.h>
#include <string.h>

#include "oled_hardware_i2c.h"
#include "ti_msp_dl_config.h"

#define ARTEMIS_UI_DEBOUNCE_MS 20U

typedef struct {
    bool stable_pressed;
    bool last_sample_pressed;
    uint32_t changed_at_ms;
} key_filter_t;

static key_filter_t key0_filter;
static key_filter_t key1_filter;
static artemis_task_id_t last_task_id = ARTEMIS_TASK_ID_3;
static artemis_ui_state_t last_state = ARTEMIS_UI_STATE_ERROR;

static bool key0_raw_pressed(void)
{
    return (DL_GPIO_readPins(GPIO_KEYS_PORT, GPIO_KEYS_PIN_KEY_0_PIN) &
            GPIO_KEYS_PIN_KEY_0_PIN) == 0U;
}

static bool key1_raw_pressed(void)
{
    return (DL_GPIO_readPins(GPIO_KEYS_PORT, GPIO_KEYS_PIN_KEY_1_PIN) &
            GPIO_KEYS_PIN_KEY_1_PIN) == 0U;
}

static bool key_filter_update(key_filter_t *filter, bool sample_pressed, uint32_t now_ms)
{
    bool pressed_event = false;

    if (sample_pressed != filter->last_sample_pressed) {
        filter->last_sample_pressed = sample_pressed;
        filter->changed_at_ms = now_ms;
    }
    if (((uint32_t) (now_ms - filter->changed_at_ms) >= ARTEMIS_UI_DEBOUNCE_MS) &&
        (filter->stable_pressed != sample_pressed)) {
        filter->stable_pressed = sample_pressed;
        pressed_event = sample_pressed;
    }
    return pressed_event;
}

static const char *task_name(artemis_task_id_t task_id)
{
    return task_id == ARTEMIS_TASK_ID_2 ? "TASK2" : "TASK3";
}

static const char *state_name(artemis_ui_state_t state)
{
    switch (state) {
        case ARTEMIS_UI_STATE_MENU:
            return "MENU";
        case ARTEMIS_UI_STATE_STARTING:
            return "START";
        case ARTEMIS_UI_STATE_RUNNING:
            return "RUN";
        case ARTEMIS_UI_STATE_DONE:
            return "DONE";
        case ARTEMIS_UI_STATE_ERROR:
        default:
            return "ERROR";
    }
}

void artemis_ui_init(void)
{
    memset(&key0_filter, 0, sizeof(key0_filter));
    memset(&key1_filter, 0, sizeof(key1_filter));
    OLED_Init();
    artemis_ui_show(ARTEMIS_TASK_ID_2, ARTEMIS_UI_STATE_MENU);
}

artemis_ui_event_t artemis_ui_poll(uint32_t now_ms)
{
    artemis_ui_event_t event;

    memset(&event, 0, sizeof(event));
    event.key0_pressed = key_filter_update(&key0_filter, key0_raw_pressed(), now_ms);
    event.key1_pressed = key_filter_update(&key1_filter, key1_raw_pressed(), now_ms);
    return event;
}

void artemis_ui_show(artemis_task_id_t task_id, artemis_ui_state_t state)
{
    char line[22];

    if ((task_id == last_task_id) && (state == last_state)) {
        return;
    }
    last_task_id = task_id;
    last_state = state;

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)" Artemis Vicon ", 8);
    (void) snprintf(line, sizeof(line), " Mode:%s", task_name(task_id));
    OLED_ShowString(0, 2, (uint8_t *)line, 8);
    (void) snprintf(line, sizeof(line), " State:%s", state_name(state));
    OLED_ShowString(0, 3, (uint8_t *)line, 8);
    OLED_ShowString(0, 5, (uint8_t *)" KEY0:SELECT", 8);
    OLED_ShowString(0, 6, (uint8_t *)" KEY1:START ", 8);
}
